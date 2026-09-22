// SPDX-License-Identifier: Apache-2.0 OR MIT
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <vector>

#include "../avbd_cpu.h"
#include "../avbd_sim.h"
#include "../cloth_grid.h"

namespace {

constexpr int SUBSTEPS = 400; // 2.0 s at H = 5 ms

struct Tile {
	AvbdCpu solver;
	PanelTile info;
	std::vector<float> triK, bendK, attachK;

	void setup(int iters) {
		const float invHSq = 1.0f / (ClothSim::H * ClothSim::H);
		const ClothMesh &m = info.mesh;
		solver.setupMesh(m.nVerts(), m.positions.data(), m.positions.data(),
				m.mass.data(), invHSq);
		triK.assign(m.nTri(), ClothSim::MEMBRANE_K);
		bendK.assign(m.nBend(), ClothSim::BEND_K);
		attachK.assign(m.nAttach(), ClothSim::ATTACH_K);
		solver.uploadTriangles(m.nTri(), m.triIdx.data(), m.triInvUV.data(), triK.data());
		solver.uploadBendings(m.nBend(), m.bendIdx.data(), m.bendWeight.data(),
				m.bendNTarget.data(), bendK.data());
		solver.uploadAttachments(m.nAttach(), m.attachVert.data(), m.attachFixed.data(),
				attachK.data());
		solver.restrictToOwned(info.nOwned);
		(void)iters;
	}
};

// Returns final hang depth (mm) and the ideal N-core wallclock (ms): the total
// per-tile solve time divided by the tile count, since the tiles run at once.
void run(uint32_t nTiles, int iters, float &hangMm, double &parallelMs) {
	ClothMesh panel = build_cloth_mesh(10, 14, 0.5f, 0.7f, 1.2f, 0.3f);
	const uint32_t nV = panel.nVerts();
	std::vector<PanelTile> parts = split_connected_panel(panel, nTiles);
	std::vector<Tile> tiles(nTiles);
	for (uint32_t t = 0; t < nTiles; ++t) {
		tiles[t].info = std::move(parts[t]);
		tiles[t].setup(iters);
	}

	std::vector<float> gpos = panel.positions;
	std::vector<float> gvel(3 * nV, 0.0f);
	std::vector<std::vector<float>> ownedNew(nTiles);
	double solveMs = 0.0;

	for (int s = 0; s < SUBSTEPS; ++s) {
		for (uint32_t t = 0; t < nTiles; ++t) {
			Tile &tile = tiles[t];
			const uint32_t nLocal = tile.info.mesh.nVerts();
			std::vector<float> lpos(3 * nLocal), lpred(3 * nLocal);
			for (uint32_t l = 0; l < nLocal; ++l) {
				const uint32_t g = tile.info.globalId[l];
				lpos[3 * l + 0] = gpos[3 * g + 0];
				lpos[3 * l + 1] = gpos[3 * g + 1];
				lpos[3 * l + 2] = gpos[3 * g + 2];
			}
			for (uint32_t l = 0; l < nLocal; ++l) {
				const uint32_t g = tile.info.globalId[l];
				const float gy = (l < tile.info.nOwned) ? (ClothSim::H * ClothSim::H * ClothSim::GRAVITY_Y) : 0.0f;
				lpred[3 * l + 0] = lpos[3 * l + 0] + ClothSim::H * gvel[3 * g + 0];
				lpred[3 * l + 1] = lpos[3 * l + 1] + ClothSim::H * gvel[3 * g + 1] + gy;
				lpred[3 * l + 2] = lpos[3 * l + 2] + ClothSim::H * gvel[3 * g + 2];
			}
			tile.solver.updateState(lpos.data(), lpred.data());
			const auto t0 = std::chrono::steady_clock::now();
			for (int it = 0; it < iters; ++it) {
				tile.solver.step();
			}
			const auto t1 = std::chrono::steady_clock::now();
			solveMs += std::chrono::duration<double, std::milli>(t1 - t0).count();
			std::vector<float> lnew;
			tile.solver.readPositions(lnew);
			ownedNew[t].assign(3 * tile.info.nOwned, 0.0f);
			for (uint32_t l = 0; l < tile.info.nOwned; ++l) {
				ownedNew[t][3 * l + 0] = lnew[3 * l + 0];
				ownedNew[t][3 * l + 1] = lnew[3 * l + 1];
				ownedNew[t][3 * l + 2] = lnew[3 * l + 2];
			}
		}
		std::vector<float> newpos = gpos;
		for (uint32_t t = 0; t < nTiles; ++t) {
			for (uint32_t l = 0; l < tiles[t].info.nOwned; ++l) {
				const uint32_t g = tiles[t].info.globalId[l];
				newpos[3 * g + 0] = ownedNew[t][3 * l + 0];
				newpos[3 * g + 1] = ownedNew[t][3 * l + 1];
				newpos[3 * g + 2] = ownedNew[t][3 * l + 2];
			}
		}
		for (uint32_t i = 0; i < 3 * nV; ++i) {
			gvel[i] = (newpos[i] - gpos[i]) / ClothSim::H;
		}
		gpos = newpos;
	}

	float lo = gpos[1];
	for (uint32_t i = 0; i < nV; ++i) {
		if (gpos[3 * i + 1] < lo) {
			lo = gpos[3 * i + 1];
		}
	}
	hangMm = (1.2f - lo) * 1000.0f;
	parallelMs = solveMs / double(nTiles) / double(SUBSTEPS / 4.0);
}

} // namespace

int main() {
	const int iters[] = { 10, 20, 40, 80 };
	std::printf("curtain 10x14, mass 0.3, 2.0 s sim; parallelMs = ideal per-frame solve on N cores\n");
	std::printf("%7s  %6s  %10s  %14s\n", "tiles", "iters", "hang(mm)", "parallelMs(native)");
	for (uint32_t nt : { 1u, 8u }) {
		for (int it : iters) {
			float hang;
			double pms;
			run(nt, it, hang, pms);
			std::printf("%7u  %6d  %10.0f  %14.3f\n", nt, it, hang, pms);
		}
	}
	std::printf("\nSame per-frame wallclock lands on the same column of parallelMs: compare hang\n");
	std::printf("across tiles=1 vs tiles=8 at equal parallelMs. Sandbox cost is ~100x native.\n");
	return 0;
}
