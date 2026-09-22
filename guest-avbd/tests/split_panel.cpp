// SPDX-License-Identifier: Apache-2.0 OR MIT

#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <thread>
#include <vector>

#include "../avbd_sim.h"

namespace {

constexpr uint32_t NX = 10, NY = 14;
constexpr uint32_t NTILES = 4;
constexpr int SUBSTEPS = 300;

struct Tile {
	AvbdCpu solver;
	PanelTile info;
	std::vector<float> triK, bendK, attachK;

	void setup() {
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
	}
};

std::vector<float> run_decomposed(uint32_t nTiles, bool parallel, double *outMs) {
	ClothMesh panel = build_cloth_mesh(NX, NY, 0.5f, 0.7f, 1.2f, 0.05f);
	const uint32_t nV = panel.nVerts();
	std::vector<Tile> tiles(nTiles);
	{
		std::vector<PanelTile> parts = split_connected_panel(panel, nTiles);
		for (uint32_t t = 0; t < nTiles; ++t) {
			tiles[t].info = std::move(parts[t]);
			tiles[t].setup();
		}
	}

	std::vector<float> gpos = panel.positions;
	std::vector<float> gvel(3 * nV, 0.0f);
	std::vector<std::vector<float>> ownedNew(nTiles);

	auto solveTile = [&](uint32_t t) {
		Tile &tile = tiles[t];
		const uint32_t nLocal = tile.info.mesh.nVerts();
		std::vector<float> lpos(3 * nLocal), lpred(3 * nLocal);
		for (uint32_t l = 0; l < nLocal; ++l) {
			const uint32_t g = tile.info.globalId[l];
			for (int d = 0; d < 3; ++d) {
				lpos[3 * l + d] = gpos[3 * g + d];
			}
		}
		for (uint32_t l = 0; l < nLocal; ++l) {
			const uint32_t g = tile.info.globalId[l];
			const float gy = (l < tile.info.nOwned) ? (ClothSim::H * ClothSim::H * ClothSim::GRAVITY_Y) : 0.0f;
			lpred[3 * l + 0] = lpos[3 * l + 0] + ClothSim::H * gvel[3 * g + 0];
			lpred[3 * l + 1] = lpos[3 * l + 1] + ClothSim::H * gvel[3 * g + 1] + gy;
			lpred[3 * l + 2] = lpos[3 * l + 2] + ClothSim::H * gvel[3 * g + 2];
		}
		tile.solver.updateState(lpos.data(), lpred.data());
		for (int it = 0; it < ClothSim::ITERS; ++it) {
			tile.solver.step();
		}
		std::vector<float> lnew;
		tile.solver.readPositions(lnew);
		ownedNew[t].assign(3 * tile.info.nOwned, 0.0f);
		for (uint32_t l = 0; l < tile.info.nOwned; ++l) {
			for (int d = 0; d < 3; ++d) {
				ownedNew[t][3 * l + d] = lnew[3 * l + d];
			}
		}
	};

	auto t0 = std::chrono::steady_clock::now();
	for (int s = 0; s < SUBSTEPS; ++s) {
		if (parallel) {
			std::vector<std::thread> pool;
			for (uint32_t t = 0; t < nTiles; ++t) {
				pool.emplace_back([t, &solveTile]() { solveTile(t); });
			}
			for (auto &th : pool) {
				th.join();
			}
		} else {
			for (uint32_t t = 0; t < nTiles; ++t) {
				solveTile(t);
			}
		}
		std::vector<float> newpos = gpos;
		for (uint32_t t = 0; t < nTiles; ++t) {
			for (uint32_t l = 0; l < tiles[t].info.nOwned; ++l) {
				const uint32_t g = tiles[t].info.globalId[l];
				for (int d = 0; d < 3; ++d) {
					newpos[3 * g + d] = ownedNew[t][3 * l + d];
				}
			}
		}
		for (uint32_t i = 0; i < 3 * nV; ++i) {
			gvel[i] = (newpos[i] - gpos[i]) / ClothSim::H;
		}
		gpos = newpos;
	}
	auto t1 = std::chrono::steady_clock::now();
	if (outMs != nullptr) {
		*outMs = std::chrono::duration<double, std::milli>(t1 - t0).count();
	}
	return gpos;
}

float lowest_y(const std::vector<float> &pos) {
	float lo = pos[1];
	for (size_t i = 0; i < pos.size() / 3; ++i) {
		if (pos[3 * i + 1] < lo) {
			lo = pos[3 * i + 1];
		}
	}
	return lo;
}

} // namespace

int main() {
	int fails = 0;

	double serialMs = 0.0, parMs = 0.0;
	std::vector<float> serial = run_decomposed(NTILES, false, &serialMs);
	std::vector<float> par = run_decomposed(NTILES, true, &parMs);

	const bool det_ok = avbd_digest(serial) == avbd_digest(par);
	std::printf("%s  split solve is schedule-independent  [%016llx vs %016llx]\n",
			det_ok ? "PASS" : "FAIL",
			(unsigned long long)avbd_digest(serial), (unsigned long long)avbd_digest(par));
	fails += det_ok ? 0 : 1;

	const float sag = 1.2f - lowest_y(par);
	const bool drape_ok = sag > 0.03f;
	std::printf("%s  split panel still drapes  [%.0f mm, about %.1f golf balls]\n",
			drape_ok ? "PASS" : "FAIL", sag * 1000.0f, sag / 0.0427f);
	fails += drape_ok ? 0 : 1;

	std::vector<float> mono = run_decomposed(1, false, nullptr);
	const float monoSag = 1.2f - lowest_y(mono);
	const float rel = std::fabs(sag - monoSag) / monoSag;
	const bool close_ok = rel < 0.25f;
	std::printf("%s  %u-tile drape within 25%% of monolithic  [%.0f mm vs %.0f mm, %.0f%%]\n",
			close_ok ? "PASS" : "FAIL", NTILES, sag * 1000.0f, monoSag * 1000.0f, rel * 100.0f);
	fails += close_ok ? 0 : 1;

	const double speedup = serialMs / parMs;
	const bool fast_ok = speedup > 1.3;
	std::printf("%s  multi-core speedup %.2fx on one panel  [%u tiles, %.0f ms serial / %.0f ms parallel]\n",
			fast_ok ? "PASS" : "FAIL", speedup, NTILES, serialMs, parMs);
	fails += fast_ok ? 0 : 1;

	std::printf("%s\n", fails == 0 ? "all split-panel checks OK" : "SPLIT-PANEL FAILED");
	return fails == 0 ? 0 : 1;
}
