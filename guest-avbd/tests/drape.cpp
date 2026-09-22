// SPDX-License-Identifier: Apache-2.0 OR MIT

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <vector>

#include "../avbd_sim.h"

namespace {

constexpr int SUBSTEPS = 400; // ~2 s of sim

struct Result {
	std::vector<float> pos;
	float pinnedY = 0.0f; // mean y of the two pinned corners
	float sag = 0.0f;     // panelTop - lowest vertex
};

Result simulate(float gravityScale) {
	const uint32_t nx = 10, ny = 14;
	ClothSim sim;
	sim.setup(build_cloth_mesh(nx, ny, 0.5f, 0.7f, 1.2f, 0.05f));
	for (int s = 0; s < SUBSTEPS; ++s) {
		const uint32_t nV = sim.mesh.nVerts();
		for (uint32_t i = 0; i < nV; ++i) {
			sim.predicted[3*i+0] = sim.pos[3*i+0] + ClothSim::H * sim.vel[3*i+0];
			sim.predicted[3*i+1] = sim.pos[3*i+1] + ClothSim::H * sim.vel[3*i+1]
					+ ClothSim::H * ClothSim::H * ClothSim::GRAVITY_Y * gravityScale;
			sim.predicted[3*i+2] = sim.pos[3*i+2] + ClothSim::H * sim.vel[3*i+2];
		}
		sim.solver.updateState(sim.pos.data(), sim.predicted.data());
		for (int it = 0; it < ClothSim::ITERS; ++it) {
			sim.solver.step();
		}
		std::vector<float> np;
		sim.solver.readPositions(np);
		for (uint32_t i = 0; i < 3 * nV; ++i) {
			sim.vel[i] = (np[i] - sim.pos[i]) / ClothSim::H;
		}
		sim.pos = np;
	}

	Result r;
	r.pos = sim.pos;
	auto vid = [ny](uint32_t ix, uint32_t iy) { return ix * ny + iy; };
	r.pinnedY = 0.5f * (sim.pos[3 * vid(0, ny - 1) + 1] + sim.pos[3 * vid(nx - 1, ny - 1) + 1]);
	float lowest = sim.pos[1];
	for (uint32_t i = 0; i < sim.mesh.nVerts(); ++i) {
		lowest = std::fmin(lowest, sim.pos[3 * i + 1]);
	}
	r.sag = 1.2f - lowest;
	return r;
}

} // namespace

int main() {
	int fails = 0;

	Result a = simulate(1.0f);
	Result b = simulate(1.0f);
	const uint64_t da = avbd_digest(a.pos), db = avbd_digest(b.pos);

	const bool pinned_ok = std::fabs(a.pinnedY - 1.2f) < 1e-3f;
	std::printf("%s  pinned corners hold  [mean y %.5f, want 1.2]\n",
			pinned_ok ? "PASS" : "FAIL", a.pinnedY);
	fails += pinned_ok ? 0 : 1;

	const bool drape_ok = a.sag > 0.03f;
	std::printf("%s  cloth sags between the pins  [%.0f mm, about %.1f golf balls]\n",
			drape_ok ? "PASS" : "FAIL", a.sag * 1000.0f, a.sag / 0.0427f);
	fails += drape_ok ? 0 : 1;

	const bool det_ok = da == db;
	std::printf("%s  bit-identical run to run  [%016llx vs %016llx]\n",
			det_ok ? "PASS" : "FAIL", (unsigned long long)da, (unsigned long long)db);
	fails += det_ok ? 0 : 1;

	Result c = simulate(0.989f);
	const bool ctrl_ok = avbd_digest(c.pos) != da;
	std::printf("%s  perturbed gravity moves the digest\n", ctrl_ok ? "PASS" : "FAIL");
	fails += ctrl_ok ? 0 : 1;

	std::printf("%s\n", fails == 0 ? "all drape checks OK" : "DRAPE FAILED");
	return fails == 0 ? 0 : 1;
}
