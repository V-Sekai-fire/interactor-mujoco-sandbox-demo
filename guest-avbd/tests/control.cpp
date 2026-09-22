// SPDX-License-Identifier: Apache-2.0 OR MIT
#include <cstdint>
#include <cstdio>
#include <vector>

#include "../avbd_sim.h"

static float hang_mm(uint32_t nx, uint32_t ny, int iters) {
	ClothSim sim;
	sim.setup(build_cloth_mesh(nx, ny, 0.5f, 0.7f, 1.2f, 0.3f));
	sim.iters = iters;
	sim.run(400);
	float lo = sim.pos[1];
	for (size_t i = 0; i < sim.pos.size() / 3; ++i) {
		if (sim.pos[3 * i + 1] < lo) {
			lo = sim.pos[3 * i + 1];
		}
	}
	return (1.2f - lo) * 1000.0f;
}

int main() {
	const uint32_t sizes[][2] = { { 4, 6 }, { 5, 7 }, { 6, 8 }, { 8, 11 }, { 10, 14 }, { 14, 20 } };
	std::printf("%9s  %6s  %10s  %11s  %9s\n",
			"grid", "verts", "hang@20", "hang@320", "converged");
	for (auto &s : sizes) {
		const uint32_t nx = s[0], ny = s[1];
		const float rt = hang_mm(nx, ny, 20);
		const float conv = hang_mm(nx, ny, 320);
		std::printf("%4ux%-4u  %6u  %8.0f mm  %8.0f mm  %7.0f%%\n",
				nx, ny, nx * ny, rt, conv, conv > 0 ? rt / conv * 100.0f : 0.0f);
	}
	std::printf("\nThe grid whose hang@20 reaches ~100%% of hang@320 converges within the\n");
	std::printf("realtime iteration budget: reduce the mesh to there.\n");
	return 0;
}
