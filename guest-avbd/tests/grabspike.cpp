// SPDX-License-Identifier: Apache-2.0 OR MIT
#include <cmath>
#include <cstdio>

#include "../avbd_sim.h"

static float vmax_move(const std::vector<float> &a, const std::vector<float> &b) {
	float m = 0.0f;
	for (size_t i = 0; i < a.size(); i += 3) {
		const float dx = a[i] - b[i], dy = a[i + 1] - b[i + 1], dz = a[i + 2] - b[i + 2];
		m = std::max(m, std::sqrt(dx * dx + dy * dy + dz * dz));
	}
	return m;
}

static float kinetic(const std::vector<float> &vel) {
	float s = 0.0f;
	for (float v : vel) {
		s += v * v;
	}
	return s;
}

int main() {
	ClothSim sim;
	sim.setup(build_cloth_mesh(6, 8, 0.5f, 0.7f, 1.2f, 0.3f));
	sim.iters = 20;
	sim.damp = 0.98f;
	for (int i = 0; i < 400; ++i) {
		sim.step();
	}
	int gv = 0;
	for (uint32_t k = 0; k < sim.mesh.nVerts(); ++k) {
		if (sim.pos[3 * k + 1] < sim.pos[3 * gv + 1]) {
			gv = int(k);
		}
	}
	std::printf("pre-grab: KE=%.5f\n", kinetic(sim.vel));
	std::vector<float> before = sim.pos;

	sim.grab(gv);            // mouse-down: pin at current pos, no drag
	sim.drag(sim.pos[3 * gv], sim.pos[3 * gv + 1], sim.pos[3 * gv + 2]);
	for (int frame = 0; frame < 20; ++frame) {
		for (int s = 0; s < 4; ++s) {
			sim.step();
		}
		std::printf("f%02d  mesh_max_vs_pregrab=%.4f  KE=%.5f  finite=%d\n",
				frame, vmax_move(sim.pos, before), kinetic(sim.vel), sim.finite() ? 1 : 0);
	}
	return 0;
}
