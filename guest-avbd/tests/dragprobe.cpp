// SPDX-License-Identifier: Apache-2.0 OR MIT
#include <cmath>
#include <cstdio>

#include "../avbd_sim.h"

static float vdist(const std::vector<float> &a, const std::vector<float> &b, int v) {
	const float dx = a[3 * v + 0] - b[3 * v + 0];
	const float dy = a[3 * v + 1] - b[3 * v + 1];
	const float dz = a[3 * v + 2] - b[3 * v + 2];
	return std::sqrt(dx * dx + dy * dy + dz * dz);
}

int main() {
	ClothSim sim;
	sim.setup(build_cloth_mesh(6, 8, 0.5f, 0.7f, 1.2f, 0.3f));
	sim.iters = 20;
	sim.damp = 0.98f;
	for (int i = 0; i < 280; ++i) {
		sim.step();
	}

	// Lowest vertex, the demo's grab target.
	int gv = 0;
	for (uint32_t k = 0; k < sim.mesh.nVerts(); ++k) {
		if (sim.pos[3 * k + 1] < sim.pos[3 * gv + 1]) {
			gv = int(k);
		}
	}
	const float ox = sim.pos[3 * gv + 0], oy = sim.pos[3 * gv + 1], oz = sim.pos[3 * gv + 2];
	sim.grab(gv);
	sim.snapshot();

	// A SMALL drag: 20 mm (about one AAA battery long) straight sideways.
	const float tx = ox + 0.02f, ty = oy, tz = oz;
	float ax = ox, ay = oy, az = oz;
	std::vector<float> before = sim.pos;
	int reverts = 0;
	float peak = 0.0f;
	for (int frame = 0; frame < 40; ++frame) {
		for (int s = 0; s < 4; ++s) {
			sim.step();
		}
		if (!sim.finite()) {
			sim.restore();
			++reverts;
		} else {
			sim.snapshot();
		}
		ax += (tx - ax) * 0.18f;
		ay += (ty - ay) * 0.18f;
		az += (tz - az) * 0.18f;
		sim.drag(ax, ay, az);
		const float gvmove = vdist(sim.pos, before, gv);
		float meshmax = 0.0f;
		for (uint32_t k = 0; k < sim.mesh.nVerts(); ++k) {
			meshmax = std::max(meshmax, vdist(sim.pos, before, int(k)));
		}
		peak = std::max(peak, gvmove);
		if (frame < 12 || (frame % 5) == 0) {
			std::printf("f%02d anchor=%.3f grabbed_moved=%.4f mesh_max=%.4f finite=%d\n",
					frame, ax - ox, gvmove, meshmax, sim.finite() ? 1 : 0);
		}
	}
	std::printf("SUMMARY drag=0.020 m  peak_grabbed_move=%.4f m  reverts=%d\n", peak, reverts);
	return 0;
}
