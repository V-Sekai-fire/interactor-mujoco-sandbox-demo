// SPDX-License-Identifier: Apache-2.0 OR MIT
#include <cmath>
#include <cstdio>

#include "../avbd_sim.h"

int main() {
	ClothSim sim;
	sim.setup(build_cloth_mesh(6, 8, 0.5f, 0.7f, 1.2f, 0.3f));
	sim.iters = 20;
	sim.damp = 0.98f;
	for (int i = 0; i < 400; ++i) {
		sim.step();
	}
	std::vector<float> prev = sim.pos;
	const float dtFrame = 4.0f * ClothSim::H;
	float t = 0.0f, maxJump = 0.0f, maxSway = 0.0f;
	bool finite = true;
	for (int f = 0; f < 600; ++f) {
		t += dtFrame;
		// Bar sways sideways 0.12 m at 0.4 rad/s, a slow flagpole wobble.
		const float bx = std::sin(t * 0.4f) * 0.12f;
		sim.moveBar(bx, 0.0f, 0.0f);
		for (int s = 0; s < 4; ++s) {
			sim.step();
		}
		if (!sim.finite()) { finite = false; break; }
		for (uint32_t k = 0; k < sim.mesh.nVerts(); ++k) {
			const float jx = sim.pos[3 * k + 0] - prev[3 * k + 0];
			const float jy = sim.pos[3 * k + 1] - prev[3 * k + 1];
			const float jz = sim.pos[3 * k + 2] - prev[3 * k + 2];
			maxJump = std::max(maxJump, std::sqrt(jx * jx + jy * jy + jz * jz));
		}
		maxSway = std::max(maxSway, std::fabs(bx));
		prev = sim.pos;
	}
	std::printf("bar sway 0.12 m @ 0.4 rad/s:  max_frame_jump=%.4f m  finite=%d\n",
			maxJump, finite ? 1 : 0);
	return finite ? 0 : 1;
}
