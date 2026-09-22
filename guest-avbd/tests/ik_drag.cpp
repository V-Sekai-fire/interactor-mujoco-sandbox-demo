// SPDX-License-Identifier: Apache-2.0 OR MIT
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <vector>

#include "../avbd_sim.h"

int main() {
	ClothSim sim;
	sim.setup(build_cloth_mesh(6, 8, 0.5f, 0.7f, 1.2f, 0.3f));
	sim.iters = 20;
	for (int i = 0; i < 280; ++i) {
		sim.step();
	}
	const bool clean = sim.finite();

	const int v = 16;
	sim.grab(v);
	sim.snapshot();
	float ox = sim.pos[3 * v + 0], oy = sim.pos[3 * v + 1], oz = sim.pos[3 * v + 2];
	// a far, likely-unsolvable target 0.25 m away
	const float len = std::sqrt(1.0f + 0.64f + 0.36f);
	float tx = ox + (1.0f / len) * 0.25f, ty = oy - (0.8f / len) * 0.25f, tz = oz + (0.6f / len) * 0.25f;

	float ax = ox, ay = oy, az = oz;
	float sx = ox, sy = oy, sz = oz;
	int reverts = 0;
	for (int frame = 0; frame < 60; ++frame) {
		for (int s = 0; s < 4; ++s) {
			sim.step();
		}
		if (!sim.finite()) {
			sim.restore();
			ax = sx; ay = sy; az = sz;
			++reverts;
		} else {
			sim.snapshot();
			sx = ax; sy = ay; sz = az;
		}
		ax += (tx - ax) * 0.18f;
		ay += (ty - ay) * 0.18f;
		az += (tz - az) * 0.18f;
		sim.drag(ax, ay, az);
	}
	if (!sim.finite()) {
		sim.restore();
	}
	const bool end_finite = sim.finite();
	const float reached = std::sqrt((sx - ox) * (sx - ox) + (sy - oy) * (sy - oy) + (sz - oz) * (sz - oz));

	std::printf("settle_finite=%d  reached=%.3f/0.250 m  reverts=%d  end_finite=%d\n",
			clean ? 1 : 0, reached, reverts, end_finite ? 1 : 0);
	std::printf("%s\n", end_finite ? "IK fallback held: never left a finite state" : "FAILED: ended non-finite");
	return end_finite ? 0 : 1;
}
