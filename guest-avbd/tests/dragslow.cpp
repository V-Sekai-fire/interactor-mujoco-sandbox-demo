// SPDX-License-Identifier: Apache-2.0 OR MIT
#include <cmath>
#include <cstdio>

#include "../avbd_sim.h"

// A slow drag: move the grabbed vertex's anchor at a fixed slow speed and report
// how tightly the vertex tracks the anchor (lag) and whether the rest of the
// panel spikes. Slow input should give slow, tight motion for a usable grab.
static void run(float grabK) {
	ClothSim sim;
	sim.setup(build_cloth_mesh(6, 8, 0.5f, 0.7f, 1.2f, 0.3f));
	sim.iters = 20;
	sim.damp = 0.98f;
	sim.grabK = grabK;
	for (int i = 0; i < 400; ++i) {
		sim.step();
	}
	int gv = 0;
	for (uint32_t k = 0; k < sim.mesh.nVerts(); ++k) {
		if (sim.pos[3 * k + 1] < sim.pos[3 * gv + 1]) {
			gv = int(k);
		}
	}
	const float ox = sim.pos[3 * gv + 0], oy = sim.pos[3 * gv + 1], oz = sim.pos[3 * gv + 2];
	sim.grab(gv);

	// Mimic the demo's slow Lissajous animation and the 0.18 anchor lerp, at
	// 60 fps (4 substeps of 5 ms). Report the tightest and loosest tracking and
	// the largest excursion of any vertex from the grab origin.
	const float dtFrame = 4.0f * ClothSim::H;
	float t = 0.0f;
	float anchorX = ox, anchorY = oy, anchorZ = oz;
	std::vector<float> prev = sim.pos;
	float maxJump = 0.0f, maxExcursion = 0.0f, maxAnchor = 0.0f;
	for (int f = 0; f < 480; ++f) {
		t += dtFrame;
		const float tx = ox + std::sin(t * 0.3f) * 0.05f;
		const float ty = oy + std::cos(t * 0.24f) * 0.03f;
		const float tz = oz;
		anchorX += (tx - anchorX) * 0.18f;
		anchorY += (ty - anchorY) * 0.18f;
		anchorZ += (tz - anchorZ) * 0.18f;
		sim.drag(anchorX, anchorY, anchorZ);
		for (int s = 0; s < 4; ++s) {
			sim.step();
		}
		maxAnchor = std::max(maxAnchor, std::fabs(anchorX - ox));
		for (uint32_t k = 0; k < sim.mesh.nVerts(); ++k) {
			const float jx = sim.pos[3 * k + 0] - prev[3 * k + 0];
			const float jy = sim.pos[3 * k + 1] - prev[3 * k + 1];
			const float jz = sim.pos[3 * k + 2] - prev[3 * k + 2];
			maxJump = std::max(maxJump, std::sqrt(jx * jx + jy * jy + jz * jz));
			const float ex = sim.pos[3 * k + 0] - ox, ey = sim.pos[3 * k + 1] - oy, ez = sim.pos[3 * k + 2] - oz;
			maxExcursion = std::max(maxExcursion, std::sqrt(ex * ex + ey * ey + ez * ez));
		}
		prev = sim.pos;
	}
	std::printf("grabK=%.0f  anchor_amp=%.3f  max_frame_jump=%.4f m  max_excursion=%.3f m\n",
			grabK, maxAnchor, maxJump, maxExcursion);
}

int main() {
	std::printf("slow Lissajous (amp 0.05 m, freq 0.3 rad/s), demo lerp 0.18:\n");
	run(120.0f);
	run(2000.0f);
	return 0;
}
