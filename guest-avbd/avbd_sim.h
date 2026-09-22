// SPDX-License-Identifier: Apache-2.0 OR MIT
#ifndef AVBD_SIM_H
#define AVBD_SIM_H

#include <cstdint>
#include <vector>

#include "avbd_cpu.h"
#include "cloth_grid.h"

struct ClothSim {
	static constexpr float H = 5e-3f;        // substep, 5 ms
	static constexpr int ITERS = 40;         // AVBD outer iterations per substep
	static constexpr float MEMBRANE_K = 2.0f;
	static constexpr float BEND_K = 0.1f;
	static constexpr float ATTACH_K = 1e5f;
	static constexpr float GRAVITY_Y = -9.81f;

	AvbdCpu solver;
	ClothMesh mesh;
	std::vector<float> pos, vel, predicted;
	std::vector<float> triK, bendK, attachK;

	int iters = ITERS;

	int grabbed = -1;
	std::vector<uint32_t> dragAttachVert;
	std::vector<float> dragAttachFixed, dragAttachK;

	void setup(const ClothMesh &m) {
		mesh = m;
		const float invHSq = 1.0f / (H * H);
		solver.setupMesh(mesh.nVerts(), mesh.positions.data(), mesh.positions.data(),
				mesh.mass.data(), invHSq);
		triK.assign(mesh.nTri(), MEMBRANE_K);
		bendK.assign(mesh.nBend(), BEND_K);
		attachK.assign(mesh.nAttach(), ATTACH_K);
		solver.uploadTriangles(mesh.nTri(), mesh.triIdx.data(), mesh.triInvUV.data(), triK.data());
		solver.uploadBendings(mesh.nBend(), mesh.bendIdx.data(), mesh.bendWeight.data(),
				mesh.bendNTarget.data(), bendK.data());
		solver.uploadAttachments(mesh.nAttach(), mesh.attachVert.data(),
				mesh.attachFixed.data(), attachK.data());
		pos = mesh.positions;
		vel.assign(3 * mesh.nVerts(), 0.0f);
		predicted.assign(3 * mesh.nVerts(), 0.0f);
	}

	void step() {
		const uint32_t nV = mesh.nVerts();
		for (uint32_t i = 0; i < nV; ++i) {
			predicted[3*i+0] = pos[3*i+0] + H * vel[3*i+0];
			predicted[3*i+1] = pos[3*i+1] + H * vel[3*i+1] + H * H * GRAVITY_Y;
			predicted[3*i+2] = pos[3*i+2] + H * vel[3*i+2];
		}
		solver.updateState(pos.data(), predicted.data());
		for (int it = 0; it < iters; ++it) {
			solver.step();
		}
		std::vector<float> np;
		solver.readPositions(np);
		for (uint32_t i = 0; i < 3 * nV; ++i) {
			vel[i] = (np[i] - pos[i]) / H;
		}
		pos = np;
	}

	void run(int substeps) {
		for (int s = 0; s < substeps; ++s) {
			step();
		}
	}

	void grab(int v) {
		if (v < 0 || uint32_t(v) >= mesh.nVerts()) {
			return;
		}
		grabbed = v;
		dragAttachVert = mesh.attachVert;
		dragAttachFixed = mesh.attachFixed;
		dragAttachK = attachK;
		dragAttachVert.push_back(uint32_t(v));
		dragAttachFixed.push_back(pos[3 * v + 0]);
		dragAttachFixed.push_back(pos[3 * v + 1]);
		dragAttachFixed.push_back(pos[3 * v + 2]);
		dragAttachK.push_back(ATTACH_K);
		solver.uploadAttachments(uint32_t(dragAttachVert.size()), dragAttachVert.data(),
				dragAttachFixed.data(), dragAttachK.data());
	}

	void drag(float x, float y, float z) {
		if (grabbed < 0 || dragAttachVert.empty()) {
			return;
		}
		const size_t last = dragAttachVert.size() - 1;
		dragAttachFixed[3 * last + 0] = x;
		dragAttachFixed[3 * last + 1] = y;
		dragAttachFixed[3 * last + 2] = z;
		solver.updateAttachmentFixedPos(dragAttachFixed.data());
	}

	void release() {
		grabbed = -1;
		solver.uploadAttachments(mesh.nAttach(), mesh.attachVert.data(),
				mesh.attachFixed.data(), attachK.data());
	}
};

inline uint64_t avbd_digest(const std::vector<float> &v) {
	uint64_t h = 1469598103934665603ull;
	const unsigned char *b = reinterpret_cast<const unsigned char *>(v.data());
	for (size_t i = 0; i < v.size() * sizeof(float); ++i) {
		h ^= b[i];
		h *= 1099511628211ull;
	}
	return h;
}

#endif // AVBD_SIM_H
