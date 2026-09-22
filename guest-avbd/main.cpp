// SPDX-License-Identifier: Apache-2.0 OR MIT
// AVBD cloth guest. One RISC-V sandbox instance owns one cloth island and
// steps it on the interpreted CPU, so the solve is bit-identical across hosts.
// The host (Godot) runs one instance per island across a thread pool for
// multi-core; each instance is independent, so parallelism never changes a
// result. Arguments are native-typed for the unboxed-argument ABI, matching
// the sibling MuJoCo guest.

#include <api.hpp>

#include <cstdint>
#include <vector>

#include "avbd_sim.h"

static ClothSim g_sim;
static bool g_ready = false;

// Build an nx-by-ny panel and set up the solver. 0.5 x 0.7 m, ~a fat quarter.
static Variant avbd_load(int nx, int ny) {
	if (nx < 2 || ny < 2) {
		return false;
	}
	g_sim = ClothSim();
	g_sim.setup(build_cloth_mesh(uint32_t(nx), uint32_t(ny), 0.5f, 0.7f, 1.2f, 0.05f));
	g_ready = true;
	return true;
}

// Advance `substeps` fixed 5 ms substeps of AVBD (40 outer iterations each).
static Variant avbd_step(int substeps) {
	if (!g_ready) {
		return false;
	}
	for (int i = 0; i < substeps; ++i) {
		g_sim.step();
	}
	return true;
}

static Variant avbd_nverts() {
	if (!g_ready) {
		return 0;
	}
	return int(g_sim.mesh.nVerts());
}

// Set outer iterations per substep (fewer = faster, shallower drape).
static Variant avbd_set_iters(int n) {
	if (!g_ready || n < 1) {
		return false;
	}
	g_sim.iters = n;
	return true;
}

// Current vertex positions as x, y, z. Streamed per frame for the mesh.
static Variant avbd_verts() {
	std::vector<double> out;
	if (!g_ready) {
		return PackedArray<double>(out);
	}
	out.reserve(g_sim.pos.size());
	for (float v : g_sim.pos) {
		out.push_back(double(v));
	}
	return PackedArray<double>(out);
}

// Triangle vertex indices, three per face. Fixed connectivity, read once.
static Variant avbd_faces() {
	std::vector<double> out;
	if (!g_ready) {
		return PackedArray<double>(out);
	}
	out.reserve(g_sim.mesh.faces.size());
	for (uint32_t idx : g_sim.mesh.faces) {
		out.push_back(double(idx));
	}
	return PackedArray<double>(out);
}

// FNV-1a digest of the position state — the cross-host determinism check.
static Variant avbd_hash() {
	if (!g_ready) {
		return 0;
	}
	return int64_t(avbd_digest(g_sim.pos));
}

// Interactive drag: pin a vertex, move its anchor, then let it go.
static Variant avbd_grab(int v) {
	if (!g_ready) {
		return false;
	}
	g_sim.grab(v);
	return true;
}

static Variant avbd_drag(float x, float y, float z) {
	if (!g_ready) {
		return false;
	}
	g_sim.drag(x, y, z);
	return true;
}

static Variant avbd_release() {
	if (!g_ready) {
		return false;
	}
	g_sim.release();
	return true;
}

int main() {
	ADD_API_FUNCTION(avbd_load, "bool", "int nx, int ny", "Build an nx-by-ny cloth panel and set up AVBD");
	ADD_API_FUNCTION(avbd_step, "bool", "int substeps", "Advance the AVBD solve by N substeps");
	ADD_API_FUNCTION(avbd_nverts, "int", "", "Number of cloth vertices");
	ADD_API_FUNCTION(avbd_set_iters, "bool", "int n", "Set AVBD outer iterations per substep");
	ADD_API_FUNCTION(avbd_verts, "PackedFloat64Array", "", "Vertex positions as x,y,z");
	ADD_API_FUNCTION(avbd_faces, "PackedFloat64Array", "", "Triangle vertex indices, three per face");
	ADD_API_FUNCTION(avbd_hash, "int", "", "Digest of the position state for the determinism check");
	ADD_API_FUNCTION(avbd_grab, "bool", "int v", "Pin vertex v to a drag anchor at its current position");
	ADD_API_FUNCTION(avbd_drag, "bool", "float x, float y, float z", "Move the grabbed anchor to x,y,z");
	ADD_API_FUNCTION(avbd_release, "bool", "", "Release the grabbed vertex and restore the pins");
	halt();
}
