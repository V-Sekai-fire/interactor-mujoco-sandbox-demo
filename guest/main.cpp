#include <api.hpp>
#include <mujoco/mujoco.h>

#include <cctype>
#include <clocale>
#include <cstdint>
#include <cstdlib>
#include <cmath>
#include <cstring>
#include <string>
#include <vector>

#include "model_data.h"

// A physics guest: MuJoCo linked into a sandbox program, running a cat's
// cradle. The host steps it one frame at a time, so the tick rate belongs to
// the caller and a run can be replayed rather than raced.
//
// The model is compiled in rather than handed over at runtime. A machine that
// can be serialised has the flat arena off, and host-to-guest transfer
// allocates through that arena, so a guest that carries its own model is the
// one that can also be migrated.

// MuJoCo sizes its model buffer from an offset starting at zero but assigns
// the pointers inside it by absolute address, both rounded to 64 bytes
// (engine_io.c mj_setPtrModel / safeAddToBufferSize). The two agree only when
// the buffer itself starts 64-byte aligned, and the guest's malloc is wrapped
// to the sandbox arena, which does not promise that.
static void *aligned64_malloc(size_t n) {
	void *raw = malloc(n + 64 + sizeof(void *));
	if (raw == nullptr) {
		return nullptr;
	}
	uintptr_t base = (uintptr_t)raw + sizeof(void *);
	uintptr_t aligned = (base + 63) & ~(uintptr_t)63;
	((void **)aligned)[-1] = raw;
	return (void *)aligned;
}

static void aligned64_free(void *p) {
	if (p != nullptr) {
		free(((void **)p)[-1]);
	}
}

static mjModel *g_model = nullptr;
static mjData *g_data = nullptr;
static std::vector<unsigned char> g_buffer;

static Variant mjc_version() {
	return (int)mj_version();
}

static Variant mjc_load(Variant bytes) {
	PackedArray<uint8_t> arr = bytes.as_byte_array();
	const std::vector<uint8_t> v = arr.fetch();
	if (v.empty()) {
		return false;
	}

	if (g_data != nullptr) {
		mj_deleteData(g_data);
		g_data = nullptr;
	}
	if (g_model != nullptr) {
		mj_deleteModel(g_model);
		g_model = nullptr;
	}

	// The buffer outlives the call: mjModel keeps pointing into it.
	g_buffer.assign(v.begin(), v.end());
	g_model = mj_loadModelBuffer(g_buffer.data(), (int)g_buffer.size());
	if (g_model == nullptr) {
		return false;
	}
	g_data = mj_makeData(g_model);
	return g_data != nullptr;
}

static bool load_xml_text(const char *text, int len) {
	if (g_data != nullptr) {
		mj_deleteData(g_data);
		g_data = nullptr;
	}
	if (g_model != nullptr) {
		mj_deleteModel(g_model);
		g_model = nullptr;
	}

	static mjVFS vfs;
	mj_defaultVFS(&vfs);
	const char *name = "model.xml";
	if (mj_addBufferVFS(&vfs, name, text, len) != 0) {
		mj_deleteVFS(&vfs);
		return false;
	}

	char err[512] = { 0 };
	g_model = mj_loadXML(name, &vfs, err, (int)sizeof(err));
	mj_deleteVFS(&vfs);
	if (g_model == nullptr) {
		print("mj_loadXML failed");
		print(err);
		return false;
	}
	g_data = mj_makeData(g_model);
	if (g_data != nullptr) {
		// A keyframe opens the model in the middle of its motion rather than
		// from rest. Without applying it the keyframe is data nothing reads.
		if (g_model->nkey > 0) {
			mj_resetDataKeyframe(g_model, g_data, 0);
		}
		// Positions are only valid once the kinematics have run; otherwise the
		// first frame reports every geom at the origin.
		mj_forward(g_model, g_data);
	}
	return g_data != nullptr;
}




// The model the guest ships with, needing nothing from the host.
static Variant mjc_load_builtin() {
	return load_xml_text(MODEL_XML, (int)sizeof(MODEL_XML));
}

// Kept so the host-to-guest path stays exercised once it works again; deleting
// it would hide that it is still broken.
static Variant mjc_load_xml(Variant text) {
	const std::vector<uint8_t> v = text.as_byte_array().fetch();
	if (v.empty()) {
		return false;
	}
	return load_xml_text((const char *)v.data(), (int)v.size());
}

static Variant mjc_step() {
	if (g_model == nullptr || g_data == nullptr) {
		return -1.0;
	}
	mj_step(g_model, g_data);
	return (double)g_data->time;
}

static Variant mjc_nq() {
	return g_model == nullptr ? 0 : (int)g_model->nq;
}

// Equality constraints. Zero means the loop never closed and the figure is a
// chain pretending to be a cradle.
static Variant mjc_neq() {
	return g_model == nullptr ? 0 : (int)g_model->neq;
}

// Active contacts. A threaded figure keeps its crossings touching; zero means
// the string fell through itself.
static Variant mjc_ncon() {
	return g_data == nullptr ? 0 : (int)g_data->ncon;
}

static Variant mjc_qpos() {
	if (g_model == nullptr || g_data == nullptr) {
		return PackedArray<double>(std::vector<double>());
	}
	const std::vector<double> q(g_data->qpos, g_data->qpos + g_model->nq);
	return PackedArray<double>(q);
}

// A digest over the full integration state, which is what a transfer has to
// carry. Comparing digests is how a migrated run is shown to be the same run
// rather than merely a running one.
static Variant mjc_digest() {
	if (g_model == nullptr || g_data == nullptr) {
		return 0;
	}
	const int n = mj_stateSize(g_model, mjSTATE_INTEGRATION);
	std::vector<double> state(n);
	mj_getState(g_model, g_data, state.data(), mjSTATE_INTEGRATION);

	uint64_t h = 1469598103934665603ULL;
	const unsigned char *p = (const unsigned char *)state.data();
	for (size_t i = 0; i < state.size() * sizeof(double); i++) {
		h ^= (uint64_t)p[i];
		h *= 1099511628211ULL;
	}
	// Godot ints are signed; keep it positive so the two sides print alike.
	return (int64_t)(h & 0x7FFFFFFFFFFFFFFFULL);
}

// Body count including the world body at index 0.
static Variant mjc_nbody() {
	return g_model == nullptr ? 0 : (int)g_model->nbody;
}

// Every body as x, y, z, qw, qx, qy, qz, so the host can draw the figure
// without knowing anything about the model.
static Variant mjc_bodies() {
	if (g_model == nullptr || g_data == nullptr) {
		return PackedArray<double>(std::vector<double>());
	}
	std::vector<double> out;
	out.reserve((size_t)g_model->nbody * 7);
	for (int i = 0; i < g_model->nbody; i++) {
		out.push_back(g_data->xpos[i * 3 + 0]);
		out.push_back(g_data->xpos[i * 3 + 1]);
		out.push_back(g_data->xpos[i * 3 + 2]);
		out.push_back(g_data->xquat[i * 4 + 0]);
		out.push_back(g_data->xquat[i * 4 + 1]);
		out.push_back(g_data->xquat[i * 4 + 2]);
		out.push_back(g_data->xquat[i * 4 + 3]);
	}
	return PackedArray<double>(out);
}

// Every geom as type, three sizes, position and orientation. This is what a
// host draws: a body frame sits at its joint, not at the shape hanging off it.
static Variant mjc_geoms() {
	Array out = Array::Create();
	if (g_model == nullptr || g_data == nullptr) {
		return out;
	}
	for (int i = 0; i < g_model->ngeom; i++) {
		out.push_back((double)g_model->geom_type[i]);
		out.push_back(g_model->geom_size[i * 3 + 0]);
		out.push_back(g_model->geom_size[i * 3 + 1]);
		out.push_back(g_model->geom_size[i * 3 + 2]);
		out.push_back(g_data->geom_xpos[i * 3 + 0]);
		out.push_back(g_data->geom_xpos[i * 3 + 1]);
		out.push_back(g_data->geom_xpos[i * 3 + 2]);
		double quat[4];
		mju_mat2Quat(quat, g_data->geom_xmat + i * 9);
		out.push_back(quat[0]);
		out.push_back(quat[1]);
		out.push_back(quat[2]);
		out.push_back(quat[3]);
	}
	return out;
}

static Variant mjc_ngeom() {
	return g_model == nullptr ? 0 : (int)g_model->ngeom;
}

// Hold one hinge at an angle, as a hand holding a ball does: the joint is
// placed and its velocity cleared, so releasing it starts from rest rather
// than from whatever the solver had accumulated.
static Variant mjc_hold(Variant index, Variant angle) {
	if (g_model == nullptr || g_data == nullptr) {
		return false;
	}
	const int i = (int)(int64_t)index;
	if (i < 0 || i >= g_model->nq) {
		return false;
	}
	g_data->qpos[i] = (double)angle;
	g_data->qvel[i] = 0.0;
	mj_forward(g_model, g_data);
	return true;
}

// Where each hinge sits and how far its ball hangs below it, so the host can
// turn a point in space into an angle without being told the model.
static Variant mjc_pivots() {
	std::vector<double> out;
	if (g_model == nullptr || g_data == nullptr) {
		return PackedArray<double>(out);
	}
	for (int j = 0; j < g_model->njnt; j++) {
		const int b = g_model->jnt_bodyid[j];
		const double px = g_data->xanchor[j * 3 + 0];
		const double pz = g_data->xanchor[j * 3 + 2];
		// The ball is the last geom on the body; its distance from the anchor
		// is the pendulum length.
		double len = 0.0;
		for (int g = 0; g < g_model->ngeom; g++) {
			if (g_model->geom_bodyid[g] != b || g_model->geom_type[g] != mjGEOM_SPHERE) {
				continue;
			}
			const double dx = g_data->geom_xpos[g * 3 + 0] - px;
			const double dz = g_data->geom_xpos[g * 3 + 2] - pz;
			len = sqrt(dx * dx + dz * dz);
		}
		out.push_back(px);
		out.push_back(pz);
		out.push_back(len);
	}
	return PackedArray<double>(out);
}

// Simulated time, so a restored run can be shown resuming rather than restarting.
static Variant mjc_time() {
	return g_data == nullptr ? 0.0 : (double)g_data->time;
}

// Lowest point of the figure, in millimetres. Sag is what shows the cradle is
// held rather than fallen.
static Variant mjc_lowest_mm() {
	if (g_model == nullptr || g_data == nullptr) {
		return 0.0;
	}
	// Geoms, not bodies: a body frame sits at its joint, so measuring those
	// reports the height of the beam the figure hangs from.
	double lowest = 1e9;
	for (int i = 0; i < g_model->ngeom; i++) {
		const double z = g_data->geom_xpos[i * 3 + 2];
		if (z < lowest) {
			lowest = z;
		}
	}
	return lowest * 1000.0;
}

int main() {
	// Initialises the ctype tables tinyxml2 needs to find attributes.
	setlocale(LC_ALL, "C");

	mju_user_malloc = aligned64_malloc;
	mju_user_free = aligned64_free;

	ADD_API_FUNCTION(mjc_version, "int", "", "MuJoCo library version");
	ADD_API_FUNCTION(mjc_load, "bool", "PackedByteArray mjb", "Load an MJB model from memory");
	ADD_API_FUNCTION(mjc_load_builtin, "bool", "", "Load the model compiled into this guest");
	ADD_API_FUNCTION(mjc_load_xml, "bool", "PackedByteArray xml", "Load an MJCF model from XML bytes");
	ADD_API_FUNCTION(mjc_step, "float", "", "Advance one step, returning simulated time");
	ADD_API_FUNCTION(mjc_nq, "int", "", "Number of generalised coordinates");
	ADD_API_FUNCTION(mjc_neq, "int", "", "Number of equality constraints");
	ADD_API_FUNCTION(mjc_ncon, "int", "", "Active contacts this step");
	ADD_API_FUNCTION(mjc_nbody, "int", "", "Number of bodies");
	ADD_API_FUNCTION(mjc_bodies, "PackedFloat64Array", "", "Body transforms as x,y,z,qw,qx,qy,qz");
	ADD_API_FUNCTION(mjc_ngeom, "int", "", "Number of geoms");
	ADD_API_FUNCTION(mjc_geoms, "Array", "", "Geoms as type, size xyz, pos xyz, quat wxyz");
	ADD_API_FUNCTION(mjc_hold, "bool", "int index, float angle", "Hold a hinge at an angle");
	ADD_API_FUNCTION(mjc_pivots, "PackedFloat64Array", "", "Per joint: anchor x, anchor z, pendulum length");
	ADD_API_FUNCTION(mjc_time, "float", "", "Simulated time");
	ADD_API_FUNCTION(mjc_qpos, "PackedFloat64Array", "", "Generalised positions");
	ADD_API_FUNCTION(mjc_digest, "int", "", "Digest of the full integration state");
	ADD_API_FUNCTION(mjc_lowest_mm, "float", "", "Lowest body height in millimetres");
	halt();
}
