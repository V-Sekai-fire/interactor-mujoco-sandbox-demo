#include <api.hpp>
#include <mujoco/mujoco.h>

#include <vector>

// A physics guest: MuJoCo linked into a sandbox program. The host steps it one
// frame at a time, so the tick rate belongs to the caller and a run can be
// replayed rather than raced.
//
// The guest has no filesystem, so a model arrives as MJB bytes compiled on the
// host and is loaded from memory. mj_loadModel takes a path and is unreachable
// here; mj_loadModelBuffer is the one that works.

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

// One step, returned as the simulated time so the host can see it advance.
static Variant mjc_step() {
	if (g_model == nullptr || g_data == nullptr) {
		return -1.0;
	}
	mj_step(g_model, g_data);
	return (double)g_data->time;
}

static Variant mjc_nq() {
	if (g_model == nullptr) {
		return 0;
	}
	return (int)g_model->nq;
}

// Generalised positions: what a caller watches to know the simulation moved,
// and what a migration has to carry.
static Variant mjc_qpos() {
	Array out = Array::Create();
	if (g_model == nullptr || g_data == nullptr) {
		return out;
	}
	for (int i = 0; i < g_model->nq; i++) {
		out.push_back((double)g_data->qpos[i]);
	}
	return out;
}

int main() {
	ADD_API_FUNCTION(mjc_version, "int", "", "MuJoCo library version");
	ADD_API_FUNCTION(mjc_load, "bool", "PackedByteArray mjb", "Load an MJB model from memory");
	ADD_API_FUNCTION(mjc_step, "float", "", "Advance one step, returning simulated time");
	ADD_API_FUNCTION(mjc_nq, "int", "", "Number of generalised coordinates");
	ADD_API_FUNCTION(mjc_qpos, "Array", "", "Generalised positions");
	halt();
}
