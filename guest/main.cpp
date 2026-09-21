#include <api.hpp>
#include <mujoco/mujoco.h>

#include <cctype>
#include <clocale>
#include <cstdint>
#include <cstdlib>
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
	return g_data != nullptr;
}

// tinyxml2 decides where an attribute starts by calling isspace. A statically
// linked glibc guest that never ran libc startup has no ctype table, so this
// reports what the parser is actually seeing.
static Variant mjc_ctype_check() {
	Array out = Array::Create();
	out.push_back(isspace(32) ? 1 : 0);
	out.push_back(isspace(9) ? 1 : 0);
	out.push_back(isalpha(97) ? 1 : 0);
	return out;
}

// The guest links with --wrap on strlen/strcmp/strncmp/memcmp. tinyxml2 finds
// an attribute's closing quote with strncmp against a one-character tag, which
// is the only place a parse of plain elements never reaches.
static Variant mjc_str_check() {
	Array out = Array::Create();
	const char *q = "\"a\"";
	const char endTag[2] = { '"', 0 };
	out.push_back((int)strlen(endTag));
	out.push_back(strncmp(q, endTag, 1) == 0 ? 1 : 0);
	out.push_back(strncmp(q + 1, endTag, 1) != 0 ? 1 : 0);
	out.push_back(strncmp(q + 2, endTag, 1) == 0 ? 1 : 0);
	out.push_back(memcmp(q, endTag, 1) == 0 ? 1 : 0);
	out.push_back(strcmp("ab", "ab") == 0 ? 1 : 0);
	return out;
}

// Whether this build can construct an mjModel at all. The XML parses; the
// library then reports its own buffer size mismatch, so the smallest possible
// model is the shortest path to that failure.
static Variant mjc_xml_selftest() {
	// Raw literals, so what the parser sees is what is written here.
	static const char *cases[] = {
		R"XML(<mujoco/>)XML",
		R"XML(<mujoco model="a"/>)XML",
		R"XML(<mujoco><worldbody/></mujoco>)XML",
		R"XML(<mujoco><worldbody><body pos="0 0 1"><geom type="sphere" size="0.1"/></body></worldbody></mujoco>)XML",
	};
	Array out = Array::Create();
	for (int i = 0; i < 4; i++) {
		static mjVFS vfs;
		mj_defaultVFS(&vfs);
		const int len = (int)strlen(cases[i]);
		if (mj_addBufferVFS(&vfs, "m.xml", cases[i], len) != 0) {
			mj_deleteVFS(&vfs);
			out.push_back("vfs-failed");
			continue;
		}
		char err[256] = { 0 };
		mjModel *m = mj_loadXML("m.xml", &vfs, err, (int)sizeof(err));
		mj_deleteVFS(&vfs);
		if (m != nullptr) {
			mj_deleteModel(m);
			out.push_back("ok");
		} else {
			out.push_back(err);
		}
	}
	return out;
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

// Lowest point of the figure, in millimetres. Sag is what shows the cradle is
// held rather than fallen.
static Variant mjc_lowest_mm() {
	if (g_model == nullptr || g_data == nullptr) {
		return 0.0;
	}
	double lowest = 1e9;
	for (int i = 1; i < g_model->nbody; i++) {
		const double z = g_data->xpos[i * 3 + 2];
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

	ADD_API_FUNCTION(mjc_str_check, "Array", "", "wrapped string functions as the guest sees them");
	ADD_API_FUNCTION(mjc_ctype_check, "Array", "", "isspace/isalpha as the guest sees them");
	ADD_API_FUNCTION(mjc_xml_selftest, "Array", "", "Whether a minimal model can be built");
	ADD_API_FUNCTION(mjc_version, "int", "", "MuJoCo library version");
	ADD_API_FUNCTION(mjc_load, "bool", "PackedByteArray mjb", "Load an MJB model from memory");
	ADD_API_FUNCTION(mjc_load_builtin, "bool", "", "Load the model compiled into this guest");
	ADD_API_FUNCTION(mjc_load_xml, "bool", "PackedByteArray xml", "Load an MJCF model from XML bytes");
	ADD_API_FUNCTION(mjc_step, "float", "", "Advance one step, returning simulated time");
	ADD_API_FUNCTION(mjc_nq, "int", "", "Number of generalised coordinates");
	ADD_API_FUNCTION(mjc_neq, "int", "", "Number of equality constraints");
	ADD_API_FUNCTION(mjc_ncon, "int", "", "Active contacts this step");
	ADD_API_FUNCTION(mjc_qpos, "PackedFloat64Array", "", "Generalised positions");
	ADD_API_FUNCTION(mjc_digest, "int", "", "Digest of the full integration state");
	ADD_API_FUNCTION(mjc_lowest_mm, "float", "", "Lowest body height in millimetres");
	halt();
}
