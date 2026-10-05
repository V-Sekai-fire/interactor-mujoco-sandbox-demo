#include <api.hpp>
#include <mujoco/mujoco.h>

#include <cctype>
#include <clocale>
#include <cstdint>
#include <cstdlib>
#include <cstdio>
#include <cmath>
#include <cstring>
#include <string>
#include <vector>

#include "model_data.h"
#include "station.h"

// Arguments are declared as native types, not Variant. The host unboxes them
// into registers when unboxed arguments are on, and a Variant parameter then
// faults on entry. Under the boxed ABI it is the other way round, and a typed
// parameter reads the argument pointer as an integer and returns garbage
// without complaining, which is the worse of the two failures.
//
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

static Variant mjc_load(PackedArray<uint8_t> mjb) {
	const std::vector<uint8_t> v = mjb.fetch();
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
static Variant mjc_load_xml(PackedArray<uint8_t> xml) {
	const std::vector<uint8_t> v = xml.fetch();
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

// Each active contact as geom0, geom1, pos x, y, z, dist — six doubles a
// contact. pos is the midpoint between the two geoms, which is the crossing
// point when the geoms are two strokes' capsules.
static Variant mjc_contacts() {
	std::vector<double> out;
	if (g_model == nullptr || g_data == nullptr) {
		return PackedArray<double>(out);
	}
	out.reserve((size_t)g_data->ncon * 6);
	for (int i = 0; i < g_data->ncon; i++) {
		const mjContact &c = g_data->contact[i];
		out.push_back((double)c.geom[0]);
		out.push_back((double)c.geom[1]);
		out.push_back(c.pos[0]);
		out.push_back(c.pos[1]);
		out.push_back(c.pos[2]);
		out.push_back(c.dist);
	}
	return PackedArray<double>(out);
}

// Crossings between strokes, computed by the engine's collision. Each stroke
// (a run of `counts[i]` points from the flat `points` xyz array) becomes one
// freejointed body of capsules of radius proximity/2, so a contact between
// capsules of different strokes is a crossing at the contact midpoint. Contacts
// within merge_epsilon of an earlier one are the same crossing and are dropped.
// Returns the crossings as flat x, y, z triples. The host-facing surface the
// curvenet pipeline calls, shaped like cassie_graph's cg_cycles.
static Variant mj_crossings(PackedArray<float> points, PackedArray<int32_t> counts,
		double proximity) {
	// Coalesce contacts within a few capsule radii; a tessellated crossing then
	// counts once rather than once per touching segment pair.
	const double merge_epsilon = proximity * 3.0 > 0.03 ? proximity * 3.0 : 0.03;
	const std::vector<float> pts = points.fetch();
	const std::vector<int32_t> cnt = counts.fetch();
	std::vector<double> out;
	if (cnt.empty()) {
		return PackedArray<double>(out);
	}
	// Geoms are numbered in emission order: stroke i contributes counts[i]-1 capsules.
	std::vector<int> g2s;
	for (size_t si = 0; si < cnt.size(); si++) {
		const int caps = cnt[si] > 0 ? cnt[si] - 1 : 0;
		for (int i = 0; i < caps; i++) {
			g2s.push_back((int)si);
		}
	}
	const double r = proximity * 0.5;
	std::string s = "<mujoco><option gravity='0 0 0'/><worldbody>";
	size_t off = 0;
	char buf[256];
	for (size_t si = 0; si < cnt.size(); si++) {
		const int n = cnt[si];
		s += "<body pos='0 0 0'><freejoint/>";
		for (int i = 0; i + 1 < n; i++) {
			const float *a = &pts[(off + (size_t)i) * 3];
			const float *b = &pts[(off + (size_t)i + 1) * 3];
			snprintf(buf, sizeof(buf),
				"<geom type='capsule' size='%g' fromto='%g %g %g %g %g %g'/>",
				(double)r, (double)a[0], (double)a[1], (double)a[2],
				(double)b[0], (double)b[1], (double)b[2]);
			s += buf;
		}
		s += "</body>";
		off += (size_t)(n > 0 ? n : 0);
	}
	s += "</worldbody></mujoco>";
	if (!load_xml_text(s.c_str(), (int)s.size())) {
		return PackedArray<double>(out);
	}
	const double me2 = (double)merge_epsilon * (double)merge_epsilon;
	for (int k = 0; k < g_data->ncon; k++) {
		const mjContact &c = g_data->contact[k];
		const int a = c.geom[0], b = c.geom[1];
		if (a < 0 || b < 0 || a >= (int)g2s.size() || b >= (int)g2s.size()) {
			continue;
		}
		if (g2s[a] == g2s[b]) {
			continue;
		}
		bool merged = false;
		for (size_t j = 0; j + 2 < out.size(); j += 3) {
			const double dx = c.pos[0] - out[j], dy = c.pos[1] - out[j + 1], dz = c.pos[2] - out[j + 2];
			if (dx * dx + dy * dy + dz * dz < me2) {
				merged = true;
				break;
			}
		}
		if (!merged) {
			out.push_back(c.pos[0]);
			out.push_back(c.pos[1]);
			out.push_back(c.pos[2]);
		}
	}
	return PackedArray<double>(out);
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

// Which hinge a later mjc_hold applies to. Split from the angle because a
// two-Variant guest function faults on the way in; one argument each works.
static int g_held = -1;

static Variant mjc_select(int index) {
	const int i = index;
	if (g_model == nullptr || i < -1 || i >= g_model->nq) {
		g_held = -1;
		return false;
	}
	g_held = i;
	return true;
}

// Hold the selected hinge at an angle, as a hand holding a ball does: the
// joint is placed and its velocity cleared, so releasing it starts from rest
// rather than from whatever the solver had accumulated.
static Variant mjc_hold(double angle) {
	if (g_model == nullptr || g_data == nullptr || g_held < 0 || g_held >= g_model->nq) {
		return false;
	}
	g_data->qpos[g_held] = angle;
	g_data->qvel[g_held] = 0.0;
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

// The station's colliders, terrain and player capsule (station.h has the layouts). Returns the
// geom count, or -1 when the input or the model is refused.
static Variant mjc_load_primitives(PackedArray<double> prims, PackedArray<double> grid, int nrow, int ncol,
		PackedArray<double> hsize) {
	std::string error;
	const std::string xml = station::build_mjcf(prims.fetch(), grid.fetch(), nrow, ncol, hsize.fetch(), error);
	if (xml.empty()) {
		print(error.c_str());
		return -1;
	}
	if (!load_xml_text(xml.c_str(), (int)xml.size())) {
		return -1;
	}
	return (int)g_model->ngeom;
}

static Variant mjc_ray(PackedArray<double> origin, PackedArray<double> dir, double maxdist, int exclude_body) {
	const std::vector<double> o = origin.fetch();
	const std::vector<double> v = dir.fetch();
	if (o.size() != 3 || v.size() != 3) {
		return PackedArray<double>(station::ray(nullptr, nullptr, nullptr, nullptr, 0.0, -1));
	}
	return PackedArray<double>(station::ray(g_model, g_data, o.data(), v.data(), maxdist, exclude_body));
}

static Variant mjc_ray_group(PackedArray<double> origin, PackedArray<double> dir, double maxdist, int exclude_body,
		int group) {
	const std::vector<double> o = origin.fetch();
	const std::vector<double> v = dir.fetch();
	if (o.size() != 3 || v.size() != 3) {
		return PackedArray<double>(station::ray(nullptr, nullptr, nullptr, nullptr, 0.0, -1));
	}
	return PackedArray<double>(station::ray(g_model, g_data, o.data(), v.data(), maxdist, exclude_body, group));
}

static Variant mjc_player_band(double radius, double bottom, double top) {
	station::set_player_band(radius, bottom, top);
	return radius > 0.0 && top > bottom;
}

static Variant mjc_player_contacts() {
	return PackedArray<double>(station::player_contacts(g_model, g_data));
}

static bool set_state(PackedArray<double> values, mjtNum *dst, int count) {
	const std::vector<double> v = values.fetch();
	if (g_model == nullptr || dst == nullptr || v.size() != (size_t)count) {
		return false;
	}
	for (size_t i = 0; i < v.size(); i++) {
		dst[i] = v[i];
	}
	return true;
}

// Each point inside the loaded model moves out to the surface it would exit
// through along a horizontal ray from the vertical axis (axis_x, axis_z), plus
// thickness; a first hit facing the point means outside, and it stays.
// Answers the points, then how many moved.
static Variant mj_push_out(PackedArray<double> points, double axis_x, double axis_z, double thickness) {
	std::vector<double> out = points.fetch();
	int moved = 0;
	for (size_t i = 0; i + 2 < out.size(); i += 3) {
		double d[3] = { out[i] - axis_x, 0.0, out[i + 2] - axis_z };
		const double len = sqrt(d[0] * d[0] + d[2] * d[2]);
		if (!(len > 1e-9)) {
			continue;
		}
		d[0] /= len;
		d[2] /= len;
		const std::vector<double> hit = station::ray(g_model, g_data, &out[i], d, 4.0, -1);
		if (hit[0] < 0.5 || hit[5] * d[0] + hit[6] * d[1] + hit[7] * d[2] <= 0.0) {
			continue;
		}
		for (int k = 0; k < 3; k++) {
			out[i + k] = hit[2 + k] + d[k] * thickness;
		}
		moved++;
	}
	out.push_back((double)moved);
	return PackedArray<double>(out);
}

static Variant mjc_set_qpos(PackedArray<double> qpos) {
	return set_state(qpos, g_data == nullptr ? nullptr : g_data->qpos, g_model == nullptr ? 0 : (int)g_model->nq);
}

static Variant mjc_set_qvel(PackedArray<double> qvel) {
	return set_state(qvel, g_data == nullptr ? nullptr : g_data->qvel, g_model == nullptr ? 0 : (int)g_model->nv);
}

// Kinematics and collision for the current state, with no integration.
static Variant mjc_forward() {
	if (g_model == nullptr || g_data == nullptr) {
		return false;
	}
	mj_forward(g_model, g_data);
	return true;
}

static Variant mjc_mocap_set(int index, PackedArray<double> pose) {
	const std::vector<double> p = pose.fetch();
	if (g_model == nullptr || g_data == nullptr || index < 0 || index >= g_model->nmocap || p.size() != 7) {
		return false;
	}
	for (int k = 0; k < 3; k++) {
		g_data->mocap_pos[index * 3 + k] = p[k];
	}
	for (int k = 0; k < 4; k++) {
		g_data->mocap_quat[index * 4 + k] = p[3 + k];
	}
	return true;
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
	ADD_API_FUNCTION(mjc_contacts, "PackedFloat64Array", "", "Active contacts as geom0, geom1, pos xyz, dist");
	ADD_API_FUNCTION(mj_crossings, "PackedFloat64Array", "PackedFloat32Array points, PackedInt32Array counts, float proximity", "Stroke crossings as coalesced x,y,z triples");
	ADD_API_FUNCTION(mjc_nbody, "int", "", "Number of bodies");
	ADD_API_FUNCTION(mjc_bodies, "PackedFloat64Array", "", "Body transforms as x,y,z,qw,qx,qy,qz");
	ADD_API_FUNCTION(mjc_ngeom, "int", "", "Number of geoms");
	ADD_API_FUNCTION(mjc_geoms, "Array", "", "Geoms as type, size xyz, pos xyz, quat wxyz");
	ADD_API_FUNCTION(mjc_select, "bool", "int index", "Choose the hinge a hold applies to");
	ADD_API_FUNCTION(mjc_hold, "bool", "float angle", "Hold the selected hinge at an angle");
	ADD_API_FUNCTION(mjc_pivots, "PackedFloat64Array", "", "Per joint: anchor x, anchor z, pendulum length");
	ADD_API_FUNCTION(mjc_time, "float", "", "Simulated time");
	ADD_API_FUNCTION(mjc_qpos, "PackedFloat64Array", "", "Generalised positions");
	ADD_API_FUNCTION(mjc_digest, "int", "", "Digest of the full integration state");
	ADD_API_FUNCTION(mjc_lowest_mm, "float", "", "Lowest body height in millimetres");
	ADD_API_FUNCTION(mjc_load_primitives, "int", "PackedFloat64Array prims, PackedFloat64Array grid, int nrow, int ncol, PackedFloat64Array hsize", "Load station colliders, terrain and the player capsule; returns the geom count or -1");
	ADD_API_FUNCTION(mj_push_out, "PackedFloat64Array", "PackedFloat64Array points, float axis_x, float axis_z, float thickness", "Move points inside the model out through its surface; last value is the count moved");
	ADD_API_FUNCTION(mjc_ray, "PackedFloat64Array", "PackedFloat64Array origin, PackedFloat64Array dir, float maxdist, int exclude_body", "Nearest hit: hit, dist, point xyz, normal xyz, geom");
	ADD_API_FUNCTION(mjc_set_qpos, "bool", "PackedFloat64Array qpos", "Set every generalised position");
	ADD_API_FUNCTION(mjc_set_qvel, "bool", "PackedFloat64Array qvel", "Set every generalised velocity");
	ADD_API_FUNCTION(mjc_forward, "bool", "", "Kinematics and collision without integrating");
	ADD_API_FUNCTION(mjc_mocap_set, "bool", "int index, PackedFloat64Array pose", "Place a mocap body at x,y,z,qw,qx,qy,qz");
	ADD_API_FUNCTION(mjc_ray_group, "PackedFloat64Array", "PackedFloat64Array origin, PackedFloat64Array dir, float maxdist, int exclude_body, int group", "Nearest hit within one geom group (1 is walkable)");
	ADD_API_FUNCTION(mjc_player_band, "bool", "float radius, float bottom, float top", "Make the next load's player a vertical cylinder over [bottom, top]");
	ADD_API_FUNCTION(mjc_player_contacts, "PackedFloat64Array", "", "Player contacts: other geom, normal xyz into the player, dist");
	halt();
}
