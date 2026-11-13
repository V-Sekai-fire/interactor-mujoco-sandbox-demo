#include "human.h"

#include "human_files.h"

#include <cstdlib>
#include <cstring>

// The stock model is edited here and nowhere else, so no second copy of its XML exists:
//   1. the six pelvis joints become one free joint named "root", as the retargeted clips expect;
//   2. a floor plane is added;
//   3. a mesh geom goes when it neither collides nor gives its body mass, and then every mesh,
//      texture and material nothing refers to goes with it. A body without an <inertial> takes
//      its mass from its geoms, so the meshes of such a body stay and no inertia is typed by hand;
//   4. a mesh that stays is read here from OpenUSD text and given to the model as points and
//      triangles. The stock XML names an STL or OBJ file; the embedded mesh has that name with
//      .usda in its place, and MuJoCo's own mesh readers are not linked.
namespace human {

static const char *const PELVIS_JOINTS[] = { "pelvis_tz", "pelvis_ty", "pelvis_tx", "pelvis_tilt", "pelvis_list", "pelvis_rotation" };

std::vector<std::string> files() {
	std::vector<std::string> out;
	for (int i = 0; i < HUMAN_FILE_COUNT; i++) {
		out.push_back(HUMAN_FILES[i].name);
	}
	return out;
}

// "a/b/../c/d.stl" -> "a/c/d.stl".
static std::string normal(const std::string &path) {
	std::vector<std::string> parts;
	size_t at = 0;
	while (at <= path.size()) {
		size_t end = path.find('/', at);
		if (end == std::string::npos) {
			end = path.size();
		}
		const std::string part = path.substr(at, end - at);
		if (part == "..") {
			if (!parts.empty()) {
				parts.pop_back();
			}
		} else if (!part.empty() && part != ".") {
			parts.push_back(part);
		}
		at = end + 1;
	}
	std::string out;
	for (size_t i = 0; i < parts.size(); i++) {
		out += (i == 0 ? "" : "/") + parts[i];
	}
	return out;
}

// The numbers of one array attribute of a .usda file, as text: what stands between "name = ["
// and the closing bracket. nullptr when the attribute is absent.
static const char *array_of(const char *begin, const char *end, const char *name, const char **stop) {
	const std::string key = std::string(name) + " = [";
	const char *at = begin;
	while (at + key.size() <= end && memcmp(at, key.data(), key.size()) != 0) {
		at++;
	}
	if (at + key.size() > end) {
		return nullptr;
	}
	at += key.size();
	const char *close = at;
	while (close < end && *close != ']') {
		close++;
	}
	*stop = close;
	return at;
}

// Reads the one Mesh of a .usda file written by tools/human/vendor.exs: every face a triangle.
static bool read_usda(const unsigned char *data, const unsigned char *data_end, std::vector<float> &points, std::vector<int> &faces) {
	const char *begin = (const char *)data;
	const char *end = (const char *)data_end;
	const char *stop = nullptr;

	const char *at = array_of(begin, end, "faceVertexCounts", &stop);
	if (at == nullptr) {
		return false;
	}
	int triangles = 0;
	while (at < stop) {
		char *next = nullptr;
		const long count = strtol(at, &next, 10);
		if (next == at) {
			at++;
			continue;
		}
		if (count != 3) {
			return false;
		}
		triangles++;
		at = next;
	}

	at = array_of(begin, end, "faceVertexIndices", &stop);
	if (at == nullptr) {
		return false;
	}
	while (at < stop) {
		char *next = nullptr;
		const long index = strtol(at, &next, 10);
		if (next == at) {
			at++;
			continue;
		}
		faces.push_back((int)index);
		at = next;
	}

	at = array_of(begin, end, "points", &stop);
	if (at == nullptr) {
		return false;
	}
	while (at < stop) {
		char *next = nullptr;
		const double value = strtod(at, &next);
		if (next == at) {
			at++;
			continue;
		}
		points.push_back((float)value);
		at = next;
	}

	if (triangles == 0 || (int)faces.size() != triangles * 3 || points.empty() || points.size() % 3 != 0) {
		return false;
	}
	for (int index : faces) {
		if (index < 0 || index >= (int)(points.size() / 3)) {
			return false;
		}
	}
	return true;
}

mjModel *build(int withhold, std::string &error, std::vector<std::string> &kept) {
	static mjVFS vfs;
	mj_defaultVFS(&vfs);
	for (int i = 0; i < HUMAN_FILE_COUNT; i++) {
		// Only the XML files go to MuJoCo's reader: the meshes are read below.
		const size_t length = strlen(HUMAN_FILES[i].name);
		if (i == withhold || length < 4 || strcmp(HUMAN_FILES[i].name + length - 4, ".xml") != 0) {
			continue;
		}
		const int size = (int)(HUMAN_FILES[i].end - HUMAN_FILES[i].begin);
		if (mj_addBufferVFS(&vfs, HUMAN_FILES[i].name, HUMAN_FILES[i].begin, size) != 0) {
			error = std::string("could not add ") + HUMAN_FILES[i].name;
			mj_deleteVFS(&vfs);
			return nullptr;
		}
	}

	char err[1024] = { 0 };
	mjSpec *spec = mj_parseXML(HUMAN_FILES[0].name, &vfs, err, (int)sizeof(err));
	if (spec == nullptr) {
		error = std::string("parse: ") + err;
		mj_deleteVFS(&vfs);
		return nullptr;
	}

	// 3. Mesh geoms that neither collide nor give mass.
	std::vector<mjsElement *> gone;
	std::vector<std::string> used;
	for (mjsElement *e = mjs_firstElement(spec, mjOBJ_GEOM); e != nullptr; e = mjs_nextElement(spec, e)) {
		mjsGeom *geom = mjs_asGeom(e);
		if (geom->type != mjGEOM_MESH) {
			continue;
		}
		const mjsBody *body = mjs_getParent(e);
		const bool collides = geom->contype != 0 || geom->conaffinity != 0;
		const bool gives_mass = body != nullptr && !body->explicitinertial;
		if (collides || gives_mass) {
			used.push_back(mjs_getString(geom->meshname));
		} else {
			gone.push_back(e);
		}
	}
	for (mjsElement *e = mjs_firstElement(spec, mjOBJ_MESH); e != nullptr; e = mjs_nextElement(spec, e)) {
		const std::string name = mjs_getString(mjs_getName(e));
		bool found = false;
		for (const std::string &u : used) {
			found = found || u == name;
		}
		if (found) {
			// The parser has already made the path relative to the model directory.
			mjsMesh *mesh = mjs_asMesh(e);
			std::string file = normal(mjs_getString(mesh->file));
			const size_t dot = file.rfind('.');
			file = file.substr(0, dot == std::string::npos ? file.size() : dot) + ".usda";
			int at = -1;
			for (int i = 0; i < HUMAN_FILE_COUNT; i++) {
				if (i != withhold && file == HUMAN_FILES[i].name) {
					at = i;
				}
			}
			std::vector<float> points;
			std::vector<int> faces;
			if (at < 0 || !read_usda(HUMAN_FILES[at].begin, HUMAN_FILES[at].end, points, faces)) {
				error = (at < 0 ? "mesh not embedded: " : "mesh not readable: ") + file;
				mj_deleteSpec(spec);
				mj_deleteVFS(&vfs);
				return nullptr;
			}
			mjs_setFloat(mesh->uservert, points.data(), (int)points.size());
			mjs_setInt(mesh->userface, faces.data(), (int)faces.size());
			mjs_setString(mesh->file, "");
			mjs_setString(mesh->content_type, "");
			kept.push_back(file);
		} else {
			gone.push_back(e);
		}
	}
	for (mjsElement *e = mjs_firstElement(spec, mjOBJ_MATERIAL); e != nullptr; e = mjs_nextElement(spec, e)) {
		gone.push_back(e);
	}
	for (mjsElement *e = mjs_firstElement(spec, mjOBJ_TEXTURE); e != nullptr; e = mjs_nextElement(spec, e)) {
		gone.push_back(e);
	}

	// 1. The free root.
	mjsBody *pelvis = mjs_findBody(spec, "pelvis");
	if (pelvis == nullptr) {
		error = "no pelvis body";
		mj_deleteSpec(spec);
		mj_deleteVFS(&vfs);
		return nullptr;
	}
	for (const char *name : PELVIS_JOINTS) {
		mjsElement *joint = mjs_findElement(spec, mjOBJ_JOINT, name);
		if (joint == nullptr) {
			error = std::string("no joint ") + name;
			mj_deleteSpec(spec);
			mj_deleteVFS(&vfs);
			return nullptr;
		}
		gone.push_back(joint);
	}
	for (mjsElement *e : gone) {
		if (mjs_delete(spec, e) != 0) {
			error = std::string("delete: ") + mjs_getError(spec);
			mj_deleteSpec(spec);
			mj_deleteVFS(&vfs);
			return nullptr;
		}
	}
	mjs_setName(mjs_addFreeJoint(pelvis)->element, "root");

	// The keyframe is 85 values for the six-joint root, so it no longer fits the model.
	gone.clear();
	for (mjsElement *e = mjs_firstElement(spec, mjOBJ_KEY); e != nullptr; e = mjs_nextElement(spec, e)) {
		gone.push_back(e);
	}
	for (mjsElement *e : gone) {
		mjs_delete(spec, e);
	}

	// 2. The floor.
	mjsGeom *floor = mjs_addGeom(mjs_findBody(spec, "world"), nullptr);
	mjs_setName(floor->element, "floor");
	floor->type = mjGEOM_PLANE;
	floor->size[0] = 0.0;
	floor->size[1] = 0.0;
	floor->size[2] = 0.05;
	floor->contype = 1;
	floor->conaffinity = 1;

	mjModel *model = mj_compile(spec, &vfs);
	if (model == nullptr) {
		error = std::string("compile: ") + mjs_getError(spec);
	}
	mj_deleteSpec(spec);
	mj_deleteVFS(&vfs);
	return model;
}

} // namespace human
