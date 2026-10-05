#include "station.h"

#include <cmath>
#include <cstdio>

namespace station {

namespace {

// MuJoCo cylinders, capsules and height fields stand along local z; Godot's up is y. A -90 degree
// turn about x carries local z onto y.
const double kZToY[4] = { 0.70710678118654752440, -0.70710678118654752440, 0.0, 0.0 };

double g_band_radius = 0.0;
double g_band_bottom = 0.0;
double g_band_top = 0.0;

void append(std::string &s, const char *fmt, double a, double b, double c) {
	char buf[160];
	snprintf(buf, sizeof(buf), fmt, a, b, c);
	s += buf;
}

void append_quat(std::string &s, const double q[4]) {
	char buf[128];
	snprintf(buf, sizeof(buf), " quat='%.17g %.17g %.17g %.17g'", q[0], q[1], q[2], q[3]);
	s += buf;
}

void append_geom(std::string &s, const double *p, bool at_origin) {
	const int type = (int)p[0];
	double q[4] = { p[7], p[8], p[9], p[10] };
	if (at_origin) {
		q[0] = 1.0;
		q[1] = q[2] = q[3] = 0.0;
	}
	if (type == kCylinder) {
		double turned[4];
		mju_mulQuat(turned, q, kZToY);
		append(s, "<geom type='cylinder' size='%.17g %.17g'", p[4], p[5], 0.0);
		append_quat(s, turned);
	} else {
		append(s, "<geom type='box' size='%.17g %.17g %.17g'", p[4], p[5], p[6]);
		append_quat(s, q);
		if (type == kWalkBox) {
			s += " group='1'";
		}
	}
	if (!at_origin) {
		append(s, " pos='%.17g %.17g %.17g'", p[1], p[2], p[3]);
	}
	s += "/>";
}

} // namespace

std::string build_mjcf(const std::vector<double> &prims, const std::vector<double> &grid, int nrow, int ncol,
		const std::vector<double> &hsize, std::string &error) {
	if (prims.size() % kPrimStride != 0) {
		error = "primitive list is not a multiple of 12 doubles";
		return "";
	}
	const bool terrain = !grid.empty();
	double lo = 0.0, hi = 0.0;
	if (terrain) {
		if (nrow < 2 || ncol < 2 || grid.size() != (size_t)nrow * (size_t)ncol || hsize.size() != 4) {
			error = "terrain needs nrow, ncol >= 2, nrow * ncol heights and four sizes";
			return "";
		}
		lo = hi = grid[0];
		for (double h : grid) {
			lo = h < lo ? h : lo;
			hi = h > hi ? h : hi;
		}
		if (!(hi > lo)) {
			error = "flat terrain: send it as a box";
			return "";
		}
	}

	std::string s = "<mujoco model='station'><option gravity='0 0 0'/>";
	if (terrain) {
		char head[256];
		snprintf(head, sizeof(head), "<asset><hfield name='terrain' nrow='%d' ncol='%d' size='%.17g %.17g %.17g 1' elevation='",
				nrow, ncol, hsize[0], hsize[1], hi - lo);
		s += head;
		// Normalised here so MuJoCo's own normalisation is the identity.
		char buf[32];
		for (int r = 0; r < nrow; r++) {
			for (int c = 0; c < ncol; c++) {
				snprintf(buf, sizeof(buf), "%.17g ", (grid[(size_t)r * ncol + c] - lo) / (hi - lo));
				s += buf;
			}
		}
		s += "'/></asset>";
	}
	s += "<worldbody>";
	if (terrain) {
		s += "<geom type='hfield' hfield='terrain' group='1'";
		append(s, " pos='%.17g %.17g %.17g'", hsize[2], lo, hsize[3]);
		append_quat(s, kZToY);
		s += "/>";
	}
	const size_t n = prims.size() / kPrimStride;
	for (size_t i = 0; i < n; i++) {
		const double *p = &prims[i * kPrimStride];
		if (p[11] != 0.0) {
			continue;
		}
		append_geom(s, p, false);
	}
	for (size_t i = 0; i < n; i++) {
		const double *p = &prims[i * kPrimStride];
		if (p[11] == 0.0) {
			continue;
		}
		s += "<body mocap='true'";
		append(s, " pos='%.17g %.17g %.17g'", p[1], p[2], p[3]);
		append_quat(s, p + 7);
		s += ">";
		append_geom(s, p, true);
		s += "</body>";
	}
	s += "<body name='player'><joint type='slide' axis='1 0 0'/><joint type='slide' axis='0 1 0'/>"
		 "<joint type='slide' axis='0 0 1'/>";
	if (g_band_radius > 0.0) {
		append(s, "<geom type='cylinder' size='%.17g %.17g' pos='0 %.17g 0'", g_band_radius,
				(g_band_top - g_band_bottom) * 0.5, (g_band_top + g_band_bottom) * 0.5);
	} else {
		const double half = kPlayerHeight * 0.5 - kPlayerRadius;
		append(s, "<geom type='capsule' size='%.17g %.17g' pos='0 %.17g 0'", kPlayerRadius, half, kPlayerHeight * 0.5);
	}
	append_quat(s, kZToY);
	s += "/></body></worldbody></mujoco>";
	return s;
}

void set_player_band(double radius, double bottom, double top) {
	const bool valid = radius > 0.0 && top > bottom;
	g_band_radius = valid ? radius : 0.0;
	g_band_bottom = valid ? bottom : 0.0;
	g_band_top = valid ? top : 0.0;
}

mjModel *load_mjcf(const std::string &xml, std::string &error) {
	mjVFS vfs;
	mj_defaultVFS(&vfs);
	if (mj_addBufferVFS(&vfs, "station.xml", xml.data(), (int)xml.size()) != 0) {
		mj_deleteVFS(&vfs);
		error = "could not stage the model";
		return nullptr;
	}
	char err[512] = { 0 };
	mjModel *m = mj_loadXML("station.xml", &vfs, err, (int)sizeof(err));
	mj_deleteVFS(&vfs);
	if (m == nullptr) {
		error = err;
	}
	return m;
}

std::vector<double> ray(const mjModel *m, const mjData *d, const double origin[3], const double dir[3],
		double maxdist, int exclude_body, int group) {
	std::vector<double> out(9, 0.0);
	out[8] = -1.0;
	const double len = sqrt(dir[0] * dir[0] + dir[1] * dir[1] + dir[2] * dir[2]);
	if (m == nullptr || d == nullptr || !(len > 0.0)) {
		return out;
	}
	const double unit[3] = { dir[0] / len, dir[1] / len, dir[2] / len };
	int geom = -1;
	double normal[3] = { 0.0, 0.0, 0.0 };
	mjtByte groups[mjNGROUP] = { 0 };
	if (group >= 0 && group < mjNGROUP) {
		groups[group] = 1;
	}
	const double dist = mj_ray(m, d, origin, unit, group < 0 ? nullptr : groups, 1, exclude_body, &geom, normal);
	if (dist < 0.0 || dist > maxdist) {
		return out;
	}
	out[0] = 1.0;
	out[1] = dist;
	for (int k = 0; k < 3; k++) {
		out[2 + k] = origin[k] + dist * unit[k];
		out[5 + k] = normal[k];
	}
	out[8] = (double)geom;
	return out;
}

std::vector<double> player_contacts(const mjModel *m, const mjData *d) {
	std::vector<double> out;
	if (m == nullptr || d == nullptr || m->nbody < 2) {
		return out;
	}
	const int player = m->nbody - 1;
	for (int i = 0; i < d->ncon; i++) {
		const mjContact &c = d->contact[i];
		const bool first = m->geom_bodyid[c.geom[0]] == player;
		const bool second = m->geom_bodyid[c.geom[1]] == player;
		if (first == second) {
			continue;
		}
		// The frame's normal points from geom[0] to geom[1].
		const double sign = second ? 1.0 : -1.0;
		out.push_back((double)(second ? c.geom[0] : c.geom[1]));
		for (int k = 0; k < 3; k++) {
			out.push_back(sign * c.frame[k]);
		}
		out.push_back(c.dist);
	}
	return out;
}

} // namespace station
