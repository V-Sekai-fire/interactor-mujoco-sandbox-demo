#include "witness/doctest.h"

#include "station.h"

#include <mujoco/mujoco.h>

#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

namespace {

void *aligned64_malloc(size_t n) {
	void *raw = malloc(n + 64 + sizeof(void *));
	if (raw == nullptr) {
		return nullptr;
	}
	uintptr_t base = (uintptr_t)raw + sizeof(void *);
	uintptr_t aligned = (base + 63) & ~(uintptr_t)63;
	((void **)aligned)[-1] = raw;
	return (void *)aligned;
}

void aligned64_free(void *p) {
	if (p != nullptr) {
		free(((void **)p)[-1]);
	}
}

struct AlignedAllocator {
	AlignedAllocator() {
		mju_user_malloc = aligned64_malloc;
		mju_user_free = aligned64_free;
	}
};

AlignedAllocator g_installed;

struct Station {
	mjModel *m = nullptr;
	mjData *d = nullptr;
	std::string error;

	Station(const std::vector<double> &prims, const std::vector<double> &grid, int nrow, int ncol,
			const std::vector<double> &hsize) {
		const std::string xml = station::build_mjcf(prims, grid, nrow, ncol, hsize, error);
		if (!xml.empty()) {
			m = station::load_mjcf(xml, error);
		}
		if (m != nullptr) {
			d = mj_makeData(m);
			mj_forward(m, d);
		}
	}
	~Station() {
		if (d != nullptr) {
			mj_deleteData(d);
		}
		if (m != nullptr) {
			mj_deleteModel(m);
		}
	}
	int player() const { return m->nbody - 1; }
	std::vector<double> cast(double ox, double oy, double oz, double dx, double dy, double dz) const {
		const double o[3] = { ox, oy, oz };
		const double v[3] = { dx, dy, dz };
		return station::ray(m, d, o, v, 100.0, player());
	}
};

void box(std::vector<double> &prims, double cx, double cy, double cz, double hx, double hy, double hz) {
	const double p[station::kPrimStride] = { station::kBox, cx, cy, cz, hx, hy, hz, 1, 0, 0, 0, 0 };
	prims.insert(prims.end(), p, p + station::kPrimStride);
}

void walk_box(std::vector<double> &prims, double cx, double cy, double cz, double hx, double hy, double hz) {
	const double p[station::kPrimStride] = { station::kWalkBox, cx, cy, cz, hx, hy, hz, 1, 0, 0, 0, 0 };
	prims.insert(prims.end(), p, p + station::kPrimStride);
}

// The player as the walker uses it: a 0.3 m cylinder over [0.45, 1.7] above the feet.
struct BandStation : Station {
	BandStation(const std::vector<double> &prims, double fx, double fy, double fz) :
			Station((station::set_player_band(0.3, 0.45, 1.7), prims), {}, 0, 0, {}) {
		station::set_player_band(0.0, 0.0, 0.0);
		if (d != nullptr) {
			d->qpos[0] = fx;
			d->qpos[1] = fy;
			d->qpos[2] = fz;
			mj_forward(m, d);
		}
	}
};

void cylinder(std::vector<double> &prims, double cx, double cy, double cz, double r, double hh) {
	const double p[station::kPrimStride] = { station::kCylinder, cx, cy, cz, r, hh, 0, 1, 0, 0, 0, 0 };
	prims.insert(prims.end(), p, p + station::kPrimStride);
}

// A 4 x 4 field whose interior heights all differ, so a flipped row or column order is visible.
// Every height normalises exactly over [0, 2], so the field holds them without float rounding.
const std::vector<double> kGrid = {
	0.0, 0.5, 1.0, 1.5,
	2.0, 0.0, 1.5, 0.5,
	1.0, 2.0, 0.5, 0.0,
	1.5, 1.0, 0.0, 2.0,
};
const std::vector<double> kSize = { 3.0, 3.0, 0.0, 0.0 };

double worst_vertex_error(const Station &s, const std::vector<double> &expected) {
	double worst = 0.0;
	for (int r = 1; r <= 2; r++) {
		for (int c = 1; c <= 2; c++) {
			const double x = -3.0 + 2.0 * c;
			const double z = -3.0 + 2.0 * r;
			const std::vector<double> hit = s.cast(x, 10.0, z, 0.0, -1.0, 0.0);
			const double err = hit[0] == 1.0 ? fabs(hit[3] - expected[(size_t)r * 4 + c]) : 1e9;
			worst = err > worst ? err : worst;
		}
	}
	return worst;
}

// Contacts and the player's geom pose after each placement, as raw bytes.
std::vector<uint8_t> walk_bytes(const std::vector<double> &prims) {
	Station s(prims, kGrid, 4, 4, kSize);
	std::vector<uint8_t> out;
	for (int i = 0; i <= 20; i++) {
		const double q[3] = { 0.30 * i / 20.0, 0.0, 0.0 };
		for (int k = 0; k < 3; k++) {
			s.d->qpos[k] = q[k];
		}
		mj_forward(s.m, s.d);
		for (int c = 0; c < s.d->ncon; c++) {
			const mjContact &con = s.d->contact[c];
			const uint8_t *p = (const uint8_t *)&con.dist;
			out.insert(out.end(), p, p + sizeof(con.dist));
			p = (const uint8_t *)con.pos;
			out.insert(out.end(), p, p + sizeof(con.pos));
			p = (const uint8_t *)con.geom;
			out.insert(out.end(), p, p + sizeof(con.geom));
		}
		const int g = s.m->ngeom - 1;
		const uint8_t *p = (const uint8_t *)(s.d->geom_xpos + g * 3);
		out.insert(out.end(), p, p + 3 * sizeof(mjtNum));
	}
	return out;
}

std::vector<double> walk_prims(double face_shift) {
	std::vector<double> prims;
	box(prims, 1.0 + face_shift, 0.85, 0.0, 0.5, 1.0, 1.0);
	cylinder(prims, -1.0, 1.0, 0.0, 0.4, 1.0);
	return prims;
}

} // namespace

TEST_CASE("[unit] a ray straight at a box meets its face at the right distance") {
	std::vector<double> prims;
	box(prims, 0.0, 1.0, -5.0, 1.0, 1.0, 0.5);
	Station s(prims, {}, 0, 0, {});
	REQUIRE_MESSAGE(s.m != nullptr, s.error);
	const std::vector<double> hit = s.cast(0.0, 1.0, 0.0, 0.0, 0.0, -1.0);
	CHECK(hit[0] == 1.0);
	CHECK(fabs(hit[1] - 4.5) < 1e-12);
	CHECK(fabs(hit[7] - 1.0) < 1e-12);
}

TEST_CASE("[falsification] a ray running parallel above the box misses it") {
	std::vector<double> prims;
	box(prims, 0.0, 1.0, -5.0, 1.0, 1.0, 0.5);
	Station s(prims, {}, 0, 0, {});
	REQUIRE(s.m != nullptr);
	const std::vector<double> hit = s.cast(0.0, 3.0, 0.0, 0.0, 0.0, -1.0);
	CHECK(hit[0] == 0.0);
}

TEST_CASE("[unit] a cylinder stands up along y") {
	std::vector<double> prims;
	cylinder(prims, 0.0, 1.0, -3.0, 0.5, 1.0);
	Station s(prims, {}, 0, 0, {});
	REQUIRE(s.m != nullptr);
	const std::vector<double> top = s.cast(0.0, 10.0, -3.0, 0.0, -1.0, 0.0);
	CHECK(top[0] == 1.0);
	CHECK(fabs(top[3] - 2.0) < 1e-12);
	const std::vector<double> side = s.cast(0.0, 1.0, 0.0, 0.0, 0.0, -1.0);
	CHECK(fabs(side[1] - 2.5) < 1e-12);
}

TEST_CASE("[unit] the terrain holds the grid's heights at its interior vertices") {
	Station s({}, kGrid, 4, 4, kSize);
	REQUIRE_MESSAGE(s.m != nullptr, s.error);
	CHECK(worst_vertex_error(s, kGrid) <= 1e-9);
}

TEST_CASE("[falsification] a grid shifted by one column does not match") {
	std::vector<double> shifted(kGrid.size());
	for (int r = 0; r < 4; r++) {
		for (int c = 0; c < 4; c++) {
			shifted[(size_t)r * 4 + c] = kGrid[(size_t)r * 4 + (c + 1) % 4];
		}
	}
	Station s({}, kGrid, 4, 4, kSize);
	REQUIRE(s.m != nullptr);
	CHECK(worst_vertex_error(s, shifted) > 1e-9);
}

TEST_CASE("[falsification] a flat grid and a ragged primitive list are refused") {
	std::string error;
	CHECK(station::build_mjcf({}, std::vector<double>(16, 1.0), 4, 4, kSize, error).empty());
	CHECK(station::build_mjcf(std::vector<double>(13, 0.0), {}, 0, 0, {}, error).empty());
}

TEST_CASE("[property] the same walk gives the same contacts, bit for bit") {
	const std::vector<uint8_t> a = walk_bytes(walk_prims(0.0));
	const std::vector<uint8_t> b = walk_bytes(walk_prims(0.0));
	REQUIRE(!a.empty());
	CHECK(a.size() == b.size());
	CHECK(memcmp(a.data(), b.data(), a.size() < b.size() ? a.size() : b.size()) == 0);
}

TEST_CASE("[falsification] a box moved by one ulp changes the contacts") {
	const std::vector<uint8_t> a = walk_bytes(walk_prims(0.0));
	const std::vector<uint8_t> b = walk_bytes(walk_prims(nextafter(1.0, 2.0) - 1.0));
	CHECK((a.size() != b.size() || memcmp(a.data(), b.data(), a.size()) != 0));
}

TEST_CASE("[unit] a walkable ray stands on the walk top under a solid box") {
	std::vector<double> prims;
	box(prims, 0.0, 0.2, 0.0, 1.0, 0.2, 1.0);
	walk_box(prims, 0.0, -0.25, 0.0, 2.0, 0.25, 2.0);
	Station s(prims, {}, 0, 0, {});
	REQUIRE_MESSAGE(s.m != nullptr, s.error);
	const double o[3] = { 0.0, 5.0, 0.0 };
	const double v[3] = { 0.0, -1.0, 0.0 };
	const std::vector<double> walkable = station::ray(s.m, s.d, o, v, 100.0, s.player(), 1);
	CHECK(walkable[0] == 1.0);
	CHECK(fabs(walkable[3] - 0.0) < 1e-12);
}

TEST_CASE("[falsification] the same ray over every group stops on the solid box") {
	std::vector<double> prims;
	box(prims, 0.0, 0.2, 0.0, 1.0, 0.2, 1.0);
	walk_box(prims, 0.0, -0.25, 0.0, 2.0, 0.25, 2.0);
	Station s(prims, {}, 0, 0, {});
	REQUIRE(s.m != nullptr);
	const std::vector<double> any = s.cast(0.0, 5.0, 0.0, 0.0, -1.0, 0.0);
	CHECK(fabs(any[3] - 0.4) < 1e-12);
}

TEST_CASE("[unit] a wall overlapping the band pushes the player straight out") {
	std::vector<double> prims;
	box(prims, 1.0, 1.0, 0.0, 0.8, 1.0, 1.0);
	BandStation s(prims, 0.0, 0.0, 0.0);
	REQUIRE_MESSAGE(s.m != nullptr, s.error);
	const std::vector<double> c = station::player_contacts(s.m, s.d);
	REQUIRE(!c.empty());
	REQUIRE(c.size() % 5 == 0);
	double deepest = 0.0;
	for (size_t i = 0; i < c.size(); i += 5) {
		CHECK(c[i + 1] < -0.999);
		CHECK(fabs(c[i + 2]) < 1e-9);
		deepest = c[i + 4] < deepest ? c[i + 4] : deepest;
	}
	CHECK(fabs(deepest + 0.1) < 1e-6);
}

TEST_CASE("[falsification] a box whose top is below the band does not touch it") {
	std::vector<double> prims;
	box(prims, 0.5, 0.2, 0.0, 0.8, 0.2, 1.0);
	BandStation s(prims, 0.0, 0.0, 0.0);
	REQUIRE(s.m != nullptr);
	CHECK(station::player_contacts(s.m, s.d).empty());
}
