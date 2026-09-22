#include "witness/doctest.h"

#include <mujoco/mujoco.h>

#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <functional>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// Four coordinate frames and the transforms between them:
//
//   UI (screen pixels, 2D)
//     |  unproject a pixel through the camera -- runs in Godot, see the note
//     v                                          at the foot of this file
//   Egocentric 3D (the ray's own view frame, -Z forward)
//     |  the ray's world pose (looking down -Z)
//     v                        The lasso scores balls by their angle from -Z
//   Godot (Y-up world, 3D)     here; that CONVENTION is tested.
//     |  holder.affine_inverse  -- the -90-degrees-about-X holder rotation.
//     v                            Its CONVENTION is tested here.
//   MuJoCo (Z-up model, 3D)
//     |  ray -> swing plane -> hinge angle -> ball position
//     v                            Tested against real MuJoCo kinematics.
//
// The drag bug lived at the bottom edge: the hold angle had its x term negated,
// so dragging right swung the ball left. These tests pin each boundary's
// convention with a falsification control, so a sign flip at any of them is
// caught rather than seen only on screen.

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

struct Vec3 {
	double x, y, z;
};

int sign(double v) {
	if (v > 1e-9) {
		return 1;
	}
	if (v < -1e-9) {
		return -1;
	}
	return 0;
}

Vec3 make_vec3(witness::RNG &rng, const witness::Level &) {
	return { rng.int_range(-1000, 1000) / 1000.0,
			 rng.int_range(-1000, 1000) / 1000.0,
			 rng.int_range(-1000, 1000) / 1000.0 };
}

// -------- The Godot <-> MuJoCo boundary --------
//
// host.gd rotates the holder -90 degrees about X so a Z-up model draws in a
// Y-up world. That rotation maps a MuJoCo vector to a Godot one; its inverse
// maps a Godot vector (a camera ray) back into the model frame, which is what
// _pointer_in_model does with holder.affine_inverse().
//
// Rx(-90): (mx, my, mz) -> (mx, mz, -my).

Vec3 mj_to_godot(Vec3 m) {
	return { m.x, m.z, -m.y };
}

Vec3 godot_to_mj(Vec3 g) {
	return { g.x, -g.z, g.y };
}

// A wrong sign on the rotation: +90 about X. The convention tests must reject
// this, or they are not testing the direction of the turn.
Vec3 mj_to_godot_wrong(Vec3 m) {
	return { m.x, -m.z, m.y };
}

// -------- Inside the MuJoCo frame --------

const double PIVOT_Z = 0.20;
const double PIVOT_X = 0.0;
const double L = 0.13;

mjModel *pendulum() {
	static mjModel *m = nullptr;
	if (m != nullptr) {
		return m;
	}
	static const char *xml =
			R"XML(<mujoco><compiler angle="radian"/><worldbody>
<body name="arm" pos="0 0 0.20">
  <joint name="h" type="hinge" axis="0 1 0" pos="0 0 0"/>
  <geom name="ball" type="sphere" size="0.01" pos="0 0 -0.13"/>
</body>
</worldbody></mujoco>)XML";
	mjVFS vfs;
	mj_defaultVFS(&vfs);
	mj_addBufferVFS(&vfs, "p.xml", xml, (int)strlen(xml));
	char err[256] = { 0 };
	m = mj_loadXML("p.xml", &vfs, err, (int)sizeof(err));
	mj_deleteVFS(&vfs);
	return m;
}

int sphere_geom(const mjModel *m) {
	for (int i = 0; i < m->ngeom; i++) {
		if (m->geom_type[i] == mjGEOM_SPHERE) {
			return i;
		}
	}
	return -1;
}

void ball_at(double theta, double &x, double &z) {
	mjModel *m = pendulum();
	static mjData *d = nullptr;
	if (d == nullptr) {
		d = mj_makeData(m);
	}
	d->qpos[0] = theta;
	d->qvel[0] = 0.0;
	mj_forward(m, d);
	const int g = sphere_geom(m);
	x = d->geom_xpos[g * 3 + 0];
	z = d->geom_xpos[g * 3 + 2];
}

// Where a ray, already in the MuJoCo frame, crosses the plane the figure swings
// in (y = 0). This is the core of _pointer_in_model, after the camera ray has
// been mapped through holder.affine_inverse(). Returns false when the ray runs
// parallel to the plane, matching host.gd's |d.y| < 1e-6 guard.
bool swing_plane(Vec3 o, Vec3 d, Vec3 &hit) {
	if (std::fabs(d.y) < 1e-6) {
		return false;
	}
	const double t = -o.y / d.y;
	hit = { o.x + t * d.x, o.y + t * d.y, o.z + t * d.z };
	return true;
}

double hold_angle(double tx, double tz) {
	return atan2(PIVOT_X - tx, PIVOT_Z - tz);
}

double hold_angle_flipped(double tx, double tz) {
	return atan2(tx - PIVOT_X, PIVOT_Z - tz);
}

double make_angle(witness::RNG &rng, const witness::Level &) {
	return rng.int_range(-1200, 1200) / 1000.0;
}

double make_offset(witness::RNG &rng, const witness::Level &) {
	const double mag = rng.int_range(10, 100) / 1000.0;
	return rng.int_range(-100, 100) < 0 ? -mag : mag;
}

// -------- Egocentric 3D (the ray's own view frame, -Z forward) --------
//
// The lasso builds a frame looking down the ray and scores each ball by the
// angle between it and -Z, so an imprecise aim still picks the ball the pointer
// is nearest to. That angle is what this frame is for.

double v3len(Vec3 a) {
	return std::sqrt(a.x * a.x + a.y * a.y + a.z * a.z);
}

double v3dot(Vec3 a, Vec3 b) {
	return a.x * b.x + a.y * b.y + a.z * b.z;
}

double angle_between(Vec3 a, Vec3 b) {
	double c = v3dot(a, b) / (v3len(a) * v3len(b));
	if (c > 1.0) {
		c = 1.0;
	}
	if (c < -1.0) {
		c = -1.0;
	}
	return std::acos(c);
}

// A ball's egocentric angular offset: the angle between the eye-to-ball vector
// and the ray direction. Zero straight ahead, larger off to the side.
double eye_angle(Vec3 eye, Vec3 ray_dir, Vec3 ball) {
	return angle_between({ ball.x - eye.x, ball.y - eye.y, ball.z - eye.z }, ray_dir);
}

// -------- Handedness --------
//
// MuJoCo and Godot are both right-handed; the holder rotation between them must
// be a proper rotation (determinant +1). A determinant of -1 is a reflection,
// which flips handedness and mirrors the scene.

double det3_rows(Vec3 r0, Vec3 r1, Vec3 r2) {
	return r0.x * (r1.y * r2.z - r1.z * r2.y) -
			r0.y * (r1.x * r2.z - r1.z * r2.x) +
			r0.z * (r1.x * r2.y - r1.y * r2.x);
}

} // namespace

// ============ Godot <-> MuJoCo ============

TEST_CASE("[unit] the holder rotation maps MuJoCo up to Godot up") {
	const Vec3 up = mj_to_godot({ 0, 0, 1 });     // MuJoCo +z (up)
	CHECK(doctest::Approx(up.x) == 0.0);
	CHECK(doctest::Approx(up.y) == 1.0);          // Godot +y (up)
	CHECK(doctest::Approx(up.z) == 0.0);
	const Vec3 across = mj_to_godot({ 1, 0, 0 }); // x is shared by both frames
	CHECK(doctest::Approx(across.x) == 1.0);
	CHECK(doctest::Approx(across.y) == 0.0);
}

TEST_CASE("[property] Godot->MuJoCo->Godot is the identity") {
	witness::Generator<Vec3> gen = &make_vec3;
	std::function<bool(const Vec3 &)> pred = [](const Vec3 &g) {
		const Vec3 r = mj_to_godot(godot_to_mj(g));
		return std::fabs(r.x - g.x) < 1e-12 &&
				std::fabs(r.y - g.y) < 1e-12 &&
				std::fabs(r.z - g.z) < 1e-12;
	};
	witness::Trial t = witness::resolve<Vec3>("holder round trip", gen, pred);
	CHECK(t.outcome == witness::Outcome::PROVABLY_NONE);
}

TEST_CASE("[falsification] a +90 holder rotation does not round-trip") {
	// If the turn went the other way, up would land on -y and the inverse would
	// no longer undo it. The property above has to be able to see that.
	witness::Generator<Vec3> gen = &make_vec3;
	std::function<bool(const Vec3 &)> pred = [](const Vec3 &g) {
		const Vec3 r = mj_to_godot_wrong(godot_to_mj(g));
		return std::fabs(r.y - g.y) < 1e-12;
	};
	witness::Trial t = witness::resolve<Vec3>("wrong holder turn", gen, pred);
	CHECK(t.outcome == witness::Outcome::FOUND);
}

// ============ UI ray -> MuJoCo swing plane ============

TEST_CASE("[unit] a ray straight down meets the swing plane on it") {
	Vec3 hit;
	REQUIRE(swing_plane({ 0.03, 0.5, 0.1 }, { 0, -1, 0 }, hit));
	CHECK(doctest::Approx(hit.y).epsilon(1e-9) == 0.0);
	CHECK(doctest::Approx(hit.x) == 0.03); // straight down keeps x and z
	CHECK(doctest::Approx(hit.z) == 0.1);
}

TEST_CASE("[property] the intersection always lands on the swing plane") {
	witness::Generator<Vec3> gen = &make_vec3;
	std::function<bool(const Vec3 &)> pred = [](const Vec3 &d) {
		if (std::fabs(d.y) < 1e-6) {
			return true; // rejected rays are handled by the guard test below
		}
		Vec3 hit;
		// A fixed origin above the plane; the direction is what varies.
		if (!swing_plane({ 0.0, 0.4, 0.0 }, d, hit)) {
			return false;
		}
		return std::fabs(hit.y) < 1e-9;
	};
	witness::Trial t = witness::resolve<Vec3>("hit on plane", gen, pred);
	CHECK(t.outcome == witness::Outcome::PROVABLY_NONE);
}

TEST_CASE("[falsification] a ray parallel to the plane is rejected") {
	// The guard host.gd relies on: a pointer whose ray never descends has no
	// point on the swing plane, and must not be treated as if it did.
	Vec3 hit;
	CHECK_FALSE(swing_plane({ 0.0, 0.4, 0.0 }, { 1, 0, 0 }, hit));
}

// ============ Egocentric 3D (the lasso's view frame) ============

TEST_CASE("[unit] a ball straight ahead has zero egocentric angle") {
	const Vec3 eye = { 0, 0, 0 };
	const Vec3 dir = { 0, 0, -1 };
	CHECK(eye_angle(eye, dir, { 0, 0, -2 }) < 1e-9);
}

TEST_CASE("[unit] a ball square to the ray is a right angle from it") {
	const Vec3 eye = { 0, 0, 0 };
	const Vec3 dir = { 0, 0, -1 };
	CHECK(doctest::Approx(eye_angle(eye, dir, { 1, 0, 0 })).epsilon(1e-6) == M_PI / 2.0);
}

TEST_CASE("[property] egocentric angle ignores how far down the ray a ball is") {
	// Depth along the ray must not change the snapping score's angular term,
	// or a near ball would always beat a far one on the same line of sight.
	witness::Generator<double> gen = [](witness::RNG &rng, const witness::Level &) -> double {
		return rng.int_range(100, 3000) / 1000.0;
	};
	std::function<bool(const double &)> pred = [](const double &t) {
		const Vec3 eye = { 0, 0, 0 };
		const Vec3 dir = { 0, 0, -1 };
		return eye_angle(eye, dir, { 0, 0, -t }) < 1e-9;
	};
	witness::Trial trial = witness::resolve<double>("angle vs depth", gen, pred);
	CHECK(trial.outcome == witness::Outcome::PROVABLY_NONE);
}

TEST_CASE("[falsification] angular snapping disagrees with nearest-in-space") {
	// The whole reason the lasso works in the egocentric frame: an on-axis ball
	// further away is the intended target, but it is not the nearest point in
	// space. A raycast-nearest picker would choose the off-axis near ball. If
	// these ever agreed, the egocentric frame would be buying nothing.
	const Vec3 eye = { 0, 0, 0 };
	const Vec3 dir = { 0, 0, -1 };
	const Vec3 on_axis_far = { 0.0, 0.0, -2.0 };
	const Vec3 off_axis_near = { 0.5, 0.0, -0.5 };

	const bool angular_picks_far =
			eye_angle(eye, dir, on_axis_far) < eye_angle(eye, dir, off_axis_near);
	const bool nearest_picks_far =
			v3len(on_axis_far) < v3len(off_axis_near);

	CHECK(angular_picks_far);        // the lasso picks the ball you aimed at
	CHECK_FALSE(nearest_picks_far);  // a raycast-nearest would not
}

// ============ Handedness ============

TEST_CASE("[unit] the holder rotation is a proper rotation, not a reflection") {
	// The rows of the MuJoCo->Godot map (mx,my,mz)->(mx,mz,-my).
	const double d = det3_rows({ 1, 0, 0 }, { 0, 0, 1 }, { 0, -1, 0 });
	CHECK(doctest::Approx(d) == 1.0); // +1 keeps handedness; -1 would mirror
}

TEST_CASE("[falsification] a single-axis flip is a reflection and is caught") {
	// (mx,my,mz)->(mx,my,-mz) negates one axis: determinant -1, a mirror.
	const double d = det3_rows({ 1, 0, 0 }, { 0, 1, 0 }, { 0, 0, -1 });
	CHECK(d < 0.0);
}

// ============ Quaternion convention (MuJoCo wxyz vs Godot xyzw) ============

TEST_CASE("[unit] a MuJoCo wxyz quaternion rotates as expected") {
	// 90 degrees about +z: takes +x to +y in a right-handed frame. MuJoCo
	// stores the quaternion w-first.
	const double s = std::sin(M_PI / 4.0);
	const mjtNum q[4] = { std::cos(M_PI / 4.0), 0, 0, s }; // w, x, y, z
	const mjtNum v[3] = { 1, 0, 0 };
	mjtNum r[3];
	mju_rotVecQuat(r, v, q);
	CHECK(doctest::Approx(r[0]).epsilon(1e-6) == 0.0);
	CHECK(doctest::Approx(r[1]).epsilon(1e-6) == 1.0);
	CHECK(doctest::Approx(r[2]).epsilon(1e-6) == 0.0);
}

TEST_CASE("[falsification] reading the quaternion in the wrong order rotates differently") {
	// The guest emits [w,x,y,z]; host.gd feeds Godot Quaternion(x,y,z,w). If the
	// reorder were skipped, the four numbers would be read as [x,y,z,w] and mean
	// a different rotation. Modelled here as the cyclic mis-read [z,w,x,y].
	const mjtNum q[4] = { 0.6, 0.1, 0.7, std::sqrt(1.0 - 0.36 - 0.01 - 0.49) };
	const mjtNum mis[4] = { q[3], q[0], q[1], q[2] };
	const mjtNum v[3] = { 0.3, -0.5, 0.8 };
	mjtNum a[3], b[3];
	mju_rotVecQuat(a, v, q);
	mju_rotVecQuat(b, v, mis);
	const double diff = std::fabs(a[0] - b[0]) + std::fabs(a[1] - b[1]) + std::fabs(a[2] - b[2]);
	CHECK(diff > 1e-3); // the two orderings are genuinely different rotations
}

// ============ Row-major vs column-major matrix storage ============

TEST_CASE("[unit] mat2quat reads the matrix row-major, as MuJoCo stores it") {
	// A 90-degrees-about-z rotation, written row-major as MuJoCo keeps it.
	const mjtNum R[9] = { 0, -1, 0,
						  1, 0, 0,
						  0, 0, 1 };
	const mjtNum v[3] = { 1, 0, 0 };

	mjtNum by_mat[3];
	mju_mulMatVec(by_mat, R, v, 3, 3); // row-major R * v -> (0,1,0)

	mjtNum q[4], by_quat[3];
	mju_mat2Quat(q, R);
	mju_rotVecQuat(by_quat, v, q);

	// The quaternion from the matrix reproduces the matrix's own action.
	for (int i = 0; i < 3; i++) {
		CHECK(doctest::Approx(by_quat[i]).epsilon(1e-6) == by_mat[i]);
	}
	CHECK(doctest::Approx(by_mat[1]).epsilon(1e-6) == 1.0);
}

TEST_CASE("[falsification] reading the matrix column-major flips the rotation") {
	// The same rotation transposed is what a column-major reader would see.
	// For a non-symmetric rotation the two disagree, so the storage order is
	// not a free choice.
	const mjtNum R[9] = { 0, -1, 0,
						  1, 0, 0,
						  0, 0, 1 };
	mjtNum Rt[9];
	mju_transpose(Rt, R, 3, 3);
	const mjtNum v[3] = { 1, 0, 0 };

	mjtNum row_major[3], col_major[3];
	mju_mulMatVec(row_major, R, v, 3, 3);
	mju_mulMatVec(col_major, Rt, v, 3, 3);

	const double diff = std::fabs(row_major[0] - col_major[0]) +
			std::fabs(row_major[1] - col_major[1]) +
			std::fabs(row_major[2] - col_major[2]);
	CHECK(diff > 1e-3);
}

// ============ MuJoCo hinge frame ============

TEST_CASE("[unit] at rest the ball hangs straight below the pivot") {
	double x, z;
	ball_at(0.0, x, z);
	CHECK(doctest::Approx(x).epsilon(1e-6) == PIVOT_X);
	CHECK(doctest::Approx(z).epsilon(1e-6) == PIVOT_Z - L);
}

TEST_CASE("[unit] a positive hinge angle moves the ball to -x") {
	double x, z;
	ball_at(0.5, x, z);
	CHECK(x < PIVOT_X);
}

TEST_CASE("[property] ball position is (-L sin, pivot_z - L cos) of the angle") {
	witness::Generator<double> gen = &make_angle;
	std::function<bool(const double &)> pred = [](const double &theta) {
		double x, z;
		ball_at(theta, x, z);
		return std::fabs(x - (PIVOT_X - L * std::sin(theta))) < 1e-6 &&
				std::fabs(z - (PIVOT_Z - L * std::cos(theta))) < 1e-6;
	};
	witness::Trial t = witness::resolve<double>("hinge frame", gen, pred);
	CHECK(t.outcome == witness::Outcome::PROVABLY_NONE);
}

TEST_CASE("[property] the string length never changes with the angle") {
	witness::Generator<double> gen = &make_angle;
	std::function<bool(const double &)> pred = [](const double &theta) {
		double x, z;
		ball_at(theta, x, z);
		const double dx = x - PIVOT_X;
		const double dz = z - PIVOT_Z;
		return std::fabs(std::sqrt(dx * dx + dz * dz) - L) < 1e-6;
	};
	witness::Trial t = witness::resolve<double>("string invariant", gen, pred);
	CHECK(t.outcome == witness::Outcome::PROVABLY_NONE);
}

// ============ The whole pointer -> hinge path ============

TEST_CASE("[property] the hold angle lands the ball on the pointer's side") {
	witness::Generator<double> gen = &make_offset;
	std::function<bool(const double &)> pred = [](const double &tx) {
		const double tz = 0.05;
		double x, z;
		ball_at(hold_angle(tx, tz), x, z);
		return sign(x - PIVOT_X) == sign(tx - PIVOT_X);
	};
	witness::Trial t = witness::resolve<double>("drag toward pointer", gen, pred);
	CHECK(t.outcome == witness::Outcome::PROVABLY_NONE);
}

TEST_CASE("[falsification] the flipped hold formula sends the ball the wrong way") {
	// The shipped bug, stated as a test: if this ever passes as PROVABLY_NONE
	// the sign error is undetectable and the property above proves nothing.
	witness::Generator<double> gen = &make_offset;
	std::function<bool(const double &)> pred = [](const double &tx) {
		const double tz = 0.05;
		double x, z;
		ball_at(hold_angle_flipped(tx, tz), x, z);
		return sign(x - PIVOT_X) == sign(tx - PIVOT_X);
	};
	witness::Trial t = witness::resolve<double>("flipped formula", gen, pred);
	CHECK(t.outcome == witness::Outcome::FOUND);
}

// Not covered here, and why: unprojecting a pixel through Camera3D (UI ->
// egocentric), Basis.looking_at building the ray frame (egocentric -> Godot),
// and Transform3D application (Godot <-> MuJoCo) all execute in the engine,
// which is not present in the RISC-V guest. This suite fixes the CONVENTION
// each of those must satisfy -- the -Z-forward view frame, the +1-determinant
// holder rotation, the w-first quaternion, the row-major matrix -- and that
// Godot performs them to match is confirmed by the headless direction check
// and by watching the toy on screen.
