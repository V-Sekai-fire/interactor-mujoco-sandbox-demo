#include "witness/doctest.h"

#include <mujoco/mujoco.h>

#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <functional>
#include <string>

// MJCF loading, through MuJoCo's own API rather than tinyxml2 directly, so the
// test needs nothing the package does not already ship.

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

// Model construction needs the aligned allocator in place before any test runs.
AlignedAllocator g_installed;

// Returns true when the XML builds a model, and fills err otherwise.
bool loads(const std::string &xml, char *err, int errlen) {
	mjVFS vfs;
	mj_defaultVFS(&vfs);
	if (mj_addBufferVFS(&vfs, "m.xml", xml.data(), (int)xml.size()) != 0) {
		mj_deleteVFS(&vfs);
		return false;
	}
	mjModel *m = mj_loadXML("m.xml", &vfs, err, errlen);
	mj_deleteVFS(&vfs);
	if (m == nullptr) {
		return false;
	}
	mj_deleteModel(m);
	return true;
}

// Attribute values MJCF accepts: a body name is a plain identifier.
std::string make_name(witness::RNG &rng, const witness::Level &lvl) {
	static const char charset[] = "abcdefghijklmnopqrstuvwxyz0123456789_";
	const uint32_t bound = lvl.fin_bound < 12 ? lvl.fin_bound : 12;
	const uint32_t n = rng.uint_range(1, bound + 1);
	std::string s;
	for (uint32_t i = 0; i < n; i++) {
		s.push_back(charset[rng.uint_range(0, sizeof(charset) - 1)]);
	}
	// A leading digit is not a valid identifier; keep the generator on-model.
	if (s[0] >= '0' && s[0] <= '9') {
		s[0] = 'a';
	}
	return s;
}

} // namespace

TEST_CASE("[unit] a model with no attributes builds") {
	char err[256] = { 0 };
	CHECK(loads("<mujoco/>", err, (int)sizeof(err)));
}

TEST_CASE("[unit] a model with one attribute builds") {
	char err[256] = { 0 };
	const bool ok = loads("<mujoco model=\"m\"/>", err, (int)sizeof(err));
	INFO("mj_loadXML said: ", err);
	CHECK(ok);
}

TEST_CASE("[property] any valid body name loads") {
	witness::Generator<std::string> gen = &make_name;
	std::function<bool(const std::string &)> pred = [](const std::string &name) {
		char err[256] = { 0 };
		const std::string xml =
				"<mujoco><worldbody><body name=\"" + name +
				"\"><geom type=\"sphere\" size=\"0.1\"/></body></worldbody></mujoco>";
		return loads(xml, err, (int)sizeof(err));
	};
	witness::Trial t = witness::resolve<std::string>("named body loads", gen, pred);
	CHECK(t.outcome != witness::Outcome::FOUND);
}

// Without this the loading property could pass because everything loads,
// including nonsense, which would say nothing about the parser.
TEST_CASE("[falsification] not every string is a valid model") {
	witness::Generator<std::string> gen = &make_name;
	std::function<bool(const std::string &)> pred = [](const std::string &s) {
		char err[256] = { 0 };
		return loads(s, err, (int)sizeof(err));
	};
	witness::Trial t = witness::resolve<std::string>("every string is a model", gen, pred);
	CHECK(t.outcome == witness::Outcome::FOUND);
}
