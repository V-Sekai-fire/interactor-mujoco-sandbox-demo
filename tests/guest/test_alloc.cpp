#include "witness/doctest.h"

#include <cstdint>
#include <cstdlib>
#include <functional>

// MuJoCo sizes its model buffer from an offset starting at zero but assigns the
// pointers inside it by absolute address, both rounded to 64 bytes. The two
// agree only when the buffer starts 64-byte aligned, and the guest's malloc is
// wrapped to the sandbox arena, which does not promise that.

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

size_t make_size(witness::RNG &rng, const witness::Level &lvl) {
	const uint32_t bound = lvl.fin_bound < 4096 ? lvl.fin_bound : 4096;
	return (size_t)rng.uint_range(1, bound + 1);
}

} // namespace

TEST_CASE("[unit] the aligned allocator returns 64-byte aligned memory") {
	void *p = aligned64_malloc(2176);
	REQUIRE(p != nullptr);
	CHECK(((uintptr_t)p & 63u) == 0u);
	aligned64_free(p);
}

TEST_CASE("[unit] an allocation is writable across its whole length") {
	const size_t n = 2224;
	unsigned char *p = (unsigned char *)aligned64_malloc(n);
	REQUIRE(p != nullptr);
	for (size_t i = 0; i < n; i++) {
		p[i] = (unsigned char)(i & 0xFF);
	}
	bool intact = true;
	for (size_t i = 0; i < n; i++) {
		if (p[i] != (unsigned char)(i & 0xFF)) {
			intact = false;
		}
	}
	CHECK(intact);
	aligned64_free(p);
}

TEST_CASE("[property] every aligned allocation is 64-byte aligned") {
	witness::Generator<size_t> gen = &make_size;
	std::function<bool(const size_t &)> pred = [](const size_t &n) {
		void *p = aligned64_malloc(n);
		if (p == nullptr) {
			return false;
		}
		const bool ok = ((uintptr_t)p & 63u) == 0u;
		aligned64_free(p);
		return ok;
	};
	witness::Trial t = witness::resolve<size_t>("64-byte aligned", gen, pred);
	CHECK(t.outcome != witness::Outcome::FOUND);
}

// The control that makes the property above mean something: plain malloc is not
// reliably 64-byte aligned, so asserting that it is must be falsified. If this
// ever reports PROVABLY_NONE the alignment property is passing for free.
TEST_CASE("[falsification] plain malloc is not 64-byte aligned") {
	witness::Generator<size_t> gen = &make_size;
	std::function<bool(const size_t &)> pred = [](const size_t &n) {
		void *p = malloc(n);
		if (p == nullptr) {
			return true;
		}
		const bool aligned = ((uintptr_t)p & 63u) == 0u;
		free(p);
		return aligned;
	};
	witness::Trial t = witness::resolve<size_t>("plain malloc is aligned", gen, pred);
	CHECK(t.outcome == witness::Outcome::FOUND);
}
