#include "witness/doctest.h"

#include <cstring>
#include <functional>
#include <string>

// The guest links with -Wl,--wrap on the string functions, so calls reach the
// sandbox's implementations. Calling them with compile-time constants does not
// test those: clang folds the comparison and answers correctly without ever
// reaching the wrapper. Everything here goes through a function pointer so the
// call survives optimisation, which is the difference between testing the
// sandbox and testing the compiler.

namespace {

// volatile, not const: clang propagates through a const function pointer and
// inlines the callee, which answers correctly without ever reaching the wrapped
// symbol. A volatile load forces a real indirect call.
int (*volatile opaque_strncmp)(const char *, const char *, size_t) = &strncmp;
int (*volatile opaque_memcmp)(const void *, const void *, size_t) = &memcmp;
size_t (*volatile opaque_strlen)(const char *) = &strlen;

struct Prefix {
	std::string haystack;
	size_t n;
};

// A string, a position inside it, and a length to compare. The interesting
// case is a match that is only a match for the first n bytes.
Prefix make_prefix(witness::RNG &rng, const witness::Level &lvl) {
	static const char charset[] = "ab\"<>/= ";
	const uint32_t bound = lvl.fin_bound < 24 ? lvl.fin_bound : 24;
	const uint32_t len = rng.uint_range(1, bound + 2);
	Prefix pf;
	for (uint32_t i = 0; i < len; i++) {
		// uint_range is inclusive and sizeof counts the terminator; sizeof - 1
		// would reach the '\0' and put a stray null in the string.
		pf.haystack.push_back(charset[rng.uint_range(0, sizeof(charset) - 2)]);
	}
	pf.n = (size_t)rng.uint_range(1, (uint32_t)pf.haystack.size() + 1);
	return pf;
}

// What strncmp must agree with, written out so the law is visible.
int reference_strncmp(const char *a, const char *b, size_t n) {
	for (size_t i = 0; i < n; i++) {
		const unsigned char ca = (unsigned char)a[i];
		const unsigned char cb = (unsigned char)b[i];
		if (ca != cb) {
			return ca < cb ? -1 : 1;
		}
		if (ca == 0) {
			return 0;
		}
	}
	return 0;
}

} // namespace

// The exact shape tinyxml2 uses to find an attribute's closing quote: compare
// one byte of a longer string against a one-character tag.
TEST_CASE("[unit] strncmp compares only the first n bytes") {
	const char haystack[] = "\"/>";
	const char endTag[2] = { '"', 0 };
	CHECK(opaque_strncmp(haystack, endTag, 1) == 0);
}

TEST_CASE("[unit] strncmp stops at n even when the strings diverge after it") {
	const char a[] = "abcXXXX";
	const char b[] = "abcYYYY";
	CHECK(opaque_strncmp(a, b, 3) == 0);
	CHECK(opaque_strncmp(a, b, 4) != 0);
}

TEST_CASE("[property] strncmp agrees with a byte-by-byte reference") {
	witness::Generator<Prefix> gen = &make_prefix;
	std::function<bool(const Prefix &)> pred = [](const Prefix &pf) {
		const char *h = pf.haystack.c_str();
		for (size_t i = 0; i < pf.haystack.size(); i++) {
			const char tag[2] = { h[i], 0 };
			const int got = opaque_strncmp(h + i, tag, 1);
			const int want = reference_strncmp(h + i, tag, 1);
			if ((got == 0) != (want == 0)) {
				return false;
			}
		}
		return true;
	};
	witness::Trial t = witness::resolve<Prefix>("strncmp matches reference", gen, pred);
	// BUDGET_HIT is not a pass: it means the space was never covered. Rule 3 --
	// a silent skip reads exactly like a pass.
	CHECK(t.outcome == witness::Outcome::PROVABLY_NONE);
}

TEST_CASE("[property] memcmp agrees with a byte-by-byte reference") {
	witness::Generator<Prefix> gen = &make_prefix;
	std::function<bool(const Prefix &)> pred = [](const Prefix &pf) {
		const char *h = pf.haystack.c_str();
		for (size_t i = 0; i < pf.haystack.size(); i++) {
			const char tag[2] = { h[i], 0 };
			if ((opaque_memcmp(h + i, tag, 1) == 0) != true) {
				return false;
			}
		}
		return true;
	};
	witness::Trial t = witness::resolve<Prefix>("memcmp matches reference", gen, pred);
	CHECK(t.outcome == witness::Outcome::PROVABLY_NONE);
}

TEST_CASE("[property] strlen through a function pointer is still correct") {
	witness::Generator<Prefix> gen = &make_prefix;
	std::function<bool(const Prefix &)> pred = [](const Prefix &pf) {
		size_t n = 0;
		while (pf.haystack.c_str()[n] != 0) {
			n++;
		}
		return opaque_strlen(pf.haystack.c_str()) == n;
	};
	witness::Trial t = witness::resolve<Prefix>("strlen uninlined", gen, pred);
	CHECK(t.outcome == witness::Outcome::PROVABLY_NONE);
}

// Without this the properties above could pass because the reference is wrong
// in the same way the implementation is.
TEST_CASE("[falsification] a reference that ignores n is caught") {
	witness::Generator<Prefix> gen = &make_prefix;
	std::function<bool(const Prefix &)> pred = [](const Prefix &pf) {
		const char *h = pf.haystack.c_str();
		for (size_t i = 0; i < pf.haystack.size(); i++) {
			const char tag[2] = { h[i], 0 };
			// Claim: comparing one byte is the same as comparing the whole
			// string. False for any match that is not at the end.
			if ((reference_strncmp(h + i, tag, 1) == 0) != (strcmp(h + i, tag) == 0)) {
				return false;
			}
		}
		return true;
	};
	witness::Trial t = witness::resolve<Prefix>("one byte equals whole string", gen, pred);
	CHECK(t.outcome == witness::Outcome::FOUND);
}
