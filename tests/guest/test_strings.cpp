#include "witness/doctest.h"

#include <cstring>
#include <functional>
#include <string>

// The guest links with -Wl,--wrap on strlen, strcmp, strncmp and memcmp, so
// these are the sandbox's implementations rather than glibc's. tinyxml2 sizes
// its copy of a document with strlen and finds an attribute's closing quote
// with strncmp; a wrong answer from either truncates or mis-scans a model
// while every hand-picked check still passes.

namespace {

// What strlen should return, counted without calling it.
size_t hand_count(const char *s) {
	size_t n = 0;
	while (s[n] != 0) {
		n++;
	}
	return n;
}

// Strings over the characters MJCF actually contains, including the quote and
// the angle brackets that attribute scanning stops on.
std::string make_xmlish(witness::RNG &rng, const witness::Level &lvl) {
	static const char charset[] = "ab01 \t\n<>=/\"'.-_";
	const uint32_t bound = lvl.fin_bound < 64 ? lvl.fin_bound : 64;
	const uint32_t n = rng.uint_range(0, bound);
	std::string s;
	s.reserve(n);
	for (uint32_t i = 0; i < n; i++) {
		s.push_back(charset[rng.uint_range(0, sizeof(charset) - 2)]);
	}
	return s;
}

} // namespace

TEST_CASE("[unit] strlen agrees with a hand count on fixed strings") {
	CHECK(strlen("") == 0u);
	CHECK(strlen("<a/>") == 4u);
	CHECK(strlen("<a b=\"c\"/>") == 10u);
}

TEST_CASE("[property] strlen agrees with a hand count") {
	witness::Generator<std::string> gen = &make_xmlish;
	std::function<bool(const std::string &)> pred = [](const std::string &s) {
		return strlen(s.c_str()) == hand_count(s.c_str());
	};
	witness::Trial t = witness::resolve<std::string>("strlen == hand count", gen, pred);
	CHECK(t.outcome != witness::Outcome::FOUND);
}

TEST_CASE("[property] a string compares equal to itself") {
	witness::Generator<std::string> gen = &make_xmlish;
	std::function<bool(const std::string &)> pred = [](const std::string &s) {
		const char *p = s.c_str();
		const size_t n = strlen(p);
		return strncmp(p, p, n) == 0 && memcmp(p, p, n) == 0 && strcmp(p, p) == 0;
	};
	witness::Trial t = witness::resolve<std::string>("self comparison", gen, pred);
	CHECK(t.outcome != witness::Outcome::FOUND);
}

// tinyxml2 looks for a closing quote with strncmp against a one-character tag.
// Scanning a string for a byte must agree with a plain loop.
TEST_CASE("[property] scanning for a quote by strncmp matches a plain loop") {
	witness::Generator<std::string> gen = &make_xmlish;
	std::function<bool(const std::string &)> pred = [](const std::string &s) {
		const char endTag[2] = { '"', 0 };
		const char *p = s.c_str();
		size_t by_strncmp = SIZE_MAX;
		for (size_t i = 0; p[i] != 0; i++) {
			if (p[i] == endTag[0] && strncmp(p + i, endTag, 1) == 0) {
				by_strncmp = i;
				break;
			}
		}
		size_t by_loop = SIZE_MAX;
		for (size_t i = 0; p[i] != 0; i++) {
			if (p[i] == '"') {
				by_loop = i;
				break;
			}
		}
		return by_strncmp == by_loop;
	};
	witness::Trial t = witness::resolve<std::string>("quote scan agrees", gen, pred);
	CHECK(t.outcome != witness::Outcome::FOUND);
}

// Negative controls. Without these the suite cannot show it is able to fail,
// and a property that always passes is indistinguishable from one that cannot
// be violated.
TEST_CASE("[falsification] a wrong strlen law is caught") {
	witness::Generator<std::string> gen = &make_xmlish;
	std::function<bool(const std::string &)> pred = [](const std::string &s) {
		return strlen(s.c_str()) == hand_count(s.c_str()) + 1;
	};
	witness::Trial t = witness::resolve<std::string>("strlen is one too many", gen, pred);
	CHECK(t.outcome == witness::Outcome::FOUND);
}

TEST_CASE("[falsification] claiming every string contains a quote is caught") {
	witness::Generator<std::string> gen = &make_xmlish;
	std::function<bool(const std::string &)> pred = [](const std::string &s) {
		return strchr(s.c_str(), '"') != nullptr;
	};
	witness::Trial t = witness::resolve<std::string>("every string has a quote", gen, pred);
	CHECK(t.outcome == witness::Outcome::FOUND);
}

// XMLDocument::Parse copies the document with memcpy before parsing it, and
// memcpy is wrapped here too. A copy that stops early leaves a buffer that
// terminates mid-attribute, which is what a failed quote scan looks like.
TEST_CASE("[property] memcpy reproduces the source exactly") {
	witness::Generator<std::string> gen = &make_xmlish;
	std::function<bool(const std::string &)> pred = [](const std::string &s) {
		const size_t n = s.size();
		std::string dst(n + 1, '\0');
		memcpy(&dst[0], s.data(), n);
		dst[n] = 0;
		if (strlen(dst.c_str()) != hand_count(s.c_str())) {
			return false;
		}
		for (size_t i = 0; i < n; i++) {
			if (dst[i] != s[i]) {
				return false;
			}
		}
		return true;
	};
	witness::Trial t = witness::resolve<std::string>("memcpy is faithful", gen, pred);
	CHECK(t.outcome != witness::Outcome::FOUND);
}

TEST_CASE("[property] memmove and memset behave") {
	witness::Generator<std::string> gen = &make_xmlish;
	std::function<bool(const std::string &)> pred = [](const std::string &s) {
		const size_t n = s.size();
		std::string buf(n + 8, 'Z');
		memcpy(&buf[0], s.data(), n);
		memmove(&buf[0] + 4, &buf[0], n);
		for (size_t i = 0; i < n; i++) {
			if (buf[i + 4] != s[i]) {
				return false;
			}
		}
		std::string zeroed(n + 1, 'Q');
		memset(&zeroed[0], 0, n + 1);
		return strlen(zeroed.c_str()) == 0;
	};
	witness::Trial t = witness::resolve<std::string>("memmove and memset", gen, pred);
	CHECK(t.outcome != witness::Outcome::FOUND);
}

// new char[n] is how tinyxml2 gets the buffer it copies into. A short or
// unwritable allocation truncates the document exactly as a bad copy would.
TEST_CASE("[property] a heap buffer holds what was written across its length") {
	witness::Generator<std::string> gen = &make_xmlish;
	std::function<bool(const std::string &)> pred = [](const std::string &s) {
		const size_t n = s.size();
		char *buf = new char[n + 1];
		memcpy(buf, s.data(), n);
		buf[n] = 0;
		const bool ok = strlen(buf) == hand_count(s.c_str());
		delete[] buf;
		return ok;
	};
	witness::Trial t = witness::resolve<std::string>("heap buffer round trip", gen, pred);
	CHECK(t.outcome != witness::Outcome::FOUND);
}
