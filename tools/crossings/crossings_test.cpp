// Host test for guest/crossings.h: build with any C++17 compiler and run.
//   c++ -std=c++17 -O2 -I guest tools/crossings/crossings_test.cpp -o /tmp/ct && /tmp/ct
// Each check prints PASS or FAIL; the planted controls must change the answer.
#include "crossings.h"

#include <array>
#include <cmath>
#include <cstdio>
#include <random>
#include <vector>

using Strokes = std::vector<std::vector<float>>;

static std::vector<float> seg(float ax, float ay, float az, float bx, float by, float bz, int n) {
	std::vector<float> p;
	for (int i = 0; i <= n; i++) {
		const float t = float(i) / float(n);
		p.insert(p.end(), {ax + (bx - ax) * t, ay + (by - ay) * t, az + (bz - az) * t});
	}
	return p;
}

static void flat(const Strokes &s, std::vector<float> &pts, std::vector<int32_t> &cnt) {
	pts.clear();
	cnt.clear();
	for (const auto &x : s) {
		pts.insert(pts.end(), x.begin(), x.end());
		cnt.push_back((int32_t)(x.size() / 3));
	}
}

static std::vector<double> fresh(const Strokes &s, double prox) {
	crossings::Finder f;
	std::vector<float> p;
	std::vector<int32_t> c;
	flat(s, p, c);
	return f.find(p, c, prox);
}

// Brute force: every segment pair of different strokes, no pruning, same coalescing rule.
static std::vector<double> brute(const Strokes &s, double prox) {
	std::vector<double> out;
	const double merge = prox * 3.0 > 0.03 ? prox * 3.0 : 0.03;
	for (size_t si = 1; si < s.size(); si++) {
		struct H { double d2, p[3]; };
		std::vector<H> hits;
		for (size_t oi = 0; oi < si; oi++) {
			for (size_t i = 0; i + 1 < s[si].size() / 3; i++) {
				for (size_t j = 0; j + 1 < s[oi].size() / 3; j++) {
					double c1[3], c2[3];
					double d2 = crossings::segment_closest(&s[si][i * 3], &s[si][i * 3 + 3], &s[oi][j * 3], &s[oi][j * 3 + 3], c1, c2);
					if (d2 < prox * prox) {
						hits.push_back({d2, {(c1[0] + c2[0]) / 2, (c1[1] + c2[1]) / 2, (c1[2] + c2[2]) / 2}});
					}
				}
			}
		}
		std::stable_sort(hits.begin(), hits.end(), [](const H &a, const H &b) { return a.d2 < b.d2; });
		for (const H &h : hits) {
			bool m = false;
			for (size_t k = 0; k + 2 < out.size(); k += 3) {
				double dx = h.p[0] - out[k], dy = h.p[1] - out[k + 1], dz = h.p[2] - out[k + 2];
				if (dx * dx + dy * dy + dz * dz < merge * merge) { m = true; break; }
			}
			if (!m) out.insert(out.end(), h.p, h.p + 3);
		}
	}
	return out;
}

static bool near_all(const std::vector<double> &got, const std::vector<std::array<double, 3>> &want, double tol) {
	if (got.size() != want.size() * 3) return false;
	for (const auto &w : want) {
		bool hit = false;
		for (size_t k = 0; k + 2 < got.size(); k += 3) {
			if (std::hypot(got[k] - w[0], got[k + 1] - w[1], got[k + 2] - w[2]) < tol) hit = true;
		}
		if (!hit) return false;
	}
	return true;
}

int main() {
	int bad = 0;
	auto check = [&](const char *name, bool ok) {
		std::printf("  %s  %s\n", ok ? "PASS" : "FAIL", name);
		bad += !ok;
	};
	// CASSIE crossing_split: three overshooting strokes, three analytic crossings.
	Strokes cassie = {seg(-1.2f, 0, 0, 1.2f, 0, 0, 16), seg(1.1f, -0.3f, 0, -0.2f, 1.8f, 0, 16), seg(0.2f, 1.8f, 0, -1.1f, -0.3f, 0, 16)};
	check("three crossings at the analytic points", near_all(fresh(cassie, 0.02), {{{0.9143, 0, 0}}, {{-0.9143, 0, 0}}, {{0, 1.4769, 0}}}, 0.01));
	Strokes far = cassie;
	far[2] = seg(5.2f, 1.8f, 0, 3.9f, -0.3f, 0, 16);
	check("control: a stroke moved away drops its two crossings", fresh(far, 0.02).size() == 3);

	// Repeated samples (zero-length segments) made MuJoCo refuse the model on every dress stroke.
	Strokes dup = cassie;
	for (auto &s : dup) s.insert(s.begin() + 3, s.begin(), s.begin() + 3);
	check("repeated samples still give the three crossings", near_all(fresh(dup, 0.02), {{{0.9143, 0, 0}}, {{-0.9143, 0, 0}}, {{0, 1.4769, 0}}}, 0.01));

	// Random strokes: pruned and incremental results equal the unpruned brute force.
	std::mt19937 rng(7);
	std::uniform_real_distribution<float> u(-0.5f, 0.5f);
	Strokes rnd;
	for (int k = 0; k < 40; k++) {
		std::vector<float> s;
		float x = u(rng), y = u(rng), z = u(rng);
		for (int i = 0; i < 30; i++) {
			s.insert(s.end(), {x, y, z});
			x += u(rng) * 0.06f; y += u(rng) * 0.06f; z += u(rng) * 0.02f;
		}
		rnd.push_back(s);
	}
	const auto want = brute(rnd, 0.02);
	check("random strokes: pruned equals brute force exactly", fresh(rnd, 0.02) == want && !want.empty());

	crossings::Finder inc;
	std::vector<double> got;
	size_t tested = 0;
	for (size_t k = 1; k <= rnd.size(); k++) {
		std::vector<float> p;
		std::vector<int32_t> c;
		flat(Strokes(rnd.begin(), rnd.begin() + k), p, c);
		got = inc.find(p, c, 0.02);
		tested = inc.strokes_tested_last();
	}
	check("incremental, one stroke per call, equals one call", got == want);
	check("incremental: the last call tested only the newest stroke", tested == 1);

	// An earlier stroke changes: the finder must start over, not reuse stale crossings.
	Strokes moved = rnd;
	for (size_t i = 0; i < moved[0].size(); i += 3) moved[0][i] += 3.0f;
	{
		std::vector<float> p;
		std::vector<int32_t> c;
		flat(moved, p, c);
		auto r = inc.find(p, c, 0.02);
		check("a changed earlier stroke resets the finder", r == brute(moved, 0.02) && inc.strokes_tested_last() == moved.size());
	}
	// A different proximity also starts over.
	{
		std::vector<float> p;
		std::vector<int32_t> c;
		flat(moved, p, c);
		auto r = inc.find(p, c, 0.05);
		check("a new proximity resets the finder", r == brute(moved, 0.05));
	}
	std::printf("RESULT: %s (%d failed)\n", bad ? "FAIL" : "PASS", bad);
	return bad ? 1 : 0;
}
