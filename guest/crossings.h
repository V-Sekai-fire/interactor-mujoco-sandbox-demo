// Stroke crossings by direct segment geometry.
//
// A stroke is a polyline. Two strokes cross where a segment of one passes within `proximity`
// of a segment of the other: the same test as two capsules of radius proximity/2 touching,
// which is what the MuJoCo version modelled, without building and compiling a model. The
// crossing point is the midpoint of the two segments' closest points. Nearby hits of ONE
// stroke pair (a tessellated stroke touches with several segment pairs) coalesce: the closest
// is kept and any later hit of that pair within merge_epsilon of a kept point is dropped. Hits
// of different pairs never merge: where three strokes meet near one point each pair keeps its
// own crossing, so a stroke is never left without the split it needs.
//
// The finder is incremental. It keeps the strokes it has seen; when the next call's strokes
// begin with the same strokes, only the new ones are tested, against everything. A call
// returns the crossings that involve the strokes new to it (every stroke, for a call that does
// not extend the last one), which is what a pen committing one stroke needs.
//
// Header only and free of the sandbox API, so a host test can include it.
#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <vector>

namespace crossings {

struct Box {
	double lo[3], hi[3];
};

inline bool boxes_touch(const Box &a, const Box &b, double pad) {
	for (int k = 0; k < 3; k++) {
		if (a.lo[k] - pad > b.hi[k] || b.lo[k] - pad > a.hi[k]) {
			return false;
		}
	}
	return true;
}

inline Box segment_box(const float *a, const float *b) {
	Box x;
	for (int k = 0; k < 3; k++) {
		x.lo[k] = std::min((double)a[k], (double)b[k]);
		x.hi[k] = std::max((double)a[k], (double)b[k]);
	}
	return x;
}

// Closest points of segments p1-q1 and p2-q2 (Ericson, Real-Time Collision Detection, 5.1.9).
// Zero-length segments are points, so a stroke with repeated samples needs no clean-up.
// Returns the squared distance; c1 and c2 receive the closest points.
inline double segment_closest(const float *p1f, const float *q1f, const float *p2f, const float *q2f,
		double c1[3], double c2[3]) {
	double p1[3], q1[3], p2[3], q2[3], d1[3], d2[3], r[3];
	for (int k = 0; k < 3; k++) {
		p1[k] = p1f[k];
		q1[k] = q1f[k];
		p2[k] = p2f[k];
		q2[k] = q2f[k];
		d1[k] = q1[k] - p1[k];
		d2[k] = q2[k] - p2[k];
		r[k] = p1[k] - p2[k];
	}
	auto dot = [](const double *u, const double *v) { return u[0] * v[0] + u[1] * v[1] + u[2] * v[2]; };
	const double a = dot(d1, d1), e = dot(d2, d2), f = dot(d2, r);
	const double eps = 1e-18;
	double s, t;
	if (a <= eps && e <= eps) {
		s = t = 0.0;
	} else if (a <= eps) {
		s = 0.0;
		t = std::clamp(f / e, 0.0, 1.0);
	} else {
		const double c = dot(d1, r);
		if (e <= eps) {
			t = 0.0;
			s = std::clamp(-c / a, 0.0, 1.0);
		} else {
			const double b = dot(d1, d2);
			const double denom = a * e - b * b;
			s = denom > eps ? std::clamp((b * f - c * e) / denom, 0.0, 1.0) : 0.0;
			t = (b * s + f) / e;
			if (t < 0.0) {
				t = 0.0;
				s = std::clamp(-c / a, 0.0, 1.0);
			} else if (t > 1.0) {
				t = 1.0;
				s = std::clamp((b - c) / a, 0.0, 1.0);
			}
		}
	}
	double d2sum = 0.0;
	for (int k = 0; k < 3; k++) {
		c1[k] = p1[k] + d1[k] * s;
		c2[k] = p2[k] + d2[k] * t;
		const double dk = c1[k] - c2[k];
		d2sum += dk * dk;
	}
	return d2sum;
}

class Finder {
public:
	// points: x,y,z per sample; counts: samples per stroke. Returns coalesced crossings as
	// x,y,z triples, the same set whichever strokes were already seen.
	std::vector<double> find(const std::vector<float> &points, const std::vector<int32_t> &counts, double proximity) {
		size_t keep = reusable(points, counts, proximity);
		if (keep == 0) {
			reset(proximity);
		} else {
			// Strokes past `keep` changed: forget them and the crossings found with them.
			if (keep < counts_.size()) {
				reset(proximity);
				keep = 0;
			}
		}
		const double prox2 = proximity * proximity;
		const double merge = proximity * 3.0 > 0.03 ? proximity * 3.0 : 0.03; // within one stroke pair
		const double merge2 = merge * merge;
		size_t off = points_.size() / 3;
		std::vector<double> fresh;
		for (size_t si = counts_.size(); si < counts.size(); si++) {
			const int n = counts[si] > 0 ? counts[si] : 0;
			Stroke s;
			s.first = off;
			s.n = n;
			s.box = {{1e300, 1e300, 1e300}, {-1e300, -1e300, -1e300}};
			for (int i = 0; i < n; i++) {
				for (int k = 0; k < 3; k++) {
					const double v = points[(off + (size_t)i) * 3 + k];
					s.box.lo[k] = std::min(s.box.lo[k], v);
					s.box.hi[k] = std::max(s.box.hi[k], v);
				}
			}
			points_.insert(points_.end(), points.begin() + (long)off * 3, points.begin() + (long)(off + (size_t)n) * 3);
			// Hits of the new stroke with every earlier stroke, closest first, so a cluster
			// keeps its best point whatever order the segments were met in.
			std::vector<Hit> hits;
			for (size_t oi = 0; oi < strokes_.size(); oi++) {
				const Stroke &o = strokes_[oi];
				if (o.n < 2 || s.n < 2 || !boxes_touch(s.box, o.box, proximity)) {
					continue;
				}
				for (int i = 0; i + 1 < s.n; i++) {
					const float *a0 = &points_[(s.first + (size_t)i) * 3];
					const float *a1 = a0 + 3;
					const Box sb = segment_box(a0, a1);
					if (!boxes_touch(sb, o.box, proximity)) {
						continue;
					}
					for (int j = 0; j + 1 < o.n; j++) {
						const float *b0 = &points_[(o.first + (size_t)j) * 3];
						const float *b1 = b0 + 3;
						if (!boxes_touch(sb, segment_box(b0, b1), proximity)) {
							continue;
						}
						double c1[3], c2[3];
						const double d2 = segment_closest(a0, a1, b0, b1, c1, c2);
						if (d2 < prox2) {
							hits.push_back({d2, oi, {(c1[0] + c2[0]) * 0.5, (c1[1] + c2[1]) * 0.5, (c1[2] + c2[2]) * 0.5}});
						}
					}
				}
			}
			std::stable_sort(hits.begin(), hits.end(), [](const Hit &x, const Hit &y) { return x.d2 < y.d2; });
			std::vector<std::pair<size_t, size_t>> kept; // (other stroke, index into fresh)
			for (const Hit &h : hits) {
				bool merged = false;
				for (const auto &k : kept) {
					if (k.first != h.other) {
						continue;
					}
					const double *q = &fresh[k.second];
					const double dx = h.p[0] - q[0], dy = h.p[1] - q[1], dz = h.p[2] - q[2];
					if (dx * dx + dy * dy + dz * dz < merge2) {
						merged = true;
						break;
					}
				}
				if (!merged) {
					kept.push_back({h.other, fresh.size()});
					fresh.insert(fresh.end(), h.p, h.p + 3);
				}
			}
			strokes_.push_back(s);
			counts_.push_back(counts[si]);
			off += (size_t)n;
		}
		return fresh;
	}

	size_t strokes_tested_last() const { return tested_; }

private:
	struct Stroke {
		size_t first;
		int n;
		Box box;
	};
	struct Hit {
		double d2;
		size_t other;
		double p[3];
	};

	// How many leading strokes of this call equal the ones already seen.
	size_t reusable(const std::vector<float> &points, const std::vector<int32_t> &counts, double proximity) {
		tested_ = counts.size();
		if (proximity != proximity_ || counts.size() < counts_.size()) {
			return 0;
		}
		if (!std::equal(counts_.begin(), counts_.end(), counts.begin())) {
			return 0;
		}
		if (points.size() < points_.size() || std::memcmp(points.data(), points_.data(), points_.size() * sizeof(float)) != 0) {
			return 0;
		}
		tested_ = counts.size() - counts_.size();
		return counts_.size();
	}

	void reset(double proximity) {
		proximity_ = proximity;
		points_.clear();
		counts_.clear();
		strokes_.clear();
	}

	double proximity_ = -1.0;
	std::vector<float> points_;
	std::vector<int32_t> counts_;
	std::vector<Stroke> strokes_;
	size_t tested_ = 0;
};

} // namespace crossings
