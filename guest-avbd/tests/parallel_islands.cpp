// SPDX-License-Identifier: Apache-2.0 OR MIT

#include <chrono>
#include <cstdint>
#include <cstdio>
#include <thread>
#include <vector>

#include "../avbd_sim.h"

namespace {

constexpr int NUM_PANELS = 8;
constexpr int SUBSTEPS = 200;

ClothMesh panel(int i) {
	return build_cloth_mesh(10 + uint32_t(i % 3), 14, 0.5f, 0.7f, 1.2f, 0.05f);
}

uint64_t solve(const ClothMesh &m) {
	ClothSim sim;
	sim.setup(m);
	sim.run(SUBSTEPS);
	return avbd_digest(sim.pos);
}

} // namespace

int main() {
	int fails = 0;

	std::vector<ClothMesh> panels;
	for (int i = 0; i < NUM_PANELS; ++i) {
		panels.push_back(panel(i));
	}

	auto t0 = std::chrono::steady_clock::now();
	std::vector<uint64_t> standalone(NUM_PANELS);
	for (int i = 0; i < NUM_PANELS; ++i) {
		standalone[i] = solve(panels[i]);
	}
	auto t1 = std::chrono::steady_clock::now();
	const double serialMs = std::chrono::duration<double, std::milli>(t1 - t0).count();

	ClothMesh combined = combine_meshes(panels);
	std::vector<ClothMesh> islands = partition_islands(combined);

	const bool count_ok = int(islands.size()) == NUM_PANELS;
	std::printf("%s  partition recovers every sub-island  [%zu islands, want %d]\n",
			count_ok ? "PASS" : "FAIL", islands.size(), NUM_PANELS);
	fails += count_ok ? 0 : 1;

	std::vector<uint64_t> par(islands.size(), 0);
	std::vector<std::thread> pool;
	auto t2 = std::chrono::steady_clock::now();
	for (size_t i = 0; i < islands.size(); ++i) {
		pool.emplace_back([i, &par, &islands]() { par[i] = solve(islands[i]); });
	}
	for (auto &th : pool) {
		th.join();
	}
	auto t3 = std::chrono::steady_clock::now();
	const double parMs = std::chrono::duration<double, std::milli>(t3 - t2).count();

	int mismatch = 0;
	for (int i = 0; i < NUM_PANELS && i < int(islands.size()); ++i) {
		if (par[i] != standalone[i]) {
			++mismatch;
		}
	}
	const bool exact_ok = count_ok && mismatch == 0;
	std::printf("%s  each island equals its standalone panel  [%d/%d match]\n",
			exact_ok ? "PASS" : "FAIL", NUM_PANELS - mismatch, NUM_PANELS);
	fails += exact_ok ? 0 : 1;

	const double speedup = serialMs / parMs;
	const unsigned hw = std::thread::hardware_concurrency();
	const bool multicore = speedup > 1.5;
	std::printf("%s  multi-core speedup %.2fx  [%d islands on %u cores, %.0f ms serial / %.0f ms parallel]\n",
			multicore ? "PASS" : "FAIL", speedup, NUM_PANELS, hw, serialMs, parMs);
	fails += multicore ? 0 : 1;

	std::printf("%s\n", fails == 0 ? "all island checks OK" : "ISLANDS FAILED");
	return fails == 0 ? 0 : 1;
}
