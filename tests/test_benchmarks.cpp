#include "doctest.h"
#include "instance_io.h"

#include <bgspprc/solver.h>
#include <bgspprc/resource.h>

#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <map>
#include <string>

using namespace bgspprc;
namespace fs = std::filesystem;

// ── Helpers ──

static std::map<std::string, double> load_optimal_csv(const std::string& path) {
    std::map<std::string, double> result;
    std::ifstream file(path);
    if (!file.is_open()) return result;

    std::string line;
    while (std::getline(file, line)) {
        if (line.empty() || line[0] == '#') continue;
        if (line.find("instance") != std::string::npos) continue;

        auto comma = line.find(',');
        if (comma == std::string::npos) continue;
        result[line.substr(0, comma)] = std::stod(line.substr(comma + 1));
    }
    return result;
}

// ── SPPCC (spprclib) benchmarks ──

static const char* SPPRCLIB_DIR =
    "../rcspp-bac/benchmarks/instances/spprclib";
static const char* SPPRCLIB_OPTIMAL =
    "../rcspp-bac/benchmarks/instances/spprclib/optimal.csv";

TEST_CASE("spprclib instances") {
    if (!fs::exists(SPPRCLIB_DIR)) {
        MESSAGE("Skipping spprclib tests: directory not found");
        return;
    }

    auto optima = load_optimal_csv(SPPRCLIB_OPTIMAL);
    if (optima.empty()) {
        MESSAGE("Skipping: no optimal.csv found");
        return;
    }

    int tested = 0, passed = 0, failed = 0;

    for (auto& entry : fs::directory_iterator(SPPRCLIB_DIR)) {
        if (entry.path().extension() != ".sppcc") continue;
        std::string name = entry.path().stem().string();

        auto it = optima.find(name);
        if (it == optima.end()) continue;

        double expected = it->second;
        if (expected >= -3.0) continue;  // skip trivial/unsolved

        auto inst = io::load_sppcc(entry.path().string());
        if (inst.n_vertices > 55) continue;  // skip large

        auto pv = inst.problem_view();

        Solver<EmptyPack> solver(pv, EmptyPack{},
            {.bucket_steps = {10.0, 1.0}, .tolerance = 1e9});
        solver.build();

        auto paths = solver.solve();

        double best = INF;
        for (const auto& p : paths)
            if (p.reduced_cost < best) best = p.reduced_cost;

        ++tested;
        // Non-elementary solver finds paths at least as good (more negative)
        if (best <= expected + 1.0) {
            ++passed;
        } else {
            ++failed;
            std::printf("  FAIL: %s expected=%.0f got=%.0f\n",
                        name.c_str(), expected, best);
        }
    }

    std::printf("spprclib: %d tested, %d passed, %d failed\n",
                tested, passed, failed);
    CHECK(tested > 0);
    CHECK(failed == 0);
}

// ── RCSPP (rcspp_dataset) — only R1xx and RC1xx (tight time windows) ──

static const char* RCSPP_DIR = "benchmarks/rcspp/rcspp/ng8";

TEST_CASE("rcspp_dataset R1 instances") {
    if (!fs::exists(RCSPP_DIR)) {
        MESSAGE("Skipping rcspp tests: directory not found");
        return;
    }

    int tested = 0, solved = 0;

    for (auto& entry : fs::directory_iterator(RCSPP_DIR)) {
        if (entry.path().extension() != ".graph") continue;
        std::string name = entry.path().stem().string();

        // Only test R1xx instances (tight TW, fast to solve)
        if (!(name[0] == 'R' && name[1] == '1')) continue;

        auto inst = io::load_rcspp_graph(entry.path().string());
        auto pv = inst.problem_view();

        Solver<EmptyPack> solver(pv, EmptyPack{},
            {.bucket_steps = {20.0, 50.0}, .tolerance = 0.0});
        solver.build();

        auto paths = solver.solve();

        ++tested;
        if (!paths.empty()) {
            ++solved;
            for (const auto& p : paths) {
                CHECK(p.reduced_cost < 0.0);
                CHECK(p.vertices.front() == inst.source);
                CHECK(p.vertices.back() == inst.sink);
            }
        }
    }

    std::printf("rcspp R1: %d tested, %d found negative-cost paths\n",
                tested, solved);
    CHECK(tested > 0);
}

TEST_CASE("rcspp_dataset RC1 instances") {
    if (!fs::exists(RCSPP_DIR)) {
        MESSAGE("Skipping rcspp tests: directory not found");
        return;
    }

    int tested = 0, solved = 0;

    for (auto& entry : fs::directory_iterator(RCSPP_DIR)) {
        if (entry.path().extension() != ".graph") continue;
        std::string name = entry.path().stem().string();

        if (!name.starts_with("RC1")) continue;

        auto inst = io::load_rcspp_graph(entry.path().string());
        auto pv = inst.problem_view();

        Solver<EmptyPack> solver(pv, EmptyPack{},
            {.bucket_steps = {20.0, 50.0}, .tolerance = 0.0});
        solver.build();

        auto paths = solver.solve();

        ++tested;
        if (!paths.empty()) {
            ++solved;
            for (const auto& p : paths) {
                CHECK(p.reduced_cost < 0.0);
                CHECK(p.vertices.front() == inst.source);
                CHECK(p.vertices.back() == inst.sink);
            }
        }
    }

    std::printf("rcspp RC1: %d tested, %d found negative-cost paths\n",
                tested, solved);
    CHECK(tested > 0);
}
