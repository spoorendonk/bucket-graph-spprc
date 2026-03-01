/// Standalone benchmark runner.
///
/// Usage: ./build/bench_run [spprclib|rcspp|all]

#include "instance_io.h"

#include <bgspprc/solver.h>
#include <bgspprc/resource.h>

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <map>
#include <string>
#include <vector>

using namespace bgspprc;
namespace fs = std::filesystem;

static std::map<std::string, double> load_csv(const std::string& path) {
    std::map<std::string, double> result;
    std::ifstream file(path);
    if (!file.is_open()) return result;
    std::string line;
    while (std::getline(file, line)) {
        if (line.empty() || line[0] == '#' || line.find("instance") != std::string::npos) continue;
        auto comma = line.find(',');
        if (comma == std::string::npos) continue;
        result[line.substr(0, comma)] = std::stod(line.substr(comma + 1));
    }
    return result;
}

struct Result {
    std::string name;
    double best_cost;
    double expected;
    double time_ms;
    int n_paths;
};

void run_spprclib() {
    std::string dir = "../rcspp-bac/benchmarks/instances/spprclib";
    if (!fs::exists(dir)) { printf("spprclib dir not found\n"); return; }

    auto optima = load_csv(dir + "/optimal.csv");
    std::vector<Result> results;

    for (auto& entry : fs::directory_iterator(dir)) {
        if (entry.path().extension() != ".sppcc") continue;
        std::string name = entry.path().stem().string();

        auto it = optima.find(name);
        double expected = (it != optima.end()) ? it->second : 0.0;
        if (expected >= -3.0) continue;

        auto inst = io::load_sppcc(entry.path().string());
        if (inst.n_vertices > 55) continue;

        auto pv = inst.problem_view();

        auto t0 = std::chrono::high_resolution_clock::now();
        Solver<EmptyPack> solver(pv, EmptyPack{},
            {.bucket_steps = {10.0, 1.0}, .tolerance = 1e9});
        solver.build();
        auto paths = solver.solve();
        auto t1 = std::chrono::high_resolution_clock::now();

        double best = paths.empty() ? 0.0 : paths[0].reduced_cost;
        double ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
        results.push_back({name, best, expected, ms, (int)paths.size()});
    }

    std::sort(results.begin(), results.end(),
              [](auto& a, auto& b) { return a.name < b.name; });

    printf("\n=== SPPRCLIB (n ≤ 55) ===\n");
    printf("%-25s %12s %12s %8s %6s %5s\n",
           "Instance", "Best", "Expected", "ms", "Paths", "OK?");
    printf("%s\n", std::string(72, '-').c_str());

    int pass = 0, fail = 0;
    for (auto& r : results) {
        bool ok = r.best_cost <= r.expected + 1.0;
        if (ok) ++pass; else ++fail;
        printf("%-25s %12.0f %12.0f %8.1f %6d %5s\n",
               r.name.c_str(), r.best_cost, r.expected, r.time_ms, r.n_paths,
               ok ? "OK" : "FAIL");
    }
    printf("\nSPPRCLIB: %d/%d passed\n", pass, pass + fail);
}

void run_rcspp() {
    std::string dir = "benchmarks/rcspp/rcspp/ng8";
    if (!fs::exists(dir)) { printf("rcspp dir not found\n"); return; }

    std::vector<Result> results;
    for (auto& entry : fs::directory_iterator(dir)) {
        if (entry.path().extension() != ".graph") continue;
        std::string name = entry.path().stem().string();

        // Only R1/RC1 instances (tight TW, tractable)
        bool is_r1 = (name[0] == 'R' && name[1] == '1');
        bool is_rc1 = name.starts_with("RC1");
        if (!is_r1 && !is_rc1) continue;

        auto inst = io::load_rcspp_graph(entry.path().string());
        auto pv = inst.problem_view();

        auto t0 = std::chrono::high_resolution_clock::now();
        Solver<EmptyPack> solver(pv, EmptyPack{},
            {.bucket_steps = {20.0, 50.0}, .tolerance = 0.0});
        solver.build();
        auto paths = solver.solve();
        auto t1 = std::chrono::high_resolution_clock::now();

        double best = paths.empty() ? 0.0 : paths[0].reduced_cost;
        double ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
        results.push_back({name, best, 0.0, ms, (int)paths.size()});
    }

    std::sort(results.begin(), results.end(),
              [](auto& a, auto& b) { return a.best_cost < b.best_cost; });

    printf("\n=== RCSPP Dataset (ng8, R1/RC1) ===\n");
    printf("%-15s %12s %8s %6s\n", "Instance", "Best Cost", "ms", "Paths");
    printf("%s\n", std::string(43, '-').c_str());

    int found_neg = 0;
    double total_ms = 0;
    for (auto& r : results) {
        printf("%-15s %12.3f %8.1f %6d\n",
               r.name.c_str(), r.best_cost, r.time_ms, r.n_paths);
        if (r.best_cost < -1e-6) ++found_neg;
        total_ms += r.time_ms;
    }
    printf("\nRCSPP R1/RC1: %d/%d found neg-cost paths, total %.1fms\n",
           found_neg, (int)results.size(), total_ms);
}

int main(int argc, char** argv) {
    bool do_spprclib = true, do_rcspp = true;
    if (argc > 1) {
        do_spprclib = std::strcmp(argv[1], "spprclib") == 0 || std::strcmp(argv[1], "all") == 0;
        do_rcspp = std::strcmp(argv[1], "rcspp") == 0 || std::strcmp(argv[1], "all") == 0;
    }
    if (do_spprclib) run_spprclib();
    if (do_rcspp) run_rcspp();
    return 0;
}
