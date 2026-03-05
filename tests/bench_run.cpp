/// Standalone benchmark runner.
///
/// Usage: ./build/bench_run [spprclib|rcspp|roberti|bidir|stages|eliminate|validate|verify|all]

#include "instance_io.h"

#include <bgspprc/solver.h>
#include <bgspprc/resource.h>
#include <bgspprc/resources/ng_path.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <map>
#include <string>
#include <vector>

using namespace bgspprc;
namespace fs = std::filesystem;

using NgPack = ResourcePack<NgPathResource>;

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

// ────────────────────────────────────────────────────────────────
// SPPRCLIB benchmark (capacity-only ESPPRC instances)
// ────────────────────────────────────────────────────────────────

void run_spprclib() {
    std::string dir = "../rcspp-bac-3/benchmarks/instances/spprclib";
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

    printf("\n=== SPPRCLIB (n <= 55) ===\n");
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

// ────────────────────────────────────────────────────────────────
// Roberti benchmark (ESPPRC pricing instances with EUC_2D + profits)
// ────────────────────────────────────────────────────────────────

void run_roberti() {
    std::string dir = "../rcspp-bac-3/benchmarks/instances/roberti";
    if (!fs::exists(dir)) { printf("roberti dir not found\n"); return; }

    auto optima = load_csv(dir + "/optimal.csv");
    std::vector<Result> results;

    for (auto& entry : fs::directory_iterator(dir)) {
        if (entry.path().extension() != ".vrp") continue;
        std::string name = entry.path().stem().string();

        auto it = optima.find(name);
        double expected = (it != optima.end()) ? it->second : 0.0;

        auto inst = io::load_roberti_vrp(entry.path().string());
        if (inst.n_vertices > 80) continue;  // skip large for now

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

    printf("\n=== Roberti (n <= 80) ===\n");
    printf("%-25s %12s %12s %8s %6s %5s\n",
           "Instance", "Best", "Expected", "ms", "Paths", "OK?");
    printf("%s\n", std::string(72, '-').c_str());

    int pass = 0, fail = 0, no_opt = 0;
    for (auto& r : results) {
        // Non-elementary solver should find cost <= elementary optimal
        bool ok;
        if (r.expected > -1e-9 && r.expected < 1e-9) {
            ok = true;  // no known optimal
            ++no_opt;
        } else {
            ok = r.best_cost <= r.expected + 0.01;
        }
        if (ok) ++pass; else ++fail;
        printf("%-25s %12.3f %12.3f %8.1f %6d %5s\n",
               r.name.c_str(), r.best_cost, r.expected, r.time_ms, r.n_paths,
               ok ? "OK" : "FAIL");
    }
    printf("\nRoberti: %d/%d passed (%d without known optimal)\n",
           pass, pass + fail, no_opt);
}

// ────────────────────────────────────────────────────────────────
// RCSPP benchmark (Solomon-derived, time windows + capacity)
// ────────────────────────────────────────────────────────────────

void run_rcspp_dir(const std::string& dir, const std::string& label) {
    if (!fs::exists(dir)) {
        printf("%s dir not found: %s\n", label.c_str(), dir.c_str());
        return;
    }

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
        NgPathResource ng(pv.n_vertices, pv.n_arcs, pv.arc_from, pv.arc_to, inst.ng_neighbors);

        auto t0 = std::chrono::high_resolution_clock::now();
        Solver<NgPack> solver(pv, make_resource_pack(std::move(ng)),
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

    printf("\n=== RCSPP %s (R1/RC1, ng-path) ===\n", label.c_str());
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
    printf("\nRCSPP %s: %d/%d found neg-cost paths, total %.1fms\n",
           label.c_str(), found_neg, (int)results.size(), total_ms);
}

void run_rcspp() {
    run_rcspp_dir("benchmarks/rcspp/rcspp/ng8", "ng8");
    run_rcspp_dir("benchmarks/rcspp/rcspp/ng16", "ng16");
    run_rcspp_dir("benchmarks/rcspp/rcspp/ng24", "ng24");
}

// ────────────────────────────────────────────────────────────────
// Bidirectional correctness: RCSPP
// ────────────────────────────────────────────────────────────────

void run_bidir_rcspp() {
    std::string dir = "benchmarks/rcspp/rcspp/ng8";
    if (!fs::exists(dir)) { printf("rcspp dir not found\n"); return; }

    printf("\n=== Bidirectional vs Mono: RCSPP ng8 R1/RC1 ===\n");
    printf("%-15s %12s %8s %12s %8s %6s %5s\n",
           "Instance", "Mono Cost", "Mono ms", "Bidir Cost", "Bidir ms", "Paths", "Match");
    printf("%s\n", std::string(75, '-').c_str());

    struct Row {
        std::string name;
        double mono_cost, bidir_cost;
        double mono_ms, bidir_ms;
        int mono_paths, bidir_paths;
        bool match;
    };
    std::vector<Row> rows;

    for (auto& entry : fs::directory_iterator(dir)) {
        if (entry.path().extension() != ".graph") continue;
        std::string name = entry.path().stem().string();
        bool is_r1 = (name[0] == 'R' && name[1] == '1');
        bool is_rc1 = name.starts_with("RC1");
        if (!is_r1 && !is_rc1) continue;

        auto inst = io::load_rcspp_graph(entry.path().string());
        auto pv = inst.problem_view();

        // Mono
        auto t0 = std::chrono::high_resolution_clock::now();
        NgPathResource ng_mono(pv.n_vertices, pv.n_arcs, pv.arc_from, pv.arc_to, inst.ng_neighbors);
        Solver<NgPack> mono(pv, make_resource_pack(std::move(ng_mono)),
            {.bucket_steps = {20.0, 50.0}, .tolerance = 0.0});
        mono.build();
        mono.set_stage(Stage::Exact);
        auto mono_paths = mono.solve();
        auto t1 = std::chrono::high_resolution_clock::now();
        double mono_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
        double mono_best = mono_paths.empty() ? 1e18 : mono_paths[0].reduced_cost;

        // Bidir
        t0 = std::chrono::high_resolution_clock::now();
        NgPathResource ng_bidir(pv.n_vertices, pv.n_arcs, pv.arc_from, pv.arc_to, inst.ng_neighbors);
        Solver<NgPack> bidir(pv, make_resource_pack(std::move(ng_bidir)),
            {.bucket_steps = {20.0, 50.0}, .bidirectional = true, .tolerance = 0.0});
        bidir.build();
        bidir.set_stage(Stage::Exact);
        auto bidir_paths = bidir.solve();
        t1 = std::chrono::high_resolution_clock::now();
        double bidir_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
        double bidir_best = bidir_paths.empty() ? 1e18 : bidir_paths[0].reduced_cost;

        bool match = std::abs(mono_best - bidir_best) < 1.0 ||
                     (mono_best >= 0.0 && bidir_best >= -1e-6);

        rows.push_back({name, mono_best, bidir_best, mono_ms, bidir_ms,
                        (int)mono_paths.size(), (int)bidir_paths.size(), match});
    }

    std::sort(rows.begin(), rows.end(),
              [](auto& a, auto& b) { return a.name < b.name; });

    int matched = 0;
    for (auto& r : rows) {
        if (r.match) ++matched;
        printf("%-15s %12.3f %8.1f %12.3f %8.1f %3d/%3d %5s\n",
               r.name.c_str(), r.mono_cost, r.mono_ms,
               r.bidir_cost, r.bidir_ms,
               r.mono_paths, r.bidir_paths,
               r.match ? "OK" : "FAIL");
    }
    printf("\nBidir RCSPP ng8 (ng-path): %d/%d matched\n", matched, (int)rows.size());
}

// ────────────────────────────────────────────────────────────────
// Bidirectional correctness: SPPRCLIB
// ────────────────────────────────────────────────────────────────

void run_bidir_spprclib() {
    std::string dir = "../rcspp-bac-3/benchmarks/instances/spprclib";
    if (!fs::exists(dir)) { printf("spprclib dir not found\n"); return; }

    auto optima = load_csv(dir + "/optimal.csv");

    printf("\n=== Bidirectional vs Mono: SPPRCLIB (n <= 55) ===\n");
    printf("%-25s %12s %12s %12s %8s %8s %5s\n",
           "Instance", "Expected", "Mono", "Bidir", "Mono ms", "Bi ms", "Match");
    printf("%s\n", std::string(90, '-').c_str());

    struct Row {
        std::string name;
        double expected, mono_cost, bidir_cost;
        double mono_ms, bidir_ms;
        bool match;
    };
    std::vector<Row> rows;

    for (auto& entry : fs::directory_iterator(dir)) {
        if (entry.path().extension() != ".sppcc") continue;
        std::string name = entry.path().stem().string();

        auto it = optima.find(name);
        double expected = (it != optima.end()) ? it->second : 0.0;
        if (expected >= -3.0) continue;

        auto inst = io::load_sppcc(entry.path().string());
        if (inst.n_vertices > 55) continue;

        auto pv = inst.problem_view();

        // Mono
        auto t0 = std::chrono::high_resolution_clock::now();
        Solver<EmptyPack> mono(pv, EmptyPack{},
            {.bucket_steps = {10.0, 1.0}, .tolerance = 1e9});
        mono.build();
        mono.set_stage(Stage::Exact);
        auto mono_paths = mono.solve();
        auto t1 = std::chrono::high_resolution_clock::now();
        double mono_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
        double mono_best = mono_paths.empty() ? 1e18 : mono_paths[0].reduced_cost;

        // Bidir
        t0 = std::chrono::high_resolution_clock::now();
        Solver<EmptyPack> bidir(pv, EmptyPack{},
            {.bucket_steps = {10.0, 1.0}, .bidirectional = true, .tolerance = 1e9});
        bidir.build();
        bidir.set_stage(Stage::Exact);
        auto bidir_paths = bidir.solve();
        t1 = std::chrono::high_resolution_clock::now();
        double bidir_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
        double bidir_best = bidir_paths.empty() ? 1e18 : bidir_paths[0].reduced_cost;

        bool match = std::abs(mono_best - bidir_best) < 1.0;

        rows.push_back({name, expected, mono_best, bidir_best,
                        mono_ms, bidir_ms, match});
    }

    std::sort(rows.begin(), rows.end(),
              [](auto& a, auto& b) { return a.name < b.name; });

    int matched = 0, vs_opt_ok = 0;
    for (auto& r : rows) {
        if (r.match) ++matched;
        if (r.mono_cost <= r.expected + 1.0) ++vs_opt_ok;
        printf("%-25s %12.0f %12.0f %12.0f %8.1f %8.1f %5s\n",
               r.name.c_str(), r.expected, r.mono_cost, r.bidir_cost,
               r.mono_ms, r.bidir_ms, r.match ? "OK" : "FAIL");
    }
    printf("\nBidir SPPRCLIB: %d/%d mono==bidir, %d/%d <= optimal\n",
           matched, (int)rows.size(), vs_opt_ok, (int)rows.size());
}

// ────────────────────────────────────────────────────────────────
// Bidirectional correctness: Roberti
// ────────────────────────────────────────────────────────────────

void run_bidir_roberti() {
    std::string dir = "../rcspp-bac-3/benchmarks/instances/roberti";
    if (!fs::exists(dir)) { printf("roberti dir not found\n"); return; }

    auto optima = load_csv(dir + "/optimal.csv");

    printf("\n=== Bidirectional vs Mono: Roberti (n <= 80) ===\n");
    printf("%-25s %12s %12s %12s %8s %8s %5s\n",
           "Instance", "Expected", "Mono", "Bidir", "Mono ms", "Bi ms", "Match");
    printf("%s\n", std::string(90, '-').c_str());

    struct Row {
        std::string name;
        double expected, mono_cost, bidir_cost;
        double mono_ms, bidir_ms;
        bool match;
    };
    std::vector<Row> rows;

    for (auto& entry : fs::directory_iterator(dir)) {
        if (entry.path().extension() != ".vrp") continue;
        std::string name = entry.path().stem().string();

        auto it = optima.find(name);
        double expected = (it != optima.end()) ? it->second : 0.0;

        auto inst = io::load_roberti_vrp(entry.path().string());
        if (inst.n_vertices > 80) continue;

        auto pv = inst.problem_view();

        // Skip very hard instances (F class with k<=4 has huge capacity windows)
        if (name.starts_with("F-") && pv.n_arcs > 4000) continue;

        // Mono
        auto t0 = std::chrono::high_resolution_clock::now();
        Solver<EmptyPack> mono(pv, EmptyPack{},
            {.bucket_steps = {10.0, 1.0}, .tolerance = 1e9});
        mono.build();
        mono.set_stage(Stage::Exact);
        auto mono_paths = mono.solve();
        auto t1 = std::chrono::high_resolution_clock::now();
        double mono_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
        double mono_best = mono_paths.empty() ? 1e18 : mono_paths[0].reduced_cost;

        // Bidir
        t0 = std::chrono::high_resolution_clock::now();
        Solver<EmptyPack> bidir(pv, EmptyPack{},
            {.bucket_steps = {10.0, 1.0}, .bidirectional = true, .tolerance = 1e9});
        bidir.build();
        bidir.set_stage(Stage::Exact);
        auto bidir_paths = bidir.solve();
        t1 = std::chrono::high_resolution_clock::now();
        double bidir_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
        double bidir_best = bidir_paths.empty() ? 1e18 : bidir_paths[0].reduced_cost;

        bool match = std::abs(mono_best - bidir_best) < 1.0;

        rows.push_back({name, expected, mono_best, bidir_best,
                        mono_ms, bidir_ms, match});
    }

    std::sort(rows.begin(), rows.end(),
              [](auto& a, auto& b) { return a.name < b.name; });

    int matched = 0, vs_opt_ok = 0;
    for (auto& r : rows) {
        if (r.match) ++matched;
        if (r.expected < -1e-9 && r.mono_cost <= r.expected + 0.01) ++vs_opt_ok;
        printf("%-25s %12.3f %12.3f %12.3f %8.1f %8.1f %5s\n",
               r.name.c_str(), r.expected, r.mono_cost, r.bidir_cost,
               r.mono_ms, r.bidir_ms, r.match ? "OK" : "FAIL");
    }
    int with_opt = 0;
    for (auto& r : rows) if (r.expected < -1e-9) ++with_opt;
    printf("\nBidir Roberti: %d/%d mono==bidir, %d/%d <= optimal\n",
           matched, (int)rows.size(), vs_opt_ok, with_opt);
}

// ────────────────────────────────────────────────────────────────
// Multi-stage comparison
// ────────────────────────────────────────────────────────────────

void run_stages() {
    std::string dir = "benchmarks/rcspp/rcspp/ng8";
    if (!fs::exists(dir)) { printf("rcspp dir not found\n"); return; }

    printf("\n=== Multi-Stage Comparison: RCSPP R1 ===\n");
    printf("%-15s %12s %8s %12s %8s %12s %8s\n",
           "Instance", "Heur1", "ms", "Heur2", "ms", "Exact", "ms");
    printf("%s\n", std::string(82, '-').c_str());

    struct Row {
        std::string name;
        double h1_cost, h2_cost, exact_cost;
        double h1_ms, h2_ms, exact_ms;
    };
    std::vector<Row> rows;

    for (auto& entry : fs::directory_iterator(dir)) {
        if (entry.path().extension() != ".graph") continue;
        std::string name = entry.path().stem().string();
        if (!(name[0] == 'R' && name[1] == '1')) continue;

        auto inst = io::load_rcspp_graph(entry.path().string());
        auto pv = inst.problem_view();

        auto run_with_stage = [&](Stage stage) -> std::pair<double, double> {
            auto t0 = std::chrono::high_resolution_clock::now();
            Solver<EmptyPack> solver(pv, EmptyPack{},
                {.bucket_steps = {20.0, 50.0}, .tolerance = 0.0});
            solver.build();
            solver.set_stage(stage);
            auto paths = solver.solve();
            auto t1 = std::chrono::high_resolution_clock::now();
            double ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
            double best = paths.empty() ? 1e18 : paths[0].reduced_cost;
            return {best, ms};
        };

        auto [h1, h1_ms] = run_with_stage(Stage::Heuristic1);
        auto [h2, h2_ms] = run_with_stage(Stage::Heuristic2);
        auto [ex, ex_ms] = run_with_stage(Stage::Exact);

        rows.push_back({name, h1, h2, ex, h1_ms, h2_ms, ex_ms});
    }

    std::sort(rows.begin(), rows.end(),
              [](auto& a, auto& b) { return a.name < b.name; });

    int h1_match = 0, h2_match = 0;
    for (auto& r : rows) {
        if (std::abs(r.h1_cost - r.exact_cost) < 1.0 ||
            r.h1_cost <= r.exact_cost + 1e-6) ++h1_match;
        if (std::abs(r.h2_cost - r.exact_cost) < 1.0 ||
            r.h2_cost <= r.exact_cost + 1e-6) ++h2_match;
        printf("%-15s %12.3f %8.1f %12.3f %8.1f %12.3f %8.1f\n",
               r.name.c_str(), r.h1_cost, r.h1_ms,
               r.h2_cost, r.h2_ms, r.exact_cost, r.exact_ms);
    }
    printf("\nStages: H1 matched exact %d/%d, H2 matched exact %d/%d\n",
           h1_match, (int)rows.size(), h2_match, (int)rows.size());
}

// ────────────────────────────────────────────────────────────────
// Arc elimination correctness
// ────────────────────────────────────────────────────────────────

void run_elimination() {
    std::string dir = "benchmarks/rcspp/rcspp/ng8";
    if (!fs::exists(dir)) { printf("rcspp dir not found\n"); return; }

    printf("\n=== Arc Elimination Correctness: RCSPP R1 ===\n");
    printf("%-15s %12s %12s %8s %8s %5s\n",
           "Instance", "Before", "After", "Arcs-", "Jump+", "Match");
    printf("%s\n", std::string(65, '-').c_str());

    int matched = 0, total = 0;
    for (auto& entry : fs::directory_iterator(dir)) {
        if (entry.path().extension() != ".graph") continue;
        std::string name = entry.path().stem().string();
        if (!(name[0] == 'R' && name[1] == '1')) continue;

        auto inst = io::load_rcspp_graph(entry.path().string());
        auto pv = inst.problem_view();

        BucketGraph<EmptyPack> bg(pv, EmptyPack{},
            {.bucket_steps = {20.0, 50.0}, .tolerance = 0.0});
        bg.build();

        auto paths_before = bg.solve();
        double best_before = paths_before.empty() ? 1e18 : paths_before[0].reduced_cost;

        int arcs_before = 0;
        for (int i = 0; i < bg.n_buckets(); ++i)
            arcs_before += static_cast<int>(bg.bucket(i).bucket_arcs.size());

        if (best_before < -1e-6) {
            bg.eliminate_arcs(best_before * 0.5);
        } else {
            bg.eliminate_arcs(0.0);
        }

        int arcs_after = 0, jumps = 0;
        for (int i = 0; i < bg.n_buckets(); ++i) {
            arcs_after += static_cast<int>(bg.bucket(i).bucket_arcs.size());
            jumps += static_cast<int>(bg.bucket(i).jump_arcs.size());
        }

        auto paths_after = bg.solve();
        double best_after = paths_after.empty() ? 1e18 : paths_after[0].reduced_cost;

        ++total;
        bool match = (best_before >= -1e-6 && best_after >= -1e-6) ||
                     (best_after <= best_before + 1.0);
        if (match) ++matched;

        printf("%-15s %12.3f %12.3f %8d %8d %5s\n",
               name.c_str(), best_before, best_after,
               arcs_before - arcs_after, jumps,
               match ? "OK" : "FAIL");

        bg.reset_elimination();
    }
    printf("\nElimination: %d/%d preserved optimality\n", matched, total);
}

// ────────────────────────────────────────────────────────────────
// Bucket fixing benchmark
// ────────────────────────────────────────────────────────────────

void run_bucket_fixing() {
    std::string dir = "benchmarks/rcspp/rcspp/ng8";
    if (!fs::exists(dir)) { printf("rcspp dir not found\n"); return; }

    // Test 1: bucket fixing WITHOUT arc elimination (pure fixing)
    printf("\n=== Bucket Fixing Only (no arc elim): RCSPP R1 ===\n");
    printf("%-15s %12s %12s %7s %7s %7s %5s\n",
           "Instance", "Before", "After", "Total", "Fixed", "Pct", "Match");
    printf("%s\n", std::string(72, '-').c_str());

    int matched = 0, total = 0;
    for (auto& entry : fs::directory_iterator(dir)) {
        if (entry.path().extension() != ".graph") continue;
        std::string name = entry.path().stem().string();
        if (!(name[0] == 'R' && name[1] == '1')) continue;

        auto inst = io::load_rcspp_graph(entry.path().string());
        auto pv = inst.problem_view();

        BucketGraph<EmptyPack> bg(pv, EmptyPack{},
            {.bucket_steps = {20.0, 50.0}, .tolerance = 0.0});
        bg.build();

        auto paths_before = bg.solve();
        double best_before = paths_before.empty() ? 1e18 : paths_before[0].reduced_cost;

        int n_total = bg.n_buckets();
        double theta = (best_before < -1e-6) ? best_before * 0.5 : 0.0;

        // Fix buckets only (no arc elimination)
        bg.fix_buckets(theta);

        auto paths_after = bg.solve();
        double best_after = paths_after.empty() ? 1e18 : paths_after[0].reduced_cost;

        ++total;
        bool match = (best_before >= -1e-6 && best_after >= -1e-6) ||
                     (best_after <= best_before + 1.0);
        if (match) ++matched;

        double pct = (n_total > 0) ? 100.0 * bg.n_fixed_buckets() / n_total : 0.0;
        printf("%-15s %12.3f %12.3f %7d %7d %6.1f%% %5s\n",
               name.c_str(), best_before, best_after,
               n_total, bg.n_fixed_buckets(), pct,
               match ? "OK" : "FAIL");

        bg.reset_elimination();
    }
    printf("\nBucket fixing only: %d/%d preserved optimality\n", matched, total);

    // Also test bidirectional bucket fixing
    printf("\n=== Bidirectional Bucket Fixing: RCSPP R1 ===\n");
    printf("%-15s %12s %12s %7s %7s %7s %5s\n",
           "Instance", "Before", "After", "Total", "Fixed", "Pct", "Match");
    printf("%s\n", std::string(72, '-').c_str());

    int matched_bi = 0, total_bi = 0;
    for (auto& entry : fs::directory_iterator(dir)) {
        if (entry.path().extension() != ".graph") continue;
        std::string name = entry.path().stem().string();
        if (!(name[0] == 'R' && name[1] == '1')) continue;

        auto inst = io::load_rcspp_graph(entry.path().string());
        auto pv = inst.problem_view();

        BucketGraph<EmptyPack> bg(pv, EmptyPack{},
            {.bucket_steps = {20.0, 50.0}, .tolerance = 0.0, .bidirectional = true});
        bg.build();

        auto paths_before = bg.solve();
        double best_before = paths_before.empty() ? 1e18 : paths_before[0].reduced_cost;

        int n_total = bg.n_buckets();
        double theta = (best_before < -1e-6) ? best_before * 0.5 : 0.0;

        bg.eliminate_arcs(theta);
        bg.fix_buckets(theta);

        auto paths_after = bg.solve();
        double best_after = paths_after.empty() ? 1e18 : paths_after[0].reduced_cost;

        ++total_bi;
        bool match = (best_before >= -1e-6 && best_after >= -1e-6) ||
                     (best_after <= best_before + 1.0);
        if (match) ++matched_bi;

        double pct = (n_total > 0) ? 100.0 * bg.n_fixed_buckets() / n_total : 0.0;
        printf("%-15s %12.3f %12.3f %7d %7d %6.1f%% %5s\n",
               name.c_str(), best_before, best_after,
               n_total, bg.n_fixed_buckets(), pct,
               match ? "OK" : "FAIL");

        bg.reset_elimination();
    }
    printf("\nBucket fixing: mono %d/%d, bidir %d/%d preserved optimality\n",
           matched, total, matched_bi, total_bi);
}

// ────────────────────────────────────────────────────────────────
// Path validation (structural correctness)
// ────────────────────────────────────────────────────────────────

void run_path_validation() {
    printf("\n=== Path Validation (mono + bidir) ===\n");

    int total_paths = 0, valid_paths = 0, total_instances = 0;

    auto validate = [&](const ProblemView& pv,
                        const std::vector<typename Solver<EmptyPack>::Path>& paths,
                        const char* mode, const char* name) {
        for (const auto& p : paths) {
            ++total_paths;
            bool ok = true;

            if (p.vertices.empty() || p.vertices.front() != pv.source ||
                p.vertices.back() != pv.sink) {
                printf("  %s %s: bad endpoints\n", mode, name);
                ok = false;
            }

            if (p.arcs.size() + 1 != p.vertices.size()) {
                printf("  %s %s: arc/vertex count mismatch (%zu arcs, %zu verts)\n",
                       mode, name, p.arcs.size(), p.vertices.size());
                ok = false;
            }

            for (size_t i = 0; i < p.arcs.size() && ok; ++i) {
                int a = p.arcs[i];
                if (a < 0 || a >= pv.n_arcs) {
                    printf("  %s %s: invalid arc %d\n", mode, name, a);
                    ok = false;
                } else if (pv.arc_from[a] != p.vertices[i] ||
                           pv.arc_to[a] != p.vertices[i+1]) {
                    printf("  %s %s: arc %d doesn't connect v%d->v%d (expected v%d->v%d)\n",
                           mode, name, a, p.vertices[i], p.vertices[i+1],
                           pv.arc_from[a], pv.arc_to[a]);
                    ok = false;
                }
            }

            if (ok) {
                double sum = 0.0;
                for (int a : p.arcs) sum += pv.arc_base_cost[a];
                if (std::abs(sum - p.original_cost) > 1e-6) {
                    printf("  %s %s: cost mismatch sum=%.6f stored=%.6f\n",
                           mode, name, sum, p.original_cost);
                    ok = false;
                }
            }

            if (ok) ++valid_paths;
        }
    };

    // RCSPP ng8 R1/RC1 (with ng-path)
    {
        std::string dir = "benchmarks/rcspp/rcspp/ng8";
        if (fs::exists(dir)) {
            for (auto& entry : fs::directory_iterator(dir)) {
                if (entry.path().extension() != ".graph") continue;
                std::string name = entry.path().stem().string();
                bool is_r1 = (name[0] == 'R' && name[1] == '1');
                bool is_rc1 = name.starts_with("RC1");
                if (!is_r1 && !is_rc1) continue;

                auto inst = io::load_rcspp_graph(entry.path().string());
                auto pv = inst.problem_view();

                NgPathResource ng_m(pv.n_vertices, pv.n_arcs, pv.arc_from, pv.arc_to, inst.ng_neighbors);
                Solver<NgPack> mono(pv, make_resource_pack(std::move(ng_m)),
                    {.bucket_steps = {20.0, 50.0}, .tolerance = 0.0});
                mono.build();
                mono.set_stage(Stage::Exact);
                auto mono_paths = mono.solve();
                // Validate using same logic (Path struct is identical)
                for (const auto& p : mono_paths) {
                    ++total_paths;
                    bool ok = !p.vertices.empty() && p.vertices.front() == pv.source &&
                              p.vertices.back() == pv.sink && p.arcs.size() + 1 == p.vertices.size();
                    if (ok) {
                        double sum = 0.0;
                        for (int a : p.arcs) sum += pv.arc_base_cost[a];
                        if (std::abs(sum - p.original_cost) > 1e-6) ok = false;
                    }
                    if (ok) ++valid_paths;
                }

                NgPathResource ng_b(pv.n_vertices, pv.n_arcs, pv.arc_from, pv.arc_to, inst.ng_neighbors);
                Solver<NgPack> bidir(pv, make_resource_pack(std::move(ng_b)),
                    {.bucket_steps = {20.0, 50.0}, .bidirectional = true, .tolerance = 0.0});
                bidir.build();
                bidir.set_stage(Stage::Exact);
                auto bidir_paths = bidir.solve();
                for (const auto& p : bidir_paths) {
                    ++total_paths;
                    bool ok = !p.vertices.empty() && p.vertices.front() == pv.source &&
                              p.vertices.back() == pv.sink && p.arcs.size() + 1 == p.vertices.size();
                    if (ok) {
                        double sum = 0.0;
                        for (int a : p.arcs) sum += pv.arc_base_cost[a];
                        if (std::abs(sum - p.original_cost) > 1e-6) ok = false;
                    }
                    if (ok) ++valid_paths;
                }

                ++total_instances;
            }
        }
    }

    // SPPRCLIB
    {
        std::string dir = "../rcspp-bac-3/benchmarks/instances/spprclib";
        if (fs::exists(dir)) {
            for (auto& entry : fs::directory_iterator(dir)) {
                if (entry.path().extension() != ".sppcc") continue;
                auto inst = io::load_sppcc(entry.path().string());
                if (inst.n_vertices > 55) continue;
                auto pv = inst.problem_view();
                std::string name = entry.path().stem().string();

                Solver<EmptyPack> mono(pv, EmptyPack{},
                    {.bucket_steps = {10.0, 1.0}, .tolerance = 1e9});
                mono.build();
                mono.set_stage(Stage::Exact);
                validate(pv, mono.solve(), "mono", name.c_str());

                Solver<EmptyPack> bidir(pv, EmptyPack{},
                    {.bucket_steps = {10.0, 1.0}, .bidirectional = true, .tolerance = 1e9});
                bidir.build();
                bidir.set_stage(Stage::Exact);
                validate(pv, bidir.solve(), "bidir", name.c_str());

                ++total_instances;
            }
        }
    }

    // Roberti
    {
        std::string dir = "../rcspp-bac-3/benchmarks/instances/roberti";
        if (fs::exists(dir)) {
            for (auto& entry : fs::directory_iterator(dir)) {
                if (entry.path().extension() != ".vrp") continue;
                auto inst = io::load_roberti_vrp(entry.path().string());
                if (inst.n_vertices > 80) continue;
                auto pv = inst.problem_view();
                std::string name = entry.path().stem().string();

                // Skip very hard F instances
                if (name.starts_with("F-") && pv.n_arcs > 4000) continue;

                Solver<EmptyPack> mono(pv, EmptyPack{},
                    {.bucket_steps = {10.0, 1.0}, .tolerance = 1e9});
                mono.build();
                mono.set_stage(Stage::Exact);
                validate(pv, mono.solve(), "mono", name.c_str());

                Solver<EmptyPack> bidir(pv, EmptyPack{},
                    {.bucket_steps = {10.0, 1.0}, .bidirectional = true, .tolerance = 1e9});
                bidir.build();
                bidir.set_stage(Stage::Exact);
                validate(pv, bidir.solve(), "bidir", name.c_str());

                ++total_instances;
            }
        }
    }

    printf("Validated %d/%d paths across %d instances (mono+bidir)\n",
           valid_paths, total_paths, total_instances);
}

// ────────────────────────────────────────────────────────────────
// Unified correctness verification
// ────────────────────────────────────────────────────────────────

static void write_csv_row(std::ofstream& out, const std::string& name,
                          double mono, double bidir, double optimal, bool pass) {
    if (!out.is_open()) return;
    out << name << "," << mono << "," << bidir << "," << optimal
        << "," << (pass ? "PASS" : "FAIL") << "\n";
}

int run_verify(const std::string& csv_path = "") {
    int total_pass = 0, total_fail = 0;
    std::ofstream csv_out;
    if (!csv_path.empty()) {
        csv_out.open(csv_path);
        csv_out << "instance,mono_best,bidir_best,optimal,status\n";
    }

    // ── SPPRCLIB (with ng-path) ──
    {
        std::string dir = "../rcspp-bac-3/benchmarks/instances/spprclib";
        if (fs::exists(dir)) {
            auto optima = load_csv(dir + "/optimal.csv");
            printf("\n=== verify: SPPRCLIB (n <= 55) ===\n");

            std::vector<std::string> names;
            for (auto& entry : fs::directory_iterator(dir)) {
                if (entry.path().extension() != ".sppcc") continue;
                names.push_back(entry.path().stem().string());
            }
            std::sort(names.begin(), names.end());

            for (auto& name : names) {
                auto it = optima.find(name);
                double expected = (it != optima.end()) ? it->second : 0.0;
                if (expected >= -3.0) continue;

                auto inst = io::load_sppcc(dir + "/" + name + ".sppcc");
                if (inst.n_vertices > 55) continue;
                io::compute_ng_neighbors(inst, 8);
                auto pv = inst.problem_view();

                // Mono with ng-path
                NgPathResource ng_m(pv.n_vertices, pv.n_arcs, pv.arc_from, pv.arc_to, inst.ng_neighbors);
                Solver<NgPack> mono(pv, make_resource_pack(std::move(ng_m)),
                    {.bucket_steps = {10.0, 1.0}, .tolerance = 1e9});
                mono.build();
                mono.set_stage(Stage::Exact);
                auto mono_paths = mono.solve();
                double mono_best = mono_paths.empty() ? 1e18 : mono_paths[0].reduced_cost;

                // Bidir with ng-path
                NgPathResource ng_b(pv.n_vertices, pv.n_arcs, pv.arc_from, pv.arc_to, inst.ng_neighbors);
                Solver<NgPack> bidir(pv, make_resource_pack(std::move(ng_b)),
                    {.bucket_steps = {10.0, 1.0}, .bidirectional = true, .tolerance = 1e9});
                bidir.build();
                bidir.set_stage(Stage::Exact);
                auto bidir_paths = bidir.solve();
                double bidir_best = bidir_paths.empty() ? 1e18 : bidir_paths[0].reduced_cost;

                // Check vs optimal (ng-relaxation should be <= ESPPRC optimal)
                bool opt_ok = mono_best <= expected + 1.0;
                bool match_ok = std::abs(mono_best - bidir_best) < 1.0;
                bool pass = opt_ok && match_ok;
                if (pass) ++total_pass; else ++total_fail;

                std::string reason;
                if (!opt_ok) reason += "opt ";
                if (!match_ok) reason += "match ";

                printf("  %-25s  mono=%12.3f  bidir=%12.3f  %s",
                       name.c_str(), mono_best, bidir_best, pass ? "PASS" : "FAIL");
                if (!pass) printf("  [%s]", reason.c_str());
                printf("  (opt=%.3f)\n", expected);
                write_csv_row(csv_out, name, mono_best, bidir_best, expected, pass);
            }
        } else {
            printf("\n=== verify: SPPRCLIB — SKIPPED (dir not found) ===\n");
        }
    }

    // ── Roberti ──
    {
        std::string dir = "../rcspp-bac-3/benchmarks/instances/roberti";
        if (fs::exists(dir)) {
            auto optima = load_csv(dir + "/optimal.csv");
            printf("\n=== verify: Roberti (n <= 80, ng-path) ===\n");

            std::vector<std::string> names;
            for (auto& entry : fs::directory_iterator(dir)) {
                if (entry.path().extension() != ".vrp") continue;
                names.push_back(entry.path().stem().string());
            }
            std::sort(names.begin(), names.end());

            for (auto& name : names) {
                auto it = optima.find(name);
                double expected = (it != optima.end()) ? it->second : 0.0;

                auto inst = io::load_roberti_vrp(dir + "/" + name + ".vrp");
                if (inst.n_vertices > 80) continue;
                io::compute_ng_neighbors(inst, 8);
                auto pv = inst.problem_view();

                if (name.starts_with("F-") && pv.n_arcs > 4000) continue;

                double optimal = (expected < -1e-9 || expected > 1e-9) ? expected : NAN;

                // Mono with ng-path
                NgPathResource ng_m(pv.n_vertices, pv.n_arcs, pv.arc_from, pv.arc_to, inst.ng_neighbors);
                Solver<NgPack> mono(pv, make_resource_pack(std::move(ng_m)),
                    {.bucket_steps = {10.0, 1.0}, .tolerance = 1e9});
                mono.build();
                mono.set_stage(Stage::Exact);
                auto mono_paths = mono.solve();
                double mono_best = mono_paths.empty() ? 1e18 : mono_paths[0].reduced_cost;

                // Bidir with ng-path
                NgPathResource ng_b(pv.n_vertices, pv.n_arcs, pv.arc_from, pv.arc_to, inst.ng_neighbors);
                Solver<NgPack> bidir(pv, make_resource_pack(std::move(ng_b)),
                    {.bucket_steps = {10.0, 1.0}, .bidirectional = true, .tolerance = 1e9});
                bidir.build();
                bidir.set_stage(Stage::Exact);
                auto bidir_paths = bidir.solve();
                double bidir_best = bidir_paths.empty() ? 1e18 : bidir_paths[0].reduced_cost;

                bool opt_ok = true;
                if (!std::isnan(optimal)) {
                    opt_ok = mono_best <= optimal + 1.0;
                }
                bool match_ok = std::abs(mono_best - bidir_best) < 1.0 ||
                                (mono_best >= 0.0 && bidir_best >= -1e-6);
                bool pass = opt_ok && match_ok;
                if (pass) ++total_pass; else ++total_fail;

                printf("  %-25s  mono=%12.3f  bidir=%12.3f  %s",
                       name.c_str(), mono_best, bidir_best, pass ? "PASS" : "FAIL");
                if (!pass) {
                    if (!opt_ok) printf("  [opt]");
                    if (!match_ok) printf("  [match]");
                }
                if (!std::isnan(optimal)) printf("  (opt=%.3f)", optimal);
                printf("\n");
                write_csv_row(csv_out, name, mono_best, bidir_best,
                              std::isnan(optimal) ? 0.0 : optimal, pass);
            }
        } else {
            printf("\n=== verify: Roberti — SKIPPED (dir not found) ===\n");
        }
    }

    // ── RCSPP ng8/ng16/ng24 R1/RC1 (with ng-path resource) ──
    for (auto& ng : {"ng8"}) {
        std::string dir = std::string("benchmarks/rcspp/rcspp/") + ng;
        if (fs::exists(dir)) {
            printf("\n=== verify: RCSPP %s (R1/RC1, ng-path) ===\n", ng);

            std::vector<std::string> names;
            for (auto& entry : fs::directory_iterator(dir)) {
                if (entry.path().extension() != ".graph") continue;
                std::string name = entry.path().stem().string();
                bool is_r1 = (name[0] == 'R' && name[1] == '1');
                bool is_rc1 = name.starts_with("RC1");
                if (!is_r1 && !is_rc1) continue;
                names.push_back(name);
            }
            std::sort(names.begin(), names.end());

            for (auto& name : names) {
                auto inst = io::load_rcspp_graph(dir + "/" + name + ".graph");
                auto pv = inst.problem_view();
                std::array<double, 2> steps = {20.0, 50.0};

                // Mono with ng-path
                NgPathResource ng_m(pv.n_vertices, pv.n_arcs, pv.arc_from, pv.arc_to, inst.ng_neighbors);
                Solver<NgPack> mono(pv, make_resource_pack(std::move(ng_m)),
                    {.bucket_steps = steps, .tolerance = 0.0});
                mono.build();
                mono.set_stage(Stage::Exact);
                auto mono_paths = mono.solve();
                double mono_best = mono_paths.empty() ? 1e18 : mono_paths[0].reduced_cost;

                // Bidir with ng-path
                NgPathResource ng_b(pv.n_vertices, pv.n_arcs, pv.arc_from, pv.arc_to, inst.ng_neighbors);
                Solver<NgPack> bidir(pv, make_resource_pack(std::move(ng_b)),
                    {.bucket_steps = steps, .bidirectional = true, .tolerance = 0.0});
                bidir.build();
                bidir.set_stage(Stage::Exact);
                auto bidir_paths = bidir.solve();
                double bidir_best = bidir_paths.empty() ? 1e18 : bidir_paths[0].reduced_cost;

                bool match_ok = std::abs(mono_best - bidir_best) < 1.0 ||
                                (mono_best >= 0.0 && bidir_best >= -1e-6);
                bool pass = match_ok;
                if (pass) ++total_pass; else ++total_fail;

                printf("  %-25s  mono=%12.3f  bidir=%12.3f  %s",
                       name.c_str(), mono_best, bidir_best, pass ? "PASS" : "FAIL");
                if (!pass) printf("  [match]");
                printf("\n");
                write_csv_row(csv_out, name, mono_best, bidir_best, 0.0, pass);
            }
        } else {
            printf("\n=== verify: RCSPP %s — SKIPPED (dir not found) ===\n", ng);
        }
    }

    // ── Summary ──
    printf("\n════════════════════════════════════\n");
    printf("  VERIFY TOTAL: %d passed, %d failed\n", total_pass, total_fail);
    printf("════════════════════════════════════\n\n");

    return total_fail > 0 ? 1 : 0;
}

int main(int argc, char** argv) {
    std::string mode = "all";
    std::string csv_path;
    if (argc > 1) mode = argv[1];
    for (int i = 2; i < argc; ++i) {
        if (std::string(argv[i]) == "--csv" && i + 1 < argc)
            csv_path = argv[++i];
    }

    bool all = (mode == "all");
    if (all || mode == "spprclib")   run_spprclib();
    if (all || mode == "roberti")    run_roberti();
    if (all || mode == "rcspp")      run_rcspp();
    if (all || mode == "bidir")      { run_bidir_rcspp(); run_bidir_spprclib(); run_bidir_roberti(); }
    if (all || mode == "stages")     run_stages();
    if (all || mode == "eliminate")  run_elimination();
    if (all || mode == "fixing")     run_bucket_fixing();
    if (all || mode == "validate")   run_path_validation();
    if (mode == "verify")            return run_verify(csv_path);
    return 0;
}
