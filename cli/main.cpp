/// bgspprc-solve — instance-driven CLI for the bucket-graph SPPRC solver.
///
/// Usage: bgspprc-solve [OPTIONS] <path>...
///
/// Arguments:
///   <path>    Instance file or directory (recurse, detect type by extension)
///
/// Options:
///   --mono          Use mono solver (default: bidir)
///   --stage STAGE   heuristic1|heuristic2|exact (default: exact)
///   --ng K          ng-neighborhood size (default: 0/off for sppcc/vrp;
///                   from file or 8 for graph; 0 disables)
///   --ng-metric M   distance|cost — metric for ng neighborhoods
///                   (default: distance for sppcc/vrp, cost for graph)
///   --steps S1,S2   Bucket step sizes (default: per-type)
///   --no-jump-arcs  Disable jump arcs (for ablation studies)
///   --max-paths N   Number of paths to return (0=all, 1=best; default: 1)
///   --theta T       Pricing threshold θ (default: -1e-6 for CG)
///   --stats         Print solve statistics after each instance
///   --csv           Machine-readable CSV output (all fields)
///   --timing        Print phase timing breakdown

#include "instance_io.h"

#include <algorithm>
#include <bgspprc/executor_thread.h>
#include <bgspprc/resource.h>
#include <bgspprc/resources/ng_path.h>
#include <bgspprc/solver.h>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <string>
#include <vector>

using namespace bgspprc;
namespace fs = std::filesystem;

using NgPack = ResourcePack<NgPathResource>;

// ── Options & Result ──

struct Options {
    bool bidir = true;
    Stage stage = Stage::Exact;
    int ng = -1;                  // -1 = use file default or 8, 0 = disable ng-path
    int ng_metric = -1;           // -1 = auto (distance for sppcc/vrp, cost for graph)
                                  //  0 = cost, 1 = distance
    double step1 = 0, step2 = 0;  // 0 = per-type default
    bool auto_steps = false;      // use per-vertex auto-computed steps
    bool no_jump_arcs = false;    // disable jump arcs (for ablation studies)
    double theta = NAN;           // NAN = use Solver default (-1e-6)
    int max_paths = 1;            // 0 = all, 1 = best only, N = top N
    bool stats = false;           // print solve statistics
    bool csv = false;             // machine-readable CSV output
    bool timing = false;          // print phase timing breakdown
    bool parallel = true;         // use parallel executor (StdThreadExecutor)
    bool parallel_bidir = true;   // concurrent fw/bw labeling (requires parallel)
};

struct Result {
    std::string name;
    std::string type;
    int n_verts = 0;
    int n_arcs = 0;
    double theta = 0;
    double cost = 0;
    int n_paths = 0;
    double ms = 0;
    std::vector<int> best_path;

    // Solve statistics (populated when --stats or --csv is set)
    int n_buckets = 0;
    int64_t n_labels_created = 0;
    int64_t n_dominance_checks = 0;
    int64_t n_non_dominated = 0;
    // Per-direction counters (fw / bw). bw is 0 for mono.
    int64_t n_dom_fw = 0;
    int64_t n_dom_bw = 0;
    int64_t n_nondom_fw = 0;
    int64_t n_nondom_bw = 0;
    int n_fixed_buckets = 0;
    int64_t n_eliminated_arcs = 0;
    std::size_t label_state_bytes = 0;

    SolveTimings timings;
};

// ── Helpers ──

template <typename Pack, Executor Exec>
typename Solver<Pack, Exec>::Options make_solver_opts(double s1, double s2, const Options& opts) {
    typename Solver<Pack, Exec>::Options so{
        .bucket_steps = {s1, s2},
        .bidirectional = opts.bidir,
        .no_jump_arcs = opts.no_jump_arcs,
        .parallel_bidir = opts.parallel_bidir,
        .max_paths = opts.max_paths,
    };
    if (!std::isnan(opts.theta)) {
        so.theta = opts.theta;
    }
    return so;
}

template <typename P, Executor Exec>
void apply_auto_steps(Solver<P, Exec>& solver) {
    auto steps = solver.compute_min_inbound_arc_resource();
    solver.set_vertex_bucket_steps(std::move(steps));
}

template <typename P, Executor Exec>
void collect_stats(const Solver<P, Exec>& solver, Result& r) {
    r.n_buckets = solver.n_buckets();
    r.n_labels_created = solver.labels_created();
    r.n_dominance_checks = solver.dominance_checks();
    r.n_non_dominated = solver.non_dominated_labels();
    r.n_dom_fw = solver.dominance_checks(Direction::Forward);
    r.n_dom_bw = solver.dominance_checks(Direction::Backward);
    r.n_nondom_fw = solver.non_dominated_labels(Direction::Forward);
    r.n_nondom_bw = solver.non_dominated_labels(Direction::Backward);
    r.n_fixed_buckets = solver.n_fixed_buckets();
    r.n_eliminated_arcs = solver.eliminated_bucket_arcs();
    r.label_state_bytes = Solver<P, Exec>::label_state_size();
}

// ── Per-type runners ──

template <Executor Exec>
static Result run_sppcc(const std::string& path, const Options& opts, Exec executor = {}) {
    auto inst = io::load_sppcc(path);

    double s1 = opts.step1 > 0 ? opts.step1 : 10.0;
    double s2 = opts.step2 > 0 ? opts.step2 : 1.0;

    Result r;
    r.name = fs::path(path).stem().string();
    r.type = "sppcc";

    if (opts.ng > 0) {
        bool use_dist = opts.ng_metric != 0;  // auto (-1) or explicit distance (1)
        io::compute_ng_neighbors(inst, opts.ng, use_dist);
        auto pv = inst.problem_view();
        r.n_verts = pv.n_vertices;
        r.n_arcs = pv.n_arcs;

        NgPathResource ng(pv.n_vertices, pv.n_arcs, pv.arc_from, pv.arc_to, inst.ng_neighbors);

        auto t0 = std::chrono::high_resolution_clock::now();
        auto so = make_solver_opts<NgPack, Exec>(s1, s2, opts);
        r.theta = so.theta;
        Solver<NgPack, Exec> solver(pv, make_resource_pack(std::move(ng)), so, executor);
        if (opts.auto_steps) {
            apply_auto_steps(solver);
        }
        solver.build();
        solver.set_stage(opts.stage);
        auto paths = solver.solve();
        auto t1 = std::chrono::high_resolution_clock::now();

        r.timings = solver.solve_timings();
        r.n_paths = static_cast<int>(paths.size());
        r.ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
        if (!paths.empty()) {
            r.cost = paths[0].reduced_cost;
            r.best_path = paths[0].vertices;
        }
        if (opts.stats || opts.csv) {
            collect_stats(solver, r);
        }
    } else {
        auto pv = inst.problem_view();
        r.n_verts = pv.n_vertices;
        r.n_arcs = pv.n_arcs;

        auto t0 = std::chrono::high_resolution_clock::now();
        auto so = make_solver_opts<EmptyPack, Exec>(s1, s2, opts);
        r.theta = so.theta;
        Solver<EmptyPack, Exec> solver(pv, EmptyPack{}, so, executor);
        if (opts.auto_steps) {
            apply_auto_steps(solver);
        }
        solver.build();
        solver.set_stage(opts.stage);
        auto paths = solver.solve();
        auto t1 = std::chrono::high_resolution_clock::now();

        r.timings = solver.solve_timings();
        r.n_paths = static_cast<int>(paths.size());
        r.ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
        if (!paths.empty()) {
            r.cost = paths[0].reduced_cost;
            r.best_path = paths[0].vertices;
        }
        if (opts.stats || opts.csv) {
            collect_stats(solver, r);
        }
    }
    return r;
}

template <Executor Exec>
static Result run_vrp(const std::string& path, const Options& opts, Exec executor = {}) {
    auto inst = io::load_roberti_vrp(path);

    double s1 = opts.step1 > 0 ? opts.step1 : 10.0;
    double s2 = opts.step2 > 0 ? opts.step2 : 1.0;

    Result r;
    r.name = fs::path(path).stem().string();
    r.type = "vrp";

    if (opts.ng > 0) {
        bool use_dist = opts.ng_metric != 0;  // auto (-1) or explicit distance (1)
        io::compute_ng_neighbors(inst, opts.ng, use_dist);
        auto pv = inst.problem_view();
        r.n_verts = pv.n_vertices;
        r.n_arcs = pv.n_arcs;

        NgPathResource ng(pv.n_vertices, pv.n_arcs, pv.arc_from, pv.arc_to, inst.ng_neighbors);

        auto t0 = std::chrono::high_resolution_clock::now();
        auto so = make_solver_opts<NgPack, Exec>(s1, s2, opts);
        r.theta = so.theta;
        Solver<NgPack, Exec> solver(pv, make_resource_pack(std::move(ng)), so, executor);
        if (opts.auto_steps) {
            apply_auto_steps(solver);
        }
        solver.build();
        solver.set_stage(opts.stage);
        auto paths = solver.solve();
        auto t1 = std::chrono::high_resolution_clock::now();

        r.timings = solver.solve_timings();
        r.n_paths = static_cast<int>(paths.size());
        r.ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
        if (!paths.empty()) {
            r.cost = paths[0].reduced_cost;
            r.best_path = paths[0].vertices;
        }
        if (opts.stats || opts.csv) {
            collect_stats(solver, r);
        }
    } else {
        auto pv = inst.problem_view();
        r.n_verts = pv.n_vertices;
        r.n_arcs = pv.n_arcs;

        auto t0 = std::chrono::high_resolution_clock::now();
        auto so = make_solver_opts<EmptyPack, Exec>(s1, s2, opts);
        r.theta = so.theta;
        Solver<EmptyPack, Exec> solver(pv, EmptyPack{}, so, executor);
        if (opts.auto_steps) {
            apply_auto_steps(solver);
        }
        solver.build();
        solver.set_stage(opts.stage);
        auto paths = solver.solve();
        auto t1 = std::chrono::high_resolution_clock::now();

        r.timings = solver.solve_timings();
        r.n_paths = static_cast<int>(paths.size());
        r.ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
        if (!paths.empty()) {
            r.cost = paths[0].reduced_cost;
            r.best_path = paths[0].vertices;
        }
        if (opts.stats || opts.csv) {
            collect_stats(solver, r);
        }
    }
    return r;
}

template <Executor Exec>
static Result run_graph(const std::string& path, const Options& opts, Exec executor = {}) {
    auto inst = io::load_rcspp_graph(path);

    // Default steps used as fallback; auto-compute overrides when no --steps
    double s1 = opts.step1 > 0 ? opts.step1 : 20.0;
    double s2 = opts.step2 > 0 ? opts.step2 : 50.0;
    bool use_auto = (opts.step1 == 0 && opts.step2 == 0);

    Result r;
    r.name = inst.name.empty() ? fs::path(path).stem().string() : inst.name;
    r.type = "graph";

    if (opts.ng == 0) {
        // --ng 0: disable ng-path, use EmptyPack
        auto pv = inst.problem_view();
        r.n_verts = pv.n_vertices;
        r.n_arcs = pv.n_arcs;

        auto t0 = std::chrono::high_resolution_clock::now();
        auto so = make_solver_opts<EmptyPack, Exec>(s1, s2, opts);
        r.theta = so.theta;
        Solver<EmptyPack, Exec> solver(pv, EmptyPack{}, so, executor);
        if (use_auto) {
            apply_auto_steps(solver);
        }
        solver.build();
        solver.set_stage(opts.stage);
        auto paths = solver.solve();
        auto t1 = std::chrono::high_resolution_clock::now();

        r.timings = solver.solve_timings();
        r.n_paths = static_cast<int>(paths.size());
        r.ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
        if (!paths.empty()) {
            r.cost = paths[0].reduced_cost;
            r.best_path = paths[0].vertices;
        }
        if (opts.stats || opts.csv) {
            collect_stats(solver, r);
        }
    } else {
        // Use ng-path resource
        int ng_k = opts.ng > 0 ? opts.ng : (inst.ng_size > 0 ? inst.ng_size : 8);
        if (inst.ng_neighbors.empty() || (opts.ng > 0 && opts.ng != inst.ng_size)) {
            bool use_dist = opts.ng_metric == 1;  // auto (-1) defaults to cost for graph
            io::compute_ng_neighbors(inst, ng_k, use_dist);
        }

        auto pv = inst.problem_view();
        r.n_verts = pv.n_vertices;
        r.n_arcs = pv.n_arcs;

        NgPathResource ng(pv.n_vertices, pv.n_arcs, pv.arc_from, pv.arc_to, inst.ng_neighbors);

        auto t0 = std::chrono::high_resolution_clock::now();
        auto so = make_solver_opts<NgPack, Exec>(s1, s2, opts);
        r.theta = so.theta;
        Solver<NgPack, Exec> solver(pv, make_resource_pack(std::move(ng)), so, executor);
        if (use_auto) {
            apply_auto_steps(solver);
        }
        solver.build();
        solver.set_stage(opts.stage);
        auto paths = solver.solve();
        auto t1 = std::chrono::high_resolution_clock::now();

        r.timings = solver.solve_timings();
        r.n_paths = static_cast<int>(paths.size());
        r.ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
        if (!paths.empty()) {
            r.cost = paths[0].reduced_cost;
            r.best_path = paths[0].vertices;
        }
        if (opts.stats || opts.csv) {
            collect_stats(solver, r);
        }
    }
    return r;
}

// ── Dispatch ──

template <Executor Exec>
static Result run_instance(const std::string& path, const Options& opts, Exec executor = {}) {
    auto ext = fs::path(path).extension().string();
    if (ext == ".sppcc") {
        return run_sppcc(path, opts, executor);
    }
    if (ext == ".vrp") {
        return run_vrp(path, opts, executor);
    }
    if (ext == ".graph") {
        return run_graph(path, opts, executor);
    }
    std::fprintf(stderr, "Unknown extension: %s\n", path.c_str());
    return {};
}

static void print_csv_header() {
    std::printf(
        "name,type,n_verts,n_arcs,theta,cost,n_paths,ms,"
        "n_buckets,n_labels_created,n_dominance_checks,"
        "n_non_dominated,n_fixed_buckets,n_eliminated_arcs,"
        "label_state_bytes,"
        "n_dominance_checks_fw,n_dominance_checks_bw,"
        "n_non_dominated_fw,n_non_dominated_bw\n");
}

static void print_result(const Result& r, const Options& opts) {
    if (r.name.empty()) {
        return;
    }

    if (opts.csv) {
        std::printf(
            "%s,%s,%d,%d,%.3g,%.3f,%d,%.1f,"
            "%d,%lld,%lld,%lld,%d,%lld,%zu,"
            "%lld,%lld,%lld,%lld\n",
            r.name.c_str(), r.type.c_str(), r.n_verts, r.n_arcs, r.theta, r.cost, r.n_paths, r.ms,
            r.n_buckets, static_cast<long long>(r.n_labels_created),
            static_cast<long long>(r.n_dominance_checks), static_cast<long long>(r.n_non_dominated),
            r.n_fixed_buckets, static_cast<long long>(r.n_eliminated_arcs), r.label_state_bytes,
            static_cast<long long>(r.n_dom_fw), static_cast<long long>(r.n_dom_bw),
            static_cast<long long>(r.n_nondom_fw), static_cast<long long>(r.n_nondom_bw));
        return;
    }

    std::printf("%-30s  %-5s  n=%-4d  arcs=%-6d  theta=%.3g  cost=%.3f  paths=%-3d  %.1fms\n",
                r.name.c_str(), r.type.c_str(), r.n_verts, r.n_arcs, r.theta, r.cost, r.n_paths,
                r.ms);
    if (!r.best_path.empty()) {
        std::printf(" ");
        for (int v : r.best_path) {
            std::printf(" %d", v);
        }
        std::printf("\n");
    }
    if (opts.stats) {
        std::printf(
            "  n_buckets=%d  n_labels_created=%lld  "
            "n_dominance_checks=%lld  n_non_dominated=%lld\n",
            r.n_buckets, static_cast<long long>(r.n_labels_created),
            static_cast<long long>(r.n_dominance_checks),
            static_cast<long long>(r.n_non_dominated));
        std::printf(
            "  n_dominance_checks_fw=%lld  n_dominance_checks_bw=%lld  "
            "n_non_dominated_fw=%lld  n_non_dominated_bw=%lld\n",
            static_cast<long long>(r.n_dom_fw), static_cast<long long>(r.n_dom_bw),
            static_cast<long long>(r.n_nondom_fw), static_cast<long long>(r.n_nondom_bw));
        std::printf(
            "  n_fixed_buckets=%d  n_eliminated_arcs=%lld  "
            "label_state_bytes=%zu\n",
            r.n_fixed_buckets, static_cast<long long>(r.n_eliminated_arcs), r.label_state_bytes);
    }
    if (opts.timing) {
        const auto& t = r.timings;
        std::printf(
            "  timing:  fw=%.3fms  bw=%.3fms  completion=%.3fms"
            "  concat=%.3fms  paths=%.3fms  sum=%.3fms\n",
            t.forward_labeling.count(), t.backward_labeling.count(), t.completion_bounds.count(),
            t.concatenation.count(), t.path_extraction.count(), t.total().count());
    }
}

template <Executor Exec>
static void run_path_with(const std::string& path, const Options& opts, Exec executor) {
    if (fs::is_directory(path)) {
        std::vector<std::string> files;
        for (auto& entry : fs::directory_iterator(path)) {
            auto ext = entry.path().extension().string();
            if (ext == ".sppcc" || ext == ".vrp" || ext == ".graph") {
                files.push_back(entry.path().string());
            }
        }
        std::sort(files.begin(), files.end());
        for (auto& f : files) {
            auto r = run_instance(f, opts, executor);
            print_result(r, opts);
        }
    } else {
        auto r = run_instance(path, opts, executor);
        print_result(r, opts);
    }
}

static void run_path(const std::string& path, const Options& opts) {
    if (opts.parallel) {
        run_path_with(path, opts, StdThreadExecutor{});
    } else {
        run_path_with(path, opts, SequentialExecutor{});
    }
}

// ── CLI parsing ──

static Stage parse_stage(const char* s) {
    if (std::strcmp(s, "heuristic1") == 0) {
        return Stage::Heuristic1;
    }
    if (std::strcmp(s, "heuristic2") == 0) {
        return Stage::Heuristic2;
    }
    if (std::strcmp(s, "exact") == 0) {
        return Stage::Exact;
    }
    std::fprintf(stderr, "Unknown stage: %s (using exact)\n", s);
    return Stage::Exact;
}

static void parse_steps(const char* s, double& s1, double& s2) {
    if (std::sscanf(s, "%lf,%lf", &s1, &s2) != 2) {
        std::fprintf(stderr, "Invalid steps: %s (expected S1,S2)\n", s);
        s1 = s2 = 0;
    }
}

int main(int argc, char** argv) {
    Options opts;
    std::vector<std::string> paths;

    for (int i = 1; i < argc; ++i) {
        if (std::strcmp(argv[i], "--mono") == 0) {
            opts.bidir = false;
        } else if (std::strcmp(argv[i], "--stage") == 0 && i + 1 < argc) {
            opts.stage = parse_stage(argv[++i]);
        } else if (std::strcmp(argv[i], "--ng") == 0 && i + 1 < argc) {
            opts.ng = std::atoi(argv[++i]);
        } else if (std::strcmp(argv[i], "--ng-metric") == 0 && i + 1 < argc) {
            ++i;
            if (std::strcmp(argv[i], "distance") == 0) {
                opts.ng_metric = 1;
            } else if (std::strcmp(argv[i], "cost") == 0) {
                opts.ng_metric = 0;
            } else {
                std::fprintf(stderr, "Unknown ng-metric: %s (use distance|cost)\n", argv[i]);
                return 1;
            }
        } else if (std::strcmp(argv[i], "--steps") == 0 && i + 1 < argc) {
            parse_steps(argv[++i], opts.step1, opts.step2);
        } else if (std::strcmp(argv[i], "--auto-steps") == 0) {
            opts.auto_steps = true;
        } else if (std::strcmp(argv[i], "--no-jump-arcs") == 0) {
            opts.no_jump_arcs = true;
        } else if (std::strcmp(argv[i], "--theta") == 0 && i + 1 < argc) {
            opts.theta = std::atof(argv[++i]);
        } else if (std::strcmp(argv[i], "--max-paths") == 0 && i + 1 < argc) {
            opts.max_paths = std::atoi(argv[++i]);
        } else if (std::strcmp(argv[i], "--stats") == 0) {
            opts.stats = true;
        } else if (std::strcmp(argv[i], "--csv") == 0) {
            opts.csv = true;
        } else if (std::strcmp(argv[i], "--timing") == 0) {
            opts.timing = true;
        } else if (std::strcmp(argv[i], "--parallel") == 0) {
            opts.parallel = true;
        } else if (std::strcmp(argv[i], "--no-parallel") == 0) {
            opts.parallel = false;
        } else if (std::strcmp(argv[i], "--no-parallel-bidir") == 0) {
            opts.parallel_bidir = false;
        } else if (argv[i][0] == '-') {
            std::fprintf(stderr, "Unknown option: %s\n", argv[i]);
            return 1;
        } else {
            paths.push_back(argv[i]);
        }
    }

    if (paths.empty()) {
        std::fprintf(stderr,
                     "Usage: bgspprc-solve [OPTIONS] <path>...\n"
                     "Options:\n"
                     "  --mono          Use mono solver (default: bidir)\n"
                     "  --stage STAGE   heuristic1|heuristic2|exact (default: exact)\n"
                     "  --ng K          ng-neighborhood size (default: 0/off for sppcc/vrp;\n"
                     "                  from file or 8 for graph; 0 disables)\n"
                     "  --ng-metric M   distance|cost — ng neighbor metric\n"
                     "                  (default: distance for sppcc/vrp, cost for graph)\n"
                     "  --steps S1,S2   Bucket step sizes\n"
                     "  --no-jump-arcs  Disable jump arcs (for ablation studies)\n"
                     "  --max-paths N   Number of paths to return (0=all, 1=best; default: 1)\n"
                     "  --theta T       Pricing threshold θ (default: -1e-6)\n"
                     "  --stats         Print solve statistics after each instance\n"
                     "  --csv           Machine-readable CSV output (all fields)\n"
                     "  --timing        Print phase timing breakdown\n"
                     "  --parallel      Use parallel executor (default: on)\n"
                     "  --no-parallel   Use sequential executor\n"
                     "  --no-parallel-bidir  Data-parallel only, sequential fw/bw labeling\n");
        return 1;
    }

    // Validate: parallel_bidir without parallel executor is meaningless.
    if (!opts.parallel && opts.parallel_bidir) {
        std::fprintf(stderr,
                     "bgspprc: warning: parallel_bidir has no effect without --parallel, "
                     "falling back to sequential\n");
        opts.parallel_bidir = false;
    }

    // Log effective configuration when --stats is set.
    if (opts.stats) {
        const char* exec_name = opts.parallel ? "StdThread" : "Sequential";
        const char* bidir_mode =
            !opts.bidir ? "off"
                        : (opts.parallel && opts.parallel_bidir ? "parallel" : "sequential");
        std::fprintf(stderr, "bgspprc: executor=%s  bidir=%s\n", exec_name, bidir_mode);
    }

    if (opts.csv) {
        print_csv_header();
    }
    for (auto& p : paths) {
        run_path(p, opts);
    }
    return 0;
}
