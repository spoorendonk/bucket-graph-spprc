/// bgspprc-solve — instance-driven CLI for the bucket-graph SPPRC solver.
///
/// Usage: bgspprc-solve [OPTIONS] <path>...
///
/// Arguments:
///   <path>    Instance file or directory (recurse, detect type by extension:
///             .sppcc, .vrp, .graph, .sppg)
///
/// Options:
///   --version       Print version and build git hash, then exit
///   --mono          Use mono solver (default: bidir)
///   --stage STAGE   heuristic1|heuristic2|exact (default: exact)
///   --ng K          ng-neighborhood size (default: 0/off for sppcc/vrp/sppg;
///                   from file or 8 for graph; 0 disables)
///   --ng-metric M   distance|cost — metric for ng neighborhoods
///                   (default: distance for sppcc/vrp, cost for graph)
///   --steps S1,S2   Bucket step sizes (default: per-type)
///   --xi X          Derive steps from ξ = X buckets per vertex (BG2021 §6.3)
///   --no-jump-arcs  Disable jump arcs (for ablation studies)
///   --max-paths N   Number of paths to return (0=all, 1=best; default: 1)
///   --theta T       Pricing threshold θ (default: -1e-6 for CG)
///   --stats         Print solve statistics after each instance
///   --csv           Machine-readable CSV output (all fields)
///   --timing        Print phase timing breakdown
///   --write-sppg DIR  Write each instance to DIR/<stem>.sppg instead of solving

#include "instance_io.h"
#include "side_resources.h"
#include "sppg_io.h"
#include "version.h"

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
#include <fstream>
#include <string>
#include <vector>

using namespace bgspprc;
namespace fs = std::filesystem;

using NgPack = ResourcePack<NgPathResource>;
using SidePack = ResourcePack<io::SideResources>;
using NgSidePack = ResourcePack<NgPathResource, io::SideResources>;

// ── Options & Result ──

struct Options {
    bool bidir = true;
    Stage stage = Stage::Exact;
    int ng = -1;                  // -1 = use file default or 8, 0 = disable ng-path
    int ng_metric = -1;           // -1 = auto: cost everywhere (parity with Pathwyse)
                                  //  0 = cost, 1 = distance
    double step1 = 0, step2 = 0;  // 0 = per-type default
    double xi = 0;                // > 0: library ξ rule instead of steps
    bool auto_steps = false;      // use per-vertex auto-computed steps
    bool no_jump_arcs = false;    // disable jump arcs (for ablation studies)
    double theta = NAN;           // NAN = use Solver default (-1e-6)
    int max_paths = 1;            // 0 = all, 1 = best only, N = top N
    bool stats = false;           // print solve statistics
    bool csv = false;             // machine-readable CSV output
    bool timing = false;          // print phase timing breakdown
    bool parallel = true;         // use parallel executor (StdThreadExecutor)
    bool parallel_bidir = true;   // concurrent fw/bw labeling (requires parallel)
    std::string write_sppg_dir;   // non-empty: convert to .sppg instead of solving
};

struct Result {
    std::string name;
    std::string type;
    std::string error;  // non-empty: the instance was not solved
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
    if (opts.xi > 0) {
        so.bucket_steps = {0.0, 0.0};
        so.xi = opts.xi;
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
        bool use_dist = opts.ng_metric == 1;  // auto (-1) defaults to cost (parity with Pathwyse)
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
        bool use_dist = opts.ng_metric == 1;  // auto (-1) defaults to cost (parity with Pathwyse)
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
    bool use_auto = (opts.step1 == 0 && opts.step2 == 0 && opts.xi == 0);

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
        int ng_k = 8;
        if (opts.ng > 0) {
            ng_k = opts.ng;
        } else if (inst.ng_size > 0) {
            ng_k = inst.ng_size;
        }
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

template <typename Pack, Executor Exec>
static void solve_into(Result& r, const ProblemView& pv, Pack pack, bool auto_steps,
                       const Options& opts, Exec executor) {
    r.n_verts = pv.n_vertices;
    r.n_arcs = pv.n_arcs;

    auto t0 = std::chrono::high_resolution_clock::now();
    auto so = make_solver_opts<Pack, Exec>(opts.step1, opts.step2, opts);
    r.theta = so.theta;
    Solver<Pack, Exec> solver(pv, std::move(pack), so, executor);
    if (auto_steps) {
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

/// Why bgspprc cannot solve a view read from .sppg, or empty if it can.
static std::string sppg_unsupported(const ProblemView& pv) {
    if (pv.n_main_resources < 1 || pv.n_main_resources > pv.n_resources) {
        return "needs 1 <= n_main_resources <= n_resources: the bucket grid is built on the "
               "main resources";
    }
    const int n_main = std::min(pv.n_main_resources, 2);
    if (pv.n_resources - n_main > io::kMaxSideResources) {
        return "more than " + std::to_string(io::kMaxSideResources) +
               " resources past the first two main ones";
    }
    for (int r = 0; r < n_main; ++r) {
        for (int v = 0; v < pv.n_vertices; ++v) {
            // The range too: [-1e308, 1e308] is finite, its width is not.
            if (!std::isfinite(pv.vertex_ub[r][v] - pv.vertex_lb[r][v])) {
                return "main resource " + std::to_string(r) +
                       " has an unbounded window at vertex " + std::to_string(v) +
                       ": the bucket grid needs finite windows";
            }
        }
    }
    return {};
}

/// Solve an .sppg instance. Its steps default to the library's ξ rule; there
/// are no ng sets in the format, so ng-path is off unless --ng K asks for
/// sets of the K cheapest outgoing arcs. Main resources past the first two,
/// and every non-main one, are enforced by io::SideResources.
template <Executor Exec>
static Result run_sppg(const std::string& path, const Options& opts, Exec executor = {}) {
    Result r;
    r.name = fs::path(path).stem().string();
    r.type = "sppg";

    auto inst = io::load_sppg(path);
    if (!inst) {
        r.error = inst.error();
        return r;
    }
    auto pv = inst->problem_view();
    r.error = sppg_unsupported(pv);
    if (!r.error.empty()) {
        return r;
    }

    const int side_first = std::min(pv.n_main_resources, 2);
    const int n_side = pv.n_resources - side_first;
    const bool ng = opts.ng > 0;
    if (ng) {
        if (opts.ng_metric == 1) {
            r.error = "--ng-metric distance needs coordinates, which sppg does not carry";
            return r;
        }
        io::compute_ng_neighbors(*inst, opts.ng, false);
    }
    auto make_ng = [&] {
        return NgPathResource(pv.n_vertices, pv.n_arcs, pv.arc_from, pv.arc_to, inst->ng_neighbors);
    };

    if (ng && n_side > 0) {
        solve_into(r, pv, make_resource_pack(make_ng(), io::SideResources(pv, side_first, n_side)),
                   opts.auto_steps, opts, executor);
    } else if (ng) {
        solve_into(r, pv, make_resource_pack(make_ng()), opts.auto_steps, opts, executor);
    } else if (n_side > 0) {
        solve_into(r, pv, make_resource_pack(io::SideResources(pv, side_first, n_side)),
                   opts.auto_steps, opts, executor);
    } else {
        solve_into(r, pv, EmptyPack{}, opts.auto_steps, opts, executor);
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
    if (ext == ".sppg") {
        return run_sppg(path, opts, executor);
    }
    Result r;
    r.name = fs::path(path).filename().string();
    r.error = "unknown extension";
    return r;
}

static bool is_instance_file(const fs::path& p) {
    const auto ext = p.extension().string();
    return ext == ".sppcc" || ext == ".vrp" || ext == ".graph" || ext == ".sppg";
}

/// Write the instance at `path` as `<dir>/<stem>.sppg`: the view the solver
/// would get, so a .graph file's ng sets are not carried over. Never
/// overwrites a file: two inputs sharing a stem, or an output that would
/// replace its own input, are refused. Returns an error message, or empty.
static std::string convert_to_sppg(const std::string& path, const std::string& dir) {
    const auto ext = fs::path(path).extension().string();
    io::Instance inst;
    if (ext == ".sppcc") {
        inst = io::load_sppcc(path);
    } else if (ext == ".vrp") {
        inst = io::load_roberti_vrp(path);
    } else if (ext == ".graph") {
        inst = io::load_rcspp_graph(path);
    } else if (ext == ".sppg") {
        auto read = io::load_sppg(path);
        if (!read) {
            return read.error();
        }
        inst = std::move(*read);
    } else {
        return "unknown extension";
    }
    const auto out_path = fs::path(dir) / (fs::path(path).stem().string() + ".sppg");
    if (fs::exists(out_path)) {
        return out_path.string() + " exists, not overwriting it";
    }
    std::ofstream out(out_path);
    if (!out) {
        return "cannot write " + out_path.string();
    }
    out << "c converted by bgspprc-solve from " << fs::path(path).filename().string() << '\n';
    io::write_sppg(out, inst.problem_view());
    out.close();
    if (!out) {
        return "cannot write " + out_path.string();
    }
    std::printf("%s\n", out_path.string().c_str());
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
    if (r.name.empty() || !r.error.empty()) {
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

/// Solve (or convert) one file. Returns false when it failed.
template <Executor Exec>
static bool run_file(const std::string& path, const Options& opts, Exec executor) {
    // The .sppcc/.vrp/.graph loaders only assert that the file opened.
    if (!std::ifstream(path).is_open()) {
        std::fprintf(stderr, "%s: cannot open\n", path.c_str());
        return false;
    }
    if (!opts.write_sppg_dir.empty()) {
        auto error = convert_to_sppg(path, opts.write_sppg_dir);
        if (!error.empty()) {
            std::fprintf(stderr, "%s: %s\n", path.c_str(), error.c_str());
            return false;
        }
        return true;
    }
    auto r = run_instance(path, opts, executor);
    if (!r.error.empty()) {
        std::fprintf(stderr, "%s: %s\n", path.c_str(), r.error.c_str());
        return false;
    }
    print_result(r, opts);
    return true;
}

/// Returns the number of files that failed.
template <Executor Exec>
static int run_path_with(const std::string& path, const Options& opts, Exec executor) {
    int failed = 0;
    if (fs::is_directory(path)) {
        std::vector<std::string> files;
        for (const auto& entry : fs::directory_iterator(path)) {
            if (is_instance_file(entry.path())) {
                files.push_back(entry.path().string());
            }
        }
        std::ranges::sort(files);
        for (auto& f : files) {
            failed += run_file(f, opts, executor) ? 0 : 1;
        }
    } else {
        failed += run_file(path, opts, executor) ? 0 : 1;
    }
    return failed;
}

static int run_path(const std::string& path, const Options& opts) {
    if (opts.parallel) {
        return run_path_with(path, opts, StdThreadExecutor{});
    }
    return run_path_with(path, opts, SequentialExecutor{});
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
        if (std::strcmp(argv[i], "--version") == 0) {
            std::printf("bgspprc %s (%s)\n", BGSPPRC_VERSION, BGSPPRC_GIT_HASH);
            return 0;
        }
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
        } else if (std::strcmp(argv[i], "--xi") == 0 && i + 1 < argc) {
            char* end = nullptr;
            opts.xi = std::strtod(argv[++i], &end);
            if (end == argv[i] || *end != '\0' || !(opts.xi > 0) || !std::isfinite(opts.xi)) {
                std::fprintf(stderr, "Invalid xi: %s (expected a positive number)\n", argv[i]);
                return 1;
            }
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
        } else if (std::strcmp(argv[i], "--write-sppg") == 0 && i + 1 < argc) {
            opts.write_sppg_dir = argv[++i];
        } else if (argv[i][0] == '-') {
            std::fprintf(stderr, "Unknown option: %s\n", argv[i]);
            return 1;
        } else {
            paths.emplace_back(argv[i]);
        }
    }

    if (opts.xi > 0 && (opts.step1 > 0 || opts.step2 > 0 || opts.auto_steps)) {
        std::fprintf(stderr, "--xi derives the steps: drop --steps and --auto-steps\n");
        return 1;
    }

    if (paths.empty()) {
        std::fprintf(stderr,
                     "Usage: bgspprc-solve [OPTIONS] <path>...\n"
                     "Options:\n"
                     "  --version       Print version and build git hash, then exit\n"
                     "  --mono          Use mono solver (default: bidir)\n"
                     "  --stage STAGE   heuristic1|heuristic2|exact (default: exact)\n"
                     "  --ng K          ng-neighborhood size (default: 0/off for sppcc/vrp/sppg;\n"
                     "                  from file or 8 for graph; 0 disables)\n"
                     "  --ng-metric M   distance|cost — ng neighbor metric\n"
                     "                  (default: cost — parity with Pathwyse buildNG)\n"
                     "  --steps S1,S2   Bucket step sizes\n"
                     "  --xi X          Derive steps from xi = X buckets per vertex (BG2021)\n"
                     "  --no-jump-arcs  Disable jump arcs (for ablation studies)\n"
                     "  --max-paths N   Number of paths to return (0=all, 1=best; default: 1)\n"
                     "  --theta T       Pricing threshold θ (default: -1e-6)\n"
                     "  --stats         Print solve statistics after each instance\n"
                     "  --csv           Machine-readable CSV output (all fields)\n"
                     "  --timing        Print phase timing breakdown\n"
                     "  --parallel      Use parallel executor (default: on)\n"
                     "  --no-parallel   Use sequential executor\n"
                     "  --no-parallel-bidir  Data-parallel only, sequential fw/bw labeling\n"
                     "  --write-sppg DIR  Write each instance to DIR/<stem>.sppg, no solve\n");
        return 1;
    }

    std::fprintf(stderr, "bgspprc %s (%s)\n", BGSPPRC_VERSION, BGSPPRC_GIT_HASH);

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
        const char* bidir_mode = "off";
        if (opts.bidir) {
            bidir_mode = opts.parallel && opts.parallel_bidir ? "parallel" : "sequential";
        }
        std::fprintf(stderr, "bgspprc: executor=%s  bidir=%s\n", exec_name, bidir_mode);
    }

    if (!opts.write_sppg_dir.empty() && !fs::is_directory(opts.write_sppg_dir)) {
        std::fprintf(stderr, "--write-sppg: %s is not a directory\n", opts.write_sppg_dir.c_str());
        return 1;
    }

    if (opts.csv && opts.write_sppg_dir.empty()) {
        print_csv_header();
    }
    int failed = 0;
    for (auto& p : paths) {
        failed += run_path(p, opts);
    }
    return failed > 0 ? 1 : 0;
}
