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
///   --ng K          ng-neighborhood size (default: from file or 8; 0 disables)
///   --steps S1,S2   Bucket step sizes (default: per-type)

#include <bgspprc/resource.h>
#include <bgspprc/resources/ng_path.h>
#include <bgspprc/solver.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <string>
#include <vector>

#include "instance_io.h"

using namespace bgspprc;
namespace fs = std::filesystem;

using NgPack = ResourcePack<NgPathResource>;

// ── Options & Result ──

struct Options {
  bool bidir = true;
  Stage stage = Stage::Exact;
  int ng = -1;  // -1 = use file default or 8, 0 = disable ng-path
  double step1 = 0, step2 = 0;  // 0 = per-type default
};

struct Result {
  std::string name;
  std::string type;
  int n_verts = 0;
  int n_arcs = 0;
  double cost = 0;
  int n_paths = 0;
  double ms = 0;
  std::vector<int> best_path;
};

// ── Per-type runners ──

static Result run_sppcc(const std::string& path, const Options& opts) {
  auto inst = io::load_sppcc(path);

  double s1 = opts.step1 > 0 ? opts.step1 : 10.0;
  double s2 = opts.step2 > 0 ? opts.step2 : 1.0;

  Result r;
  r.name = fs::path(path).stem().string();
  r.type = "sppcc";

  if (opts.ng > 0) {
    io::compute_ng_neighbors(inst, opts.ng);
    auto pv = inst.problem_view();
    r.n_verts = pv.n_vertices;
    r.n_arcs = pv.n_arcs;

    NgPathResource ng(pv.n_vertices, pv.n_arcs, pv.arc_from, pv.arc_to,
                      inst.ng_neighbors);

    auto t0 = std::chrono::high_resolution_clock::now();
    Solver<NgPack> solver(pv, make_resource_pack(std::move(ng)),
                          {.bucket_steps = {s1, s2},
                           .bidirectional = opts.bidir,
                           .tolerance = 1e9});
    solver.build();
    solver.set_stage(opts.stage);
    auto paths = solver.solve();
    auto t1 = std::chrono::high_resolution_clock::now();

    r.n_paths = static_cast<int>(paths.size());
    r.ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
    if (!paths.empty()) {
      r.cost = paths[0].reduced_cost;
      r.best_path = paths[0].vertices;
    }
  } else {
    auto pv = inst.problem_view();
    r.n_verts = pv.n_vertices;
    r.n_arcs = pv.n_arcs;

    auto t0 = std::chrono::high_resolution_clock::now();
    Solver<EmptyPack> solver(pv, EmptyPack{},
                             {.bucket_steps = {s1, s2},
                              .bidirectional = opts.bidir,
                              .tolerance = 1e9});
    solver.build();
    solver.set_stage(opts.stage);
    auto paths = solver.solve();
    auto t1 = std::chrono::high_resolution_clock::now();

    r.n_paths = static_cast<int>(paths.size());
    r.ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
    if (!paths.empty()) {
      r.cost = paths[0].reduced_cost;
      r.best_path = paths[0].vertices;
    }
  }
  return r;
}

static Result run_vrp(const std::string& path, const Options& opts) {
  auto inst = io::load_roberti_vrp(path);

  double s1 = opts.step1 > 0 ? opts.step1 : 10.0;
  double s2 = opts.step2 > 0 ? opts.step2 : 1.0;

  Result r;
  r.name = fs::path(path).stem().string();
  r.type = "vrp";

  if (opts.ng > 0) {
    io::compute_ng_neighbors(inst, opts.ng);
    auto pv = inst.problem_view();
    r.n_verts = pv.n_vertices;
    r.n_arcs = pv.n_arcs;

    NgPathResource ng(pv.n_vertices, pv.n_arcs, pv.arc_from, pv.arc_to,
                      inst.ng_neighbors);

    auto t0 = std::chrono::high_resolution_clock::now();
    Solver<NgPack> solver(pv, make_resource_pack(std::move(ng)),
                          {.bucket_steps = {s1, s2},
                           .bidirectional = opts.bidir,
                           .tolerance = 1e9});
    solver.build();
    solver.set_stage(opts.stage);
    auto paths = solver.solve();
    auto t1 = std::chrono::high_resolution_clock::now();

    r.n_paths = static_cast<int>(paths.size());
    r.ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
    if (!paths.empty()) {
      r.cost = paths[0].reduced_cost;
      r.best_path = paths[0].vertices;
    }
  } else {
    auto pv = inst.problem_view();
    r.n_verts = pv.n_vertices;
    r.n_arcs = pv.n_arcs;

    auto t0 = std::chrono::high_resolution_clock::now();
    Solver<EmptyPack> solver(pv, EmptyPack{},
                             {.bucket_steps = {s1, s2},
                              .bidirectional = opts.bidir,
                              .tolerance = 1e9});
    solver.build();
    solver.set_stage(opts.stage);
    auto paths = solver.solve();
    auto t1 = std::chrono::high_resolution_clock::now();

    r.n_paths = static_cast<int>(paths.size());
    r.ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
    if (!paths.empty()) {
      r.cost = paths[0].reduced_cost;
      r.best_path = paths[0].vertices;
    }
  }
  return r;
}

static Result run_graph(const std::string& path, const Options& opts) {
  auto inst = io::load_rcspp_graph(path);

  double s1 = opts.step1 > 0 ? opts.step1 : 20.0;
  double s2 = opts.step2 > 0 ? opts.step2 : 50.0;

  Result r;
  r.name = inst.name.empty() ? fs::path(path).stem().string() : inst.name;
  r.type = "graph";

  if (opts.ng == 0) {
    // --ng 0: disable ng-path, use EmptyPack
    auto pv = inst.problem_view();
    r.n_verts = pv.n_vertices;
    r.n_arcs = pv.n_arcs;

    auto t0 = std::chrono::high_resolution_clock::now();
    Solver<EmptyPack> solver(pv, EmptyPack{},
                             {.bucket_steps = {s1, s2},
                              .bidirectional = opts.bidir,
                              .tolerance = 0.0});
    solver.build();
    solver.set_stage(opts.stage);
    auto paths = solver.solve();
    auto t1 = std::chrono::high_resolution_clock::now();

    r.n_paths = static_cast<int>(paths.size());
    r.ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
    if (!paths.empty()) {
      r.cost = paths[0].reduced_cost;
      r.best_path = paths[0].vertices;
    }
  } else {
    // Use ng-path resource
    int ng_k = opts.ng > 0 ? opts.ng : (inst.ng_size > 0 ? inst.ng_size : 8);
    if (opts.ng > 0 && opts.ng != inst.ng_size) {
      io::compute_ng_neighbors(inst, ng_k);
    }

    auto pv = inst.problem_view();
    r.n_verts = pv.n_vertices;
    r.n_arcs = pv.n_arcs;

    NgPathResource ng(pv.n_vertices, pv.n_arcs, pv.arc_from, pv.arc_to,
                      inst.ng_neighbors);

    auto t0 = std::chrono::high_resolution_clock::now();
    Solver<NgPack> solver(pv, make_resource_pack(std::move(ng)),
                          {.bucket_steps = {s1, s2},
                           .bidirectional = opts.bidir,
                           .tolerance = 0.0});
    solver.build();
    solver.set_stage(opts.stage);
    auto paths = solver.solve();
    auto t1 = std::chrono::high_resolution_clock::now();

    r.n_paths = static_cast<int>(paths.size());
    r.ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
    if (!paths.empty()) {
      r.cost = paths[0].reduced_cost;
      r.best_path = paths[0].vertices;
    }
  }
  return r;
}

// ── Dispatch ──

static Result run_instance(const std::string& path, const Options& opts) {
  auto ext = fs::path(path).extension().string();
  if (ext == ".sppcc") return run_sppcc(path, opts);
  if (ext == ".vrp") return run_vrp(path, opts);
  if (ext == ".graph") return run_graph(path, opts);
  std::fprintf(stderr, "Unknown extension: %s\n", path.c_str());
  return {};
}

static void print_result(const Result& r) {
  if (r.name.empty()) return;
  std::printf("%-30s  %-5s  n=%-4d  arcs=%-6d  cost=%.3f  paths=%-3d  %.1fms\n",
              r.name.c_str(), r.type.c_str(), r.n_verts, r.n_arcs, r.cost,
              r.n_paths, r.ms);
  if (!r.best_path.empty()) {
    std::printf(" ");
    for (int v : r.best_path) std::printf(" %d", v);
    std::printf("\n");
  }
}

static void run_path(const std::string& path, const Options& opts) {
  if (fs::is_directory(path)) {
    std::vector<std::string> files;
    for (auto& entry : fs::directory_iterator(path)) {
      auto ext = entry.path().extension().string();
      if (ext == ".sppcc" || ext == ".vrp" || ext == ".graph")
        files.push_back(entry.path().string());
    }
    std::sort(files.begin(), files.end());
    for (auto& f : files) {
      auto r = run_instance(f, opts);
      print_result(r);
    }
  } else {
    auto r = run_instance(path, opts);
    print_result(r);
  }
}

// ── CLI parsing ──

static Stage parse_stage(const char* s) {
  if (std::strcmp(s, "heuristic1") == 0) return Stage::Heuristic1;
  if (std::strcmp(s, "heuristic2") == 0) return Stage::Heuristic2;
  if (std::strcmp(s, "exact") == 0) return Stage::Exact;
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
    } else if (std::strcmp(argv[i], "--steps") == 0 && i + 1 < argc) {
      parse_steps(argv[++i], opts.step1, opts.step2);
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
                 "  --ng K          ng-neighborhood size (default: from file or 8; 0 disables)\n"
                 "  --steps S1,S2   Bucket step sizes\n");
    return 1;
  }

  for (auto& p : paths) run_path(p, opts);
  return 0;
}
