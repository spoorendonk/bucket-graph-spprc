/// Run ALL SPPRCLIB instances (no node limit), mono + bidir.
/// Sorted by node count ascending.
/// Usage: ./build/run_spprclib_all

#include <bgspprc/resource.h>
#include <bgspprc/resources/ng_path.h>
#include <bgspprc/solver.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <map>
#include <string>
#include <vector>

#include "instance_io.h"

using namespace bgspprc;
namespace fs = std::filesystem;

using NgPack = ResourcePack<NgPathResource>;

static std::map<std::string, double> load_csv(const std::string& path) {
  std::map<std::string, double> result;
  std::ifstream file(path);
  if (!file.is_open()) return result;
  std::string line;
  while (std::getline(file, line)) {
    if (line.empty() || line[0] == '#' ||
        line.find("instance") != std::string::npos)
      continue;
    auto comma = line.find(',');
    if (comma == std::string::npos) continue;
    result[line.substr(0, comma)] = std::stod(line.substr(comma + 1));
  }
  return result;
}

int main() {
  std::string dir = "../rcspp-bac/benchmarks/instances/spprclib";
  if (!fs::exists(dir)) {
    printf("spprclib dir not found\n");
    return 1;
  }

  auto optima = load_csv(dir + "/optimal.csv");

  // Collect instances with node counts for sorting
  struct Entry {
    std::string name;
    double expected;
    int n_vertices;
  };
  std::vector<Entry> entries;

  for (auto& e : fs::directory_iterator(dir)) {
    if (e.path().extension() != ".sppcc") continue;
    std::string name = e.path().stem().string();
    auto it = optima.find(name);
    double expected = (it != optima.end()) ? it->second : 0.0;
    if (expected >= -3.0) continue;  // skip trivial

    auto inst = io::load_sppcc(e.path().string());
    entries.push_back({name, expected, inst.n_vertices});
  }

  // Sort by node count ascending
  std::sort(entries.begin(), entries.end(),
            [](auto& a, auto& b) { return a.n_vertices < b.n_vertices; });

  printf("%-25s %5s %12s %12s %12s %10s %10s %6s %5s\n", "Instance", "N",
         "Optimal", "Mono", "Bidir", "Mono ms", "Bidir ms", "Paths", "OK?");
  printf("%s\n", std::string(110, '-').c_str());

  int pass = 0, fail = 0;

  for (auto& entry : entries) {
    auto inst = io::load_sppcc(dir + "/" + entry.name + ".sppcc");
    io::compute_ng_neighbors(inst, 8);
    auto pv = inst.problem_view();

    // Mono (with ng-path)
    auto t0 = std::chrono::high_resolution_clock::now();
    NgPathResource ng_m(pv.n_vertices, pv.n_arcs, pv.arc_from, pv.arc_to,
                        inst.ng_neighbors);
    Solver<NgPack> mono(pv, make_resource_pack(std::move(ng_m)),
                        {.bucket_steps = {10.0, 1.0}, .tolerance = 1e9});
    mono.build();
    mono.set_stage(Stage::Exact);
    auto mono_paths = mono.solve();
    auto t1 = std::chrono::high_resolution_clock::now();
    double mono_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
    double mono_best = mono_paths.empty() ? 1e18 : mono_paths[0].reduced_cost;

    // Bidir (with ng-path)
    t0 = std::chrono::high_resolution_clock::now();
    NgPathResource ng_b(pv.n_vertices, pv.n_arcs, pv.arc_from, pv.arc_to,
                        inst.ng_neighbors);
    Solver<NgPack> bidir(
        pv, make_resource_pack(std::move(ng_b)),
        {.bucket_steps = {10.0, 1.0}, .bidirectional = true, .tolerance = 1e9});
    bidir.build();
    bidir.set_stage(Stage::Exact);
    auto bidir_paths = bidir.solve();
    t1 = std::chrono::high_resolution_clock::now();
    double bidir_ms =
        std::chrono::duration<double, std::milli>(t1 - t0).count();
    double bidir_best =
        bidir_paths.empty() ? 1e18 : bidir_paths[0].reduced_cost;

    // Checks
    bool opt_ok = mono_best <= entry.expected + 1.0;
    bool match_ok = std::abs(mono_best - bidir_best) < 1.0;
    bool ok = opt_ok && match_ok;

    if (ok)
      ++pass;
    else
      ++fail;

    printf("%-25s %5d %12.0f %12.0f %12.0f %10.1f %10.1f %6d %5s",
           entry.name.c_str(), pv.n_vertices, entry.expected, mono_best,
           bidir_best, mono_ms, bidir_ms, (int)mono_paths.size(),
           ok ? "OK" : "FAIL");
    if (!ok) {
      if (!opt_ok) printf(" [opt]");
      if (!match_ok) printf(" [match]");
    }
    printf("\n");
    fflush(stdout);
  }

  printf("\n%s\n", std::string(110, '=').c_str());
  printf("SPPRCLIB ALL: %d passed, %d failed out of %d\n", pass, fail,
         pass + fail);
  printf("%s\n", std::string(110, '=').c_str());

  return fail > 0 ? 1 : 0;
}
