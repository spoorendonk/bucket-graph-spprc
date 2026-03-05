/// CLI for comparing our solver against PathWyse.
///
/// Usage: ./build/bench_vs_pathwyse <instance.sppcc> <ng_size>
///
/// Runs both mono and bidir solver, outputs:
///   RESULT_MONO <name> <ng_size> <cost> <time_ms> <n_paths>
///   RESULT_BIDIR <name> <ng_size> <cost> <time_ms> <n_paths>

#include "instance_io.h"

#include <bgspprc/solver.h>
#include <bgspprc/resource.h>
#include <bgspprc/resources/ng_path.h>

#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <string>

using namespace bgspprc;
using NgPack = ResourcePack<NgPathResource>;

int main(int argc, char** argv) {
    if (argc < 3) {
        fprintf(stderr, "Usage: %s <instance.sppcc> <ng_size>\n", argv[0]);
        return 1;
    }

    std::string path = argv[1];
    int ng_size = std::atoi(argv[2]);

    // Detect format from extension
    bool is_vrp = path.ends_with(".vrp");
    auto inst = is_vrp ? io::load_roberti_vrp(path) : io::load_sppcc(path);
    io::compute_ng_neighbors(inst, ng_size);
    auto pv = inst.problem_view();

    // Extract name from filename (avoids spaces in inst.name)
    std::string name;
    {
        auto slash = path.rfind('/');
        auto dot = path.rfind('.');
        if (slash == std::string::npos) slash = 0; else slash++;
        if (dot == std::string::npos) dot = path.size();
        name = path.substr(slash, dot - slash);
    }

    // Mono
    {
        auto t0 = std::chrono::high_resolution_clock::now();
        NgPathResource ng(pv.n_vertices, pv.n_arcs, pv.arc_from, pv.arc_to, inst.ng_neighbors);
        Solver<NgPack> solver(pv, make_resource_pack(std::move(ng)),
            {.bucket_steps = {10.0, 1.0}, .tolerance = 1e9});
        solver.build();
        solver.set_stage(Stage::Exact);
        auto paths = solver.solve();
        auto t1 = std::chrono::high_resolution_clock::now();
        double ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
        double best = paths.empty() ? 1e18 : paths[0].reduced_cost;
        printf("RESULT_MONO %s %d %.6f %.1f %d\n", name.c_str(), ng_size, best, ms, (int)paths.size());
    }

    // Bidir
    {
        auto t0 = std::chrono::high_resolution_clock::now();
        NgPathResource ng(pv.n_vertices, pv.n_arcs, pv.arc_from, pv.arc_to, inst.ng_neighbors);
        Solver<NgPack> solver(pv, make_resource_pack(std::move(ng)),
            {.bucket_steps = {10.0, 1.0}, .bidirectional = true, .tolerance = 1e9});
        solver.build();
        solver.set_stage(Stage::Exact);
        auto paths = solver.solve();
        auto t1 = std::chrono::high_resolution_clock::now();
        double ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
        double best = paths.empty() ? 1e18 : paths[0].reduced_cost;
        printf("RESULT_BIDIR %s %d %.6f %.1f %d\n", name.c_str(), ng_size, best, ms, (int)paths.size());
    }

    return 0;
}
