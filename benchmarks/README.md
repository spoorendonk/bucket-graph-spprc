# Benchmarks

Benchmark suite for bgspprc: runs solver on standard SPPRC/VRPTW instance sets,
verifies optimal costs, and compares runtimes against published results.

## Directory structure

```
benchmarks/
├── instances/
│   ├── spprclib/          45 SPPRC instances (.sppcc), from cptp repo
│   │   ├── optimal.csv            reference costs (no ng)
│   │   ├── optimal-ng{8,16,24}.csv  reference costs with ng neighborhoods
│   │   └── *.sppcc               instance files
│   ├── roberti/           31 VRPTW instances (.vrp), from cptp repo
│   │   ├── optimal.csv
│   │   ├── optimal-ng{8,16,24}.csv
│   │   └── *.vrp
│   └── rcspp/             56 Solomon RCSPP instances (.graph), from rcspp_dataset
│       ├── ng8/           *.graph (optimal = 0 for all)
│       ├── ng16/
│       └── ng24/
├── bgspprc.csv            unified results (all sets, all ng values)
├── comparison_rcspp.csv   runtime comparison vs Petersen & Spoorendonk 2025 (rcspp only)
├── comparison_pathwyse.csv runtime comparison vs Pathwyse (generated)
├── pull_algo_runtimes.csv reference runtimes from Petersen & Spoorendonk 2025 (arXiv:2511.01397)
├── fetch_instances.sh     download all instance sets
├── run_benchmarks.sh      run solver, produce bgspprc.csv
├── run_comparison.sh      run solver on rcspp, compare vs paper
├── run_pathwyse.sh        build Pathwyse, convert instances, run comparison
├── convert_to_pathwyse.py convert .sppcc/.vrp/.graph to Pathwyse format
├── compare_mono_bidir.sh  compare mono vs bidir mode
└── check_optimal.sh       verify results against reference optima
```

## Instance sources

| Set | Format | Source |
|-----|--------|--------|
| spprclib | `.sppcc` | [cptp](https://github.com/spoorendonk/cptp) — SPPRC instances |
| roberti | `.vrp` | [cptp](https://github.com/spoorendonk/cptp) — Roberti VRPTW pricing instances |
| rcspp | `.graph` | [rcspp_dataset](https://github.com/spoorendonk/rcspp_dataset) — Solomon RCSPP instances |

## Quick start

```bash
# 1. Fetch instances (once)
./benchmarks/fetch_instances.sh

# 2. Build
cmake -B build -DCMAKE_CXX_COMPILER=g++-14
cmake --build build

# 3. Run all benchmarks
for ng in 8 16 24; do
  ./benchmarks/run_benchmarks.sh --ng $ng --timeout 120 benchmarks/instances/spprclib
  ./benchmarks/run_benchmarks.sh --ng $ng --timeout 120 benchmarks/instances/roberti
done

# 4. Verify
./benchmarks/check_optimal.sh
```

## Scripts

| Script | Description | Input | Output |
|--------|-------------|-------|--------|
| `fetch_instances.sh` | Download all instance sets | — | `instances/` |
| `run_benchmarks.sh` | Run solver on instances, deduplicate results | Instance files/dirs, `--ng K`, `--timeout S` | `bgspprc.csv` |
| `check_optimal.sh` | Verify costs against reference optima | `bgspprc.csv`, `optimal*.csv` | Pass/fail table |
| `run_comparison.sh` | Compare rcspp runtimes vs Petersen & Spoorendonk 2025 | `instances/rcspp/`, `pull_algo_runtimes.csv` | `comparison_rcspp.csv` |
| `run_pathwyse.sh` | Build Pathwyse, convert instances, compare both solvers | Instance files/dirs, `--ng K`, `--timeout S` | `comparison_pathwyse.csv` |
| `convert_to_pathwyse.py` | Convert instances to Pathwyse format | `.sppcc`/`.vrp`/`.graph` files | `instances/pathwyse/` |
| `compare_mono_bidir.sh` | Side-by-side mono vs bidir comparison | Instance files/dirs | Terminal table |

## How to reproduce

### Full benchmark run

```bash
# spprclib + roberti at ng=8,16,24
for ng in 8 16 24; do
  ./benchmarks/run_benchmarks.sh --ng $ng --timeout 120 benchmarks/instances/spprclib
  ./benchmarks/run_benchmarks.sh --ng $ng --timeout 120 benchmarks/instances/roberti
done

# rcspp at ng=8,16,24 (also populates bgspprc.csv)
for ng in 8 16 24; do
  ./benchmarks/run_benchmarks.sh --ng $ng --timeout 120 benchmarks/instances/rcspp/ng$ng
done
```

### Verification

```bash
./benchmarks/check_optimal.sh
# PASS = cost ≤ optimal + 0.001
# FAIL = cost too high or no result (timeout)
# SKIP = no reference optimal available
```

### Paper comparison (rcspp only)

```bash
./benchmarks/run_comparison.sh
# Compares bgspprc wall-clock times vs Petersen & Spoorendonk 2025 "Base" column
# Produces shifted geometric mean summary per ng group
```

### Pathwyse comparison

```bash
# Run on rcspp ng=8 instances (default)
./benchmarks/run_pathwyse.sh

# Run on specific instances with custom ng
./benchmarks/run_pathwyse.sh --ng 8 benchmarks/instances/rcspp/ng8

# Skip rebuilding Pathwyse (reuse previous build)
./benchmarks/run_pathwyse.sh --skip-build

# Run on all three datasets
./benchmarks/run_pathwyse.sh --ng 8 \
  benchmarks/instances/rcspp/ng8 \
  benchmarks/instances/spprclib \
  benchmarks/instances/roberti
```

Note: Pathwyse uses integer objectives internally (costs are truncated to int).
For fractional-cost instances (rcspp .graph), this introduces small rounding
differences (typically <10). The `cost_match` column uses a tolerance of 10.

## CSV columns

### `bgspprc.csv`

| Column | Description |
|--------|-------------|
| `instance` | Instance name (filename without extension) |
| `set` | Instance set (`spprclib`, `roberti`, `ng8`, `ng16`, `ng24`) |
| `ng` | ng-neighborhood size used |
| `cost` | Optimal cost found (empty on timeout/error) |
| `paths` | Number of optimal paths found |
| `time_s` | Wall-clock time in seconds |
| `timestamp` | UTC timestamp of the run |

### `comparison_rcspp.csv`

| Column | Description |
|--------|-------------|
| `instance` | Solomon instance name |
| `ng` | ng-neighborhood size (8, 16, or 24) |
| `bgspprc_s` | bgspprc wall-clock time in seconds |
| `paper_base_s` | Petersen & Spoorendonk 2025 "Base" time in seconds |
| `ratio` | `bgspprc_s / paper_base_s` |

### `comparison_pathwyse.csv`

| Column | Description |
|--------|-------------|
| `instance` | Instance name (filename without extension) |
| `ng` | ng-neighborhood size used |
| `bgspprc_s` | bgspprc wall-clock time in seconds |
| `pathwyse_s` | Pathwyse wall-clock time in seconds |
| `bgspprc_labels` | Number of paths found by bgspprc |
| `pathwyse_labels` | (empty — Pathwyse does not expose label counts) |
| `bgspprc_cost` | Optimal cost found by bgspprc |
| `pathwyse_cost` | Optimal cost found by Pathwyse (int-truncated) |
| `cost_match` | YES if costs agree within tolerance, NO otherwise |
| `ratio` | `bgspprc_s / pathwyse_s` |
