CXX      := g++-14
CXXFLAGS := -std=c++23 -Wall -Wextra -Wpedantic -O2 -I include
LDFLAGS  :=

# Header-only library — "build" just verifies compilation
.PHONY: all test bench clean

all: build/test_runner

build/test_runner: tests/test_main.cpp tests/test_resource.cpp tests/test_bucket_graph.cpp tests/test_small_instance.cpp | build
	$(CXX) $(CXXFLAGS) $^ -o $@ $(LDFLAGS)

build/bench_runner: tests/test_main.cpp tests/test_benchmarks.cpp | build
	$(CXX) $(CXXFLAGS) $^ -o $@ $(LDFLAGS)

build/bench_run: tests/bench_run.cpp | build
	$(CXX) $(CXXFLAGS) $^ -o $@ $(LDFLAGS)

build/bench_vs_pathwyse: tests/bench_vs_pathwyse.cpp | build
	$(CXX) $(CXXFLAGS) $^ -o $@ $(LDFLAGS)

build:
	mkdir -p build

test: build/test_runner
	./build/test_runner

bench: build/bench_runner
	./build/bench_runner

clean:
	rm -rf build
