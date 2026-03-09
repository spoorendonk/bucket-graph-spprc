cmake_minimum_required(VERSION 3.14)
project(bgspprc LANGUAGES CXX)

# Header-only INTERFACE library
add_library(bgspprc INTERFACE)
target_include_directories(bgspprc INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
target_compile_features(bgspprc INTERFACE cxx_std_23)

# Tests (opt-in, off by default when consumed as submodule)
option(BGSPPRC_BUILD_TESTS "Build unit tests" ${PROJECT_IS_TOP_LEVEL})

if(BGSPPRC_BUILD_TESTS)
    include(FetchContent)
    FetchContent_Declare(
        doctest
        GIT_REPOSITORY https://github.com/doctest/doctest.git
        GIT_TAG v2.4.12)
    FetchContent_MakeAvailable(doctest)

    enable_testing()

    # Compiler flags for project targets only (not propagated to consumers)
    add_compile_options(-Wall -Wextra -Wpedantic -O2)

    add_executable(test_runner
        tests/test_main.cpp
        tests/test_resource.cpp
        tests/test_bucket_graph.cpp
        tests/test_small_instance.cpp)
    target_link_libraries(test_runner PRIVATE bgspprc doctest::doctest)
    add_test(NAME unit_tests COMMAND test_runner)

    add_executable(bench_runner tests/test_main.cpp tests/test_benchmarks.cpp)
    target_link_libraries(bench_runner PRIVATE bgspprc doctest::doctest)

    add_executable(bench_run tests/bench_run.cpp)
    target_link_libraries(bench_run PRIVATE bgspprc doctest::doctest)

    add_executable(bench_vs_pathwyse tests/bench_vs_pathwyse.cpp)
    target_link_libraries(bench_vs_pathwyse PRIVATE bgspprc doctest::doctest)
endif()
