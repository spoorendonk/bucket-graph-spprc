cmake_minimum_required(VERSION 3.14)
project(bgspprc LANGUAGES CXX)

# Header-only INTERFACE library
add_library(bgspprc INTERFACE)
target_include_directories(bgspprc INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
target_compile_features(bgspprc INTERFACE cxx_std_23)

# CLI solver (opt-in, off by default when consumed as submodule)
option(BGSPPRC_BUILD_CLI "Build bgspprc-solve CLI" ${PROJECT_IS_TOP_LEVEL})

if(BGSPPRC_BUILD_CLI)
    add_executable(bgspprc-solve cli/main.cpp)
    target_link_libraries(bgspprc-solve PRIVATE bgspprc)
    target_include_directories(bgspprc-solve PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/cli)
    target_compile_options(bgspprc-solve PRIVATE -Wall -Wextra -Wpedantic -O2)
endif()

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

    if(BGSPPRC_BUILD_CLI)
        add_executable(test_cli tests/test_main.cpp tests/test_instance_io.cpp)
        target_link_libraries(test_cli PRIVATE bgspprc doctest::doctest)
        target_include_directories(test_cli PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/cli)
        target_compile_definitions(test_cli PRIVATE
            FIXTURE_DIR="${CMAKE_CURRENT_SOURCE_DIR}/tests/fixtures")
        add_test(NAME cli_unit_tests COMMAND test_cli)

        add_test(NAME cli_integration
            COMMAND bash ${CMAKE_CURRENT_SOURCE_DIR}/tests/test_cli.sh
                $<TARGET_FILE:bgspprc-solve>
                ${CMAKE_CURRENT_SOURCE_DIR}/tests/fixtures)
        set_tests_properties(cli_integration PROPERTIES TIMEOUT 30)
    endif()

    if(NOT EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/data")
        message(WARNING "Instance data not found. Run scripts/fetch_instances.sh to download benchmark instances.")
    endif()

    add_executable(bench_runner tests/test_main.cpp tests/test_benchmarks.cpp)
    target_link_libraries(bench_runner PRIVATE bgspprc doctest::doctest)
    target_include_directories(bench_runner PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/cli)
    target_compile_definitions(bench_runner PRIVATE
        DATA_DIR="${CMAKE_CURRENT_SOURCE_DIR}/data")

    add_executable(bench_run tests/bench_run.cpp)
    target_link_libraries(bench_run PRIVATE bgspprc doctest::doctest)
    target_include_directories(bench_run PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/cli)
    target_compile_definitions(bench_run PRIVATE
        DATA_DIR="${CMAKE_CURRENT_SOURCE_DIR}/data")

    add_executable(bench_vs_pathwyse tests/bench_vs_pathwyse.cpp)
    target_link_libraries(bench_vs_pathwyse PRIVATE bgspprc doctest::doctest)
    target_include_directories(bench_vs_pathwyse PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/cli)
endif()
