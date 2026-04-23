cmake_minimum_required(VERSION 3.25)
project(bgspprc LANGUAGES CXX)

# Emit compile_commands.json for clang-tidy and IDEs, but only when we
# are the top-level project — otherwise a downstream consumer embedding
# us via add_subdirectory could have its own compile db clobbered.
if(PROJECT_IS_TOP_LEVEL)
    set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
endif()

# Header-only INTERFACE library
add_library(bgspprc INTERFACE)
target_include_directories(bgspprc INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
target_compile_features(bgspprc INTERFACE cxx_std_23)

# Option to disable SIMD (for ablation studies)
option(BGSPPRC_DISABLE_SIMD "Disable SIMD in dominance checks" OFF)
if(BGSPPRC_DISABLE_SIMD)
    target_compile_definitions(bgspprc INTERFACE BGSPPRC_DISABLE_SIMD)
endif()

# Detect -march=native once at top level so both CLI and test targets can
# opt in. Not propagated through the INTERFACE library to avoid forcing a
# host-specific ISA on downstream consumers.
include(CheckCXXCompilerFlag)
check_cxx_compiler_flag(-march=native BGSPPRC_HAS_MARCH_NATIVE)
set(BGSPPRC_PROJECT_FLAGS "")
if(BGSPPRC_HAS_MARCH_NATIVE)
    list(APPEND BGSPPRC_PROJECT_FLAGS -march=native)
endif()

# CLI solver (opt-in, off by default when consumed as submodule)
option(BGSPPRC_BUILD_CLI "Build bgspprc-solve CLI" ${PROJECT_IS_TOP_LEVEL})

if(BGSPPRC_BUILD_CLI)
    # Regenerate version.h on every build so the embedded git hash tracks HEAD
    # without a reconfigure. configure_file is content-idempotent, so this
    # doesn't trigger spurious relinks when the hash hasn't changed.
    set(BGSPPRC_VERSION_HEADER ${CMAKE_CURRENT_BINARY_DIR}/generated/version.h)
    add_custom_target(bgspprc_version ALL
        BYPRODUCTS ${BGSPPRC_VERSION_HEADER}
        COMMAND ${CMAKE_COMMAND}
            -DSRC_DIR=${CMAKE_CURRENT_SOURCE_DIR}
            -DVERSION_IN=${CMAKE_CURRENT_SOURCE_DIR}/cli/version.h.in
            -DVERSION_OUT=${BGSPPRC_VERSION_HEADER}
            -P ${CMAKE_CURRENT_SOURCE_DIR}/cmake/generate_version.cmake
        COMMENT "Generating bgspprc version.h"
        VERBATIM)

    add_executable(bgspprc-solve cli/main.cpp)
    add_dependencies(bgspprc-solve bgspprc_version)
    target_link_libraries(bgspprc-solve PRIVATE bgspprc)
    target_include_directories(bgspprc-solve PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/cli
        ${CMAKE_CURRENT_BINARY_DIR}/generated)
    target_compile_options(bgspprc-solve PRIVATE
        -Wall -Wextra -Wpedantic -O2 ${BGSPPRC_PROJECT_FLAGS})

    # Parity CLI with SIMD code paths disabled, used by benchmarks to isolate
    # the SIMD contribution (the *_base modes in run_benchmarks.sh).
    add_executable(bgspprc-solve-nosimd cli/main.cpp)
    add_dependencies(bgspprc-solve-nosimd bgspprc_version)
    target_link_libraries(bgspprc-solve-nosimd PRIVATE bgspprc)
    target_include_directories(bgspprc-solve-nosimd PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/cli
        ${CMAKE_CURRENT_BINARY_DIR}/generated)
    target_compile_options(bgspprc-solve-nosimd PRIVATE
        -Wall -Wextra -Wpedantic -O2 ${BGSPPRC_PROJECT_FLAGS})
    target_compile_definitions(bgspprc-solve-nosimd PRIVATE BGSPPRC_DISABLE_SIMD)
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
    add_compile_options(-Wall -Wextra -Wpedantic -O2 ${BGSPPRC_PROJECT_FLAGS})

    set(BGSPPRC_TEST_SOURCES
        tests/test_main.cpp
        tests/test_executor.cpp
        tests/test_resource.cpp
        tests/test_bucket_graph.cpp
        tests/test_small_instance.cpp
        tests/test_stress.cpp)

    add_executable(test_runner ${BGSPPRC_TEST_SOURCES})
    target_link_libraries(test_runner PRIVATE bgspprc doctest::doctest)
    add_test(NAME unit_tests COMMAND test_runner)

    # Parity build: same sources, SIMD code paths disabled. Catches a class
    # of bugs where the SIMD lowering diverges from its scalar fallback.
    add_executable(test_runner_nosimd ${BGSPPRC_TEST_SOURCES})
    target_link_libraries(test_runner_nosimd PRIVATE bgspprc doctest::doctest)
    target_compile_definitions(test_runner_nosimd PRIVATE BGSPPRC_DISABLE_SIMD)
    add_test(NAME unit_tests_nosimd COMMAND test_runner_nosimd)

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

    add_executable(bench_runner tests/test_main.cpp tests/test_benchmarks.cpp)
    target_link_libraries(bench_runner PRIVATE bgspprc doctest::doctest)
    target_include_directories(bench_runner PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/cli)
    target_compile_definitions(bench_runner PRIVATE
        DATA_DIR="${CMAKE_CURRENT_SOURCE_DIR}/benchmarks/instances")
endif()

# Examples (opt-in, off by default when consumed as submodule)
option(BGSPPRC_BUILD_EXAMPLES "Build examples" ${PROJECT_IS_TOP_LEVEL})

if(BGSPPRC_BUILD_EXAMPLES)
    add_subdirectory(examples)
endif()
