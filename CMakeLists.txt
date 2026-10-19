cmake_minimum_required(VERSION 3.25)

# If the default g++ is too old for the C++23 this library needs, pick a newer
# one that is already installed. Stock Ubuntu 24.04 defaults to GCC 13 but
# packages 14, so without this the documented build command — which pre-push
# runs verbatim — fails there while working on newer distros. An explicit
# choice (CXX in the environment, or -DCMAKE_CXX_COMPILER) always wins.
#
# Must run before project(), which is what reads CMAKE_CXX_COMPILER.
if(NOT DEFINED CMAKE_CXX_COMPILER AND NOT DEFINED ENV{CXX})
    find_program(BGSPPRC_DEFAULT_GXX g++)
    if(BGSPPRC_DEFAULT_GXX)
        execute_process(
            COMMAND ${BGSPPRC_DEFAULT_GXX} -dumpversion
            OUTPUT_VARIABLE BGSPPRC_DEFAULT_GXX_VERSION
            OUTPUT_STRIP_TRAILING_WHITESPACE
            ERROR_QUIET)
        if(BGSPPRC_DEFAULT_GXX_VERSION VERSION_LESS 14)
            # Newest first. Use a per-candidate cache variable: find_program
            # caches its result, so reusing one name across the loop would let
            # an earlier miss or hit shadow the later candidates.
            foreach(candidate g++-15 g++-14)
                find_program(BGSPPRC_GXX_${candidate} ${candidate})
                if(BGSPPRC_GXX_${candidate})
                    set(CMAKE_CXX_COMPILER ${BGSPPRC_GXX_${candidate}})
                    message(STATUS
                        "Default g++ is ${BGSPPRC_DEFAULT_GXX_VERSION}; "
                        "using ${BGSPPRC_GXX_${candidate}} instead")
                    break()
                endif()
            endforeach()
        endif()
    endif()
endif()

project(bgspprc VERSION 0.1.0 LANGUAGES CXX)

# Fail early and legibly on a compiler that cannot do the C++23 this library
# needs. Without this the first symptom is a wall of template errors from a
# header, which reads as a code bug rather than a toolchain one.
if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU" AND CMAKE_CXX_COMPILER_VERSION VERSION_LESS 14)
    message(FATAL_ERROR
        "GCC ${CMAKE_CXX_COMPILER_VERSION} is too old; bgspprc needs GCC >= 14. "
        "Select a newer compiler, e.g. cmake -B build -DCMAKE_CXX_COMPILER=g++-14")
elseif(CMAKE_CXX_COMPILER_ID MATCHES "Clang" AND CMAKE_CXX_COMPILER_VERSION VERSION_LESS 18)
    message(FATAL_ERROR
        "Clang ${CMAKE_CXX_COMPILER_VERSION} is too old; bgspprc needs Clang >= 18. "
        "Select a newer compiler, e.g. cmake -B build -DCMAKE_CXX_COMPILER=clang++-18")
endif()

# Emit compile_commands.json for clang-tidy and IDEs, but only when we
# are the top-level project — otherwise a downstream consumer embedding
# us via add_subdirectory could have its own compile db clobbered.
if(PROJECT_IS_TOP_LEVEL)
    set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
endif()

# Point git at the tracked hooks in .githooks/. core.hooksPath is per-checkout
# local config and cannot be committed, so it has to be set at configure time —
# otherwise a fresh clone silently has no gates at all.
#
# Guarded on being the top-level project: without that, a consumer embedding us
# via add_subdirectory/FetchContent would get core.hooksPath written into *its*
# repository, pointing at our hooks.
option(BGSPPRC_INSTALL_GIT_HOOKS "Point core.hooksPath at .githooks/" ON)

if(BGSPPRC_INSTALL_GIT_HOOKS
   AND PROJECT_IS_TOP_LEVEL
   # True for the .git *file* a linked worktree has, as well as a real .git dir.
   AND EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/.git")
    # --local, not --get: a plain `git config --get` also reads global and
    # system scope, so a contributor with a global core.hooksPath would take
    # the "already set" branch and end up with no project hooks at all.
    execute_process(
        COMMAND git config --local --get core.hooksPath
        WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
        OUTPUT_VARIABLE BGSPPRC_HOOKS_PATH
        OUTPUT_STRIP_TRAILING_WHITESPACE
        ERROR_QUIET)

    if(NOT BGSPPRC_HOOKS_PATH STREQUAL ".githooks")
        execute_process(
            COMMAND git config --local core.hooksPath .githooks
            WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
            RESULT_VARIABLE BGSPPRC_HOOKS_RESULT
            ERROR_QUIET)
        # Only claim success when git actually succeeded — a read-only .git
        # would otherwise give a green configure with no hooks installed.
        if(BGSPPRC_HOOKS_RESULT EQUAL 0)
            message(STATUS "Git hooks enabled (core.hooksPath=.githooks)")
        else()
            message(WARNING
                "Could not set core.hooksPath; git hooks are NOT active. "
                "Run: git config --local core.hooksPath .githooks")
        endif()
    endif()
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
            -DBGSPPRC_VERSION=${PROJECT_VERSION}
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
        tests/test_problem_view.cpp
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

    # The benchmark tooling's pytest suite, when an interpreter with pytest is
    # found (`.venv/bin/python` first). Skipped with a warning otherwise: the
    # C++ build must not depend on a Python setup.
    # NO_CACHE, so a .venv made after a first configure is picked up.
    find_program(BGSPPRC_PYTHON NAMES python3 python
        HINTS "${CMAKE_CURRENT_SOURCE_DIR}/.venv/bin" NAMES_PER_DIR NO_CACHE)
    if(BGSPPRC_PYTHON)
        execute_process(COMMAND ${BGSPPRC_PYTHON} -c "import pytest"
            RESULT_VARIABLE BGSPPRC_HAS_PYTEST OUTPUT_QUIET ERROR_QUIET)
    endif()
    if(BGSPPRC_PYTHON AND BGSPPRC_HAS_PYTEST EQUAL 0)
        add_test(NAME python_tests
            COMMAND ${BGSPPRC_PYTHON} -m pytest -q -p no:cacheprovider
                    ${CMAKE_CURRENT_SOURCE_DIR}/benchmarks)
    else()
        message(WARNING "No Python with pytest found: python_tests is not "
                        "registered, so the gate skips the benchmark tooling's "
                        "tests. Install with: uv venv && uv pip install pytest")
    endif()
endif()

# Examples (opt-in, off by default when consumed as submodule)
option(BGSPPRC_BUILD_EXAMPLES "Build examples" ${PROJECT_IS_TOP_LEVEL})

if(BGSPPRC_BUILD_EXAMPLES)
    add_subdirectory(examples)
endif()

# Lint gates, last so every option above is defined (BGSPPRC_BUILD_EXAMPLES).
# Top level only: they read our compile_commands.json, which only a top-level
# build writes, and a consumer's ctest is not ours to lint.
if(BGSPPRC_BUILD_TESTS AND PROJECT_IS_TOP_LEVEL)
    # Every translation unit the build compiles; tidying them reaches every
    # header. See cmake/Lint.cmake.
    set(BGSPPRC_LINT_UNITS ${BGSPPRC_TEST_SOURCES} tests/test_benchmarks.cpp)
    if(BGSPPRC_BUILD_CLI)
        list(APPEND BGSPPRC_LINT_UNITS cli/main.cpp tests/test_instance_io.cpp)
    endif()
    if(BGSPPRC_BUILD_EXAMPLES)
        list(APPEND BGSPPRC_LINT_UNITS
            examples/basic_spprc.cpp examples/custom_resource.cpp)
    endif()
    include(cmake/Lint.cmake)
endif()
