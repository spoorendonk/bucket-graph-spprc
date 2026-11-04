# Lint gates as ctest tests, so the pre-push local CI enforces them (#101).
#
#   clang_format        clang-format --dry-run -Werror over every tracked C++
#                       file, the same set the pre-commit hook checks. Needs a
#                       git checkout to list them, so a source export has none.
#   clang_tidy:<path>   clang-tidy on one translation unit, warnings as errors.
#                       One test per unit, named by its path from the
#                       repository root: pre-push then tidies only the units a
#                       push reaches (headers through a stub unit, via the
#                       ninja dependency log) and caches the passes.
#
# The library is header-only, so the units below are what reach its headers;
# `.clang-tidy`'s HeaderFilterRegex reports on `include/bgspprc/` and `cli/`
# and keeps out FetchContent's sources under the build tree.
#
# The tools are pinned to the group toolchain, clang 22.1.8: the version is
# `.clang-format-version`'s first line, which the pre-commit hook reads too, and
# `requirements-dev.txt` installs it into `.venv`. `.venv/bin` is searched
# first, as the hooks do, so the pinned tools there win over the ones on PATH
# (the main checkout's when this is a linked worktree, as pre-push's is);
# NO_CACHE, so one put there after a first configure is picked up. A missing
# tool, or one of another major version (clang-format's output and
# clang-tidy's finding set both change between majors), is a configure warning
# that drops its tests, not an error: a machine without them still builds.
#
# Expects BGSPPRC_LINT_UNITS (paths relative to the source root) to be set.

set(_bgspprc_lint_hint "${CMAKE_CURRENT_SOURCE_DIR}/.venv/bin")
execute_process(COMMAND git -C "${CMAKE_CURRENT_SOURCE_DIR}" rev-parse --path-format=absolute --git-common-dir
    OUTPUT_VARIABLE _bgspprc_git_common OUTPUT_STRIP_TRAILING_WHITESPACE
    ERROR_QUIET RESULT_VARIABLE _bgspprc_git_rc)
if(_bgspprc_git_rc EQUAL 0 AND _bgspprc_git_common MATCHES "/\\.git$")
    get_filename_component(_bgspprc_main_checkout "${_bgspprc_git_common}" DIRECTORY)
    list(APPEND _bgspprc_lint_hint "${_bgspprc_main_checkout}/.venv/bin")
endif()
find_program(BGSPPRC_CLANG_FORMAT clang-format HINTS ${_bgspprc_lint_hint} NO_CACHE)
find_program(BGSPPRC_CLANG_TIDY clang-tidy HINTS ${_bgspprc_lint_hint} NO_CACHE)

file(STRINGS "${CMAKE_CURRENT_SOURCE_DIR}/.clang-format-version" _bgspprc_pin LIMIT_COUNT 1)
string(STRIP "${_bgspprc_pin}" _bgspprc_pin)
string(REGEX MATCH "^[0-9]+" _bgspprc_pin_major "${_bgspprc_pin}")
if(NOT _bgspprc_pin_major)
    message(FATAL_ERROR ".clang-format-version does not begin with a version (\"${_bgspprc_pin}\")")
endif()

# Clears <var> when the tool's major version is not the pinned one, with a
# warning naming the fix.
function(_bgspprc_lint_require_pin var)
    if(NOT ${var})
        return()
    endif()
    execute_process(COMMAND ${${var}} --version
        OUTPUT_VARIABLE _v ERROR_QUIET RESULT_VARIABLE _rc)
    string(REGEX MATCH "version ([0-9]+)\\." _m "${_v}")
    if(NOT _rc EQUAL 0 OR NOT CMAKE_MATCH_1 STREQUAL _bgspprc_pin_major)
        string(STRIP "${_v}" _v)
        message(WARNING "${${var}} is not version ${_bgspprc_pin} (\"${_v}\"): "
                        "its lint tests are not registered. Install the pinned "
                        "tools with: uv venv && uv pip install -r requirements-dev.txt")
        set(${var} "" PARENT_SCOPE)
    elseif(NOT _v MATCHES "version ${_bgspprc_pin}([^0-9]|$)")
        message(WARNING "${${var}} is not exactly ${_bgspprc_pin} (\"${_v}\"); "
                        "its lint tests run, but may differ from the group's.")
    endif()
endfunction()
_bgspprc_lint_require_pin(BGSPPRC_CLANG_FORMAT)
_bgspprc_lint_require_pin(BGSPPRC_CLANG_TIDY)

if(BGSPPRC_CLANG_FORMAT AND NOT EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/.git")
    message(STATUS "Not a git checkout: no clang_format test")
elseif(BGSPPRC_CLANG_FORMAT)
    add_test(NAME clang_format
        COMMAND ${CMAKE_COMMAND}
            -DCLANG_FORMAT=${BGSPPRC_CLANG_FORMAT}
            -DSOURCE_DIR=${CMAKE_CURRENT_SOURCE_DIR}
            -P ${CMAKE_CURRENT_SOURCE_DIR}/cmake/run_clang_format.cmake)
    set_tests_properties(clang_format PROPERTIES LABELS lint)
else()
    message(WARNING "No usable clang-format: no clang_format test is "
                    "registered, so the push gate does not check formatting.")
endif()

# Apple's clang finds the SDK by itself, so the compile database names no
# -isysroot, and LLVM's clang-tidy (the pinned wheel, or Homebrew's) then
# misses the C library headers ('signal.h' file not found). Give it the SDK.
set(_bgspprc_tidy_sysroot "")
if(APPLE)
    set(_bgspprc_sdk "${CMAKE_OSX_SYSROOT}")
    if(NOT _bgspprc_sdk)
        execute_process(COMMAND xcrun --show-sdk-path
            OUTPUT_VARIABLE _bgspprc_sdk OUTPUT_STRIP_TRAILING_WHITESPACE ERROR_QUIET)
    endif()
    if(_bgspprc_sdk)
        set(_bgspprc_tidy_sysroot "--extra-arg=-isysroot${_bgspprc_sdk}")
    endif()
endif()

if(BGSPPRC_CLANG_TIDY)
    foreach(unit IN LISTS BGSPPRC_LINT_UNITS)
        add_test(NAME "clang_tidy:${unit}"
            COMMAND ${BGSPPRC_CLANG_TIDY} -p ${CMAKE_BINARY_DIR}
                    --quiet --warnings-as-errors=* ${_bgspprc_tidy_sysroot}
                    ${CMAKE_CURRENT_SOURCE_DIR}/${unit})
        set_tests_properties("clang_tidy:${unit}" PROPERTIES LABELS lint)
    endforeach()
else()
    message(WARNING "No usable clang-tidy: no clang_tidy: tests are "
                    "registered, so the push gate runs without lint.")
endif()
