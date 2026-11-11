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

include(${CMAKE_CURRENT_LIST_DIR}/VenvHint.cmake)
find_program(BGSPPRC_CLANG_FORMAT clang-format HINTS ${_bgspprc_lint_hint} NO_CACHE)
find_program(BGSPPRC_CLANG_TIDY clang-tidy HINTS ${_bgspprc_lint_hint} NO_CACHE)

file(STRINGS "${CMAKE_CURRENT_SOURCE_DIR}/.clang-format-version" _bgspprc_pin LIMIT_COUNT 1)
string(STRIP "${_bgspprc_pin}" _bgspprc_pin)
# The leading N[.N...] of the pin, matched in as many components as it names
# (`22` takes any 22.x, `22.1.8` only 22.1.8): the rule pre-commit and
# pre-push apply, so configure registers a lint test only for a tool the hooks
# would run.
string(REGEX MATCH "^[0-9]+(\\.[0-9]+)*" _bgspprc_pin "${_bgspprc_pin}")
if(NOT _bgspprc_pin)
    message(FATAL_ERROR ".clang-format-version does not begin with a version")
endif()
string(REPLACE "." "\\." _bgspprc_pin_re "${_bgspprc_pin}")

# Clears <var> when the tool's version is not the pinned one, with a warning
# naming the fix.
function(_bgspprc_lint_require_pin var)
    if(NOT ${var})
        return()
    endif()
    execute_process(COMMAND ${${var}} --version
        OUTPUT_VARIABLE _v ERROR_QUIET RESULT_VARIABLE _rc)
    if(NOT _rc EQUAL 0 OR NOT _v MATCHES "version ${_bgspprc_pin_re}([^.0-9]|\\.[0-9]|$)")
        string(STRIP "${_v}" _v)
        message(WARNING "${${var}} is not version ${_bgspprc_pin} (\"${_v}\"): "
                        "its lint tests are not registered. Install the pinned "
                        "tools with: uv venv && uv pip install -r requirements-dev.txt")
        set(${var} "" PARENT_SCOPE)
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

# ── Script lint (#140) ──
#
#   ruff_check, ruff_format   ruff check, and ruff format --check, over every
#                             tracked *.py (rules: ruff.toml)
#   mypy                      mypy --strict over every tracked *.py
#   shellcheck                shellcheck over every tracked *.sh (.shellcheckrc)
#
# Each runs cmake/run_script_lint.cmake, which lists the files with git, so a
# source export has none of them. The tools come from PyPI, pinned in
# requirements-dev.txt (shellcheck as shellcheck-py), and are searched for in
# `.venv/bin` first like the clang tools. Each is registered only for the
# pinned version, matched in its first three components (shellcheck-py's
# fourth is its packaging revision): their findings and formatting move
# between releases. A missing or other-version tool is a configure warning
# that drops its tests.
#
# Not the hooks under .githooks/: they are wcp's (`wcp doctor` tracks their
# bytes), and a finding there is fixed upstream, not here.

file(STRINGS "${CMAKE_CURRENT_SOURCE_DIR}/requirements-dev.txt" _bgspprc_reqs
    REGEX "^[A-Za-z0-9_.-]+==")

# Sets <var> to the pinned X.Y.Z of PyPI package <pkg> (empty without a pin).
function(_bgspprc_lint_pypi_pin pkg var)
    set(_pin "")
    foreach(_line IN LISTS _bgspprc_reqs)
        if(_line MATCHES "^${pkg}==([0-9]+\\.[0-9]+\\.[0-9]+)")
            set(_pin "${CMAKE_MATCH_1}")
        endif()
    endforeach()
    set(${var} "${_pin}" PARENT_SCOPE)
endfunction()

# Registers script lint test <name>(s) for tool <exe> of PyPI package <pkg>,
# when <exe> is found at the pinned version.
function(_bgspprc_script_lint pkg exe)
    set(_names ${ARGN})
    _bgspprc_lint_pypi_pin(${pkg} _pin)
    if(NOT _pin)
        message(WARNING "requirements-dev.txt does not pin ${pkg}==X.Y.Z: "
                        "${_names} not registered.")
        return()
    endif()
    string(TOUPPER "BGSPPRC_${exe}" _var)
    find_program(${_var} ${exe} HINTS ${_bgspprc_lint_hint} NO_CACHE)
    if(${_var})
        execute_process(COMMAND ${${_var}} --version
            OUTPUT_VARIABLE _v ERROR_QUIET RESULT_VARIABLE _rc)
        string(REPLACE "." "\\." _pin_re "${_pin}")
        if(NOT _rc EQUAL 0 OR NOT _v MATCHES "(^|[^.0-9])${_pin_re}([^.0-9]|\\.[0-9]|$)")
            string(STRIP "${_v}" _v)
            message(WARNING "${${_var}} is not version ${_pin} (\"${_v}\"): "
                            "${_names} not registered. Install the pinned "
                            "tools with: uv venv && uv pip install -r requirements-dev.txt")
            return()
        endif()
    else()
        message(WARNING "No ${exe}: ${_names} not registered, so the push gate "
                        "does not run it. Install the pinned tools with: "
                        "uv venv && uv pip install -r requirements-dev.txt")
        return()
    endif()
    if(NOT EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/.git")
        message(STATUS "Not a git checkout: no ${_names} test")
        return()
    endif()
    # mypy resolves the tests' imports (pytest) in the suite's interpreter, so
    # it is registered only where python_tests is: without one, it would fail
    # on `import pytest` where the suite is merely skipped.
    set(_python "")
    if(BGSPPRC_PYTHON AND BGSPPRC_HAS_PYTEST EQUAL 0)
        set(_python "${BGSPPRC_PYTHON}")
    elseif(pkg STREQUAL "mypy")
        message(WARNING "No Python with pytest found: ${_names} not registered.")
        return()
    endif()
    foreach(_name IN LISTS _names)
        add_test(NAME ${_name}
            COMMAND ${CMAKE_COMMAND}
                -DLINT=${_name}
                -DTOOL=${${_var}}
                -DSOURCE_DIR=${CMAKE_CURRENT_SOURCE_DIR}
                -DCACHE_DIR=${CMAKE_BINARY_DIR}/lint_cache
                -DPYTHON=${_python}
                -P ${CMAKE_CURRENT_SOURCE_DIR}/cmake/run_script_lint.cmake)
        set_tests_properties(${_name} PROPERTIES LABELS lint)
    endforeach()
endfunction()

_bgspprc_script_lint(ruff ruff ruff_check ruff_format)
_bgspprc_script_lint(mypy mypy mypy)
_bgspprc_script_lint(shellcheck-py shellcheck shellcheck)
