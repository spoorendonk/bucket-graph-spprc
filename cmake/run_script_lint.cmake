# The script lint tests (cmake/Lint.cmake, #140), over the tracked files:
#
#   LINT=ruff_check    ruff check over every tracked *.py
#   LINT=ruff_format   ruff format --check over every tracked *.py
#   LINT=mypy          mypy --strict over every tracked *.py
#   LINT=shellcheck    shellcheck over every tracked *.sh
#
# Tracked rather than globbed, so the build tree, .venv and scratch files are
# never linted. The rules are the repository's: ruff.toml and .shellcheckrc,
# which the tools find beside the files; mypy takes --strict from here.
#
# Usage: cmake -DLINT=<name> -DTOOL=<exe> -DSOURCE_DIR=<root>
#              -DCACHE_DIR=<dir> [-DPYTHON=<exe>] -P run_script_lint.cmake

if(LINT STREQUAL "shellcheck")
    set(patterns *.sh)
else()
    set(patterns *.py)
endif()

execute_process(
    COMMAND git -c core.quotePath=false ls-files -- ${patterns}
    WORKING_DIRECTORY "${SOURCE_DIR}"
    OUTPUT_VARIABLE files
    OUTPUT_STRIP_TRAILING_WHITESPACE
    RESULT_VARIABLE rc)
if(NOT rc EQUAL 0)
    message(FATAL_ERROR "git ls-files failed in ${SOURCE_DIR}; nothing was checked")
endif()
string(REPLACE "\n" ";" files "${files}")
if(NOT files)
    message(FATAL_ERROR "no tracked ${patterns} files in ${SOURCE_DIR}; nothing was checked")
endif()

if(LINT STREQUAL "ruff_check")
    set(cmd "${TOOL}" check --no-fix --cache-dir "${CACHE_DIR}/ruff")
    set(fix "run ruff check --fix, or fix them by hand")
elseif(LINT STREQUAL "ruff_format")
    set(cmd "${TOOL}" format --check --cache-dir "${CACHE_DIR}/ruff")
    set(fix "run ruff format")
elseif(LINT STREQUAL "mypy")
    # The interpreter whose packages (pytest) the tests import, when known.
    set(cmd "${TOOL}" --strict --cache-dir "${CACHE_DIR}/mypy")
    if(PYTHON)
        list(APPEND cmd --python-executable "${PYTHON}")
    endif()
    set(fix "fix the types it reports")
elseif(LINT STREQUAL "shellcheck")
    set(cmd "${TOOL}" -f gcc)
    set(fix "fix them, or disable one with its reason (# shellcheck disable=SCnnnn)")
else()
    message(FATAL_ERROR "unknown LINT '${LINT}'")
endif()

execute_process(
    COMMAND ${cmd} ${files}
    WORKING_DIRECTORY "${SOURCE_DIR}"
    RESULT_VARIABLE rc)
if(NOT rc EQUAL 0)
    message(FATAL_ERROR "${LINT} found problems: ${fix}")
endif()
