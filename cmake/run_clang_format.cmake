# clang-format --dry-run -Werror over the tracked C++ files: the clang_format
# ctest test (cmake/Lint.cmake). Tracked rather than globbed, so the build
# tree, fetched dependencies and scratch files are never formatted.
#
# Usage: cmake -DCLANG_FORMAT=<exe> -DSOURCE_DIR=<root> -P run_clang_format.cmake

execute_process(
    COMMAND git -c core.quotePath=false ls-files -- *.h *.cpp
    WORKING_DIRECTORY "${SOURCE_DIR}"
    OUTPUT_VARIABLE files
    OUTPUT_STRIP_TRAILING_WHITESPACE
    RESULT_VARIABLE rc)
if(NOT rc EQUAL 0)
    message(FATAL_ERROR "git ls-files failed in ${SOURCE_DIR}; nothing was checked")
endif()
string(REPLACE "\n" ";" files "${files}")
if(NOT files)
    message(FATAL_ERROR "no tracked C++ files in ${SOURCE_DIR}; nothing was checked")
endif()

execute_process(
    COMMAND "${CLANG_FORMAT}" --dry-run -Werror ${files}
    WORKING_DIRECTORY "${SOURCE_DIR}"
    RESULT_VARIABLE rc)
if(NOT rc EQUAL 0)
    message(FATAL_ERROR "clang-format found unformatted files; run "
                        "git ls-files -z '*.h' '*.cpp' | xargs -0 clang-format -i")
endif()
