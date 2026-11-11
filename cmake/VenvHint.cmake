# Sets _bgspprc_lint_hint: the `.venv/bin` directories the pinned tools of
# requirements-dev.txt are searched for in before PATH, as the hooks do. This
# checkout's, then the main checkout's when this is a linked worktree (as
# pre-push's is). Shared by the Python test (CMakeLists.txt) and the lint
# (cmake/Lint.cmake), so mypy's interpreter and mypy come from one place.

set(_bgspprc_lint_hint "${CMAKE_CURRENT_SOURCE_DIR}/.venv/bin")
execute_process(COMMAND git -C "${CMAKE_CURRENT_SOURCE_DIR}" rev-parse --path-format=absolute --git-common-dir
    OUTPUT_VARIABLE _bgspprc_git_common OUTPUT_STRIP_TRAILING_WHITESPACE
    ERROR_QUIET RESULT_VARIABLE _bgspprc_git_rc)
if(_bgspprc_git_rc EQUAL 0 AND _bgspprc_git_common MATCHES "/\\.git$")
    get_filename_component(_bgspprc_main_checkout "${_bgspprc_git_common}" DIRECTORY)
    list(APPEND _bgspprc_lint_hint "${_bgspprc_main_checkout}/.venv/bin")
endif()
