Before starting any work, check the current state and present options. Follow these steps exactly:

## Step 1: Check for dev-std updates

1. Check if the submodule has upstream updates:
   ```bash
   git -C .dev-std fetch origin main --quiet && git -C .dev-std rev-list HEAD..origin/main --count
   ```
   If the count is > 0, tell the user how many commits behind the submodule is and suggest running `bash .dev-std/update.sh` to pull the latest and re-apply shared files. Don't auto-run — let the user decide.

2. Check if copied files are out of sync with the current submodule:
   - Compare each file in `.claude/commands/` against the matching file in `.dev-std/commands/`
   - Compare `.git/hooks/pre-commit` against `.dev-std/hooks/pre-commit.sh`
   - Compare `.git/hooks/pre-push` against `.dev-std/hooks/pre-push.sh`
   - Compare `.git/hooks/commit-msg` against `.dev-std/hooks/commit-msg.sh`

   Use `diff -q` for each pair. If any differ, list the stale files and suggest running `bash .dev-std/update.sh` to update them.

3. Check if CLAUDE.md has been fleshed out beyond the minimal template (should have build commands, test commands, project-specific conventions). If it's still minimal, suggest running `/init` to auto-detect project details.

## Step 2: Check current state

1. Run `git status` and `git branch --show-current` to see where we are.
2. Run `git branch` to see all local branches.
3. For each branch other than main, check if it's stale:
   - Last commit older than 1 week, OR
   - Merge-base with main is 25+ commits behind main HEAD
   - Flag stale branches explicitly.

## Step 3: Present options

Report findings, then present choices:

- If on a feature branch: ask whether to continue here, or switch to main for new work.
- If other local branches exist: flag them (especially stale ones) and ask if any should be cleaned up with `git branch -d`.
- If on main: run `git pull` to get latest, ready for new work.

## Step 4: Branch selection

Only after the user chooses:

- To continue existing work: stay on or check out that branch.
- To start fresh: ensure we're on main, then create a new branch with format `feat/<short-description>` or `fix/<short-description>`.

For non-trivial new work, suggest entering plan mode before implementing.

Do NOT automatically check out branches, create new ones, or delete branches without asking first.
