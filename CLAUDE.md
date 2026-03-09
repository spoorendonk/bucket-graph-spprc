# bucket-graph-spprc

## Quick Reference

```bash
# fetch benchmark instances (once)
scripts/fetch_instances.sh

# build & test (standalone)
cmake -B build -DCMAKE_CXX_COMPILER=g++-14
cmake --build build
ctest --test-dir build

# as submodule in another project
add_subdirectory(bucket-graph-spprc)
target_link_libraries(my_target PRIVATE bgspprc)
```

## Git

- Never commit directly to `main`. Always feature branches.
- Linear history only (squash-merge or rebase-merge).

## Workflow: Plan → Grind

Every task has two phases. Do not skip planning.

### Plan (default)

When given a task, **plan first**: investigate the code, propose an approach,
discuss with the user. Wait for approval before implementing (e.g. "grind", "go", "do it").

### Grind (on approval)

Execute autonomously. Build, test, fix, repeat until green.
Self-review, then fullgate: branch, PR, sync main, push.
Progress lives in files and git — not in your context window.

Only pause and ask a human when:
- A fix requires changing the public API or architecture
- You discover a bug in unrelated code you shouldn't touch
- You're stuck after multiple failed attempts

### Fullgate

Also runs standalone when user says **"fullgate"**:
branch → PR → sync (merge main **into** feature branch) → tests → docs →
push → review → build → test → push fixes → squash-merge → delete branch

### Claiming Work (GitHub)

- `gh issue edit <N> --add-label agent-wip` when starting on an issue or PR
- Check for `agent-wip` label before picking up work
- Remove label and close/merge when done

### Teams

For independent sub-tasks, launch a team. Each teammate works in its own
worktree. Lead integrates: merge, resolve conflicts, build/test the result.
