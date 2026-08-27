#!/bin/bash
# Shared helpers for the git hooks in this directory. Sourced, not executed.

# Resolve a developer tool, preferring a project virtualenv over PATH so a
# pinned copy wins over whatever the machine happens to ship.
# Echoes the resolved path on success; returns 1 when the tool is absent.
resolve_tool() {
	local name="$1" venv_bin
	for venv_bin in .venv/bin venv/bin; do
		if [ -x "$venv_bin/$name" ]; then
			printf '%s' "$venv_bin/$name"
			return 0
		fi
	done
	if command -v "$name" >/dev/null 2>&1; then
		printf '%s' "$name"
		return 0
	fi
	return 1
}

# Resolve a tool, or say out loud which gate is not running. A missing tool
# must never skip in silence — that is a gate reporting a pass it never made.
require_tool() {
	local name="$1" what="$2" path
	if path=$(resolve_tool "$name"); then
		printf '%s' "$path"
		return 0
	fi
	echo "NOTE: $name not found — skipping $what." >&2
	echo "      Install it, or put a pinned copy in .venv/bin, so this gate runs." >&2
	return 1
}

# Minimum clang major version. Floored at the default apt install on Ubuntu
# 24.04 LTS so a stock machine satisfies it. 18 and 21 were verified to format
# this tree byte-identically and to report the same fix-subset findings, so the
# range is safe to spread across contributors.
CLANG_MIN_MAJOR=18

# Resolve a clang tool and check it meets CLANG_MIN_MAJOR. Older copies format
# differently, so running one would churn the tree against everyone else.
resolve_clang_tool() {
	local name="$1" what="$2" path major
	if ! path=$(resolve_tool "$name"); then
		echo "NOTE: $name not found — skipping $what." >&2
		echo "      apt install $name (>= $CLANG_MIN_MAJOR) so this gate runs." >&2
		return 1
	fi
	major=$("$path" --version 2>/dev/null |
		sed -nE 's/.*version ([0-9]+)\..*/\1/p' | head -1)
	if [ -z "$major" ]; then
		echo "NOTE: could not read $name version — skipping $what." >&2
		return 1
	fi
	if [ "$major" -lt "$CLANG_MIN_MAJOR" ]; then
		echo "NOTE: $name is version $major, below the minimum of $CLANG_MIN_MAJOR." >&2
		echo "      Skipping $what rather than reformatting the tree against everyone else." >&2
		return 1
	fi
	printf '%s' "$path"
	return 0
}
