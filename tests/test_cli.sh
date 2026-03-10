#!/usr/bin/env bash
# Integration tests for bgspprc-solve CLI.
# Usage: test_cli.sh <solver-binary> <fixtures-dir>
set -euo pipefail

SOLVER="$1"
FIXTURES="$2"
PASS=0
FAIL=0

run_test() {
  local name="$1"
  shift
  if "$@" ; then
    echo "PASS: $name"
    PASS=$((PASS + 1))
  else
    echo "FAIL: $name"
    FAIL=$((FAIL + 1))
  fi
}

# Helpers that capture output
expect_exit() {
  local expected="$1"
  shift
  local rc=0
  "$@" >/dev/null 2>&1 || rc=$?
  [ "$rc" -eq "$expected" ]
}

expect_exit_and_output() {
  local expected_exit="$1"
  local expected_str="$2"
  shift 2
  local out rc=0
  out=$("$@" 2>&1) || rc=$?
  [ "$rc" -eq "$expected_exit" ] && echo "$out" | grep -q "$expected_str"
}

# ── Tests ──

run_test "no args → exit 1" \
  expect_exit 1 "$SOLVER"

run_test "unknown option → exit 1" \
  expect_exit 1 "$SOLVER" --bogus

run_test "single file tiny.sppcc → exit 0, has cost=" \
  expect_exit_and_output 0 "cost=" "$SOLVER" "$FIXTURES/tiny.sppcc"

run_test "single file tiny.graph → exit 0, has cost=" \
  expect_exit_and_output 0 "cost=" "$SOLVER" "$FIXTURES/tiny.graph"

run_test "single file tiny.vrp → exit 0, has cost=" \
  expect_exit_and_output 0 "cost=" "$SOLVER" "$FIXTURES/tiny.vrp"

run_test "--mono tiny.sppcc → exit 0" \
  expect_exit_and_output 0 "cost=" "$SOLVER" --mono "$FIXTURES/tiny.sppcc"

run_test "--ng 2 tiny.sppcc → exit 0" \
  expect_exit_and_output 0 "cost=" "$SOLVER" --ng 2 "$FIXTURES/tiny.sppcc"

run_test "directory mode → processes all files" \
  expect_exit_and_output 0 "cost=" "$SOLVER" "$FIXTURES"

# ── Summary ──

echo ""
echo "Results: $PASS passed, $FAIL failed"
[ "$FAIL" -eq 0 ]
