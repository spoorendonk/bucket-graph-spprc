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

run_test "default theta in output" \
  expect_exit_and_output 0 "theta=-1e-06" "$SOLVER" "$FIXTURES/tiny.sppcc"

run_test "--theta 1e9 overrides default" \
  expect_exit_and_output 0 "theta=1e+09" "$SOLVER" --theta 1e9 "$FIXTURES/tiny.sppcc"

run_test "--theta 0 shows theta=0" \
  expect_exit_and_output 0 "theta=0" "$SOLVER" --theta 0 "$FIXTURES/tiny.sppcc"

run_test "--stats prints n_buckets" \
  expect_exit_and_output 0 "n_buckets=" "$SOLVER" --stats "$FIXTURES/tiny.sppcc"

run_test "--stats prints n_labels_created" \
  expect_exit_and_output 0 "n_labels_created=" "$SOLVER" --stats "$FIXTURES/tiny.sppcc"

run_test "--stats prints label_state_bytes" \
  expect_exit_and_output 0 "label_state_bytes=" "$SOLVER" --stats "$FIXTURES/tiny.sppcc"

run_test "--csv prints header row" \
  expect_exit_and_output 0 "name,type,n_verts" "$SOLVER" --csv "$FIXTURES/tiny.sppcc"

run_test "--csv produces data row" \
  expect_exit_and_output 0 "tiny,sppcc," "$SOLVER" --csv "$FIXTURES/tiny.sppcc"

run_test "--timing prints fw=" \
  expect_exit_and_output 0 "fw=" "$SOLVER" --timing "$FIXTURES/tiny.sppcc"

run_test "--no-jump-arcs exits 0" \
  expect_exit_and_output 0 "cost=" "$SOLVER" --no-jump-arcs "$FIXTURES/tiny.sppcc"

run_test "--ng-metric distance with --ng 2" \
  expect_exit_and_output 0 "cost=" "$SOLVER" --ng 2 --ng-metric distance "$FIXTURES/tiny.sppcc"

run_test "--ng-metric cost with --ng 2" \
  expect_exit_and_output 0 "cost=" "$SOLVER" --ng 2 --ng-metric cost "$FIXTURES/tiny.sppcc"

# ── Summary ──

echo ""
echo "Results: $PASS passed, $FAIL failed"
[ "$FAIL" -eq 0 ]
