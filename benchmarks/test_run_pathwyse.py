"""Tests for run_pathwyse.sh on a fake PathWyse: a run with RUN_PREFIX unset
solves on every bash it can meet, macOS's /bin/bash 3.2 included, where an
empty array expanded under `set -u` is an unbound variable (#158)."""

import csv
import os
import shutil
import subprocess
import textwrap
from pathlib import Path

import pytest

SCRIPT = Path(__file__).parent / "run_pathwyse.sh"

# A stand-in for PathWyse: it needs the settings file run_pathwyse.sh writes
# into its working directory, and prints both releases' result lines.
FAKE_PATHWYSE = """#!/usr/bin/env bash
[[ -f pathwyse.set && -f "$1" ]] || exit 7
echo "Obj: -12"
echo "global time: 0.5"
echo "PWBucket Time : 0.5"
"""

# A stand-in for `bgspprc-solve --write-ng DIR --ng K FILE`.
FAKE_SOLVE = """#!/usr/bin/env bash
f="$(basename "$5")"
echo "0,1" > "$2/${f%.*}.ng$4.csv"
"""

SPPCC = textwrap.dedent("""\
    NAME : tiny
    TYPE : SPPCC
    DIMENSION : 3
    CAPACITY : 100
    EDGE_WEIGHT_TYPE : EXPLICIT
    EDGE_WEIGHT_FORMAT : FULL_MATRIX
    EDGE_WEIGHT_SECTION
    0 10 20
    10 0 15
    20 15 0
    NODE_WEIGHT_SECTION
    -5 -3 -7
    DEMAND_SECTION
    1 0
    2 10
    3 20
""")


def _bashes() -> list[str]:
    """Every distinct bash here: the one on PATH and /bin/bash (3.2 on macOS)."""
    found: dict[str, str] = {}
    for b in (shutil.which("bash"), "/bin/bash"):
        if b and os.access(b, os.X_OK):
            found.setdefault(os.path.realpath(b), b)
    return list(found.values())


pytestmark = pytest.mark.skipif(
    not _bashes() or shutil.which("timeout") is None,
    reason="run_pathwyse.sh needs bash and timeout",
)


def _exe(path: Path, body: str) -> Path:
    path.write_text(body)
    path.chmod(0o755)
    return path


@pytest.mark.parametrize("bash", _bashes())
@pytest.mark.parametrize("version", ["1.0", "1.1.0"])
def test_solves_with_run_prefix_unset(tmp_path: Path, bash: str, version: str) -> None:
    pw_dir = tmp_path / "pathwyse"
    (pw_dir / "bin").mkdir(parents=True)
    _exe(pw_dir / "bin" / "pathwyse", FAKE_PATHWYSE)
    instances = tmp_path / "spprclib"
    instances.mkdir()
    (instances / "tiny.sppcc").write_text(SPPCC)
    out = tmp_path / "pathwyse.csv"
    env = {
        "PATH": os.environ.get("PATH", "/usr/bin:/bin"),
        "TMPDIR": str(tmp_path),
        "PATHWYSE_DIR": str(pw_dir),
        "PATHWYSE_CSV": str(out),
        "PATHWYSE_CONVERTED_DIR": str(tmp_path / "converted"),
        "SOLVE": str(_exe(tmp_path / "solve", FAKE_SOLVE)),
    }
    res = subprocess.run(
        [bash, str(SCRIPT), "--pathwyse", version, "--skip-build", str(instances)],
        env=env,
        capture_output=True,
        text=True,
    )
    assert res.returncode == 0, res.stdout + res.stderr
    assert "unbound variable" not in res.stderr
    with open(out) as f:
        rows = list(csv.DictReader(f))
    assert len(rows) == 1
    row = rows[0]
    assert (row["instance"], row["set"], row["cost"], row["time_s"]) == (
        "tiny",
        "spprclib",
        "-12.000000",
        "0.5",
    )
    if version == "1.1.0":
        assert row["status"] == "OK"
