"""Tests for run_benchmarks.sh's CSV: each row records its run's status (#157),
so a failed run is not read as a timeout. Runs the script on a fake solver."""

import csv
import os
import shutil
import subprocess
from pathlib import Path

import pytest

SCRIPT = Path(__file__).parent / "run_benchmarks.sh"
LEGACY_HEADER = "instance,set,ng,mode,cost,paths,time_s,timestamp"

# A stand-in for bgspprc-solve, its behaviour picked by the instance's name.
FAKE_SOLVER = """#!/usr/bin/env bash
case "$(basename "${!#}")" in
  ok.*)      echo "bgspprc abc"; echo "ok  sppcc  n=3  arcs=4  cost=-1.500  paths=2  12.5ms" ;;
  crash.*)   echo "boom" >&2; exit 3 ;;
  garbled.*) echo "no result line" ;;
  slow.*)    sleep 10 ;;
esac
"""

pytestmark = pytest.mark.skipif(
    shutil.which("bash") is None or shutil.which("timeout") is None,
    reason="run_benchmarks.sh needs bash and timeout",
)


def _run(tmp_path: Path, csv_path: Path, *names: str) -> subprocess.CompletedProcess[str]:
    solver = tmp_path / "solve"
    solver.write_text(FAKE_SOLVER)
    solver.chmod(0o755)
    instances = tmp_path / "spprclib"
    instances.mkdir(exist_ok=True)
    for n in names:
        (instances / f"{n}.sppcc").touch()
    return subprocess.run(
        ["bash", str(SCRIPT), "--timeout", "1", str(instances)],
        env={
            "PATH": os.environ.get("PATH", "/usr/bin:/bin"),
            "SOLVE": str(solver),
            "BGSPPRC_CSV": str(csv_path),
        },
        capture_output=True,
        text=True,
        check=True,
    )


def _rows(csv_path: Path) -> dict[str, dict[str, str]]:
    with open(csv_path) as f:
        return {r["instance"]: r for r in csv.DictReader(f)}


def test_each_row_records_its_status(tmp_path: Path) -> None:
    out = tmp_path / "bgspprc.csv"
    _run(tmp_path, out, "ok", "crash", "garbled", "slow")
    assert out.read_text().splitlines()[0] == LEGACY_HEADER + ",status"
    rows = _rows(out)
    assert {i: r["status"] for i, r in rows.items()} == {
        "ok": "OK",
        "crash": "ERROR(3)",
        "garbled": "UNPARSED",
        "slow": "TIMEOUT",
    }
    assert (rows["ok"]["cost"], rows["ok"]["paths"], rows["ok"]["time_s"]) == (
        "-1.500",
        "2",
        "0.013",
    )
    for i in ("crash", "garbled", "slow"):
        assert rows[i]["cost"] == rows[i]["paths"] == rows[i]["time_s"] == ""


def test_legacy_csv_gains_the_status_column(tmp_path: Path) -> None:
    out = tmp_path / "bgspprc.csv"
    out.write_text(
        "\n".join(
            [
                LEGACY_HEADER,
                "solved,roberti,8,para_bidir_vec,-2,1,0.100,t",
                "blank,roberti,8,para_bidir_vec,,,,t",
                "ok,spprclib,,para_bidir_vec,,,,t",  # replaced by the rerun
            ]
        )
        + "\n"
    )
    res = _run(tmp_path, out, "ok")
    assert "added the status column" in res.stderr
    rows = _rows(out)
    assert {i: r["status"] for i, r in rows.items()} == {
        "solved": "OK",
        "blank": "TIMEOUT",
        "ok": "OK",
    }
    assert rows["ok"]["cost"] == "-1.500"
