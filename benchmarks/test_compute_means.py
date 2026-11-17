"""Tests for compute_means.py — the in-memory joins that replaced the former
run_comparison.sh / build_comparison_pathwyse.py producer scripts.

These lock in the join + cost-tolerance + TL logic that is now the sole source
of every published README number.
"""

import math
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).parent))
from compute_means import (
    classify,
    costs_agree,
    join_pathwyse,
    join_pathwyse11,
    join_rcspp,
    pathwyse11_table,
    quartiles,
)

BG_HEADER = "instance,set,ng,mode,cost,paths,time_s,timestamp"
PAPER_HEADER = "instance,ng,base_s,vectorised_s,parallel_s,bidir_s,all_s"
PW_HEADER = "instance,set,ng,cost,time_s,cost_scale,timestamp"
PW11_HEADER = "instance,set,ng,algo,parallel,cost,time_s,cost_scale,timestamp"


def _write(tmp_path: Path, name: str, header: str, rows: list[str]) -> Path:
    p = tmp_path / name
    p.write_text("\n".join([header, *rows]) + "\n")
    return p


class TestClassify:
    def test_leq_within_default_tolerance(self) -> None:
        assert classify("-10", "-10", 1_000_000) == "bg_leq"

    def test_gt(self) -> None:
        assert classify("-19", "-20", 1000) == "bg_gt"

    def test_tolerance_widens_at_low_cost_scale(self) -> None:
        # tol = max(1e-3, 100/1000) = 0.1 — bg just inside the widened band.
        assert classify("-19.95", "-20.0", 1000) == "bg_leq"

    def test_zero(self) -> None:
        assert classify("0", "-3", 1_000_000) == "bg_zero"

    def test_blank_when_either_cost_missing(self) -> None:
        assert classify("", "-3", 1_000_000) == ""
        assert classify("-3", "", 1_000_000) == ""


class TestJoinRcspp:
    def test_join(self, tmp_path: Path) -> None:
        bg = _write(
            tmp_path,
            "bgspprc.csv",
            BG_HEADER,
            [
                "A,ng8,8,para_bidir_vec,5,1,0.100,t",
                "A,ng8,8,mono_base,5,1,0.050,t",  # wrong mode → excluded
                "B,ng8,8,para_bidir_vec,,,,t",  # empty cost/time → TL
                "C,spprclib,8,para_bidir_vec,5,1,0.200,t",  # non-ng set → excluded
                "D,ng16,16,para_bidir_vec,5,1,2.000,t",
                "E,ng8,8,para_bidir_vec,5,1,0.300,t",  # not in paper → excluded
                "H,ng8,8,para_bidir_vec,5,1,0.400,t",
            ],
        )
        paper = _write(
            tmp_path,
            "pull_algo_runtimes.csv",
            PAPER_HEADER,
            [
                "A,8,0,0,0,0,0.050",
                "B,8,0,0,0,0,0.010",
                "D,16,0,0,0,0,1.000",
                "F,8,0,0,0,0,0.500",  # not in bg → excluded
                "H,8,0,0,0,0,200",  # > timeout → TL
            ],
        )
        rows = join_rcspp(bg, paper)
        # inner join, sorted by (ng, instance)
        assert [(r["instance"], r["ng"]) for r in rows] == [
            ("A", "8"),
            ("B", "8"),
            ("H", "8"),
            ("D", "16"),
        ]
        by = {r["instance"]: r for r in rows}
        assert by["A"]["bgspprc_s"] == "0.100" and by["A"]["paper_all_s"] == "0.050"
        assert by["A"]["ratio"] == "2.00"
        # bg timeout substitutes 120 in the ratio
        assert by["B"]["bgspprc_s"] == "TL" and by["B"]["ratio"] == "12000.00"
        # paper timeout substitutes 120 in the ratio
        assert by["H"]["paper_all_s"] == "TL" and by["H"]["ratio"] == "0.00"

    def test_ng_filter(self, tmp_path: Path) -> None:
        bg = _write(
            tmp_path,
            "bgspprc.csv",
            BG_HEADER,
            [
                "A,ng8,8,para_bidir_vec,5,1,0.1,t",
                "A,ng16,16,para_bidir_vec,5,1,0.2,t",
            ],
        )
        paper = _write(
            tmp_path,
            "pull_algo_runtimes.csv",
            PAPER_HEADER,
            [
                "A,8,0,0,0,0,0.05",
                "A,16,0,0,0,0,0.05",
            ],
        )
        rows = join_rcspp(bg, paper, ng_groups=(8,))
        assert [r["ng"] for r in rows] == ["8"]


class TestJoinPathwyse:
    def test_join(self, tmp_path: Path) -> None:
        bg = _write(
            tmp_path,
            "bgspprc.csv",
            BG_HEADER,
            [
                "P,spprclib,8,para_bidir_vec,-10,1,0.500,t",
                "P,spprclib,8,mono_base,-10,1,0.200,t",  # wrong mode → excluded
                "Q,roberti,8,para_bidir_vec,-20,1,1.000,t",
                "R,ng8,8,para_bidir_vec,-1,1,0.100,t",  # rcspp set → excluded
                "S,spprclib,8,para_bidir_vec,,,,t",  # empty cost/time
                "T,spprclib,16,para_bidir_vec,-5,1,2.000,t",  # not in pw → excluded
                "U,spprclib,8,para_bidir_vec,-5,1,0.300,t",
                "V,spprclib,8,para_bidir_vec,0,1,0.200,t",
            ],
        )
        pw = _write(
            tmp_path,
            "pathwyse.csv",
            PW_HEADER,
            [
                "P,spprclib,8,-10,1.000,1000000,t",
                "Q,roberti,8,-1,9.900,1000,t",  # stale dup, overwritten
                "Q,roberti,8,-19,2.000,1000,t",  # last-wins
                "S,spprclib,8,-3,0.500,1000000,t",
                "U,spprclib,8,-10,0.600,1000000,t",
                "V,spprclib,8,-3,0.400,1000000,t",
            ],
        )
        rows = join_pathwyse(bg, pw)
        assert [r["instance"] for r in rows] == ["P", "Q", "S", "U", "V"]
        tags = {r["instance"]: r["bg_leq"] for r in rows}
        assert tags == {"P": "bg_leq", "Q": "bg_leq", "S": "", "U": "bg_gt", "V": "bg_zero"}
        by = {r["instance"]: r for r in rows}
        # last-seen pathwyse row wins on duplicate keys
        assert by["Q"]["pathwyse_s"] == "2.000" and by["Q"]["pathwyse_cost"] == "-19"
        assert by["P"]["ratio"] == "0.50"
        # empty bgspprc time → blank ratio
        assert by["S"]["ratio"] == ""

    def test_set_attached_to_row(self, tmp_path: Path) -> None:
        bg = _write(
            tmp_path,
            "bgspprc.csv",
            BG_HEADER,
            [
                "P,spprclib,8,para_bidir_vec,-10,1,0.5,t",
                "Q,roberti,8,para_bidir_vec,-20,1,1.0,t",
            ],
        )
        pw = _write(
            tmp_path,
            "pathwyse.csv",
            PW_HEADER,
            [
                "P,spprclib,8,-10,1.0,1000000,t",
                "Q,roberti,8,-20,2.0,1000000,t",
            ],
        )
        rows = join_pathwyse(bg, pw)
        assert {r["instance"]: r["set"] for r in rows} == {"P": "spprclib", "Q": "roberti"}


class TestCostsAgree:
    def test_equal_within_tolerance(self) -> None:
        assert costs_agree("-10.000", "-10.0004", 1_000_000) == "eq"

    def test_lower_bgspprc_is_a_disagreement(self) -> None:
        # Same relaxation, so a lower bgspprc cost is as wrong as a higher one.
        assert costs_agree("-10.01", "-10", 1_000_000) == "ne"
        assert costs_agree("-9.99", "-10", 1_000_000) == "ne"

    def test_tolerance_widens_at_low_cost_scale(self) -> None:
        assert costs_agree("-19.95", "-20", 1000) == "eq"

    def test_unscaled_integer_costs_are_exact(self) -> None:
        # cost_scale 1 (.sppcc) converts exactly: no rounding band.
        assert costs_agree("-11", "-10", 1) == "ne"
        assert costs_agree("-10.000", "-10", 1) == "eq"

    def test_blank_on_timeout(self) -> None:
        assert costs_agree("", "-3", 1_000_000) == ""
        assert costs_agree("-3", "", 1_000_000) == ""


class TestQuartiles:
    def test_single_value(self) -> None:
        assert quartiles([2.0]) == (2.0, 2.0, 2.0)

    def test_inclusive(self) -> None:
        assert quartiles([1.0, 2.0, 3.0, 4.0, 5.0]) == (2.0, 3.0, 4.0)


def _pw11_fixture(tmp_path: Path) -> tuple[Path, Path]:
    bg = _write(
        tmp_path,
        "bgspprc.csv",
        BG_HEADER,
        [
            "P,spprclib,8,bidir_vec,-10,1,1.000,t",
            "P,spprclib,8,para_bidir_vec,-10,1,0.500,t",
            "Q,spprclib,8,bidir_vec,,,,t",  # bgspprc timeout
            "R,spprclib,8,bidir_vec,-5,1,1.000,t",  # PWBucket row missing
            "S,ng8,8,bidir_vec,-5,1,1.000,t",  # rcspp: not in this comparison
            "P,spprclib,8,mono_vec,-10,1,0.100,t",  # mode on no line
        ],
    )
    pw = _write(
        tmp_path,
        "pathwyse.csv",
        PW11_HEADER,
        [
            "P,spprclib,8,PWDefault,0,-10,9.000,1,t",  # stale dup, overwritten
            "P,spprclib,8,PWDefault,0,-10,3.000,1,t",
            "P,spprclib,8,PWBucket,0,-10,1.000,1,t",
            "P,spprclib,8,PWDefault,1,-10,2.000,1,t",
            "P,spprclib,8,PWBucket,1,-11,0.500,1,t",  # cost disagrees
            "Q,spprclib,8,PWDefault,0,-7,1.000,1,t",
            "Q,spprclib,8,PWBucket,0,,,1,t",  # PathWyse timeout
            "R,spprclib,8,PWDefault,0,-5,1.000,1,t",
            "S,ng8,8,PWDefault,0,-5,1.000,1,t",
            "S,ng8,8,PWBucket,0,-5,1.000,1,t",
        ],
    )
    return bg, pw


class TestJoinPathwyse11:
    def test_join(self, tmp_path: Path) -> None:
        rows = join_pathwyse11(*_pw11_fixture(tmp_path))
        assert [(r["line"], r["instance"]) for r in rows] == [
            ("seq", "P"),
            ("seq", "Q"),
            ("par", "P"),
        ]
        seq_p, seq_q, par_p = rows
        # last-seen PathWyse row wins on duplicate keys
        assert seq_p["PWDefault_s"] == "3.000"
        assert seq_p["PWDefault_agree"] == "eq" and seq_p["PWBucket_agree"] == "eq"
        # a timeout leaves the time blank and the agreement blank
        assert seq_q["bgspprc_s"] == "" and seq_q["PWBucket_s"] == ""
        assert seq_q["PWDefault_agree"] == ""
        assert par_p["bgspprc_s"] == "0.500" and par_p["PWBucket_agree"] == "ne"

    def test_table(self, tmp_path: Path, capsys: pytest.CaptureFixture[str]) -> None:
        out = pathwyse11_table(*_pw11_fixture(tmp_path), tl=120.0)
        lines = {tuple(c.strip() for c in ln.split("|")[3:5]): ln for ln in out.splitlines()[2:]}
        # seq: P (bg 1 s, PWDefault 3 s) and Q (bg TL, PWDefault 1 s):
        # ratios (3+1)/(1+1) = 2 and (1+1)/(120+1); geomean sqrt(2*2/121).
        cells = [c.strip() for c in lines[("seq", "PWDefault")].split("|")]
        assert cells[6] == "2/2"
        assert cells[7] == f"{math.sqrt(4 / 121):.2f}"
        assert [c.strip() for c in lines[("seq", "bgspprc")].split("|")][6] == "1/2"
        assert [c.strip() for c in lines[("seq", "PWBucket")].split("|")][6] == "1/2"
        # par: one instance, so the ratio is its own spread.
        par = [c.strip() for c in lines[("par", "PWBucket")].split("|")]
        assert par[7] == "1.00" and par[8] == "1.00–1.00"
        err = capsys.readouterr().err
        assert "cost agreement (PWBucket, n=3): eq=1  ne=1  blank=1" in err
        assert "cost agreement (PWDefault, n=3): eq=2  ne=0  blank=1" in err
