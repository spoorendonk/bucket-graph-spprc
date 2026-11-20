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
    bg_status,
    classify,
    costs_agree,
    join_pathwyse,
    join_pathwyse11,
    join_rcspp,
    modes_table,
    pathwyse11_table,
    pathwyse_table,
    quartiles,
    rcspp_table,
)

# bgspprc.csv before its status column (#157), and since.
BG_HEADER = "instance,set,ng,mode,cost,paths,time_s,timestamp"
BG_STATUS_HEADER = BG_HEADER + ",status"
PAPER_HEADER = "instance,ng,base_s,vectorised_s,parallel_s,bidir_s,all_s"
PW_HEADER = "instance,set,ng,cost,time_s,cost_scale,timestamp"
PW11_HEADER = "instance,set,ng,algo,parallel,status,cost,time_s,cost_scale,timestamp"


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
        assert costs_agree("-10.000", "-10.0004", 1_000_000, False) == "eq"

    def test_lower_bgspprc_is_a_disagreement(self) -> None:
        # Same relaxation, so a lower bgspprc cost is as wrong as a higher one.
        assert costs_agree("-10.01", "-10", 1_000_000, False) == "ne"
        assert costs_agree("-9.99", "-10", 1_000_000, False) == "ne"

    def test_tolerance_widens_at_low_cost_scale(self) -> None:
        assert costs_agree("-19.95", "-20", 1000, False) == "eq"

    def test_integer_costs_are_exact(self) -> None:
        # Integer arc costs (.sppcc) convert exactly: no rounding band.
        assert costs_agree("-11", "-10", 1, True) == "ne"
        assert costs_agree("-10.000", "-10", 1, True) == "eq"

    def test_float_costs_at_scale_one_keep_the_rounding_band(self) -> None:
        # A .vrp converted with --cost-scale 1 rounds every arc.
        assert costs_agree("-11", "-10", 1, False) == "eq"

    def test_blank_on_timeout(self) -> None:
        assert costs_agree("", "-3", 1_000_000, False) == ""
        assert costs_agree("-3", "", 1_000_000, False) == ""


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
            "P,spprclib,8,PWDefault,0,OK,-10,9.000,1,t",  # stale dup, overwritten
            "P,spprclib,8,PWDefault,0,OK,-10,3.000,1,t",
            "P,spprclib,8,PWBucket,0,OK,-10,1.000,1,t",
            "P,spprclib,8,PWDefault,1,OK,-10,2.000,1,t",
            "P,spprclib,8,PWBucket,1,OK,-11,0.500,1,t",  # cost disagrees
            "Q,spprclib,8,PWDefault,0,OK,-7,1.000,1,t",
            "Q,spprclib,8,PWBucket,0,TIMEOUT,,,1,t",  # PathWyse timeout
            "R,spprclib,8,PWDefault,0,OK,-5,1.000,1,t",
            "S,ng8,8,PWDefault,0,OK,-5,1.000,1,t",
            "S,ng8,8,PWBucket,0,OK,-5,1.000,1,t",
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

    def test_failed_pathwyse_run_is_excluded_not_a_timeout(
        self, tmp_path: Path, capsys: pytest.CaptureFixture[str]
    ) -> None:
        bg = _write(
            tmp_path,
            "bgspprc.csv",
            BG_HEADER,
            ["P,spprclib,8,bidir_vec,-10,1,1.000,t", "Q,spprclib,8,bidir_vec,-7,1,1.000,t"],
        )
        pw = _write(
            tmp_path,
            "pathwyse.csv",
            PW11_HEADER,
            [
                "P,spprclib,8,PWDefault,0,ERROR(134),,,1,t",
                "P,spprclib,8,PWBucket,0,OK,-10,1.000,1,t",
                "Q,spprclib,8,PWDefault,0,OK,-7,1.000,1,t",
                "Q,spprclib,8,PWBucket,0,UNPARSED,,,1,t",
            ],
        )
        out = pathwyse11_table(bg, pw)
        assert len(out.splitlines()) == 2  # header only: both instances failed
        err = capsys.readouterr().err
        assert (
            "excluded (failed run): P ng8 seq: bgspprc OK, PWDefault ERROR(134), PWBucket OK" in err
        )
        assert (
            "excluded (failed run): Q ng8 seq: bgspprc OK, PWDefault OK, PWBucket UNPARSED" in err
        )

    def test_failed_bgspprc_run_is_excluded_not_a_timeout(
        self, tmp_path: Path, capsys: pytest.CaptureFixture[str]
    ) -> None:
        bg = _write(
            tmp_path,
            "bgspprc.csv",
            BG_STATUS_HEADER,
            [
                "P,spprclib,8,bidir_vec,-10,1,1.000,t,OK",
                "Q,spprclib,8,bidir_vec,,,,t,ERROR(134)",
                "R,spprclib,8,bidir_vec,,,,t,TIMEOUT",
            ],
        )
        pw = _write(
            tmp_path,
            "pathwyse.csv",
            PW11_HEADER,
            [
                f"{i},spprclib,8,{a},0,OK,-7,1.000,1,t"
                for i in ("P", "Q", "R")
                for a in ("PWDefault", "PWBucket")
            ],
        )
        out = pathwyse11_table(bg, pw)
        cells = [c.strip() for c in out.splitlines()[2].split("|")]
        assert cells[4] == "bgspprc" and cells[6] == "1/2"  # P solved, R timed out
        err = capsys.readouterr().err
        assert "excluded (failed run): Q ng8 seq: bgspprc ERROR(134)" in err
        assert " P ng8" not in err and " R ng8" not in err

    def test_bgspprc_row_without_ng_does_not_break_the_join(self, tmp_path: Path) -> None:
        bg = _write(
            tmp_path,
            "bgspprc.csv",
            BG_HEADER,
            ["P,spprclib,8,bidir_vec,-10,1,1.000,t", "X,other,,bidir_vec,-1,1,1.000,t"],
        )
        pw = _write(
            tmp_path,
            "pathwyse.csv",
            PW11_HEADER,
            [
                "P,spprclib,8,PWDefault,0,OK,-10,1.000,1,t",
                "P,spprclib,8,PWBucket,0,OK,-10,1.000,1,t",
            ],
        )
        assert [r["instance"] for r in join_pathwyse11(bg, pw)] == ["P"]


class TestBgStatus:
    def test_legacy_row_reads_ok_with_a_time_else_timeout(self) -> None:
        assert bg_status({"time_s": "0.100"}) == "OK"
        assert bg_status({"time_s": ""}) == "TIMEOUT"

    def test_status_column_wins(self) -> None:
        assert bg_status({"time_s": "", "status": "ERROR(3)"}) == "ERROR(3)"
        assert bg_status({"time_s": "", "status": "UNPARSED"}) == "UNPARSED"
        assert bg_status({"time_s": "", "status": "TIMEOUT"}) == "TIMEOUT"


def _solved(table: str, *key: str) -> str:
    """The `solved` cell of the table row whose leading cells are `key`."""
    for ln in table.splitlines()[2:]:
        cells = [c.strip() for c in ln.split("|")[1:-1]]
        if tuple(cells[: len(key)]) == key:
            return cells[-1]
    raise AssertionError(f"no row {key} in\n{table}")


BG_STATUS_ROWS = (
    "A,spprclib,8,para_bidir_vec,-1,1,0.500,t,OK",
    "B,spprclib,8,para_bidir_vec,,,,t,TIMEOUT",
    "C,spprclib,8,para_bidir_vec,,,,t,ERROR(134)",
    "D,spprclib,8,para_bidir_vec,,,,t,UNPARSED",
    "A,ng8,8,para_bidir_vec,-1,1,0.500,t,OK",
    "B,ng8,8,para_bidir_vec,,,,t,TIMEOUT",
    "C,ng8,8,para_bidir_vec,,,,t,ERROR(6)",
)


class TestFailedBgspprcRunsExcluded:
    """A failed bgspprc run (ERROR/UNPARSED) is left out of every table that
    reads bgspprc.csv and named on stderr; a timeout still counts at 120 s."""

    def test_pathwyse_table(self, tmp_path: Path, capsys: pytest.CaptureFixture[str]) -> None:
        # spprclib rows only: the join keys on (instance, ng), which the rcspp
        # rows of the same names would overwrite.
        spprclib = [r for r in BG_STATUS_ROWS if ",spprclib," in r]
        bg = _write(tmp_path, "bgspprc.csv", BG_STATUS_HEADER, spprclib)
        pw = _write(
            tmp_path,
            "pathwyse.csv",
            PW_HEADER,
            [f"{i},spprclib,8,-1,1.000,1000000,t" for i in "ABCD"],
        )
        out = pathwyse_table(bg, pw)
        assert _solved(out, "spprclib", "8", "bgspprc") == "1/2"
        assert _solved(out, "spprclib", "8", "pathwyse") == "2/2"
        err = capsys.readouterr().err
        assert "excluded (failed run): C ng8: bgspprc ERROR(134)" in err
        assert "excluded (failed run): D ng8: bgspprc UNPARSED" in err
        assert "B ng8" not in err
        # --rows keeps the failed rows, with their status
        statuses = {r["instance"]: r["bgspprc_status"] for r in join_pathwyse(bg, pw)}
        assert statuses == {"A": "OK", "B": "TIMEOUT", "C": "ERROR(134)", "D": "UNPARSED"}

    def test_rcspp_table(self, tmp_path: Path, capsys: pytest.CaptureFixture[str]) -> None:
        bg = _write(tmp_path, "bgspprc.csv", BG_STATUS_HEADER, list(BG_STATUS_ROWS))
        paper = _write(tmp_path, "paper.csv", PAPER_HEADER, [f"{i},8,0,0,0,0,1.000" for i in "ABC"])
        out = rcspp_table(bg, paper)
        assert _solved(out, "8", "bgspprc") == "1/2"
        assert _solved(out, "8", "paper") == "2/2"
        assert "excluded (failed run): C ng8: bgspprc ERROR(6)" in capsys.readouterr().err
        rows = {r["instance"]: r for r in join_rcspp(bg, paper)}
        assert rows["C"]["bgspprc_s"] == "TL" and rows["C"]["bgspprc_status"] == "ERROR(6)"

    def test_modes_table(self, tmp_path: Path, capsys: pytest.CaptureFixture[str]) -> None:
        bg = _write(tmp_path, "bgspprc.csv", BG_STATUS_HEADER, list(BG_STATUS_ROWS))
        out = modes_table(bg)
        assert _solved(out, "spprclib", "8", "para_bidir_vec") == "1/2"
        assert _solved(out, "rcspp", "8", "para_bidir_vec") == "1/2"
        err = capsys.readouterr().err
        assert "excluded (failed run): C spprclib ng8 para_bidir_vec: bgspprc ERROR(134)" in err
        assert "excluded (failed run): C ng8 ng8 para_bidir_vec: bgspprc ERROR(6)" in err

    def test_legacy_csv_reads_every_blank_row_as_a_timeout(
        self, tmp_path: Path, capsys: pytest.CaptureFixture[str]
    ) -> None:
        legacy = [ln.rsplit(",", 1)[0] for ln in BG_STATUS_ROWS]
        out = modes_table(_write(tmp_path, "bgspprc.csv", BG_HEADER, legacy))
        assert _solved(out, "spprclib", "8", "para_bidir_vec") == "1/4"
        assert "excluded" not in capsys.readouterr().err
