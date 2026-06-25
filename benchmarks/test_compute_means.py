"""Tests for compute_means.py — the in-memory joins that replaced the former
run_comparison.sh / build_comparison_pathwyse.py producer scripts.

These lock in the join + cost-tolerance + TL logic that is now the sole source
of every published README number.
"""
import sys
import textwrap
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))
from compute_means import classify, join_pathwyse, join_rcspp  # noqa: E402

BG_HEADER = "instance,set,ng,mode,cost,paths,time_s,timestamp"
PAPER_HEADER = "instance,ng,base_s,vectorised_s,parallel_s,bidir_s,all_s"
PW_HEADER = "instance,set,ng,cost,time_s,cost_scale,timestamp"


def _write(tmp_path, name, header, rows):
    p = tmp_path / name
    p.write_text("\n".join([header, *rows]) + "\n")
    return p


class TestClassify:
    def test_leq_within_default_tolerance(self):
        assert classify("-10", "-10", 1_000_000) == "bg_leq"

    def test_gt(self):
        assert classify("-19", "-20", 1000) == "bg_gt"

    def test_tolerance_widens_at_low_cost_scale(self):
        # tol = max(1e-3, 100/1000) = 0.1 — bg just inside the widened band.
        assert classify("-19.95", "-20.0", 1000) == "bg_leq"

    def test_zero(self):
        assert classify("0", "-3", 1_000_000) == "bg_zero"

    def test_blank_when_either_cost_missing(self):
        assert classify("", "-3", 1_000_000) == ""
        assert classify("-3", "", 1_000_000) == ""


class TestJoinRcspp:
    def test_join(self, tmp_path):
        bg = _write(tmp_path, "bgspprc.csv", BG_HEADER, [
            "A,ng8,8,para_bidir,5,1,0.100,t",
            "A,ng8,8,mono_base,5,1,0.050,t",     # wrong mode → excluded
            "B,ng8,8,para_bidir,,,,t",            # empty cost/time → TL
            "C,spprclib,8,para_bidir,5,1,0.200,t",  # non-ng set → excluded
            "D,ng16,16,para_bidir,5,1,2.000,t",
            "E,ng8,8,para_bidir,5,1,0.300,t",     # not in paper → excluded
            "H,ng8,8,para_bidir,5,1,0.400,t",
        ])
        paper = _write(tmp_path, "pull_algo_runtimes.csv", PAPER_HEADER, [
            "A,8,0,0,0,0,0.050",
            "B,8,0,0,0,0,0.010",
            "D,16,0,0,0,0,1.000",
            "F,8,0,0,0,0,0.500",                  # not in bg → excluded
            "H,8,0,0,0,0,200",                    # > timeout → TL
        ])
        rows = join_rcspp(bg, paper)
        # inner join, sorted by (ng, instance)
        assert [(r["instance"], r["ng"]) for r in rows] == [
            ("A", "8"), ("B", "8"), ("H", "8"), ("D", "16")
        ]
        by = {r["instance"]: r for r in rows}
        assert by["A"]["bgspprc_s"] == "0.100" and by["A"]["paper_all_s"] == "0.050"
        assert by["A"]["ratio"] == "2.00"
        # bg timeout substitutes 120 in the ratio
        assert by["B"]["bgspprc_s"] == "TL" and by["B"]["ratio"] == "12000.00"
        # paper timeout substitutes 120 in the ratio
        assert by["H"]["paper_all_s"] == "TL" and by["H"]["ratio"] == "0.00"

    def test_ng_filter(self, tmp_path):
        bg = _write(tmp_path, "bgspprc.csv", BG_HEADER, [
            "A,ng8,8,para_bidir,5,1,0.1,t",
            "A,ng16,16,para_bidir,5,1,0.2,t",
        ])
        paper = _write(tmp_path, "pull_algo_runtimes.csv", PAPER_HEADER, [
            "A,8,0,0,0,0,0.05",
            "A,16,0,0,0,0,0.05",
        ])
        rows = join_rcspp(bg, paper, ng_groups=(8,))
        assert [r["ng"] for r in rows] == ["8"]


class TestJoinPathwyse:
    def test_join(self, tmp_path):
        bg = _write(tmp_path, "bgspprc.csv", BG_HEADER, [
            "P,spprclib,8,para_bidir,-10,1,0.500,t",
            "P,spprclib,8,mono_base,-10,1,0.200,t",  # wrong mode → excluded
            "Q,roberti,8,para_bidir,-20,1,1.000,t",
            "R,ng8,8,para_bidir,-1,1,0.100,t",       # rcspp set → excluded
            "S,spprclib,8,para_bidir,,,,t",          # empty cost/time
            "T,spprclib,16,para_bidir,-5,1,2.000,t",  # not in pw → excluded
            "U,spprclib,8,para_bidir,-5,1,0.300,t",
            "V,spprclib,8,para_bidir,0,1,0.200,t",
        ])
        pw = _write(tmp_path, "pathwyse.csv", PW_HEADER, [
            "P,spprclib,8,-10,1.000,1000000,t",
            "Q,roberti,8,-1,9.900,1000,t",           # stale dup, overwritten
            "Q,roberti,8,-19,2.000,1000,t",          # last-wins
            "S,spprclib,8,-3,0.500,1000000,t",
            "U,spprclib,8,-10,0.600,1000000,t",
            "V,spprclib,8,-3,0.400,1000000,t",
        ])
        rows = join_pathwyse(bg, pw)
        assert [r["instance"] for r in rows] == ["P", "Q", "S", "U", "V"]
        tags = {r["instance"]: r["bg_leq"] for r in rows}
        assert tags == {"P": "bg_leq", "Q": "bg_leq", "S": "",
                        "U": "bg_gt", "V": "bg_zero"}
        by = {r["instance"]: r for r in rows}
        # last-seen pathwyse row wins on duplicate keys
        assert by["Q"]["pathwyse_s"] == "2.000" and by["Q"]["pathwyse_cost"] == "-19"
        assert by["P"]["ratio"] == "0.50"
        # empty bgspprc time → blank ratio
        assert by["S"]["ratio"] == ""

    def test_set_attached_to_row(self, tmp_path):
        bg = _write(tmp_path, "bgspprc.csv", BG_HEADER, [
            "P,spprclib,8,para_bidir,-10,1,0.5,t",
            "Q,roberti,8,para_bidir,-20,1,1.0,t",
        ])
        pw = _write(tmp_path, "pathwyse.csv", PW_HEADER, [
            "P,spprclib,8,-10,1.0,1000000,t",
            "Q,roberti,8,-20,2.0,1000000,t",
        ])
        rows = join_pathwyse(bg, pw)
        assert {r["instance"]: r["set"] for r in rows} == {
            "P": "spprclib", "Q": "roberti"
        }
