"""Tests for convert_to_pathwyse.py instance format conversion."""
import textwrap
from pathlib import Path

import pytest

# Import the module under test
import sys

sys.path.insert(0, str(Path(__file__).parent))
from convert_to_pathwyse import parse_graph, parse_sppcc, parse_vrp, write_pathwyse


@pytest.fixture
def graph_instance(tmp_path):
    """Create a minimal .graph instance file."""
    content = textwrap.dedent("""\
        c test graph
        p test 4 4 8
        c
        c vertex a b d Q
        v 0 0 100 0 50
        v 1 10 50 10 50
        v 2 20 80 20 50
        v 3 0 100 0 50
        c
        c edge id src tgt cost time
        e 0 0 1 10.5 15
        e 1 0 2 20.0 25
        e 2 1 3 30.5 35
        e 3 2 3 5.0 10
    """)
    p = tmp_path / "test.graph"
    p.write_text(content)
    return p


@pytest.fixture
def sppcc_instance(tmp_path):
    """Create a minimal .sppcc instance file."""
    content = textwrap.dedent("""\
        NAME : test
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
    p = tmp_path / "test.sppcc"
    p.write_text(content)
    return p


@pytest.fixture
def vrp_instance(tmp_path):
    """Create a minimal .vrp instance file."""
    content = textwrap.dedent("""\
        NAME : test
        TYPE : CVRP
        DIMENSION : 3
        CAPACITY : 100
        EDGE_WEIGHT_TYPE : EUC_2D
        NODE_COORD_SECTION
        1 0 0
        2 3 4
        3 6 8
        DEMAND_SECTION
        1 0
        2 10
        3 20
        DEPOT_SECTION
        1
        -1
        PROFIT_SECTION
        1 1.0
        2 2.0
        3 3.0
        EOF
    """)
    p = tmp_path / "test.vrp"
    p.write_text(content)
    return p


class TestParseGraph:
    def test_vertex_count(self, graph_instance):
        inst = parse_graph(str(graph_instance))
        assert inst["n_vertices"] == 4

    def test_source_sink(self, graph_instance):
        inst = parse_graph(str(graph_instance))
        assert inst["source"] == 0
        assert inst["sink"] == 3

    def test_arc_count(self, graph_instance):
        inst = parse_graph(str(graph_instance))
        assert len(inst["arcs"]) == 4

    def test_has_time_windows(self, graph_instance):
        inst = parse_graph(str(graph_instance))
        assert inst["has_tw"] is True
        assert inst["tw_lb"] == [0, 10, 20, 0]
        assert inst["tw_ub"] == [100, 50, 80, 100]

    def test_demands(self, graph_instance):
        inst = parse_graph(str(graph_instance))
        assert inst["demands"] == [0, 10, 20, 0]


class TestParseSppcc:
    def test_vertex_count(self, sppcc_instance):
        inst = parse_sppcc(str(sppcc_instance))
        # 3 original + 1 sink = 4
        assert inst["n_vertices"] == 4

    def test_source_sink(self, sppcc_instance):
        inst = parse_sppcc(str(sppcc_instance))
        assert inst["source"] == 0
        assert inst["sink"] == 3

    def test_no_time_windows(self, sppcc_instance):
        inst = parse_sppcc(str(sppcc_instance))
        assert inst["has_tw"] is False


class TestParseVrp:
    def test_vertex_count(self, vrp_instance):
        inst = parse_vrp(str(vrp_instance))
        # 3 original + 1 sink = 4
        assert inst["n_vertices"] == 4

    def test_source_sink(self, vrp_instance):
        inst = parse_vrp(str(vrp_instance))
        assert inst["source"] == 0
        assert inst["sink"] == 3


class TestWritePathwyse:
    def test_graph_output(self, graph_instance, tmp_path):
        inst = parse_graph(str(graph_instance))
        outpath = tmp_path / "out.txt"
        write_pathwyse(inst, str(outpath))

        content = outpath.read_text()
        assert "NAME : test" in content
        assert "SIZE : 4" in content
        assert "ORIGIN : 0" in content
        assert "DESTINATION : 3" in content
        assert "RESOURCES : 2" in content
        assert "0 CAP" in content
        assert "1 TW" in content
        assert "EDGE_COST" in content
        assert "NODE_CONSUMPTION" in content
        assert "EDGE_CONSUMPTION" in content

    def test_sppcc_output(self, sppcc_instance, tmp_path):
        inst = parse_sppcc(str(sppcc_instance))
        outpath = tmp_path / "out.txt"
        write_pathwyse(inst, str(outpath))

        content = outpath.read_text()
        assert "RESOURCES : 1" in content
        assert "0 CAP" in content
        assert "EDGE_COST" in content
        assert "NODE_CONSUMPTION" in content
        # No time window sections for sppcc
        assert "1 TW" not in content

    def test_vrp_output(self, vrp_instance, tmp_path):
        inst = parse_vrp(str(vrp_instance))
        outpath = tmp_path / "out.txt"
        write_pathwyse(inst, str(outpath))

        content = outpath.read_text()
        assert "RESOURCES : 1" in content
        assert "EDGE_COST" in content
