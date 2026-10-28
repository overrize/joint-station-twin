"""工站 SOP 交接标准（M3.5a）：示例 SOP 合格、转换后与现有产品数据一致、改坏的 SOP 能被查出来。"""
import copy
import json
from pathlib import Path

import pytest

from jsdemo import flow, sop
from jsdemo.io import load_product, load_yaml

ROOT = Path(__file__).resolve().parents[1]
EXAMPLE = ROOT / "kb" / "sop" / "joint-v1.sop.yaml"


@pytest.fixture(scope="module")
def doc():
    return load_yaml(EXAMPLE)


def codes(res, level=None):
    return {(f["code"], f["path"]) for f in res["findings"] if level is None or f["level"] == level}


def code_set(res, level=None):
    return {c for c, _ in codes(res, level)}


def step(d, sid):
    return next(s for s in d["steps"] if s["id"] == sid)


# ---------- 示例本身 ----------
def test_example_is_buildable_not_signable(doc):
    res = sop.validate(doc)
    assert res["buildable"] is True and res["counts"]["error"] == 0
    assert res["signable"] is False            # 放件执行者/时长缺失、open_items 未决
    assert "B-MISSING" in code_set(res, "blocker") and "B-OPEN" in code_set(res, "blocker")


def test_example_dfm_review_matches_pipeline(doc):
    """SOP 几何检查出的批头间隙问题 = 流水线 DFM 阶段查出的（D3 / C12）。"""
    res = sop.validate(doc)
    found = {(f["path"], f["message"].split("距 ")[1].split(" ")[0]) for f in res["findings"] if f["code"] == "DFM-01"}
    assert found == {("steps[5].points[2]", "C12")}


def test_example_matches_json_schema(doc):
    jsonschema = pytest.importorskip("jsonschema")
    schema = json.loads((ROOT / "kb" / "contracts" / "sop-v1.schema.json").read_text(encoding="utf-8"))
    jsonschema.validate(doc, schema)


# ---------- 转换后与现有产品数据一致 ----------
def test_roundtrip_points_groups_obstacles(doc):
    got, ref = sop.to_product(doc), load_product("joint-v1")
    assert got["fastening"]["points"] == ref["fastening"]["points"]
    assert got["fastening"]["obstacles"] == ref["fastening"]["obstacles"]
    assert got["fastening"]["frame"] == ref["fastening"]["frame"]
    strip = lambda gs: [{k: v for k, v in g.items() if k != "part"} for g in gs]  # noqa: E731  part 字段语义见 docs/06
    assert strip(got["fastening"]["groups"]) == strip(ref["fastening"]["groups"])


def test_roundtrip_order_takt_ctq_bom(doc):
    got, ref = sop.to_product(doc), load_product("joint-v1")
    assert got["bom"]["assembly_order"] == ref["bom"]["assembly_order"]
    assert got["bom"]["target_takt_s"] == ref["bom"]["target_takt_s"]
    norm = lambda cs: sorted((c["id"], tuple(c.get("groups", [])), tuple(c.get("points", []))) for c in cs)  # noqa: E731
    assert norm(got["ctq"]) == norm(ref["ctq"])
    qty = lambda b: {p["id"]: p["qty"] for p in b["parts"] if "qty" in p}  # noqa: E731
    assert qty(got["bom"]) == qty(ref["bom"])


def test_roundtrip_station_policy(doc):
    pol = sop.to_station_policy(doc)
    for g, row in pol.items():
        assert row["on_ng"] == flow.NG_POLICY.get(g, flow.DEFAULT_NG)
        assert row.get("executor_retry") == flow.EXECUTOR_RETRY.get(g)


def test_mounting_offset_moves_points(doc):
    d = copy.deepcopy(doc)
    d["frame"]["mounting_in_fixture"]["xyz"] = [10.0, 0.0, 5.0]
    d["frame"]["mounting_in_fixture"]["rpy"] = [0.0, 0.0, 90.0]
    p = {x["id"]: x["xyz"] for x in sop.to_product(d)["fastening"]["points"]}
    assert p["E1"] == [10.0, 38.0, 45.0]      # (38, 0, 40) 绕 z 转 90° 再平移


# ---------- 改坏的 SOP 要能查出来 ----------
def _bad(doc, mutate):
    d = copy.deepcopy(doc)
    mutate(d)
    return sop.validate(d)


def test_cap_placed_before_encoder_screws_is_blocked(doc):
    def m(d):
        cap = step(d, "S070")
        d["steps"].remove(cap)
        d["steps"].insert(2, cap)              # 先放端盖再拧编码器
    res = _bad(doc, m)
    assert not res["buildable"]
    assert {p for c, p in codes(res, "error") if c == "E-BLOCKED"} >= {"steps[3].points[0]", "steps[3].points[1]", "steps[3].points[2]"}


def test_fasten_before_place_is_order_error(doc):
    def m(d):
        d["steps"].remove(step(d, "S040"))     # 支架没放就拧驱动板
    assert "E-ORDER" in code_set(_bad(doc, m), "error")


def test_unknown_refs(doc):
    def m(d):
        step(d, "S030")["fastens"] = "ENC-99"
        step(d, "S060")["screw"] = "M2x7-tapping"
        step(d, "S080")["ctq"] = ["CTQ-9"]
    res = _bad(doc, m)
    assert ("E-REF", "steps[2].fastens") in codes(res)
    assert ("E-REF", "steps[5].screw") in codes(res)
    assert ("E-REF", "steps[7].ctq") in codes(res)


def test_point_not_on_surface(doc):
    def m(d):
        step(d, "S030")["points"][0]["xyz"] = [0.0, 15.0, 20.0]   # 编码器顶面是 18
        step(d, "S080")["points"][0]["xyz"] = [60.0, 0.0, 40.0]   # 在端盖外
    res = _bad(doc, m)
    assert ("E-SURFACE", "steps[2].points[0]") in codes(res) and ("E-SURFACE", "steps[7].points[0]") in codes(res)


def test_missing_geometry_and_signed_with_blockers(doc):
    def m(d):
        d["parts"][4]["geometry"]["shapes"][0]["r"] = "缺失"
    assert "E-MISSING" in code_set(_bad(doc, m), "error")

    def s(d):
        d["meta"]["status"] = "signed"
        d["meta"]["signed_by"] = "某人"
    assert "E-SIGN" in code_set(_bad(doc, s), "error")


def test_unsupported_and_unit_and_source(doc):
    def m(d):
        step(d, "S080")["approach"] = [0.0, -1.0, 0.0]
        d["units"]["length"] = "inch"
    res = _bad(doc, m)
    assert "E-UNIT" in code_set(res, "error")

    def a(d):
        step(d, "S080")["approach"] = [0.0, -1.0, 0.0]
    assert "E-UNSUPPORTED" in code_set(_bad(doc, a), "error")

    def e(d):
        d.pop("example_data")                  # 真实 SOP 里不能出现 source: example
    assert "E-SOURCE" in code_set(_bad(doc, e), "error")


def test_process_and_decision_blockers(doc):
    def m(d):
        step(d, "S060")["hole"]["material"] = "PEEK"            # 工艺库没有 PEEK + M2 自攻
        step(d, "S080")["on_ng"]["retry"]["decision"] = "DEC-NG-999"
    res = _bad(doc, m)
    assert ("B-PROCESS", "steps[5].hole") in codes(res, "blocker")
    assert ("B-DECISION", "steps[7].on_ng.retry.decision") in codes(res, "blocker")
    assert res["buildable"] is True and res["signable"] is False


def test_run_writes_only_build(tmp_path):
    res = sop.run(EXAMPLE, out_root=tmp_path)
    out = Path(res["out"])
    for f in ("validation.json", "product/bom.yaml", "product/fastening_points.yaml", "product/ctq.yaml",
              "station_policy.yaml", "assembly.yaml"):
        assert (out / f).exists(), f
    assert load_yaml(out / "product" / "fastening_points.yaml")["example_data"] is True
