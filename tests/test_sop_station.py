"""按 SOP 建工站（M3.5b）：流水线吃 SOP；joint-v1 走 SOP 与走 kb 结果一致；换一个结构不同的产品也能走通；
孪生按 SOP 工步放件。孪生报告不存在时跳过对应测试。"""
import copy
import json
from pathlib import Path

import pytest

from jsdemo.io import load_yaml
from jsdemo.pipeline import Runner

ROOT = Path(__file__).resolve().parents[1]
SOP_J = ROOT / "kb" / "sop" / "joint-v1.sop.yaml"
SOP_H = ROOT / "kb" / "sop" / "sensor-hub-v1.sop.yaml"


@pytest.fixture(scope="module")
def run_j(tmp_path_factory):
    out = tmp_path_factory.mktemp("st_sop_j")
    return out, Runner(out, yes=True, quiet=True, sop=str(SOP_J)).run()


@pytest.fixture(scope="module")
def run_h(tmp_path_factory):
    out = tmp_path_factory.mktemp("st_sop_h")
    return out, Runner(out, yes=True, quiet=True, sop=str(SOP_H)).run()


def test_joint_via_sop_equals_kb(run, run_j):
    out, rep = run_j
    assert rep["status"] == "accepted"
    assert (out / "flow.yaml").read_text(encoding="utf-8") == (run["out"] / "flow.yaml").read_text(encoding="utf-8")
    # 走 SOP 时多了放件臂握手（M4 ②：停靠、区域空闲、等放件计数）；去掉握手后与走 kb 的程序逐条一致
    from jsdemo import codegen
    ops = json.loads((out / "arm_program.json").read_text(encoding="utf-8"))
    hand = [o for o in ops if o.get("purpose") == "park" or o.get("name") in ("zone_free", "unit_done") or o.get("until") in ("placed_count", "zone_left")]
    # C、D、E 三组之前各握手一次；D 组前放件臂中途换手，多一段「等离开 → 收回区域 → 取钉 → 回停靠 → 交出区域」；整件完成后停靠 + 通知下线
    assert len(hand) == 4 * 3 + 4 + 2
    core = [o for o in ops if not any(o is h for h in hand)]
    kb_ops = json.loads((run["out"] / "arm_program.json").read_text(encoding="utf-8"))
    assert core == kb_ops
    assert codegen.urscript(core) == (run["out"] / "arm_program.urscript").read_text(encoding="utf-8")
    st = load_yaml(out / "station.yaml")
    assert st["product_source"]["assembly"] == "assembly.yaml" and (out / "assembly.yaml").exists()


def test_second_product_runs_through(run_h):
    out, rep = run_h
    assert rep["status"] == "accepted"
    assert rep["stages"]["intake"]["product"] == "sensor-hub-v1" and rep["stages"]["intake"]["points"] == 10
    assert rep["stages"]["intake"]["sop"]["signable"] is False            # 借用治具，缺失项未补
    assert "DEC-004" in rep["decisions"]                                  # 借用治具如实记决策草稿
    flow = load_yaml(out / "flow.yaml")
    assert [s["group"] for s in flow["sequence"]][:1] == ["A"]
    assert {s["group"]: s["on_ng"][-1] for s in flow["sequence"]} == {"A": "continue", "K": "stop_unit", "B": "continue"}
    # 产品绕 z 转 90° 装夹：A1 (40, 22, 11.6) → 治具系 (-22, 40, 11.6)
    pts = {p["id"]: p["xyz"] for p in load_yaml(out / "product" / "fastening_points.yaml")["points"]}
    assert pts["A1"] == [-22.0, 40.0, 11.6]


def test_invalid_sop_is_sent_back(tmp_path):
    d = copy.deepcopy(load_yaml(SOP_H))
    d["steps"] = [s for s in d["steps"] if s["id"] != "S050"]              # 线缆座没放就拧压板
    bad = tmp_path / "bad.sop.yaml"
    bad.write_text(json.dumps(d, ensure_ascii=False), encoding="utf-8")   # JSON 是合法 YAML
    out = tmp_path / "st"
    rep = Runner(out, yes=True, quiet=True, sop=str(bad)).run()
    assert rep["status"] == "stopped:sop_invalid"
    assert not (out / "station.yaml").exists()
    v = json.loads((out / "sop_validation.json").read_text(encoding="utf-8"))
    assert any(f["code"] == "E-ORDER" for f in v["findings"])


# ---------- 孪生按 SOP 放件 ----------
def _twin(path):
    p = ROOT / "build" / path
    if not p.exists():
        pytest.skip(f"还没有 {path}")
    return json.loads(p.read_text(encoding="utf-8"))


def _place_ids(sop_path):
    return [s["part"] for s in load_yaml(sop_path)["steps"] if s["type"] == "place"]


def test_joint_twin_places_parts_in_sop_order():
    rep = _twin("twin/twin_report_dfmfix.json")
    assert [p["part"] for p in rep["place_steps"]] == _place_ids(SOP_J)
    cap = next(p for p in rep["place_steps"] if p["part"] == "CAP-01")
    first_e = next(s for s in rep["segments"] if s.get("point", "") and s["point"].startswith("E"))
    assert cap["t_s"] < sum(s["measured_s"] for s in rep["segments"][: first_e["i"]]) + 1.0   # 端盖在拧 E 组之前放
    if cap.get("by") == "handler_arm":   # M4 ②：放件臂放，时长为孪生实测；SOP 执行者缺失如实标出
        assert cap["duration_source"] == "measured" and cap["actor_assumed"] is True
    else:
        assert cap["duration_source"] == "assumed"                                            # SOP 里时长缺失


def test_second_product_twin_ok():
    rep = _twin("twin_station_hub/twin_report.json")
    assert rep["product"] == "sensor-hub-v1"
    assert rep["ok"] is True and rep["twin_ok"] is True and rep["collisions"] == []
    assert rep["points_arrived"] == rep["points_total"] == 10
    assert [p["part"] for p in rep["place_steps"]] == _place_ids(SOP_H)
    assert all(p["duration_source"] == "sop" for p in rep["place_steps"])


def test_second_product_stop_unit():
    rep = _twin("twin_station_hub/twin_report_demo_k1slip.json")
    assert rep["twin_ok"] is True and rep["unit_result"] == "NG"
    assert rep["results"][-1]["point"] == "K1" and [i["point"] for i in rep["issues"]] == ["K1"]
