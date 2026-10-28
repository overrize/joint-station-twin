"""工站设计标准（jsdemo.station/1，用户 2026-10-26：链路里只有上游方案 agent，布局、治具、工站设计都由它给）：
示例能通过；每类问题都能查出来并定位。"""
import copy
from pathlib import Path

import pytest

from jsdemo import station
from jsdemo.io import load_yaml

ROOT = Path(__file__).resolve().parents[1]
EX = load_yaml(ROOT / "kb" / "station" / "joint-v1.station.yaml")


def codes(spec):
    return {f["code"] for f in station.validate(spec)["findings"]}


def test_example_buildable():
    r = station.validate(EX)
    assert r["buildable"] and r["counts"]["error"] == 0
    assert "B-SOP" in {f["code"] for f in r["findings"]}          # joint-v1 的 SOP 还有缺失项，工站设计随之不能签字
    assert not r["signable"]


def mod(spec, mid):
    return next(m for m in spec["modules"] if m["id"] == mid)


@pytest.mark.parametrize("edit,code", [
    (lambda s: mod(s, "R2").__setitem__("module", "ARM-XYZ"), "E-REF"),                             # 库里没有的型号
    (lambda s: mod(s, "FD1")["lanes"].pop(), "E-FEED"),                                              # 料道漏了 M3x8
    (lambda s: mod(s, "FD1")["pose"].__setitem__("xyz", [990, 200, -30]), "E-REACH"),                # 供钉器挪远，出钉位够不到
    (lambda s: mod(s, "R2")["pose"].__setitem__("xyz", [120, 60, 0]), "E-OVERLAP"),                  # 两台臂挤在一起
    (lambda s: s["process"]["allocation"].pop(0), "E-ALLOC"),                                         # 工步没分配
    (lambda s: s["process"]["allocation"][2].__setitem__("resource", "R2"), "E-ALLOC"),              # 拧紧分给放件臂
    (lambda s: s.__setitem__("zones", []), "E-ZONE"),                                                 # 两臂并行却没有互斥分区
    (lambda s: s["handling"]["tray_slots"].pop(), "E-FEED"),                                          # 端盖没有料盘穴位
    (lambda s: mod(s, "R1")["pose"].__setitem__("xyz", [0, -1500, 0]), "E-OUTSIDE"),                 # 出了工站空间
])
def test_problems_are_caught(edit, code):
    s = copy.deepcopy(EX)
    edit(s)
    assert code in codes(s)


def test_new_fixture_suggestion():
    s = copy.deepcopy(EX)
    s["fixture"] = {"pose": {"xyz": [450, 0, 0], "yaw": 0}, "product_mounting": "from_sop",
                    "suggestion": {"type": "nest", "locating": "两销一面", "clamping": "缺失",
                                   "datums": [{"id": "D1", "xyz": [0, 0, 0]}, {"id": "D2", "xyz": [50, 0, 0]}, {"id": "D3", "xyz": [100, 0, 0]}]}}
    c = codes(s)
    assert "E-GEOM" in c          # 三个基准点共线
    assert "B-MISSING" in c       # 夹紧方式缺失
    assert "R-FIXTURE" in c       # 新治具交人确认
    assert "E-REF" in c           # 与 SOP 装夹所用治具不一致（SOP 写的是 FX-JOINT-V1）


def test_layout_comes_from_spec():
    """布局全部来自工站设计：关键位置与示例逐项一致，模块外形取模块库。"""
    L = station.to_layout(EX)
    assert L["HANDLER_MOUNT"] == {"xyz_mm": [820.0, -200.0, 0.0], "yaw_deg": 180.0}
    assert L["TRAY_STOP"]["xyz_mm"] == mod(EX, "CV1")["stops"]["tray_stop"] and L["OUT_STOP"]["xyz_mm"] == mod(EX, "CV1")["stops"]["out_stop"]
    assert L["TOOL_DOCKS"] == {"GR-2F85": [640.0, -520.0], "GR-VG1": [900.0, -520.0]} and L["TOOL_VIA"] == mod(EX, "TC1")["via"]
    assert L["FEEDER"]["xyz_mm"] == mod(EX, "FD1")["pose"]["xyz"] and L["FEEDER"]["size_mm"] == [180.0, 110.0, 60.0]
    assert L["H_HOME_J"] == mod(EX, "R2")["home_joints_deg"] and L["PARK_J"] == mod(EX, "R1")["park_joints_deg"]
    assert L["FIXTURE"] == {"id": "FX-JOINT-V1", "xyz_mm": [450.0, 0.0, 0.0], "yaw_deg": 0.0}


@pytest.fixture
def moved(tmp_path):
    """示例工站设计改几处布局后跑流水线到 generate：出件位沿 y 挪 20、供钉器沿 x 挪 10、拧紧臂停靠位换一个角。"""
    from jsdemo import layout
    from jsdemo.io import dump_yaml
    from jsdemo.pipeline import Runner
    s = copy.deepcopy(EX)
    s["meta"]["sop"] = str(ROOT / EX["meta"]["sop"])
    mod(s, "CV1")["stops"]["out_stop"][1] += 20
    mod(s, "FD1")["pose"]["xyz"][0] += 10
    mod(s, "R1")["park_joints_deg"][0] = 95
    sp = tmp_path / "moved.station.yaml"
    dump_yaml(s, sp)
    try:
        Runner(tmp_path / "base", yes=True, quiet=True, station_spec=str(ROOT / "kb" / "station" / "joint-v1.station.yaml")).run(stop_after="generate")
        rep = Runner(tmp_path / "out", yes=True, quiet=True, station_spec=str(sp)).run(stop_after="generate")
        yield tmp_path / "base", tmp_path / "out", rep
    finally:
        layout.use(None)   # 别把改过的布局留给后面的测试


def test_programs_follow_spec(moved):
    """流水线直接读工站设计：改了布局，生成的放件规划与两臂程序跟着变，其余不变。"""
    import json
    base, out, rep = moved
    assert rep["stages"]["intake"]["station_spec"]["explicit"] and rep["stages"]["intake"]["station_spec"]["buildable"]
    h0, h1 = load_yaml(base / "handling.yaml"), load_yaml(out / "handling.yaml")
    assert h1["unload"]["place_world"][1] == pytest.approx(h0["unload"]["place_world"][1] + 20)
    assert h1["items"] == h0["items"]
    a0 = json.loads((base / "arm_program.json").read_text(encoding="utf-8"))
    a1 = json.loads((out / "arm_program.json").read_text(encoding="utf-8"))
    feed = lambda ops: [o["pose"]["xyz_mm"] for o in ops if o.get("purpose") == "feed"]   # noqa: E731
    assert feed(a1) and all(p1[0] == pytest.approx(p0[0] + 10) and p1[1:] == p0[1:] for p0, p1 in zip(feed(a0), feed(a1)))
    assert {tuple(o["joints_deg"]) for o in a1 if o.get("purpose") == "park"} == {(95.0, -90.0, 90.0, -90.0, -90.0, 0.0)}
    fast = lambda ops: [o for o in ops if o.get("purpose") not in ("feed", "park")]   # noqa: E731
    assert fast(a1) == fast(a0)                                                         # 紧固点等与布局无关的部分不变
    assert load_yaml(out / "station.yaml")["layout_source"]["station_spec"].endswith("moved.station.yaml")


def test_invalid_spec_rejected(tmp_path):
    from jsdemo import layout
    from jsdemo.io import dump_yaml
    from jsdemo.pipeline import Runner
    s = copy.deepcopy(EX)
    s["meta"]["sop"] = str(ROOT / EX["meta"]["sop"])
    s["zones"] = []                                   # 两臂并行却没有互斥分区
    sp = tmp_path / "bad.station.yaml"
    dump_yaml(s, sp)
    try:
        rep = Runner(tmp_path / "out", yes=True, quiet=True, station_spec=str(sp)).run()
    finally:
        layout.use(None)
    assert rep["status"] == "stopped:station_invalid"
    assert not (tmp_path / "out" / "arm_program.json").exists()
    v = load_yaml(tmp_path / "out" / "station_validation.json")
    assert "E-ZONE" in {f["code"] for f in v["findings"]}


@pytest.mark.parametrize("edit,code", [
    (lambda s: mod(s, "R1").pop("park_joints_deg"), "E-SCHEMA"),                 # 并行时拧紧臂没有停靠位
    (lambda s: mod(s, "FD1")["pose"].__setitem__("yaw", 90), "E-UNSUPPORTED"),   # v1 供钉器只支持 yaw 0
])
def test_generator_fields_required(edit, code):
    s = copy.deepcopy(EX)
    edit(s)
    assert code in codes(s)
