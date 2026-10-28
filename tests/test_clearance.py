"""机械臂程序离线干涉检查（M4 ③）：结论与孪生一致，新障碍物能被查出来。"""
import copy
from pathlib import Path

import pytest

from jsdemo import clearance
from jsdemo.io import load_library, load_modules, load_yaml
from jsdemo.pipeline import Runner

ROOT = Path(__file__).resolve().parents[1]
SOP_J = ROOT / "kb" / "sop" / "joint-v1.sop.yaml"
SOP_H = ROOT / "kb" / "sop" / "sensor-hub-v1.sop.yaml"


@pytest.fixture(scope="module")
def stations(tmp_path_factory):
    out = {}
    for name, sop in (("joint", SOP_J), ("hub", SOP_H)):
        d = tmp_path_factory.mktemp(name)
        out[name] = (d, Runner(d, yes=True, quiet=True, sop=str(sop)).run())
    return out


def test_joint_only_known_d3(stations):
    d, rep = stations["joint"]
    c = rep["stages"]["verify"]["clearance"]
    assert rep["stages"]["verify"]["checks"]["clearance"] is True
    assert {(h["point"], h["obstacle"]) for h in c["known_hits"]} == {("D3", "C12")}   # 与孪生基线、DFM-01 一致
    assert c["new_hits"] == []
    assert (d / "clearance.yaml").exists()


def test_hub_clean(stations):
    _, rep = stations["hub"]
    c = rep["stages"]["verify"]["clearance"]
    assert c["ok"] and not c["known_hits"] and not c["new_hits"]


def test_new_obstacle_is_caught(stations):
    """端盖上加一个 DFM 没见过的高元件，紧挨 E1：应报新干涉、检查不通过。"""
    d, rep = stations["joint"]
    asm = copy.deepcopy(load_yaml(d / "assembly.yaml"))
    flow = load_yaml(d / "flow.yaml")
    e1 = next(s for s in flow["sequence"] if s["point"] == "E1")["engage"]["xyz"]
    cap = next(p for p in asm["parts"] if p["id"] == "CAP-01")
    cap.setdefault("components", []).append(
        {"id": "X99", "name": "测试障碍", "shape": {"type": "box", "center": [e1[0] + 4.0, e1[1], e1[2] + 5.0], "size": [3.0, 3.0, 10.0]}})
    env = load_modules()["EX-TS1"]["envelope"]
    c = clearance.check(flow, asm, env, load_library()["screws"], rep["stages"]["dfm"]["issues"])
    assert not c["ok"]
    assert any(h["point"] == "E1" and h["obstacle"] == "X99" for h in c["new_hits"])


def test_kb_run_skips_without_geometry(run):
    v = run["report"]["stages"]["verify"]
    assert v["clearance"] is None and "clearance" not in v["checks"]   # 没有外形就不声称查过


def test_envelope_matches_twin_scene():
    """离线检查用的工具包络与孪生场景里的执行器一致。"""
    import sys
    sys.path.insert(0, str(ROOT))
    from sim.isaac._common import SCENE
    seg = {s["name"]: s for s in load_modules()["EX-TS1"]["envelope"]["segments"]}
    ex, parts, fh = SCENE["executor"], SCENE["executor_parts"], SCENE["float_head"]
    assert seg["bit"]["r"] == parts["bit"]["radius"] and seg["bit"]["z"][1] == parts["bit"]["length"]
    assert seg["body"]["r"] == parts["body"]["radius"] and seg["body"]["z"] == [ex["length"] - parts["body"]["length"], ex["length"]]
    assert seg["float_head"]["r"] == fh["radius"]
    assert seg["float_head"]["z"] == [ex["length"] - fh["center_from_flange"] - fh["length"] / 2, ex["length"] - fh["center_from_flange"] + fh["length"] / 2]
