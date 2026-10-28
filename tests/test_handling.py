"""放件（M4 ②）：输送线送料盘、放件臂放件，拧紧臂只等信号；孪生报告不存在时跳过孪生部分。"""
import json
from pathlib import Path

import pytest

from jsdemo import clearance, codegen, composer, handling
from jsdemo.io import load_modules, load_rules, load_yaml
from jsdemo.pipeline import Runner

ROOT = Path(__file__).resolve().parents[1]
SOP_J = ROOT / "kb" / "sop" / "joint-v1.sop.yaml"
SOP_H = ROOT / "kb" / "sop" / "sensor-hub-v1.sop.yaml"


@pytest.fixture(scope="module")
def joint(tmp_path_factory):
    out = tmp_path_factory.mktemp("hd_joint")
    return out, Runner(out, yes=True, quiet=True, sop=str(SOP_J)).run()


def test_handler_added_only_when_needed(joint, tmp_path):
    out, rep = joint
    roles = [m["role"] for m in rep["stages"]["compose"]["station"]["modules"]]
    assert {"handler_arm", "gripper", "tool_changer", "tray"} <= set(roles)
    assert "DEC-005" in rep["decisions"]
    hub = Runner(tmp_path / "hub", yes=True, quiet=True, sop=str(SOP_H)).run()   # 放件全是人工 / 上游：不加放件臂
    assert "handler_arm" not in [m["role"] for m in hub["stages"]["compose"]["station"]["modules"]]
    assert not (tmp_path / "hub" / "handler_program.json").exists()


def test_plan_follows_sop_order_and_handoffs(joint):
    out, _ = joint
    h = load_yaml(out / "handling.yaml")
    sop_places = [s["part"] for s in load_yaml(SOP_J)["steps"] if s["type"] == "place"]
    assert [it["part"] for it in h["items"]] == sop_places
    assert all(it["actor_assumed"] for it in h["items"])          # SOP 执行者缺失：演示交给放件臂并标出
    assert h["handoff_before_point"] == {"C1": 2, "D1": 4, "E1": 5}
    assert h["source"] == "demo_placeholder"


def test_fastening_arm_waits_and_never_places(joint):
    out, _ = joint
    ops = json.loads((out / "arm_program.json").read_text(encoding="utf-8"))
    assert not any(o["op"] == "grip" for o in ops)                # 拧紧臂程序里没有放件动作
    waits = [o["value"] for o in ops if o.get("until") == "placed_count"]
    assert waits == [2, 4, 5]
    for i, o in enumerate(ops):   # 等待前一定先退到停靠位、给区域空闲；两者与等待之间只允许去供钉器取钉，不许进治具
        if o.get("until") == "placed_count":
            j = max(k for k in range(i) if ops[k].get("purpose") == "park")
            assert ops[j + 1] == {"op": "signal", "name": "zone_free", "value": True}
            between = ops[j + 2:i]
            assert all(x.get("purpose") == "feed" or x.get("until") == "feed_ok" for x in between)


def test_handler_program_boundary(joint):
    out, rep = joint
    forbidden = load_rules()["golden"]["arm_program_forbidden_tokens"]
    src = (out / "handler_program.urscript").read_text(encoding="utf-8") + (out / "handler_program.json").read_text(encoding="utf-8")
    assert not [t for t in forbidden if t in src]
    ops = json.loads((out / "handler_program.json").read_text(encoding="utf-8"))
    grips = [(o["part"], o["value"]) for o in ops if o["op"] == "grip" and o.get("phase") != "unload"]
    unload = [(o["part"], o["value"]) for o in ops if o["op"] == "grip" and o.get("phase") == "unload"]
    assert unload == [("UNIT", True), ("UNIT", False)]                 # M4 ③：放完后吸起整件放到下游
    assert len(grips) == 10 and all(grips[i][1] and not grips[i + 1][1] and grips[i][0] == grips[i + 1][0] for i in range(0, 10, 2))
    assert rep["stages"]["verify"]["checks"]["reach"] is True


def test_handler_clearance(joint):
    out, rep = joint
    c = rep["stages"]["verify"]["clearance"]["handler"]
    assert c["ok"] and len(c["steps"]) == 5
    # 套在编码器外面的驱动板支架按环形计算，不会被实心外接圆误报
    brk = next(r for r in c["steps"] if r["part"] == "BRK-01")
    assert brk["min_gap_mm"] is not None and brk["min_gap_mm"] >= 0


def test_handler_clearance_catches_blocking_part(joint):
    """把端盖放件挪到编码器之前：后放的编码器会被端盖挡住，应报干涉。"""
    out, _ = joint
    asm = load_yaml(out / "assembly.yaml")
    steps = asm["steps"]
    cap = next(s for s in steps if s.get("part") == "CAP-01")
    steps.remove(cap)
    steps.insert(1, cap)
    mods = load_modules()
    st = load_yaml(out / "station.yaml")
    F = composer.fixture_pose(mods, st["fixture_frame"])
    pl = handling.plan(asm, mods, st, F)
    c = clearance.check_handler(pl, asm, mods, F)
    assert not c["ok"] and any(h["obstacle"].startswith("CAP-01") for h in c["new_hits"])


def test_urscript_handler_signals(joint):
    out, _ = joint
    src = (out / "handler_program.urscript").read_text(encoding="utf-8")
    assert "get_standard_digital_in(3)" in src and "get_standard_digital_in(2)" in src
    assert src.count("write_output_integer_register(0,") == 3
    arm = (out / "arm_program.urscript").read_text(encoding="utf-8")
    assert arm.count("read_input_integer_register(0)") == 3


# ---------- 孪生 ----------
def _twin(path):
    p = ROOT / "build" / path
    if not p.exists():
        pytest.skip(f"还没有 {path}")
    return json.loads(p.read_text(encoding="utf-8"))


def test_twin_handler_places_all():
    rep = _twin("twin/twin_report_dfmfix.json")
    if not rep.get("handler"):
        pytest.skip("孪生报告是放件臂之前的版本")
    assert rep["handler"]["placed"] == rep["handler"]["items"] == 5
    assert all(p["by"] == "handler_arm" and p["place_err_mm"] < 0.1 for p in rep["place_steps"])
    assert rep["collisions"] == [] and rep["twin_ok"] is True
    assert rep["takt"]["handler_s"] > 0 and rep["takt"]["fasten_arm_s"] > 0


def test_tool_chosen_by_geometry(joint):
    """用户 2026-10-25：按零件外形自动选末端。joint-v1：壳体内撑、编码器外夹、支架内撑（指尖抬到编码器上方），
    驱动板太薄、端盖超出开口 → 吸盘；中间快换一次。"""
    out, _ = joint
    h = load_yaml(out / "handling.yaml")
    got = {it["part"]: (it["tool"], it["grasp"]["mode"]) for it in h["items"]}
    assert got == {"HSG-01": ("GR-2F85", "internal"), "ENC-01": ("GR-2F85", "external"), "BRK-01": ("GR-2F85", "internal"),
                   "PCB-01": ("GR-VG1", "vacuum"), "CAP-01": ("GR-VG1", "vacuum")}
    brk = next(it for it in h["items"] if it["part"] == "BRK-01")["grasp"]
    assert brk["tcp_part"][2] >= 18.0 and brk["engage_mm"] >= 5.0     # 指尖在编码器顶面（z=18）以上，接触 ≥ 5 mm
    assert h["tool_changes"] == 1
    assert any("超出开合范围" in w for w in next(it for it in h["items"] if it["part"] == "CAP-01")["why_tool"])


def test_fetch_screw_during_tool_change(joint):
    """放件臂中途换手时拧紧臂不干等（用户 2026-10-26）：放件臂到区域外给「已离开」，拧紧臂收回区域、取 D1 的螺钉、
    回停靠位、交出区域；放件臂回来先等区域空闲再放驱动板。D1 不再重复取钉。"""
    out, _ = joint
    h = load_yaml(out / "handling.yaml")
    assert h["tool_change_window_before_point"] == ["D1"]
    ops = json.loads((out / "arm_program.json").read_text(encoding="utf-8"))
    i = next(k for k, o in enumerate(ops) if o.get("until") == "zone_left")
    assert ops[i - 2]["purpose"] == "park" and ops[i - 1] == {"op": "signal", "name": "zone_free", "value": True}
    assert ops[i + 1] == {"op": "signal", "name": "zone_free", "value": False}
    j = next(k for k in range(i, len(ops)) if ops[k].get("until") == "placed_count")
    feed = [o for o in ops[i + 2:j] if o.get("purpose") == "feed"]
    assert feed and {o["point"] for o in feed} == {"D1"}
    assert ops[j - 2].get("prefetch") == "D1" and ops[j - 1] == {"op": "signal", "name": "zone_free", "value": True}
    assert ops[j]["value"] == 4 and ops[j + 1] == {"op": "signal", "name": "zone_free", "value": False}
    assert ops[j + 2]["point"] == "D1" and ops[j + 2].get("purpose") is None   # 直接去 D1 接近位
    assert sum(1 for o in ops if o.get("purpose") == "feed" and o["point"] == "D1") == len(feed)
    hops = json.loads((out / "handler_program.json").read_text(encoding="utf-8"))
    sig = [k for k, o in enumerate(hops) if o.get("name") == "zone_left"]
    assert [hops[k]["value"] for k in sig] == [True, False]
    a, b = sig
    assert hops[a - 1].get("via") and hops[a + 1].get("tool_change")                     # 到了快换中间点（区域外）才说已离开
    brk = max(k for k in range(a) if hops[k]["op"] == "grip" and not hops[k]["value"])
    assert hops[brk]["part"] == "BRK-01"                                                    # 支架放完才离开
    assert hops[b - 1] == {"op": "wait", "until": "zone_free"}                              # 回来先等区域空闲
    pcb = next(k for k in range(b, len(hops)) if hops[k]["op"] == "grip" and not hops[k]["value"])
    assert hops[pcb]["part"] == "PCB-01"

