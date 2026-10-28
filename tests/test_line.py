"""流水线（M4 ③，用户 2026-10-25）：装好的整件由放件臂吸到下游托盘送走，两台串行。孪生报告不存在时跳过孪生部分。"""
import json
from pathlib import Path

import pytest

from jsdemo.io import load_yaml
from jsdemo.layout import OUT_STOP
from jsdemo.pipeline import Runner

ROOT = Path(__file__).resolve().parents[1]
SOP_J = ROOT / "kb" / "sop" / "joint-v1.sop.yaml"


@pytest.fixture(scope="module")
def joint(tmp_path_factory):
    out = tmp_path_factory.mktemp("line_joint")
    return out, Runner(out, yes=True, quiet=True, sop=str(SOP_J)).run()


def test_fastening_arm_signals_unit_done(joint):
    out, _ = joint
    ops = json.loads((out / "arm_program.json").read_text(encoding="utf-8"))
    assert ops[-2].get("purpose") == "park" and ops[-1] == {"op": "signal", "name": "unit_done", "value": True}
    assert "set_standard_digital_out(2, True)" in (out / "arm_program.urscript").read_text(encoding="utf-8")


def test_unload_phase(joint):
    out, _ = joint
    h = load_yaml(out / "handling.yaml")
    ul = h["unload"]
    assert ul["tool"] == "GR-VG1" and ul["unit_height_mm"] == 40.0
    assert ul["place_world"][:2] == OUT_STOP["xyz_mm"][:2]
    ops = json.loads((out / "handler_program.json").read_text(encoding="utf-8"))
    ph = [o for o in ops if o.get("phase") == "unload"]
    assert ph[0] == {"op": "wait", "until": "unit_done", "phase": "unload"}
    assert [(o["part"], o["value"]) for o in ph if o["op"] == "grip"] == [("UNIT", True), ("UNIT", False)]
    # 每个循环结束时工具放回快换架：下一台从空手开始，程序开头先去接工具
    tools = [o for o in ops if o["op"] == "tool"]
    assert tools[0]["action"] == "attach" and tools[-1]["action"] == "release" and tools[-1].get("phase") == "unload"
    # 去快换架前都经过避开放件臂基座的中间点
    for i, o in enumerate(ops):
        if o["op"] == "tool":
            assert any(x.get("via") for x in ops[max(0, i - 4):i])


def test_twin_two_units():
    p = ROOT / "build" / "twin" / "twin_report_dfmfix_line_line.json"
    if not p.exists():
        pytest.skip("还没有两台流水线的孪生报告（run_flow.py --dfm-fix --units 2 --label line）")
    d = json.loads(p.read_text(encoding="utf-8"))
    assert d["summary"]["n"] >= 2 and d["summary"]["twin_ok"] is True      # 用户 2026-10-26：三台连续流动
    for k, u in enumerate(d["units"], start=1):
        assert u["line"]["unit"] == k and u["line"]["sn"].endswith(f"-{k}")
        assert u["unit_result"] == "OK" and u["collisions"] == []
        assert u["points_arrived"] == u["points_total"] == 13
        assert u["handler"]["placed"] == 5
    assert d["units"][1]["line"]["t_start_s"] > d["units"][0]["line"]["t_start_s"] + d["units"][0]["line"]["unit_s"]   # 治具同一时刻只装一台
    # 连续流动：开工间隔 = 上一台的周期（下一盘配件已在排队，不再等料盘退回再送进来）
    for a, b in zip(d["units"], d["units"][1:]):
        assert a["line"]["interval_s"] == pytest.approx(b["line"]["t_start_s"] - a["line"]["t_start_s"], abs=0.05)
        assert a["line"]["interval_s"] <= a["line"]["cycle_s"] + 0.5
