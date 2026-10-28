"""固件生成（用户 2026-10-26：模块化——由工站设计直接得到控制程序与各模块配置）。"""
import re
from pathlib import Path

import pytest

from jsdemo import firmware
from jsdemo.io import load_modules, load_yaml

ROOT = Path(__file__).resolve().parents[1]
SPEC = ROOT / "kb" / "station" / "joint-v1.station.yaml"


@pytest.fixture(scope="module")
def fw(tmp_path_factory):
    out = tmp_path_factory.mktemp("fw")
    h = ROOT / "build" / "station" / "handling.yaml"
    r = firmware.generate(SPEC, out, h if h.exists() else None)
    return out, r


def test_io_map_complete_and_unique(fw):
    out, r = fw
    io = load_yaml(out / "io_map.yaml")
    addrs = [s["plc"] for s in io["signals"]]
    assert len(addrs) == len(set(addrs))                                      # PLC 地址不重复
    spec, mods = load_yaml(SPEC), load_modules()
    expected = 0
    for m in spec["modules"]:
        for sig in mods[m["module"]].get("io", {}).get("signals", []):
            expected += len(m.get("lanes", [])) if sig.get("per_lane") else len(m.get("slots", [])) if sig.get("per_slot") else 1
    assert len(io["signals"]) == expected                                     # 每个模块接口信号都分配了
    for s in io["signals"]:                                                   # 模块 out 接 PLC 输入，in 接输出
        assert s["plc"].startswith("%I") == (s["dir"] == "out")
    assert not r["over_capacity"]


def test_robot_io_matches_urscript(fw, run):
    out, _ = fw
    io = load_yaml(out / "io_map.yaml")
    r1 = {x["signal"]: x["io"] for x in io["robot_io"] if x["robot"] == "R1"}
    src = (run["out"] / "arm_program.urscript").read_text(encoding="utf-8")
    assert f"set_standard_digital_out({r1['ready'][2:]}, True)" in src
    assert f"get_standard_digital_in({r1['exec_done'][2:]})" in src and f"get_standard_digital_in({r1['exec_fault'][2:]})" in src


def test_st_declares_what_it_uses(fw):
    out, _ = fw
    st = (out / "station.st").read_text(encoding="utf-8")
    body = st.split("END_VAR", 1)[1]
    declared = set(re.findall(r"^\s+(\w+)\s+(?:AT|:)", st.split("END_VAR", 1)[0], re.M))
    declared |= {"State", "ZoneOwner", "ItemsPerUnit", "UnitsDone", "UnitsOK", "AndonGreen", "AndonYellow", "AndonRed"}
    code = re.sub(r"\(\*.*?\*\)", "", body, flags=re.S)
    used = {w for w in re.findall(r"\b[A-Za-z_][A-Za-z0-9_]*\b", code)
            if w not in {"IF", "THEN", "ELSE", "END_IF", "CASE", "OF", "END_CASE", "AND", "OR", "NOT", "TRUE", "FALSE", "END_PROGRAM"}}
    assert used <= declared, used - declared
    assert "R2_zone_free    := R1_zone_free;" in st                         # 两臂握手由 PLC 转接


def test_module_configs(fw):
    out, _ = fw
    fd = load_yaml(out / "config" / "FD1_feeder.yaml")
    assert [ln["screw"] for ln in fd["lanes"]] == ["M2x5-machine", "M2x6-tapping", "M3x8-machine"]
    tc = load_yaml(out / "config" / "TC1_tool_changer.yaml")
    assert [s["tool"].split()[1] for s in tc["slots"]] == ["GR-2F85", "GR-VG1"]
    if (out / "config" / "grasp_table.yaml").exists():
        g = load_yaml(out / "config" / "grasp_table.yaml")
        assert {r["part"]: r["mode"] for r in g["items"]}["HSG-01"] == "internal"
