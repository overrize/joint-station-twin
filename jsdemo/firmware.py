"""固件 / 控制程序生成（用户 2026-10-26：模块化——由上游工站设计直接得到各模块的控制程序与配置）。

输入：工站设计（jsdemo.station/1）+ 模块库的接口描述（kb/modules 的 io 段）+ 放件规划（handling.yaml，可选）。
输出（build/<工站>/firmware/）：
  io_map.yaml        I/O 分配表：每个模块信号 ↔ 工站控制器（PLC）地址；两台机械臂之间的握手由 PLC 转接
  station.st         工站总控程序（IEC 61131-3 结构化文本）：总状态机、治具区互斥、换盘、下线送出、灯塔
  config/*.yaml      各模块配置：供钉器料道、夹爪抓取表、快换工位表、料盘穴位表
机械臂程序（URScript）仍由 jsdemo.codegen 生成，I/O 编号与本表一致（测试核对）。确定性生成，不经过 LLM。

    python -m jsdemo.firmware kb/station/joint-v1.station.yaml [--handling build/station/handling.yaml] [--out build/station/firmware]
"""
from __future__ import annotations

import argparse
from pathlib import Path

from .io import ROOT, dump_yaml, load_modules, load_yaml, write_text

# 机械臂侧 I/O 约定：与 jsdemo.codegen.urscript / urscript_handler 一致（测试核对）
ARM_IO = {
    "fastening_arm": [
        ("DO", 0, "ready", "已锁定、可以拧 → 执行器 start"), ("DO", 1, "zone_free", "已让出治具区"), ("DO", 2, "unit_done", "整件拧完"),
        ("DI", 0, "exec_done", "← 执行器 done"), ("DI", 1, "exec_fault", "← 执行器 fault"), ("DI", 4, "feed_ok", "← 执行器 bit_vacuum_ok"),
        ("DI", 5, "zone_left", "← 放件臂已离开治具区（中途换手），可去取下一颗螺钉"),
        ("REG_IN", 0, "placed_count", "← 放件臂已放件数"),
    ],
    "handling_arm": [
        ("DO", 0, "zone_left", "已离开治具区（中途换手）→ 拧紧臂"), ("DI", 2, "zone_free", "← 拧紧臂已让出治具区"), ("DI", 3, "tray_ready", "← 输送线 tray_at_stop"), ("DI", 5, "unit_done", "← 拧紧臂整件拧完"),
        ("TOOL_DO", 0, "vacuum_on", "→ 吸盘 vacuum_on"), ("REG_OUT", 0, "placed_count", "→ 拧紧臂"), ("REG_OUT", 1, "unloaded", "整件已放到下游托盘"),
    ],
}
IEC = {"bool": ("X", "BOOL"), "int": ("W", "INT"), "real": ("D", "REAL")}


def _expand(sig, inst, mod):
    """per_lane / per_slot 信号展开成每路一个。"""
    if sig.get("per_lane"):
        return [(f"{sig['name']}_{i}", sig) for i in range(1, len(inst.get("lanes", [])) + 1 or mod.get("lanes", 1) + 1)]
    if sig.get("per_slot"):
        return [(f"{sig['name']}_{s['slot']}", sig) for s in inst.get("slots", [])]
    return [(sig["name"], sig)]


def io_map(spec: dict, modules: dict) -> dict:
    """按模块接口分配 PLC 地址：模块 out → PLC 输入（%I），模块 in → PLC 输出（%Q）。"""
    ctr = next(m for m in spec["modules"] if m["role"] == "controller")
    plc = modules[ctr["module"]].get("plc", {})
    nxt = {"IX": 0, "QX": 0, "IW": 0, "QW": 0, "ID": 0, "QD": 0}
    rows = []
    for m in spec["modules"]:
        mod = modules.get(m["module"], {})
        for name, sig in [x for s in mod.get("io", {}).get("signals", []) for x in _expand(s, m, mod)]:
            kind, iec = IEC[sig["type"]]
            area = ("I" if sig["dir"] == "out" else "Q") + kind
            idx = nxt[area]
            nxt[area] += 1
            addr = f"%{area}{idx // 8}.{idx % 8}" if kind == "X" else f"%{area}{idx}"
            rows.append({"module": m["id"], "model": m["module"], "signal": name, "dir": sig["dir"], "type": iec,
                         "plc": addr, "var": f"{m['id']}_{name}", "desc": sig.get("desc", "")})
    robot = []
    for m in spec["modules"]:
        for kind, idx, name, desc in ARM_IO.get(m["role"], []):
            robot.append({"robot": m["id"], "io": f"{kind}{idx}", "signal": name, "desc": desc, "var": f"{m['id']}_{name}"})
    used = {"digital_in": nxt["IX"], "digital_out": nxt["QX"], "words": nxt["IW"] + nxt["QW"] + nxt["ID"] + nxt["QD"]}
    over = [k for k in ("digital_in", "digital_out") if used[k] > plc.get(k, 0)]
    return {"controller": ctr["id"], "plc": plc, "used": used, "over_capacity": over, "signals": rows, "robot_io": robot,
            "note": "方向以模块为准：模块 out 接 PLC 输入（%I），模块 in 接 PLC 输出（%Q）。机械臂侧编号与 URScript 一致。"}


def station_st(spec: dict, io: dict, handling: dict | None) -> str:
    """工站总控 PLC 程序（IEC 61131-3 ST）。状态机与孪生 run_line 的流程一致：
    等料盘 → 装配（放件臂放件 / 拧紧臂拧紧，治具区互斥）→ 下线（放件臂把整件放到下游托盘）→ 送出 → 下一台。"""
    ids = {m["role"]: m["id"] for m in spec["modules"]}
    r1, r2, cv = ids.get("fastening_arm", "R1"), ids.get("handling_arm"), ids.get("conveyor", "CV1")
    n_items = len(handling["items"]) if handling else 0
    zones = ", ".join(z["id"] for z in spec["zones"])
    L = [f"(* 工站 {spec['meta']['station']} 总控程序：由 jsdemo.firmware 依据工站设计 {spec['meta']['station']}（jsdemo.station/1）生成，请勿手改 *)",
         f"(* 互斥分区：{zones}；握手：zone_free / placed_count / unit_done，中途换手 zone_left（control.interlock = handshake） *)",
         "PROGRAM Station", "VAR"]
    for r in io["signals"]:
        L.append(f"    {r['var']:<28} AT {r['plc']:<8} : {r['type']};   (* {r['model']} {r['signal']}：{r['desc']} *)")
    for r in io["robot_io"]:
        L.append(f"    {r['var']:<28} : {'INT' if 'REG' in r['io'] else 'BOOL'};   (* {r['robot']} {r['io']}：{r['desc']} *)")
    L += ["    State          : INT := 0;      (* 0 等料盘 1 装配 2 下线 3 送出 9 故障 *)",
          "    ZoneOwner      : INT := 0;      (* 治具区当前归属：1 拧紧臂 2 放件臂 0 空 *)",
          f"    ItemsPerUnit   : INT := {n_items};",
          "    UnitsDone      : INT := 0;",
          "    UnitsOK        : INT := 0;",
          "    AndonGreen, AndonYellow, AndonRed : BOOL;",
          "END_VAR", "",
          "(* ---- 机械臂之间的握手由 PLC 转接（两台臂不直接接线） ---- *)"]
    if r2:
        L += [f"{r2}_zone_free    := {r1}_zone_free;",
              f"{r1}_placed_count := {r2}_placed_count;",
              f"{r2}_unit_done    := {r1}_unit_done;",
              f"{r1}_zone_left    := {r2}_zone_left;",
              f"{r2}_tray_ready   := {cv}_tray_at_stop;", ""]
    L += ["(* ---- 治具区互斥：谁让出、谁进入，同一时刻只有一个 ---- *)",
          f"IF {r1}_zone_free THEN ZoneOwner := 2; ELSE ZoneOwner := 1; END_IF;", "",
          "(* ---- 总状态机 ---- *)",
          "CASE State OF",
          f"  0: (* 等料盘：排队位的料盘送到放件位 *)",
          f"     {cv}_tray_advance := {cv}_tray_at_prestop AND NOT {cv}_tray_at_stop;",
          f"     IF {cv}_tray_at_stop THEN {cv}_tray_advance := FALSE; State := 1; END_IF;",
          f"  1: (* 装配：放件臂放件、拧紧臂拧紧；料盘取空就下沉回流 *)",
          (f"     {cv}_tray_return := ({r1}_placed_count >= ItemsPerUnit);" if r2 else "     ;"),
          f"     IF {r1}_unit_done THEN State := 2; END_IF;",
          f"  2: (* 下线：放件臂把整件放到下游托盘 *)",
          (f"     IF {r2}_unloaded = 1 THEN State := 3; END_IF;" if r2 else "     State := 3;"),
          f"  3: (* 送出：托盘送往下游缓存，下一台 *)",
          f"     {cv}_out_advance := NOT {cv}_buffer_full;",
          f"     IF {cv}_pallet_at_out THEN {cv}_out_advance := FALSE; UnitsDone := UnitsDone + 1; State := 0; END_IF;",
          "  9: (* 故障：等人复位 *)",
          "     ;",
          "END_CASE;", "",
          "(* ---- 灯塔：绿 正常 / 黄 执行器复拧 / 红 不合格或故障 ---- *)"]
    ex = ids.get("executor", "EX1")
    fd = ids.get("feeder", "FD1")
    L += [f"AndonYellow := {ex}_retry_active;",
          f"AndonRed    := {ex}_fault OR {fd}_jam OR (State = 9);",
          "AndonGreen  := NOT AndonYellow AND NOT AndonRed;",
          "END_PROGRAM", ""]
    return "\n".join(L)


def module_configs(spec: dict, modules: dict, handling: dict | None) -> dict:
    inst = {m["id"]: m for m in spec["modules"]}
    out = {}
    for m in spec["modules"]:
        if m["role"] == "feeder":
            out[f"{m['id']}_feeder.yaml"] = {"module": m["module"], "lanes": m.get("lanes", []),
                                             "lane_capacity_pcs": modules[m["module"]].get("lane_capacity_pcs"), "feed_method": spec["process"].get("screw_feed")}
        if m["role"] == "tool_changer":
            out[f"{m['id']}_tool_changer.yaml"] = {"module": m["module"], "slots": [
                {"slot": s["slot"], "xy": s["xy"], "tool": next((g["id"] + " " + g["module"] for g in inst.values()
                                                                 if g.get("dock", {}).get("slot") == s["slot"] and g.get("dock", {}).get("changer") == m["id"]), None)}
                for s in m.get("slots", [])], "dock_z_mm": modules[m["module"]].get("dock_z_mm"), "change_s": modules[m["module"]].get("change_s")}
        if m["role"] == "tray":
            out[f"{m['id']}_tray.yaml"] = {"module": m["module"], "count": m.get("count", 1), "slots": spec.get("handling", {}).get("tray_slots", [])}
    if handling:
        rows = []
        for it in handling["items"]:
            g = it["grasp"]
            rows.append({"part": it["part"], "step": it["step"], "tool": it["tool"], "mode": g["mode"],
                         **({k: g[k] for k in ("grip_deg", "preset_deg", "diameter_mm", "phi_deg", "engage_mm") if k in g}),
                         "tcp_from_flange_mm": g.get("tip_from_flange_mm"), "rule": g.get("rule")})
        out["grasp_table.yaml"] = {"note": "放件臂抓取表：由 jsdemo.handling 按零件外形与末端实测几何规划", "items": rows}
    return out


def generate(spec_path: Path, out: Path, handling_path: Path | None = None) -> dict:
    spec = load_yaml(spec_path)
    modules = load_modules()
    handling = load_yaml(handling_path) if handling_path and Path(handling_path).exists() else None
    io = io_map(spec, modules)
    dump_yaml(io, out / "io_map.yaml")
    write_text(station_st(spec, io, handling), out / "station.st")
    files = ["io_map.yaml", "station.st"]
    for name, data in module_configs(spec, modules, handling).items():
        dump_yaml(data, out / "config" / name)
        files.append(f"config/{name}")
    write_text("# 固件与配置（由 jsdemo.firmware 依据工站设计生成）\n\n" + "\n".join(f"- `{f}`" for f in files) + "\n", out / "README.md")
    return {"files": files, "io_used": io["used"], "over_capacity": io["over_capacity"]}


def _main():
    ap = argparse.ArgumentParser(description="由工站设计生成 I/O 表、PLC 程序与模块配置")
    ap.add_argument("spec")
    ap.add_argument("--handling", default=str(ROOT / "build" / "station" / "handling.yaml"))
    ap.add_argument("--out", default=str(ROOT / "build" / "station" / "firmware"))
    a = ap.parse_args()
    r = generate(Path(a.spec), Path(a.out), Path(a.handling))
    print(f"固件 → {a.out}：{len(r['files'])} 个文件，PLC 数字量 入 {r['io_used']['digital_in']} / 出 {r['io_used']['digital_out']}"
          + (f"，超出容量 {r['over_capacity']}" if r["over_capacity"] else ""))


if __name__ == "__main__":
    _main()
