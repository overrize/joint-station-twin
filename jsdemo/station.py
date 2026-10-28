"""工站设计（jsdemo.station/1）校验：上游方案 agent 给出布局、治具建议与工站设计，工具链据此生成控制程序与孪生。

链路里只有一个 agent（用户 2026-10-26）：布局坐标、治具、分工、分区都由它给；这里只做确定性校验，
不替它改坐标。校验分四级，与 SOP 一致：error（结构错或做不到，打回）/ blocker（缺失或待确认，不能签字）/
review（交人判断）/ warning（估算、占位、可疑）。

    python -m jsdemo.station kb/station/joint-v1.station.yaml
"""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

from . import sop as sopmod
from .geometry import Pose
from .io import ROOT, load_library, load_modules, load_yaml, write_text
from . import layout

SCHEMA = "jsdemo.station/1"
MISSING = "缺失"
ROLES = {"fastening_arm", "handling_arm", "executor", "float_head", "gripper", "tool_changer", "feeder", "conveyor", "tray",
         "baseplate", "controller", "interface_box", "gateway", "vision_locator"}
ROLE_TYPE = {"fastening_arm": "arm", "handling_arm": "arm", "executor": "executor", "float_head": "float_head", "gripper": "gripper",
             "tool_changer": "tool_changer", "feeder": "feeder", "conveyor": "conveyor", "tray": "tray", "baseplate": "baseplate",
             "controller": "controller", "interface_box": "interface_box", "gateway": "gateway", "vision_locator": "vision_locator"}
REQUIRED_ROLES = {"fastening_arm", "executor", "feeder", "controller"}
SOFT_SOURCES = {"estimate", "placeholder"}
ARM_BASE_CLEAR_MM = 200.0     # 两台臂基座中心最小间距（垫高块 180 × 180）


class Findings:
    def __init__(self):
        self.items = []

    def add(self, level, code, path, message, fix=None):
        self.items.append({"level": level, "code": code, "path": path, "message": message, **({"fix": fix} if fix else {})})


def _miss(v) -> bool:
    return v == MISSING


def _num(v) -> bool:
    return isinstance(v, (int, float)) and not isinstance(v, bool)


def _vec(v, n=3) -> bool:
    return isinstance(v, list) and len(v) == n and all(_num(x) for x in v)


def _pose_of(m) -> Pose | None:
    p = m.get("pose")
    if not isinstance(p, dict) or not _vec(p.get("xyz")) or not _num(p.get("yaw", 0.0)):
        return None
    return Pose.from_xyz_rpy(p["xyz"], (0.0, 0.0, p.get("yaw", 0.0)))


def nest_points(feeder_pose: Pose, lanes: int, geo: dict) -> list[list[float]]:
    """供钉器各路出钉位顶面中心（工站坐标系）。外形取模块库 geometry，位姿取工站设计。"""
    sx, sy, sz = geo["size_mm"]
    out = []
    for i in range(lanes):
        lx = (i - (lanes - 1) / 2) * geo["lane_pitch_mm"]
        out.append(feeder_pose.apply([lx, -sy / 2 - geo["outlet_len_mm"] - geo["nest_size_mm"][1] / 2, sz / 2]))
    return out


def _reach(F, arm_id, arm, base: Pose, pts, path, what):
    for name, p in pts:
        d = math.dist(base.t, p)
        if not (arm["min_reach_mm"] <= d <= arm["reach_mm"]):
            F.add("error", "E-REACH", path, f"{arm_id} 够不到{what} {name}：距基座 {d:.0f} mm，可达 {arm['min_reach_mm']}–{arm['reach_mm']} mm",
                  "调整该模块或机械臂的位姿")


def validate(spec: dict, *, modules=None, library=None) -> dict:
    modules = modules or load_modules()
    library = library or load_library()
    F = Findings()
    for key in ("schema", "meta", "units", "frame", "modules", "fixture", "process", "zones", "control", "open_items"):
        if key not in spec:
            F.add("error", "E-SCHEMA", key, f"缺少顶层字段 {key}")
    if spec.get("schema") != SCHEMA:
        F.add("error", "E-SCHEMA", "schema", f"schema 应为 {SCHEMA}")
    if spec.get("units") != {"length": "mm", "angle": "deg", "time": "s"}:
        F.add("error", "E-UNIT", "units", "units 固定为 {length: mm, angle: deg, time: s}")
    if any(i["level"] == "error" for i in F.items):
        return _result(F)

    meta = spec["meta"]
    for k in ("station", "product", "sop", "generated_by", "status"):
        if k not in meta:
            F.add("error", "E-SCHEMA", f"meta.{k}", f"缺少 meta.{k}")
    sop_path = ROOT / meta.get("sop", "")
    sop = load_yaml(sop_path) if meta.get("sop") and sop_path.exists() else None
    if sop is None:
        F.add("error", "E-REF", "meta.sop", f"找不到 SOP {meta.get('sop')}（工站设计必须引用一份 jsdemo.sop/1）")
        return _result(F)
    sres = sopmod.validate(sop, modules=modules)
    if not sres["signable"]:
        F.add("blocker", "B-SOP", "meta.sop", f"引用的 SOP 还不能签字（缺失 {sres['counts']['blocker']}、error {sres['counts']['error']}），工站设计随之不能签字")
    if sop["meta"]["product"] != meta.get("product"):
        F.add("error", "E-REF", "meta.product", f"产品 {meta.get('product')} 与 SOP 的 {sop['meta']['product']} 不一致")

    env = spec["frame"].get("envelope", {})
    if not (_vec(env.get("x"), 2) and _vec(env.get("y"), 2)):
        F.add("error", "E-SCHEMA", "frame.envelope", "需要 envelope.x / envelope.y（工站可用空间，mm）")
        env = None

    # ---------- 模块 ----------
    inst = {}
    for i, m in enumerate(spec["modules"]):
        path = f"modules[{i}]"
        mid, cat, role = m.get("id"), m.get("module"), m.get("role")
        if not mid or mid in inst:
            F.add("error", "E-DUP", path, f"实例 id {mid!r} 缺失或重复")
            continue
        inst[mid] = m
        if role not in ROLES:
            F.add("error", "E-SCHEMA", f"{path}.role", f"不认识的 role {role!r}（可用：{sorted(ROLES)}）")
        if _miss(cat):
            F.add("blocker", "B-MISSING", f"{path}.module", f"{mid} 的型号缺失", "从模块库选型")
            continue
        if cat not in modules:
            F.add("error", "E-REF", f"{path}.module", f"{mid}：模块库里没有 {cat}", "只能选库里的型号；库里没有就写进 open_items")
            continue
        if role in ROLE_TYPE and modules[cat]["type"] != ROLE_TYPE[role]:
            F.add("error", "E-REF", f"{path}.module", f"{mid}：{cat} 是 {modules[cat]['type']}，不能当 {role}")
        if "pose" in m:
            P = _pose_of(m)
            if P is None:
                F.add("error", "E-SCHEMA", f"{path}.pose", f"{mid} 的位姿应为 {{xyz: [x, y, z], yaw: deg}}")
            elif env and not (env["x"][0] <= P.t[0] <= env["x"][1] and env["y"][0] <= P.t[1] <= env["y"][1]):
                F.add("error", "E-OUTSIDE", f"{path}.pose", f"{mid} 在工站可用空间外：{P.t[:2]}")
        if "mount" in m and m["mount"].get("parent") not in inst and m["mount"].get("parent") not in [x.get("id") for x in spec["modules"]]:
            F.add("error", "E-REF", f"{path}.mount", f"{mid} 装在不存在的 {m['mount'].get('on')} 上")
        if role == "gripper":
            if m.get("tool_of") not in [x.get("id") for x in spec["modules"]]:
                F.add("error", "E-REF", f"{path}.tool_of", f"{mid} 的 tool_of 指向不存在的机械臂")
        # 生成器直接用的字段（布局从工站设计读，缺了就生成不了程序）
        if role in ("fastening_arm", "handling_arm"):
            for k in ("home_joints_deg",) + (("park_joints_deg",) if role == "fastening_arm" and spec.get("process", {}).get("parallel") else ()):
                v = m.get(k)
                if not (isinstance(v, list) and len(v) == 6 and all(isinstance(x, (int, float)) for x in v)):
                    F.add("error", "E-SCHEMA", f"{path}.{k}", f"{mid} 需要 {k}：6 个关节角（deg）"
                          + ("；两臂并行时拧紧臂要有让出治具上方的停靠位" if k == "park_joints_deg" else ""))
        if role == "feeder" and "pose" in m and abs(float(m["pose"].get("yaw", 0.0) or 0.0)) > 1e-9:
            F.add("error", "E-UNSUPPORTED", f"{path}.pose.yaw", f"{mid}：v1 供钉器只支持 yaw 0（出钉口朝 -y）")
        if role == "tool_changer" and not all(_vec(s.get("xy"), 2) for s in m.get("slots", [])):
            F.add("error", "E-SCHEMA", f"{path}.slots", f"{mid} 的每个工位要给 xy（工站坐标系）")
    roles = {}
    for m in inst.values():
        if m.get("module") in modules:   # 型号不在库里的已报 E-REF，后续检查跳过
            roles.setdefault(m.get("role"), []).append(m)
    for r in REQUIRED_ROLES - set(roles):
        F.add("error", "E-MISSING", "modules", f"缺少 {r}")

    # 两台臂基座不能挤在一起
    arms = [m for m in inst.values() if m.get("role") in ("fastening_arm", "handling_arm") and _pose_of(m)]
    for a in range(len(arms)):
        for b in range(a + 1, len(arms)):
            d = math.dist(_pose_of(arms[a]).t[:2], _pose_of(arms[b]).t[:2])
            if d < ARM_BASE_CLEAR_MM:
                F.add("error", "E-OVERLAP", "modules", f"{arms[a]['id']} 与 {arms[b]['id']} 基座间距 {d:.0f} mm < {ARM_BASE_CLEAR_MM:.0f}")

    # 快换架工位
    for m in roles.get("gripper", []):
        dk = m.get("dock")
        if dk:
            tc = inst.get(dk.get("changer"))
            if not tc or dk.get("slot") not in [s.get("slot") for s in tc.get("slots", [])]:
                F.add("error", "E-REF", f"modules.{m['id']}.dock", f"{m['id']} 的快换工位 {dk} 不存在")
    if len(roles.get("gripper", [])) > 1 and not roles.get("tool_changer"):
        F.add("error", "E-MISSING", "modules", "一台臂配了多个末端，但没有快换架")

    # ---------- 治具 ----------
    fx = spec["fixture"]
    FX = None
    if not isinstance(fx.get("pose"), dict) or not _vec(fx["pose"].get("xyz")):
        F.add("error", "E-SCHEMA", "fixture.pose", "需要治具位姿 {xyz, yaw}（工站坐标系）")
    else:
        FX = Pose.from_xyz_rpy(fx["pose"]["xyz"], (0.0, 0.0, fx["pose"].get("yaw", 0.0)))
    if fx.get("use"):
        fm = modules.get(fx["use"])
        if not fm or fm["type"] != "fixture":
            F.add("error", "E-REF", "fixture.use", f"模块库里没有治具 {fx['use']}", "改为给出 suggestion（新治具设计建议）")
        elif fm.get("product") != meta.get("product"):
            F.add("review", "R-FIXTURE", "fixture.use", f"借用 {fx['use']}（为 {fm.get('product')} 设计），定位与夹持是否适配需确认")
        if sop["frame"]["mounting_in_fixture"].get("fixture") not in (fx["use"], MISSING):
            F.add("error", "E-REF", "fixture.use", f"与 SOP 装夹所用治具 {sop['frame']['mounting_in_fixture'].get('fixture')} 不一致")
    else:
        sg = fx.get("suggestion") or {}
        if not sg:
            F.add("error", "E-MISSING", "fixture", "既没有 use（库里的治具），也没有 suggestion（新治具建议）")
        else:
            ds = sg.get("datums", [])
            if len(ds) < 3 or not all(_vec(d.get("xyz")) for d in ds):
                F.add("error", "E-SCHEMA", "fixture.suggestion.datums", "新治具至少给 3 个基准点 {id, xyz}（治具坐标系），现场自标定要用")
            else:
                a, b, c = (ds[i]["xyz"] for i in range(3))
                cr = [(b[1] - a[1]) * (c[2] - a[2]) - (b[2] - a[2]) * (c[1] - a[1]), (b[2] - a[2]) * (c[0] - a[0]) - (b[0] - a[0]) * (c[2] - a[2]),
                      (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])]
                if math.hypot(*cr) < 1e-6:
                    F.add("error", "E-GEOM", "fixture.suggestion.datums", "前 3 个基准点共线，定不出治具坐标系")
            for k in ("type", "locating", "clamping"):
                if _miss(sg.get(k)) or k not in sg:
                    F.add("blocker", "B-MISSING", f"fixture.suggestion.{k}", f"新治具的 {k} 缺失")
            F.add("review", "R-FIXTURE", "fixture.suggestion", "新治具为设计建议，需结构工程师确认后才能制作")
            sop_fx = sop["frame"]["mounting_in_fixture"].get("fixture")
            if sop_fx not in (MISSING, sg.get("id")):
                F.add("error", "E-REF", "fixture.suggestion", f"SOP 的装夹写的是治具 {sop_fx}，与新治具建议不一致",
                      "新治具给一个 id，并让 SOP 的 frame.mounting_in_fixture.fixture 引用它")

    # ---------- 供钉 ----------
    product = sopmod.to_product(sop)
    screw_kinds = sorted({g["screw"] for g in product["fastening"]["groups"]})
    for m in roles.get("feeder", []):
        lanes = m.get("lanes", [])
        cat = modules.get(m.get("module"), {})
        if len(lanes) > cat.get("lanes", 0):
            F.add("error", "E-CAPACITY", f"modules.{m['id']}.lanes", f"{m['module']} 只有 {cat.get('lanes')} 路，配了 {len(lanes)} 路")
        have = {ln.get("screw") for ln in lanes}
        for sk in screw_kinds:
            if sk not in have:
                F.add("error", "E-FEED", f"modules.{m['id']}.lanes", f"SOP 用到的螺钉 {sk} 没有分配料道")
        for ln in lanes:
            if ln.get("screw") not in library["screws"]:
                F.add("error", "E-REF", f"modules.{m['id']}.lanes", f"料道 {ln.get('lane')} 的螺钉 {ln.get('screw')} 不在工艺库")

    # ---------- 分工 ----------
    steps = {s["id"]: s for s in sop["steps"]}
    alloc = spec["process"].get("allocation", [])
    seen = {}
    for i, a in enumerate(alloc):
        path = f"process.allocation[{i}]"
        st = steps.get(a.get("sop_step"))
        if not st:
            F.add("error", "E-REF", path, f"SOP 里没有工步 {a.get('sop_step')}")
            continue
        if a["sop_step"] in seen:
            F.add("error", "E-DUP", path, f"工步 {a['sop_step']} 重复分配")
        seen[a["sop_step"]] = a
        res = a.get("resource")
        if res in ("operator", "upstream"):
            if st["type"] == "fasten":
                F.add("error", "E-ALLOC", path, f"拧紧工步 {st['id']} 只能由拧紧臂完成")
            continue
        r = inst.get(res)
        if not r:
            F.add("error", "E-REF", path, f"资源 {res} 不存在")
            continue
        if st["type"] == "fasten" and r.get("role") != "fastening_arm":
            F.add("error", "E-ALLOC", path, f"拧紧工步 {st['id']} 分给了 {r.get('role')}")
        if st["type"] == "place" and r.get("role") not in ("handling_arm",):
            F.add("error", "E-ALLOC", path, f"放件工步 {st['id']} 分给了 {r.get('role')}（放件用 handling_arm，或 operator / upstream）")
        if st["type"] == "place" and st.get("actor") not in ("robot", MISSING):
            F.add("review", "R-ALLOC", path, f"SOP 写 {st['id']} 由 {st.get('actor')} 完成，工站设计改给 {res}")
        tool = a.get("tool")
        if tool not in ("auto", None) and (tool not in inst or (inst[tool].get("tool_of") != res and inst[tool].get("mount", {}).get("parent") != res)):
            F.add("error", "E-REF", path, f"工具 {tool} 不属于 {res}")
    for sid in steps:
        if sid not in seen:
            F.add("error", "E-ALLOC", "process.allocation", f"SOP 工步 {sid} 没有分配资源")

    # 料盘穴位 ↔ 放件臂的放件工步
    tray = (roles.get("tray") or [None])[0]
    slots = spec.get("handling", {}).get("tray_slots", [])
    handled = [steps[a["sop_step"]]["part"] for a in alloc if a.get("sop_step") in steps and steps[a["sop_step"]]["type"] == "place"
               and inst.get(a.get("resource"), {}).get("role") == "handling_arm"]
    if handled and not tray:
        F.add("error", "E-MISSING", "modules", "放件臂要取件，但没有料盘")
    if tray and len(slots) > modules[tray["module"]]["slots"]:
        F.add("error", "E-CAPACITY", "handling.tray_slots", f"{tray['module']} 只有 {modules[tray['module']]['slots']} 穴")
    in_tray = [s.get("part") for s in slots]
    for pid in handled:
        if pid not in in_tray:
            F.add("error", "E-FEED", "handling.tray_slots", f"放件臂要放 {pid}，料盘里没有它的穴位")
    for pid in in_tray:
        if pid not in handled:
            F.add("warning", "W-TRAY", "handling.tray_slots", f"料盘里有 {pid}，但没有分给放件臂的放件工步")

    # ---------- 可达 ----------
    if FX is not None:
        M = sopmod._mount(sop)
        pts = [(p["id"], FX.apply(p["xyz"])) for p in product["fastening"]["points"]]
        for r in roles.get("fastening_arm", []):
            B = _pose_of(r)
            if B:
                _reach(F, r["id"], modules[r["module"]], B, pts, f"modules.{r['id']}", "紧固点")
                for fd in roles.get("feeder", []):
                    P = _pose_of(fd)
                    if P:
                        nps = nest_points(P, modules[fd["module"]]["lanes"], modules[fd["module"]]["geometry"])
                        _reach(F, r["id"], modules[r["module"]], B, [(f"{fd['id']} 第 {i + 1} 路出钉位", p) for i, p in enumerate(nps)],
                               f"modules.{fd['id']}", "出钉位")
        for r in roles.get("handling_arm", []):
            B = _pose_of(r)
            cv = (roles.get("conveyor") or [{}])[0]
            stops = cv.get("stops", {})
            tg = [("治具", FX.t)]
            for k in ("tray_stop", "out_stop"):
                if _vec(stops.get(k)):
                    tg.append((k, stops[k]))
                elif r and handled:
                    F.add("error", "E-MISSING", f"modules.{cv.get('id')}.stops.{k}", f"输送线缺少 {k}")
            for tc in roles.get("tool_changer", []):
                tg += [(f"{tc['id']} 工位 {s['slot']}", [s["xy"][0], s["xy"][1], modules[tc["module"]]["dock_z_mm"]]) for s in tc.get("slots", [])]
            if B:
                _reach(F, r["id"], modules[r["module"]], B, tg, f"modules.{r['id']}", "")
        _ = M

    # ---------- 分区与控制 ----------
    zone_ids = set()
    for i, z in enumerate(spec["zones"]):
        zone_ids.add(z.get("id"))
        for rid in z.get("exclusive", []):
            if rid not in inst:
                F.add("error", "E-REF", f"zones[{i}].exclusive", f"分区 {z.get('id')} 引用了不存在的 {rid}")
        b = z.get("box", {})
        if not (_vec(b.get("center")) and _vec(b.get("size"))):
            F.add("error", "E-SCHEMA", f"zones[{i}].box", "分区用 box {center, size} 描述")
    two_arms = len([m for m in inst.values() if m.get("role") in ("fastening_arm", "handling_arm")]) >= 2
    if two_arms and spec["process"].get("parallel") and FX is not None:
        inside = [z for z in spec["zones"] if _vec(z.get("box", {}).get("center")) and
                  all(abs(FX.t[k] - z["box"]["center"][k]) <= z["box"]["size"][k] / 2 for k in range(2))]
        if not inside:
            F.add("error", "E-ZONE", "zones", "两臂并行，但治具不在任何互斥分区里", "给治具上方加一个 exclusive: [拧紧臂, 放件臂] 的分区")
    if spec["control"].get("interlock") not in ("handshake",):
        F.add("error", "E-SCHEMA", "control.interlock", "v1 只支持 handshake（zone_free / placed_count / unit_done）")

    # ---------- 缺失与来源 ----------
    def walk(v, path):
        if _miss(v):
            F.add("blocker", "B-MISSING", path, "缺失")
        elif isinstance(v, dict):
            if v.get("source") in SOFT_SOURCES:
                F.add("warning", "W-SOFT", path, f"来源为 {v['source']}，签字前需确认")
            for k, x in v.items():
                walk(x, f"{path}.{k}")
        elif isinstance(v, list):
            for k, x in enumerate(v):
                walk(x, f"{path}[{k}]")
    for key in ("targets", "line", "fixture", "handling", "control"):
        walk(spec.get(key), key)
    for i, it in enumerate(spec.get("open_items", [])):
        F.add("blocker", "B-OPEN", f"open_items[{i}]", it.get("text", str(it)))
    if meta.get("status") == "signed" and any(i["level"] in ("error", "blocker") for i in F.items):
        F.add("error", "E-SIGN", "meta.status", "有 error / blocker 不能标 signed")
    return _result(F)


def _result(F: Findings) -> dict:
    counts = {k: sum(1 for i in F.items if i["level"] == k) for k in ("error", "blocker", "review", "warning")}
    return {"schema": SCHEMA, "counts": counts, "buildable": counts["error"] == 0,
            "signable": counts["error"] == 0 and counts["blocker"] == 0, "findings": F.items}


def to_layout(spec: dict) -> dict:
    """工站设计 → 工具链用的布局数据（同 jsdemo.layout.from_spec）。"""
    return layout.from_spec(spec)


def run(path: Path, out_root: Path | None = None) -> dict:
    spec = load_yaml(path)
    res = validate(spec)
    out = (out_root or ROOT / "build" / "station_spec") / spec.get("meta", {}).get("station", "unknown")
    write_text(json.dumps({"station_spec": str(path), **res}, ensure_ascii=False, indent=2), out / "validation.json")
    return res


def _main():
    ap = argparse.ArgumentParser(description="校验工站设计（jsdemo.station/1）")
    ap.add_argument("path")
    a = ap.parse_args()
    res = run(Path(a.path))
    c = res["counts"]
    print(f"工站设计 {a.path}：error {c['error']}、缺失 {c['blocker']}、待审 {c['review']}、警告 {c['warning']}；"
          f"{'可生成' if res['buildable'] else '打回'}，{'可签字' if res['signable'] else '不可签字'}")
    for f in res["findings"]:
        print(f"  [{f['level']}] {f['code']} {f['path']}：{f['message']}")


if __name__ == "__main__":
    _main()
