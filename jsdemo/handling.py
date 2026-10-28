"""放件（M4 ②）：输送线送配套料盘，放件臂（第二台机械臂）从料盘取件、放到治具上，拧紧臂只等信号。

用户 2026-10-25 定：
- 输送线送件 + 第二台机械臂放件；demo 阶段布局、料盘、吸盘、快换架都用占位值并标 demo；
- 取件末端：Robotiq 2F-85 两指夹爪（外夹 / 内撑）+ 真空吸盘，放件臂上加快换；
- 末端由规划器按零件外形自动选：能夹、且放到位时手指与本体不碰已装零件 → 夹爪；否则能吸 → 吸盘；都不行就报出来。

其余约定：
- 哪些放件工步归放件臂：SOP 里 actor 为 robot 的；actor「缺失」的演示时也交给放件臂（标 assumed）；
  upstream 的开始即在位；operator 的仍由人工放（孪生里落位动画）。
- 料盘：零件按放件顺序进槽位，零件偏航与装到治具上时相同，搬运中工具不转。
- 两台臂不同时进治具上方：拧紧臂在需要新零件前退到停靠位、给「区域空闲」，等放件计数到数再进；
  放件臂放完一批回停靠位、写放件计数。孪生按这个握手顺序执行（两臂不并行），节拍里分开记。
- 程序都在各自基座坐标系里：放件臂程序写放件臂基座系位姿，与拧紧臂一样由治具位姿换算，点位源头仍在治具坐标系。
"""
from __future__ import annotations

import math

from .geometry import Pose
from . import layout   # 布局取当前工站设计（jsdemo.station/1）

TRANSIT_CLEAR_MM = 40.0
NEAR_MM = 10.0          # 快速降到目标上方这么高，最后一段慢速（取件与放下）
DOCK_CLEAR_MM = 60.0    # 快换：工位上方这么高为接近点
ACTORS_FOR_HANDLER = {"robot", "缺失"}
FINGER_CIRCLES = (-7.5, 0.0, 7.5)   # 单指（厚 × 宽）用沿宽度方向的 3 个小圆覆盖，圆半径 = 厚度 / 2 + 1.3


def _top(sh):
    return sh["z"][1] if sh["type"] != "box" else sh["center"][2] + sh["size"][2] / 2


def _bottom(sh):
    return sh["z"][0] if sh["type"] != "box" else sh["center"][2] - sh["size"][2] / 2


def _covers_origin(sh) -> bool:
    if sh["type"] == "cylinder":
        cx, cy = sh.get("xy", [0.0, 0.0])
        return math.hypot(cx, cy) < sh["r"]
    if sh["type"] == "box":
        c, s = sh["center"], sh["size"]
        return abs(c[0]) < s[0] / 2 and abs(c[1]) < s[1] / 2
    return False


def pick_point(part: dict, cup_r: float) -> dict:
    """吸盘：零件坐标系里的吸取点（吸盘面中心）。"""
    shapes = part["shapes"]
    cover = [s for s in shapes if _covers_origin(s)]
    if cover:
        top = max(_top(s) for s in cover)
        return {"xyz": [0.0, 0.0, top], "rule": "覆盖原点的简化体顶面中心"}
    rings = [s for s in shapes if s["type"] == "ring" and s["r_out"] - s["r_in"] >= 2 * cup_r]
    if rings:
        ring = max(rings, key=_top)
        rm = (ring["r_in"] + ring["r_out"]) / 2
        cx, cy = ring.get("xy", [0.0, 0.0])
        comps = [c["shape"] for c in part.get("components", [])]
        for k in range(16):
            a = 2 * math.pi * k / 16
            x, y = cx + rm * math.cos(a), cy + rm * math.sin(a)
            if all(math.hypot(x - c.get("xy", [0, 0])[0], y - c.get("xy", [0, 0])[1]) > cup_r + c.get("r", 0.0) + 2.0
                   for c in comps if c["type"] == "cylinder"):
                return {"xyz": [round(x, 3), round(y, 3), _top(ring)], "rule": f"环面中线、避开元件（{round(math.degrees(a))}°）"}
    raise ValueError(f"{part['id']}：找不到吸盘能吸的平面（demo 规则只认覆盖原点的顶面或足够宽的环面）")


def extent_r(part: dict) -> float:
    r = 0.0
    for s in part["shapes"]:
        cx, cy = s.get("xy", [0.0, 0.0]) if s["type"] != "box" else s["center"][:2]
        rr = s.get("r", s.get("r_out")) if s["type"] != "box" else math.hypot(s["size"][0], s["size"][1]) / 2
        r = max(r, math.hypot(cx, cy) + rr)
    return r


# ---------- 两指夹爪 ----------
def _interp(table, col_in, val, col_out):
    rows = sorted(table, key=lambda r: r[col_in])
    if not rows[0][col_in] <= val <= rows[-1][col_in]:
        return None
    for a, b in zip(rows, rows[1:]):
        if a[col_in] <= val <= b[col_in]:
            f = 0.0 if b[col_in] == a[col_in] else (val - a[col_in]) / (b[col_in] - a[col_in])
            return a[col_out] + (b[col_out] - a[col_out]) * f
    return None


def finger_segments(g: dict, mode: str, r_contact: float, phi: float) -> list[dict]:
    """夹爪在 TCP（两指尖中点）坐标系里的外形：两根手指各 3 个小圆 + 本体大圆柱。z 从指尖向上量。
    phi：开合方向在治具系里的角度。外夹时手指在 r_contact 外侧，内撑时在内侧。"""
    t, w, L = g["finger_mm"]["thickness"], g["finger_mm"]["width"], g["finger_mm"]["length"]
    rc = r_contact + t / 2 if mode == "external" else r_contact - t / 2
    u, v = (math.cos(phi), math.sin(phi)), (-math.sin(phi), math.cos(phi))
    segs = []
    for side in (1, -1):
        for k, off in enumerate(FINGER_CIRCLES):
            segs.append({"name": f"finger{'L' if side > 0 else 'R'}{k}", "z": [0.0, L], "r": t / 2 + 1.3,
                         "dxy": [side * u[0] * rc + v[0] * off, side * u[1] * rc + v[1] * off]})
    b = g["body_mm"]
    segs.append({"name": "gripper_body", "z": [b["from_tip"], b["from_tip"] + 120.0], "r": b["radius"]})
    return segs


def grasp_2f(part: dict, g: dict, blocked) -> tuple[dict | None, list[str]]:
    """两指夹爪的抓取：逐个圆柱 / 环简化体试外夹与内撑，指尖从零件底面往上找第一个放到位不碰的高度。
    blocked(segs, tcp_part) → 碰撞描述或 None：在零件装配位姿下检查这些工具段是否碰已装零件。返回 (抓取, 试过的原因)。"""
    rules, tab = g["grip_rules"], g["open_table"]
    clr, eng = rules["clearance_mm"], rules["min_engage_mm"]
    tried = []
    for k, sh in enumerate(part["shapes"]):
        if sh["type"] not in ("cylinder", "ring"):
            continue
        cx, cy = sh.get("xy", [0.0, 0.0])
        modes = [("external", sh["r"] if sh["type"] == "cylinder" else sh["r_out"])]
        if sh["type"] == "ring":
            modes.append(("internal", sh["r_in"]))
        for mode, r in modes:
            D = 2 * r
            if mode == "external":
                grip = _interp(tab, 1, D, 0)          # 内侧间距 = 外径
                pre = _interp(tab, 1, D + 2 * clr, 0)
            else:
                grip = _interp(tab, 2, D, 0)          # 外侧跨度 = 内孔
                pre = _interp(tab, 2, D - 2 * clr, 0)
            if grip is None or pre is None:
                tried.append(f"s{k} {mode} Ø{D:.1f}：超出开合范围")
                continue
            tip_off = _interp(tab, 0, grip, 3)
            z0, z1 = sh["z"]
            tip = z0 + rules["tip_above_bottom_mm"]
            found = None
            while z1 - tip >= eng - 1e-9 and not found:
                for phi_deg in range(0, 180, 15):
                    segs = finger_segments(g, mode, r, math.radians(phi_deg))
                    hit = blocked(segs, [cx, cy, tip], k)
                    if hit is None:
                        found = (phi_deg, segs)
                        break
                if not found:
                    tip += 0.5
            if not found:
                tried.append(f"s{k} {mode} Ø{D:.1f}：放到位时手指或本体碰已装零件（接触高度 ≥ {eng} mm 的位置都不行）")
                continue
            return ({"tool": g["id"], "mode": mode, "shape": k, "diameter_mm": round(D, 3), "grip_deg": round(grip, 3),
                     "preset_deg": round(pre, 3), "tip_from_flange_mm": round(tip_off, 3), "phi_deg": found[0],
                     "engage_mm": round(min(z1, tip + g["finger_mm"]["length"]) - tip, 3),
                     "tcp_part": [cx, cy, round(tip, 3)], "segments": found[1],
                     "rule": f"{'外夹' if mode == 'external' else '内撑'} Ø{D:.1f}，指尖离底 {tip - z0:.1f} mm"}, tried)
    return None, tried


def _placed_before(assembly, step_id):
    out = set()
    for st in assembly["steps"]:
        if st["id"] == step_id:
            break
        if st["type"] == "place":
            out.add(st["part"])
    return out


def plan(assembly: dict, modules: dict, station: dict, F: Pose) -> dict | None:
    """放件规划；没有归放件臂的工步时返回 None（工站不加放件臂）。"""
    from . import clearance

    steps = [s for s in assembly["steps"] if s["type"] == "place" and s["actor"] in ACTORS_FOR_HANDLER]
    if not steps:
        return None
    ids = {m["role"]: [] for m in station["modules"]}
    for m in station["modules"]:
        ids[m["role"]].append(m["id"])
    grippers = [modules[i] for i in ids["gripper"]]
    two_f = [g for g in grippers if g["principle"] == "parallel_2f"]
    vac = [g for g in grippers if g["principle"] == "vacuum"]
    tray = modules[ids["tray"][0]]
    parts = {p["id"]: p for p in assembly["parts"]}
    if len(steps) > tray["slots"]:
        raise ValueError(f"料盘只有 {tray['slots']} 穴，要放 {len(steps)} 个零件")
    T = Pose.from_xyz_rpy(layout.TRAY_STOP["xyz_mm"], (0.0, 0.0, layout.TRAY_STOP["yaw_deg"]))
    H = Pose.from_xyz_rpy(layout.HANDLER_MOUNT["xyz_mm"], (0.0, 0.0, layout.HANDLER_MOUNT["yaw_deg"]))
    nx, ny = tray["slot_grid"]
    px, py = tray["slot_pitch_mm"]
    items, issues = [], []
    for i, st in enumerate(steps):
        part = parts[st["part"]]
        pose_f = Pose.from_xyz_rpy(part["pose_in_fixture"]["xyz"], part["pose_in_fixture"]["rpy"])
        placed = _placed_before(assembly, st["id"])
        yaw_p = math.radians(part["pose_in_fixture"]["rpy"][2])

        def blocked(segs, tcp_part, own_shape, part=part, placed=placed, pose_f=pose_f, yaw_p=yaw_p):
            """工具段（TCP 坐标，工具竖直向下、不转）放在零件装配位姿处，与已装零件和本零件其他简化体比对。"""
            tw = pose_f.apply(tcp_part)
            c, s_ = math.cos(yaw_p), math.sin(yaw_p)
            for seg in segs:
                d = seg.get("dxy", [0.0, 0.0])
                # 夹爪外形的 dxy 在零件坐标系里给出（与 phi 一致），换到治具系
                q = [tw[0] + c * d[0] - s_ * d[1], tw[1] + s_ * d[0] + c * d[1], tw[2]]
                z = [q[2] + seg["z"][0], q[2] + seg["z"][1]]
                for pid, oid, pose, sh in clearance._obstacles(assembly, placed):
                    qq = clearance._to_part(q, pose)
                    gap = clearance._gap(qq[:2], seg["r"], [qq[2] + seg["z"][0], qq[2] + seg["z"][1]], sh)
                    if gap is not None and gap < 0:
                        return f"{seg['name']} 碰 {oid}"
                for k2, sh in enumerate(part["shapes"]):
                    if k2 == own_shape:
                        continue
                    qq = pose_f.inverse().apply(q)
                    gap = clearance._gap(qq[:2], seg["r"], [qq[2] + seg["z"][0], qq[2] + seg["z"][1]], sh)
                    if gap is not None and gap < 0:
                        return f"{seg['name']} 碰本件 s{k2}"
            return None

        choice, reasons = None, []
        for g in two_f:
            gr, tried = grasp_2f(part, g, blocked)
            reasons += [f"{g['id']} {t}" for t in tried]
            if gr:
                choice = gr
                break
        if not choice:
            for g in vac:
                try:
                    pk = pick_point(part, g["envelope"]["segments"][0]["r"])
                except ValueError as e:
                    reasons.append(f"{g['id']}：{e}")
                    continue
                choice = {"tool": g["id"], "mode": "vacuum", "tcp_part": pk["xyz"], "rule": pk["rule"],
                          "tip_from_flange_mm": g["envelope"]["segments"][-1]["z"][1]}
                break
        if not choice:
            raise ValueError(f"{part['id']}：库里没有能取放它的末端：" + "；".join(reasons))
        if 2 * extent_r(part) > tray["slot_max_diameter_mm"]:
            issues.append(f"{part['id']} 外径 {2 * extent_r(part):.0f} mm 超过料盘穴 {tray['slot_max_diameter_mm']} mm（demo 仍放入）")
        col, row = i % nx, i // nx
        slot_xy = [(col - (nx - 1) / 2) * px, (row - (ny - 1) / 2) * py]
        tcp_part = choice["tcp_part"]
        z_min = min(_bottom(s) for s in part["shapes"])
        yaw_world = (F.compose(pose_f)).yaw_deg()
        # 零件在料盘里：底面贴盘面，偏航与装好后在世界里的偏航一致
        in_tray = Pose.from_xyz_rpy([slot_xy[0], slot_xy[1], -z_min], (0.0, 0.0, yaw_world - layout.TRAY_STOP["yaw_deg"]))
        pick_w = T.compose(in_tray).apply(tcp_part)
        place_w = F.compose(pose_f).apply(tcp_part)
        # 工具偏航：两指夹爪的开合方向（夹爪 y 轴）对准 phi；工具朝下（绕 x 转 180°）后 y → -y，所以偏航 = phi + 90°
        tool_yaw = (yaw_world + choice["phi_deg"] + 90.0) if choice["mode"] != "vacuum" else 0.0
        items.append({"step": st["id"], "part": part["id"], "name": part["name"], "actor": st["actor"],
                      "actor_assumed": st["actor"] != "robot", "slot": i + 1, "slot_xy": slot_xy,
                      "part_in_tray": {"xyz": in_tray.t, "yaw_deg": round(yaw_world - layout.TRAY_STOP["yaw_deg"], 6)},
                      "tool": choice["tool"], "grasp": {k: v for k, v in choice.items() if k != "segments"},
                      "tool_yaw_world_deg": round(tool_yaw, 3), "why_tool": reasons,
                      "pick_point_part": {"xyz": tcp_part, "rule": choice["rule"]}, "hang_mm": round(tcp_part[2] - z_min, 3),
                      "pick_world": [round(v, 3) for v in pick_w], "place_world": [round(v, 3) for v in place_w]})
    # 搬运高度：治具上所有零件顶面与料盘上零件顶面之上，再留出最长的悬挂件和余量
    tops = [F.compose(Pose.from_xyz_rpy(p["pose_in_fixture"]["xyz"], p["pose_in_fixture"]["rpy"])).apply([0, 0, max(_top(s) for s in p["shapes"])])[2]
            for p in assembly["parts"]]
    tops += [it["pick_world"][2] for it in items]
    transit_z = round(max(tops) + max(it["hang_mm"] for it in items) + TRANSIT_CLEAR_MM, 3)

    # 每个拧紧工步开始前，放件臂必须已经放完的件数（只在数变大时握手）
    need, n, prev = {}, 0, 0
    handled = {it["step"] for it in items}
    for st in assembly["steps"]:
        if st["type"] == "place" and st["id"] in handled:
            n += 1
        elif st["type"] == "fasten" and n > prev:
            need[st["points"][0]] = n
            prev = n
    # 换手窗口（用户 2026-10-26：拧紧臂不该干等放件臂换执行器）：一批里中途要换工具时，放件臂离开治具去快换架，
    # 拧紧臂趁这段去取下一点的螺钉。记下这些握手点
    window, lo = [], 0
    for pt, hi in sorted(need.items(), key=lambda kv: kv[1]):
        batch = items[lo:hi]
        if any(a["tool"] != b["tool"] for a, b in zip(batch, batch[1:])):
            window.append(pt)
        lo = hi
    tools = list(dict.fromkeys(it["tool"] for it in items))
    # 下线（M4 ③，用户 2026-10-25：放件臂吸起整件放到下游）：吸盘吸整件最高面的中心；整件底面落到下游托盘面上
    vac_ids = [g["id"] for g in vac]
    unload = None
    if vac_ids:
        top_z = max(max(_top(s) for s in p["shapes"]) for p in assembly["parts"])
        bot_z = min(min(_bottom(s) for s in p["shapes"]) for p in assembly["parts"])
        top_parts = [p for p in assembly["parts"] if any(_top(s) == top_z and _covers_origin(s) for s in p["shapes"])]
        if top_parts:
            pick = F.apply([0.0, 0.0, top_z])
            O = layout.OUT_STOP["xyz_mm"]
            unload = {"tool": vac_ids[0], "pick_world": [round(v, 3) for v in pick],
                      "place_world": [O[0], O[1], round(O[2] + (top_z - bot_z), 3)], "unit_height_mm": round(top_z - bot_z, 3),
                      "transit_z_mm": round(max(tops) + (top_z - bot_z) + TRANSIT_CLEAR_MM, 3), "out_stop": layout.OUT_STOP,
                      "rule": f"吸 {top_parts[0]['id']} 顶面中心（整件最高面）"}
        else:
            issues.append("整件最高面不在原点上方，demo 规则吸不了整件，下线跳过")
    return {"handler_mount": layout.HANDLER_MOUNT, "tray_stop": layout.TRAY_STOP, "tray": tray["id"], "tools": tools,
            "tool_docks": {t: layout.TOOL_DOCKS[t] for t in ids["gripper"] if t in layout.TOOL_DOCKS},
            "tool_changer": ids.get("tool_changer", [None])[0], "start_tool": items[0]["tool"],
            "tool_changes": sum(1 for a, b in zip(items, items[1:]) if a["tool"] != b["tool"]),
            "transit_z_mm": transit_z, "items": items, "handoff_before_point": need,
            "tool_change_window_before_point": window, "unload": unload,
            "issues": issues, "source": "demo_placeholder", "H": H}


def handler_ops(pl: dict, modules: dict) -> list[dict]:
    """放件臂中间指令（放件臂基座系）。没有紧固逻辑，只有搬运、末端开合、快换与握手。
    每条运动带 tcp_mm（当前工具的 TCP 偏移）：两指夹爪取两指尖中点，吸盘取吸盘面，快换对接取法兰面（0）。"""
    Hi = pl["H"].inverse()
    hyaw = pl["H"].yaw_deg()

    def pose(w, yaw_world=0.0):
        return {"xyz_mm": [round(v, 3) for v in Hi.apply(w)], "rpy": [180, 0, 0], "yaw_offset_deg": round(yaw_world - hyaw, 4)}

    ops = [{"op": "movej", "joints_deg": layout.H_HOME_J}]
    need = sorted(pl["handoff_before_point"].values())
    zone, cur, left = False, None, False    # 开机时工具都在快换架上：先去接第一件要用的（与料盘进场同时进行，节拍加速）
    dock_z = modules[pl["tool_changer"]]["dock_z_mm"] if pl["tool_changer"] else 150.0
    for k, it in enumerate(pl["items"], start=1):
        if k == 1 and it["tool"] == cur:
            ops.append({"op": "wait", "until": "tray_ready"})
        if it["tool"] != cur:   # 快换：放下当前工具、接上下一个（法兰对接，TCP = 法兰面）
            mid_batch = zone   # 这一批还没放完、治具区域还在放件臂手里：到中间点（区域外）就告诉拧紧臂「已离开」，交出区域
            for t, act in (((cur, "release"),) if cur else ()) + ((it["tool"], "attach"),):
                d = pl["tool_docks"][t]
                at, up = [d[0], d[1], dock_z], [d[0], d[1], dock_z + DOCK_CLEAR_MM]
                ops += [{"op": "movel", "tool_change": t, "pose": pose(layout.TOOL_VIA), "speed": "fast", "tcp_mm": 0.0, "via": True}]
                if mid_batch:
                    ops.append({"op": "signal", "name": "zone_left", "value": True})
                    zone, mid_batch, left = False, False, True   # 回来放下一件前重新等区域空闲
                ops += [
                        {"op": "movel", "tool_change": t, "pose": pose(up), "speed": "fast", "tcp_mm": 0.0},
                        {"op": "movel", "tool_change": t, "pose": pose(at), "speed": "slow", "tcp_mm": 0.0},
                        {"op": "tool", "action": act, "tool": t},
                        {"op": "movel", "tool_change": t, "pose": pose(up), "speed": "fast", "tcp_mm": 0.0}]
            cur = it["tool"]
            if k == 1:   # 工具接好，再等料盘到位
                ops.append({"op": "wait", "until": "tray_ready"})
        g = it["grasp"]
        tcp = g["tip_from_flange_mm"]
        yaw = it["tool_yaw_world_deg"]
        two_f = g["mode"] != "vacuum"
        up_pick = [it["pick_world"][0], it["pick_world"][1], pl["transit_z_mm"]]
        up_place = [it["place_world"][0], it["place_world"][1], pl["transit_z_mm"]]
        near_pick = [it["pick_world"][0], it["pick_world"][1], it["pick_world"][2] + NEAR_MM]
        near_place = [it["place_world"][0], it["place_world"][1], it["place_world"][2] + NEAR_MM]
        mv = lambda w, sp: {"op": "movel", "part": it["part"], "pose": pose(w, yaw), "speed": sp, "tcp_mm": tcp}   # noqa: E731
        ops.append(mv(up_pick, "fast"))
        if two_f:   # 下降前把手指预置到比零件大（外夹）或比内孔小（内撑）
            ops.append({"op": "gripper", "part": it["part"], "angle_deg": g["preset_deg"], "tool": cur})
        ops += [mv(near_pick, "fast"), mv(it["pick_world"], "slow"),
                {"op": "grip", "part": it["part"], "value": True, "tool": cur, "mode": g["mode"],
                 **({"grip_deg": g["grip_deg"], "preset_deg": g["preset_deg"]} if two_f else {})},
                mv(up_pick, "fast")]
        if not zone:
            ops.append({"op": "wait", "until": "zone_free"})
            if left:
                ops.append({"op": "signal", "name": "zone_left", "value": False})   # 重新进入治具区域
                left = False
            zone = True
        ops += [mv(up_place, "fast"), dict(mv(near_place, "fast"), leg="place"), dict(mv(it["place_world"], "slow"), leg="place"),
                {"op": "grip", "part": it["part"], "value": False, "tool": cur, "mode": g["mode"],
                 **({"preset_deg": g["preset_deg"]} if two_f else {})},
                mv(up_place, "fast")]
        if k in need:   # 这一批放完：退到料盘上方待命点（治具区域外）就告诉拧紧臂，不再回原位（节拍加速，用户 2026-10-26）
            T = layout.TRAY_STOP["xyz_mm"]
            ops += [{"op": "movel", "part": it["part"], "pose": pose([T[0], T[1], pl["transit_z_mm"]], yaw), "speed": "fast", "tcp_mm": tcp,
                     "purpose": "standby"},
                    {"op": "signal", "name": "placed_count", "value": k}]
            zone = False
    if ops[-1]["op"] != "signal":
        ops += [{"op": "movej", "joints_deg": layout.H_HOME_J}, {"op": "signal", "name": "placed_count", "value": len(pl["items"])}]
    ul = pl.get("unload")
    if ul:   # 下线段：拧紧臂整件拧完、退到停靠位后，吸起整件放到下游托盘，再把工具放回快换架（每个循环从空手开始）
        ph = {"phase": "unload"}
        ops.append({"op": "wait", "until": "unit_done", **ph})
        if ul["tool"] != cur:
            for t, act in ((cur, "release"), (ul["tool"], "attach")):
                d = pl["tool_docks"][t]
                ops += [{"op": "movel", "tool_change": t, "pose": pose(layout.TOOL_VIA), "speed": "fast", "tcp_mm": 0.0, "via": True, **ph},
                        {"op": "movel", "tool_change": t, "pose": pose([d[0], d[1], dock_z + DOCK_CLEAR_MM]), "speed": "fast", "tcp_mm": 0.0, **ph},
                        {"op": "movel", "tool_change": t, "pose": pose([d[0], d[1], dock_z]), "speed": "slow", "tcp_mm": 0.0, **ph},
                        {"op": "tool", "action": act, "tool": t, **ph},
                        {"op": "movel", "tool_change": t, "pose": pose([d[0], d[1], dock_z + DOCK_CLEAR_MM]), "speed": "fast", "tcp_mm": 0.0, **ph}]
            cur = ul["tool"]
        tcp = modules[ul["tool"]]["envelope"]["segments"][-1]["z"][1]
        tz = ul["transit_z_mm"]
        mv = lambda w, sp, **k: {"op": "movel", "part": "UNIT", "pose": pose(w), "speed": sp, "tcp_mm": tcp, **ph, **k}   # noqa: E731
        p, q = ul["pick_world"], ul["place_world"]
        ops += [mv([p[0], p[1], tz], "fast"), mv([p[0], p[1], p[2] + NEAR_MM], "fast"), mv(p, "slow"),
                {"op": "grip", "part": "UNIT", "value": True, "tool": cur, "mode": "vacuum", **ph},
                mv([p[0], p[1], tz], "fast"), mv([q[0], q[1], tz], "fast"), mv([q[0], q[1], q[2] + NEAR_MM], "fast"), mv(q, "slow"),
                {"op": "grip", "part": "UNIT", "value": False, "tool": cur, "mode": "vacuum", **ph},
                mv([q[0], q[1], tz], "fast")]
        d = pl["tool_docks"][cur]
        ops += [{"op": "movel", "tool_change": cur, "pose": pose(layout.TOOL_VIA), "speed": "fast", "tcp_mm": 0.0, "via": True, **ph},   # 直线去快换架会过放件臂基座正上方（逆解失败，孪生实测），经中间点
                {"op": "movel", "tool_change": cur, "pose": pose([d[0], d[1], dock_z + DOCK_CLEAR_MM]), "speed": "fast", "tcp_mm": 0.0, **ph},
                {"op": "movel", "tool_change": cur, "pose": pose([d[0], d[1], dock_z]), "speed": "slow", "tcp_mm": 0.0, **ph},
                {"op": "tool", "action": "release", "tool": cur, **ph},
                {"op": "movel", "tool_change": cur, "pose": pose([d[0], d[1], dock_z + DOCK_CLEAR_MM]), "speed": "fast", "tcp_mm": 0.0, **ph},
                {"op": "movej", "joints_deg": layout.H_HOME_J, **ph}, {"op": "signal", "name": "unloaded", "value": True, **ph}]
    return ops
