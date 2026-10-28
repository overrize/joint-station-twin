"""机械臂程序的离线干涉检查（M4 ③）：工具沿程序里每段直线运动扫过，和当时已放好的零件、元件简化体比对。

- 工具：执行器模块的外形包络（kb/modules 的 envelope，沿工具轴从 TCP 向上的几段圆柱）；拿着螺钉时，
  TCP 到批头尖端之间再加一段螺钉（头径 2d，长 = 杆长 + 头高 0.7d，与孪生一致）。
- 障碍物：assembly.yaml 里零件的 shapes 与 components，按 SOP 工步只算拧这个点之前已放好的零件。
- 只查工具，不查机械臂连杆；只支持工具竖直向下（v1 只有竖直拧紧）。
- 结果按点位汇总。与 DFM 已记录的问题（同一点位、同一障碍物）对得上的标 known，其余为新问题。
"""
from __future__ import annotations

import math

STEP_MM = 1.0
EPS_MM = 0.01          # 贴合（螺钉头压在被紧固件顶面上）不算干涉
HEAD_D_RATIO, HEAD_H_RATIO = 2.0, 0.7


def _screw_len(screw_id: str) -> float:
    return float(screw_id.split("-")[0].split("x")[1])


def _tool(envelope: dict, screw: dict | None) -> list[dict]:
    """返回工具各段 {name, z:[下, 上], r}，z 相对 TCP 向上。"""
    segs = [dict(s) for s in envelope["segments"]]
    if screw:
        h = screw["len"] + HEAD_H_RATIO * screw["d"]
        segs.insert(0, {"name": "screw", "z": [0.0, h], "r": HEAD_D_RATIO * screw["d"] / 2})
        segs[1]["z"] = [max(segs[1]["z"][0], h), segs[1]["z"][1]]
    return segs


def _to_part(p, pose):
    """治具坐标 → 零件坐标（零件只绕 z 转）。"""
    yaw = math.radians(pose["rpy"][2])
    dx, dy = p[0] - pose["xyz"][0], p[1] - pose["xyz"][1]
    c, s = math.cos(-yaw), math.sin(-yaw)
    return [c * dx - s * dy, s * dx + c * dy, p[2] - pose["xyz"][2]]


def _gap(xy, r, z, shape) -> float | None:
    """竖直圆柱段（圆心 xy、半径 r、高度 z）与简化体的水平间隙（mm，负数为干涉）；z 不重叠返回 None。"""
    sz = shape["z"] if shape["type"] != "box" else [shape["center"][2] - shape["size"][2] / 2, shape["center"][2] + shape["size"][2] / 2]
    if min(z[1], sz[1]) - max(z[0], sz[0]) <= EPS_MM:
        return None
    t = shape["type"]
    if t == "cylinder":
        cx, cy = shape.get("xy", [0.0, 0.0])
        return math.hypot(xy[0] - cx, xy[1] - cy) - r - shape["r"]
    if t == "ring":
        cx, cy = shape.get("xy", [0.0, 0.0])
        d = math.hypot(xy[0] - cx, xy[1] - cy)
        return max(shape["r_in"] - (d + r), d - r - shape["r_out"])   # 在内孔里或在外圈外都算有间隙
    if t == "box":
        yaw = math.radians(shape.get("yaw", 0.0))
        dx, dy = xy[0] - shape["center"][0], xy[1] - shape["center"][1]
        c, s = math.cos(-yaw), math.sin(-yaw)
        lx, ly = c * dx - s * dy, s * dx + c * dy
        ex, ey = max(abs(lx) - shape["size"][0] / 2, 0.0), max(abs(ly) - shape["size"][1] / 2, 0.0)
        return math.hypot(ex, ey) - r
    raise ValueError(f"不认识的简化体 {t}")


def _obstacles(assembly: dict, placed: set[str]):
    for p in assembly["parts"]:
        if p["id"] not in placed:
            continue
        for k, sh in enumerate(p["shapes"]):
            yield p["id"], f"{p['id']}/s{k}", p["pose_in_fixture"], sh
        for c in p.get("components", []):
            yield p["id"], c["id"], p["pose_in_fixture"], c["shape"]


def check(flow: dict, assembly: dict, envelope: dict, screws: dict, dfm_issues: list[dict], feed: dict | None = None, F=None) -> dict:
    """feed / F：取钉（DEC-FEED-001）时，上一点 → 出钉位上方（空批头）、出钉位上方 → 本点接近位（拿着螺钉）代替直接转移。"""
    groups = {}
    order, placed_before = [], {}
    placed = set()
    for st in assembly["steps"]:
        if st["type"] == "place":
            placed.add(st["part"])
        elif st["type"] == "fasten":
            for pt in st["points"]:
                placed_before[pt] = set(placed)
                groups[pt] = st
    known = {(i.get("point"), i.get("obstacle")) for i in dfm_issues}

    # 程序里的直线段：上一点接近位 → 本点接近位 → 下压位（拿着螺钉）→ 接近位（螺钉已拧入）
    rows = []
    prev = None
    for s in flow["sequence"]:
        pt = s["point"]
        g = groups[pt]
        sc = {"d": screws[g["screw"]]["d_mm"], "len": _screw_len(g["screw"])}
        legs = []
        if feed and F is not None:
            from .layout import FEED_CLEAR_MM

            px, py, pz = feed[pt]["pick"]
            above = F.inverse().apply([px, py, pz + FEED_CLEAR_MM])
            if prev is not None:
                legs.append(("to_feed", prev, above, None))
            legs.append(("from_feed", above, s["approach"]["xyz"], sc))
        elif prev is not None:
            legs.append(("transfer", prev, s["approach"]["xyz"], None))
        legs += [("approach", s["approach"]["xyz"], s["engage"]["xyz"], sc), ("retract", s["engage"]["xyz"], s["approach"]["xyz"], None)]
        prev = s["approach"]["xyz"]
        hits, min_gap = {}, math.inf
        for leg, a, b, held in legs:
            tool = _tool(envelope, held)
            n = max(1, math.ceil(math.dist(a, b) / STEP_MM))
            for k in range(n + 1):
                tcp = [a[i] + (b[i] - a[i]) * k / n for i in range(3)]
                for part, oid, pose, sh in _obstacles(assembly, placed_before[pt]):
                    q = _to_part(tcp, pose)
                    for seg in tool:
                        gap = _gap(q[:2], seg["r"], [q[2] + seg["z"][0], q[2] + seg["z"][1]], sh)
                        if gap is None:
                            continue
                        min_gap = min(min_gap, gap)
                        if gap < 0:
                            h = hits.setdefault((oid, seg["name"]), {"obstacle": oid, "part": part, "tool": seg["name"], "legs": set(), "depth_mm": 0.0})
                            h["legs"].add(leg)
                            h["depth_mm"] = max(h["depth_mm"], round(-gap, 3))
        hit_list = []
        for h in hits.values():
            h["legs"] = sorted(h["legs"])
            h["known"] = (pt, h["obstacle"]) in known
            hit_list.append(h)
        rows.append({"point": pt, "min_gap_mm": round(min_gap, 3) if min_gap < math.inf else None, "hits": hit_list})
    new = [dict(h, point=r["point"]) for r in rows for h in r["hits"] if not h["known"]]
    known_hits = [dict(h, point=r["point"]) for r in rows for h in r["hits"] if h["known"]]
    return {"method": "工具包络沿程序直线段扫掠（步长 1 mm），对已放零件与元件的简化体；不含机械臂连杆",
            "envelope_source": envelope.get("source"), "points": rows, "new_hits": new, "known_hits": known_hits,
            "ok": not new}


def check_handler(handling_plan: dict, assembly: dict, modules: dict, F) -> dict:
    """放件臂（M4 ②）：吸盘 + 吸着的零件（按它自己的圆柱 / 环简化体；长方体取外接圆）沿放件路径（搬运高度 → 目标上方 → 放到位 → 抬起）扫过，
    对已在治具上的零件。只查治具上方这一段；料盘一侧与机械臂连杆不查。"""
    Fi = F.inverse()
    parts = {p["id"]: p for p in assembly["parts"]}
    order = [s for s in assembly["steps"] if s["type"] == "place"]
    rows = []
    for it in handling_plan["items"]:
        placed = set()
        for st in order:
            if st["id"] == it["step"]:
                break
            placed.add(st["part"])
        part = parts[it["part"]]
        pk = it["pick_point_part"]["xyz"]
        yaw = math.radians(part["pose_in_fixture"]["rpy"][2])
        c, s_ = math.cos(yaw), math.sin(yaw)
        dxy = [-(c * pk[0] - s_ * pk[1]), -(s_ * pk[0] + c * pk[1])]   # 零件原点相对吸盘中心（治具系）
        body = []
        for k, sh in enumerate(part["shapes"]):   # 吸着的零件：每个简化体随吸盘走，z 相对吸盘面
            sx, sy = sh.get("xy", [0.0, 0.0]) if sh["type"] != "box" else sh["center"][:2]
            r_out = sh.get("r", sh.get("r_out")) if sh["type"] != "box" else math.hypot(sh["size"][0], sh["size"][1]) / 2
            body.append({"name": f"carried:{part['id']}/s{k}", "z": [_s_bot(sh) - pk[2], _s_top(sh) - pk[2]],
                         "r": r_out, "r_in": sh.get("r_in", 0.0) if sh["type"] == "ring" else 0.0,
                         "dxy": [dxy[0] + c * sx - s_ * sy, dxy[1] + s_ * sx + c * sy]})
        g = modules[it["tool"]]
        gr = it["grasp"]
        if gr["mode"] == "vacuum":
            tool = [dict(sg) for sg in g["envelope"]["segments"]]
        else:   # 两指夹爪：手指与本体（dxy 在零件坐标系，随零件偏航换到治具系）
            from .handling import finger_segments

            r = gr["diameter_mm"] / 2
            tool = []
            for sg in finger_segments(g, gr["mode"], r, math.radians(gr["phi_deg"])):
                d = sg.get("dxy", [0.0, 0.0])
                tool.append(dict(sg, dxy=[c * d[0] - s_ * d[1], s_ * d[0] + c * d[1]]))
        place = Fi.apply(it["place_world"])
        up = [place[0], place[1], Fi.apply([0, 0, handling_plan["transit_z_mm"]])[2]]
        legs = [("lower", up, place, tool + body), ("lift", place, up, tool)]
        hits, min_gap = {}, math.inf
        for leg, a, b, segs in legs:
            n = max(1, math.ceil(math.dist(a, b) / STEP_MM))
            for k in range(n + 1):
                tcp = [a[i] + (b[i] - a[i]) * k / n for i in range(3)]
                for pid, oid, pose, sh in _obstacles(assembly, placed):
                    for seg in segs:
                        d = seg.get("dxy", [0.0, 0.0])
                        q = _to_part([tcp[0] + d[0], tcp[1] + d[1], tcp[2]], pose)
                        gap = _gap_ring(q[:2], seg.get("r_in", 0.0), seg["r"], [q[2] + seg["z"][0], q[2] + seg["z"][1]], sh)
                        if gap is None:
                            continue
                        min_gap = min(min_gap, gap)
                        if gap < 0:
                            h = hits.setdefault((oid, seg["name"]), {"obstacle": oid, "part": pid, "tool": seg["name"], "legs": set(), "depth_mm": 0.0})
                            h["legs"].add(leg)
                            h["depth_mm"] = max(h["depth_mm"], round(-gap, 3))
        for h in hits.values():
            h["legs"] = sorted(h["legs"])
        rows.append({"step": it["step"], "part": it["part"], "min_gap_mm": round(min_gap, 3) if min_gap < math.inf else None,
                     "hits": list(hits.values())})
    new = [dict(h, step=r["step"]) for r in rows for h in r["hits"]]
    return {"method": "吸盘包络 + 吸着零件外接圆柱，沿治具上方的放件路径扫掠（步长 1 mm），对已放零件；不含料盘一侧与连杆",
            "envelope_source": "demo_placeholder（吸盘）/ 本机实测（2F-85）", "steps": rows, "new_hits": new, "ok": not new}


def _s_top(sh):
    return sh["z"][1] if sh["type"] != "box" else sh["center"][2] + sh["size"][2] / 2


def _s_bot(sh):
    return sh["z"][0] if sh["type"] != "box" else sh["center"][2] - sh["size"][2] / 2


def _gap_ring(xy, r_in, r_out, z, shape) -> float | None:
    """竖直圆环段（内径 r_in，r_in=0 为实心）与简化体的水平间隙。两环（或环与圆柱）不重叠的条件：
    相离，或一个整个落在另一个的内孔里。长方体只按实心外圆算（保守）。"""
    if r_in <= 0 or shape["type"] == "box":
        return _gap(xy, r_out, z, shape)
    sz = shape["z"]
    if min(z[1], sz[1]) - max(z[0], sz[0]) <= EPS_MM:
        return None
    cx, cy = shape.get("xy", [0.0, 0.0])
    d = math.hypot(xy[0] - cx, xy[1] - cy)
    b_in, b_out = (shape["r_in"], shape["r_out"]) if shape["type"] == "ring" else (0.0, shape["r"])
    return max(d - r_out - b_out, r_in - (d + b_out), b_in - (d + r_out))
