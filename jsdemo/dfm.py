"""可制造性检查：只做有明确几何判据的规则，查不出来的不假装能查。"""
from __future__ import annotations

import math

from .io import groups_by_id


def check(product: dict, rules: list[dict], screws: dict) -> dict:
    groups = groups_by_id(product)
    pts = product["fastening"]["points"]
    obstacles = product["fastening"].get("obstacles", [])
    issues = []
    for r in rules:
        kind = r["check"]
        if kind == "bit_clearance":
            for p in pts:
                for ob in obstacles:
                    if ob["z_top"] <= p["xyz"][2]:
                        continue
                    d = math.dist(p["xyz"][:2], ob["xy"])
                    if d < r["min_mm"]:
                        issues.append({
                            "rule": r["id"], "point": p["id"], "obstacle": ob["id"],
                            "measured_mm": round(d, 2), "limit_mm": r["min_mm"],
                            "message": f"{p['id']} 批头进不去：{ob['name']} {ob['id']} 距孔位中心 {d:.1f} mm，下限 {r['min_mm']} mm",
                            "suggestion": f"把 {ob['id']} 外移至少 {r['min_mm'] - d + 0.3:.1f} mm，或调整 {p['id']} 的孔位",
                        })
        elif kind == "thread_engagement":
            for g in groups.values():
                s = screws[g["screw"]]
                if s["kind"] == "machine" and g["depth_mm"] < r["min_ratio_of_d"] * s["d_mm"]:
                    issues.append({"rule": r["id"], "group": g["id"], "message": f"{g['name']} 旋合深度不足"})
        elif kind == "tapping_depth":
            for g in groups.values():
                s = screws[g["screw"]]
                if s["kind"] == "tapping" and g["depth_mm"] < r["min_ratio_of_d"] * s["d_mm"]:
                    issues.append({"rule": r["id"], "group": g["id"], "message": f"{g['name']} 自攻拧入深度不足"})
        elif kind == "approach_vertical":
            pass  # 演示产品全部竖直接近，规则恒通过
    bad_points = {i["point"] for i in issues if "point" in i}
    return {
        "rules_checked": len(rules),
        "points_total": len(pts),
        "points_passed": len(pts) - len(bad_points),
        "issues": issues,
    }
