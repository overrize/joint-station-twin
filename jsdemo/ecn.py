"""设计变更：算出受影响的点位和文件，只重做受影响的部分。"""
from __future__ import annotations

from .io import clone, groups_by_id, points
from .procmodel import recommend


def apply(product: dict, ecn: dict) -> dict:
    new = clone(product)
    g = groups_by_id(new)
    changed_groups = set()
    for c in ecn["changes"]:
        if c["target"] == "group":
            grp = g[c["id"]]
            if grp.get(c["field"]) != c["from"]:
                raise ValueError(f"变更前值不符：{c['id']}.{c['field']} 当前是 {grp.get(c['field'])}，变更单写的是 {c['from']}")
            grp[c["field"]] = c["to"]
            changed_groups.add(c["id"])
    return {"product": new, "changed_groups": sorted(changed_groups)}


def impact(old_product: dict, ecn: dict, library: dict) -> dict:
    res = apply(old_product, ecn)
    new = res["product"]
    groups = groups_by_id(new)
    all_pts = [p["id"] for p in points(new)]
    hit = [p["id"] for p in points(new) if p["group"] in res["changed_groups"]]
    windows = {}
    for gid in res["changed_groups"]:
        gg = groups[gid]
        windows[gid] = recommend(gg["material"], gg["screw"], gg.get("hole_d_mm"), library)
    files = [f"params/{p}.yaml" for p in hit] + [f"docs/wi.md#{p}" for p in hit] + [f"docs/control_plan.csv#{p}" for p in hit]
    return {
        "ecn": ecn["id"], "title": ecn["title"],
        "changed_groups": res["changed_groups"],
        "affected_points": hit,
        "unaffected_points": [p for p in all_pts if p not in hit],
        "new_windows": windows,
        "files_to_update": files,
        "refai_points": hit,
        "blocked": [gid for gid, w in windows.items() if w["status"] == "unknown"],
    }
