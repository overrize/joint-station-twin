"""节拍估算：按模块节拍逐点累加。Isaac 孪生里实测后用实测值替换。"""
from __future__ import annotations


def estimate(station: dict, modules: dict, n_points: int, target_s: float) -> dict:
    by_role = {m["role"]: modules[m["id"]] for m in station["modules"]}
    move = by_role["arm"]["move_s_avg"]
    fasten = by_role["executor"]["fasten_s_base"]
    extra = by_role["vision_locator"]["extra_s_per_point"] if "vision_locator" in by_role else 0.0
    per_point = move + fasten + extra
    load = by_role["conveyor"]["load_unload_s"]
    total = round(n_points * per_point + load, 2)
    return {"per_point_s": round(per_point, 2), "points": n_points, "load_unload_s": load,
            "total_s": total, "target_s": target_s, "ok": total <= target_s}
