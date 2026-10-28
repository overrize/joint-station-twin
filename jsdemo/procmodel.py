"""工艺模型（演示版）：库里有的直接引用，库内样本之间插值推荐，超出范围返回未知。

原则：宁可擦除，不要误纠。没见过的组合不给数。
命令行：
  python -m jsdemo.procmodel recommend --material PA66 --screw M2x6-tapping --hole 1.6
  python -m jsdemo.procmodel judge --curve process/curves/D1_ok.csv --min 0.15 --max 0.21 --screw M2x6-tapping --depth 5
"""
from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path

from .io import load_library


def _fmt_id(material: str, screw: str, hole: float | None) -> str:
    s = screw.replace("-machine", "").replace("-tapping", "T").replace("x", "-")
    h = f"-H{int(round(hole * 10))}" if hole is not None else ""
    return f"W-{material}-{s}{h}"


def recommend(material: str, screw: str, hole_d: float | None = None, library: dict | None = None) -> dict:
    lib = library or load_library()
    cands = [e for e in lib["entries"] if e["material"] == material and e["screw"] == screw]
    if not cands:
        return {"status": "unknown", "reason": f"库中没有 {material} + {screw} 的样本", "material": material, "screw": screw}

    # 无孔径维度（机螺钉）：精确匹配
    if hole_d is None or all("hole_d_mm" not in e for e in cands):
        e = cands[0]
        return {"status": "library", "id": e["id"], "window_nm": e["window_nm"], "target_nm": e["target_nm"],
                "stages": e["stages"], "confidence": e["confidence"], "sources": [e["id"]]}

    with_h = sorted([e for e in cands if "hole_d_mm" in e], key=lambda e: e["hole_d_mm"])
    for e in with_h:
        if abs(e["hole_d_mm"] - hole_d) < 1e-9:
            return {"status": "library", "id": e["id"], "window_nm": e["window_nm"], "target_nm": e["target_nm"],
                    "stages": e["stages"], "confidence": e["confidence"], "sources": [e["id"]]}

    lo, hi = with_h[0]["hole_d_mm"], with_h[-1]["hole_d_mm"]
    max_ex = lib.get("model", {}).get("max_extrapolation_mm", 0.0)
    if hole_d < lo - max_ex or hole_d > hi + max_ex or len(with_h) < 2:
        return {"status": "unknown", "reason": f"孔径 {hole_d} mm 超出样本范围 {lo}–{hi} mm", "material": material, "screw": screw}

    # 找相邻两点做线性插值（范围内）或用端点两点外推（允许的小范围内）
    a, b = with_h[0], with_h[1]
    for i in range(len(with_h) - 1):
        if with_h[i]["hole_d_mm"] <= hole_d <= with_h[i + 1]["hole_d_mm"]:
            a, b = with_h[i], with_h[i + 1]
            break
    else:
        if hole_d > hi:
            a, b = with_h[-2], with_h[-1]
    t = (hole_d - a["hole_d_mm"]) / (b["hole_d_mm"] - a["hole_d_mm"])
    lerp = lambda x, y: round(x + (y - x) * t, 3)
    inside = lo <= hole_d <= hi
    return {
        "status": "model",
        "id": _fmt_id(material, screw, hole_d),
        "window_nm": [lerp(a["window_nm"][0], b["window_nm"][0]), lerp(a["window_nm"][1], b["window_nm"][1])],
        "target_nm": lerp(a["target_nm"], b["target_nm"]),
        "stages": a["stages"],
        "confidence": "medium" if inside else "low",
        "sources": [a["id"], b["id"]],
        "note": "模型推荐，首件收敛后可回写工艺库",
    }


# ---------------- 曲线判定 ----------------

def expected_angle_deg(screw_spec: dict, depth_mm: float) -> float:
    return depth_mm / screw_spec["pitch_mm"] * 360.0


def judge(rows: list[tuple[float, float]], window: list[float], screw_spec: dict, depth_mm: float) -> dict:
    """rows: [(angle_deg, torque_nm)]。返回判定和失效模式。"""
    if not rows:
        return {"result": "NG", "mode": "no_data"}
    final = rows[-1][1]
    peak = max(t for _, t in rows)
    total = rows[-1][0] - rows[0][0]
    lo, hi = window
    exp = expected_angle_deg(screw_spec, depth_mm)
    if final < lo and peak >= 0.9 * lo and final < 0.7 * peak:
        mode = "slip"
    elif final > hi:
        mode = "over"
    elif final < lo:
        mode = "under"
    elif total < 0.85 * exp:
        mode = "float"
    else:
        mode = "ok"
    return {"result": "OK" if mode == "ok" else "NG", "mode": mode, "final_nm": round(final, 4),
            "peak_nm": round(peak, 4), "total_angle_deg": round(total, 1), "expected_angle_deg": round(exp, 1)}


MODE_CN = {"ok": "合格", "slip": "滑牙", "over": "扭矩超差", "under": "扭矩不足", "float": "浮高", "no_data": "无数据"}


def synth_curve(kind: str, window: list[float], target: float, screw_spec: dict, depth_mm: float, step_deg: float = 10.0) -> list[tuple[float, float]]:
    """生成演示曲线（确定性）。kind: ok | slip | float。真实部署时由执行器上传真实曲线。"""
    exp = expected_angle_deg(screw_spec, depth_mm)
    total = exp * (0.6 if kind == "float" else 1.02)
    seat = total - 120.0
    rows = []
    a = 0.0
    tapping = screw_spec["kind"] == "tapping"
    base = target * (0.35 if tapping else 0.12)
    while a <= total + 1e-9:
        if a < seat:
            t = base * min(1.0, a / 200.0) + 0.01 * math.sin(a / 40.0) * target
        else:
            k = (a - seat) / (total - seat)
            t = base + (target - base) * (k ** 1.6)
        rows.append((round(a, 1), round(max(t, 0.0), 4)))
        a += step_deg
    if kind == "slip":
        peak = window[1] * 0.98
        rows = [(x, min(y, peak)) for x, y in rows]
        last_a = rows[-1][0]
        rows += [(last_a + 10 * i, round(peak * (1 - 0.15 * i), 4)) for i in range(1, 5)]
    return rows


def read_curve(path: Path | str) -> list[tuple[float, float]]:
    with open(path, "r", encoding="utf-8") as f:
        r = csv.DictReader(f)
        return [(float(x["angle_deg"]), float(x["torque_nm"])) for x in r]


def write_curve(rows, path: Path | str) -> None:
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    with open(path, "w", encoding="utf-8", newline="") as f:
        w = csv.writer(f)
        w.writerow(["angle_deg", "torque_nm"])
        w.writerows(rows)


def _main():
    ap = argparse.ArgumentParser(prog="procmodel")
    sp = ap.add_subparsers(dest="cmd", required=True)
    r = sp.add_parser("recommend")
    r.add_argument("--material", required=True)
    r.add_argument("--screw", required=True)
    r.add_argument("--hole", type=float)
    j = sp.add_parser("judge")
    j.add_argument("--curve", required=True)
    j.add_argument("--min", type=float, required=True)
    j.add_argument("--max", type=float, required=True)
    j.add_argument("--screw", required=True)
    j.add_argument("--depth", type=float, required=True)
    a = ap.parse_args()
    lib = load_library()
    if a.cmd == "recommend":
        out = recommend(a.material, a.screw, a.hole, lib)
    else:
        out = judge(read_curve(a.curve), [a.min, a.max], lib["screws"][a.screw], a.depth)
    print(json.dumps(out, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    _main()
