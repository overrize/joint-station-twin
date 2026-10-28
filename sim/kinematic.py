"""简化运动学检查与自标定模拟。不依赖 Isaac Sim，用于测试和在没有 GPU 的环境里验证业务流。

Isaac 孪生上线后，这里的结果作为对照：两边结论不一致时，以 Isaac 为准并回查原因。
"""
from __future__ import annotations

import random

from jsdemo.geometry import Pose, calibrate_fixture, norm, pose_error


def reach_check(ops: list[dict], arm: dict) -> dict:
    bad = []
    for o in ops:
        if o["op"] != "movel":
            continue
        d = norm(o["pose"]["xyz_mm"])
        if not (arm["min_reach_mm"] <= d <= arm["reach_mm"]):
            bad.append({"point": o["point"], "dist_mm": round(d, 1)})
    return {"ok": not bad, "violations": bad, "checked": sum(1 for o in ops if o["op"] == "movel")}


def simulate_calibration(fixture: dict, true_offset: dict, noise_mm: float = 0.0, seed: int = 7) -> dict:
    """模拟现场自标定：治具实际位姿相对名义有偏差，机械臂触碰三个基准点，算出实际位姿。

    true_offset: {"xyz_mm": [dx,dy,dz], "rpy_deg": [r,p,y]}，叠加在名义位姿上。
    """
    rnd = random.Random(seed)
    from jsdemo import layout
    fx = layout.current().get("FIXTURE")   # 名义位姿 = 工站设计 fixture.pose
    nom = fx if fx and fx["id"] == fixture.get("id") else fixture["nominal_pose_in_base"]
    F_nom = Pose.from_xyz_rpy(nom["xyz_mm"], (0, 0, nom["yaw_deg"]))
    off = Pose.from_xyz_rpy(true_offset["xyz_mm"], true_offset["rpy_deg"])
    F_true = F_nom.compose(off)
    datums = [d["xyz"] for d in fixture["datums"]]
    measured = [[v + rnd.gauss(0, noise_mm) for v in F_true.apply(p)] for p in datums]
    F_cal = calibrate_fixture(datums, measured)
    return {
        "before": pose_error(F_nom, F_true),
        "after": pose_error(F_cal, F_true),
        "calibrated_xyz_mm": [round(v, 4) for v in F_cal.t],
        "calibrated_yaw_deg": round(F_cal.yaw_deg(), 4),
    }
