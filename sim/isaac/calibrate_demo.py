"""任务 M3.3（原 M7）：现场自标定演示。

1. 场景里的治具相对名义位姿放偏（默认平移 [0.8, -0.5, 0.2] mm，旋转 [0.1, -0.05, 0.3]°），模拟现场安装误差；
2. 机械臂（只知道名义位姿）依次到三个基准球（DT1~DT3）上方，下探到球顶；实测值 = 场景中基准球实际中心 + 高斯噪声
   （默认 0.02 mm；真实设备由执行器接触力判断，这里没有模拟触碰传感与多次触碰拟合球心的过程）；
3. 调用 jsdemo.geometry.calibrate_fixture 算出治具实际位姿；
4. 用新位姿重新生成机械臂程序（jsdemo.codegen.neutral），在同一次仿真里重跑整件；
5. 写 build/twin/calibration_report.json：标定前后误差、重跑结果；重跑整件另写 twin_report_calibrated.json / gateway_calibrated.jsonl。

纯数学部分已经在 sim/kinematic.py 和 tests/test_calibration.py 里验证过，这里只做孪生里的演示。
也可以单独重跑：run_flow.py --fixture-pose build/twin/calibration_report.json

运行（先 build_scene.py）：
    F:\\isaacsim\\python.bat sim\\isaac\\calibrate_demo.py
    F:\\isaacsim\\python.bat sim\\isaac\\calibrate_demo.py --headless
"""
from __future__ import annotations

import argparse
import json
import random
import sys
from datetime import datetime

from _common import SCENE, STATION_DIR, TWIN_DIR, load_outputs

ap = argparse.ArgumentParser()
ap.add_argument("--headless", action="store_true")
ap.add_argument("--offset-xyz", type=float, nargs=3, default=[0.8, -0.5, 0.2], metavar=("DX", "DY", "DZ"), help="mm")
ap.add_argument("--offset-rpy", type=float, nargs=3, default=[0.1, -0.05, 0.3], metavar=("R", "P", "Y"), help="deg")
ap.add_argument("--noise-mm", type=float, default=0.02)
ap.add_argument("--seed", type=int, default=7)
ap.add_argument("--sn", default="TWIN-0002")
ap.add_argument("--record", action="store_true", help="录屏：mp4 + 字幕 srt 到 build/twin/media/calibration/（M4.1）")
args, _ = ap.parse_known_args()

from isaacsim import SimulationApp  # noqa: E402

app = SimulationApp({"headless": args.headless})

from isaacsim.core.rendering_manager import ViewportManager  # noqa: E402

from jsdemo import codegen, composer  # noqa: E402
from jsdemo.geometry import Pose, calibrate_fixture, norm, pose_error, sub  # noqa: E402
from twin import Twin  # noqa: E402

CAL_TRANS_TOL_MM = 0.1     # docs/03-plan.md M3.3 验收
PROBE_ABOVE_MM = 30.0      # 下探起点在球顶上方的高度（演示用，与紧固点 approach 高度一致）
PROBE_DWELL_S = 0.5        # 触碰后停留（演示用）
SCENE_READBACK_TOL_MM = 0.005  # 治具写入场景的允许偏差：取探测噪声默认值 0.02 mm 的 1/4


def pose_json(F: Pose) -> dict:
    return {"xyz_mm": [round(v, 6) for v in F.t], "R": [[round(v, 12) for v in r] for r in F.R],
            "yaw_deg": round(F.yaw_deg(), 6)}


def err_json(e: dict) -> dict:
    return {k: round(v, 6) for k, v in e.items()}


exit_code = 1
try:
    station, flow, ops, modules = load_outputs()
    fx = modules[station["fixture_frame"]]
    F_nom = composer.fixture_pose(modules, station["fixture_frame"])
    F_target = F_nom.compose(Pose.from_xyz_rpy(args.offset_xyz, args.offset_rpy))  # 与 sim/kinematic.simulate_calibration 同一约定

    twin = Twin(app, station, ops, modules, headless=args.headless, sn=args.sn,
                record_dir=(TWIN_DIR / "media" / "calibration") if args.record else None)
    # 真值 = 场景读回（见 Twin.place_fixture）；写入目标与读回之差应远小于探测噪声
    readback = twin.place_fixture(F_target)
    F_true = twin.fixture_true
    # M3.4 起产品带碰撞，D3 会如实撞上 C12（DFM-01 未关闭）；本演示只验证定位，重跑时按 DFM 建议外移障碍物（只改场景）
    dfm = json.loads((STATION_DIR / "report.json").read_text(encoding="utf-8"))["stages"]["dfm"]["issues"]
    dfm_fix = twin.apply_dfm_fix(dfm)
    if max(readback.values()) > SCENE_READBACK_TOL_MM:
        raise RuntimeError(f"治具位姿写入场景后基准球读回偏差过大（mm）：{readback}")
    if not args.headless or args.record:
        twin.camera_main = {"eye": [0.15, -1.85, 1.15], "target": [0.45, -0.12, 0.18]}
        ViewportManager.set_camera_view("/OmniverseKit_Persp", **twin.camera_main)
    twin.caption(f"现场自标定：治具放偏 平移 {args.offset_xyz} mm、旋转 {args.offset_rpy}°；机械臂只知道名义位姿")

    # ---------- 1. 下探三个基准点（机械臂只知道名义位姿） ----------
    twin.start()
    rnd = random.Random(args.seed)
    rpy, yaw = [180, 0, 0], round(F_nom.yaw_deg(), 4)
    r = SCENE["datum_radius"]
    probes, probe_ok = [], True
    for d in fx["datums"]:
        top = F_nom.apply([d["xyz"][0], d["xyz"][1], d["xyz"][2] + r])
        above = F_nom.apply([d["xyz"][0], d["xyz"][1], d["xyz"][2] + r + PROBE_ABOVE_MM])
        t0 = twin.t
        twin.caption(f"下探基准点 {d['id']}")
        m1 = twin.movel({"xyz_mm": above, "rpy": rpy, "yaw_offset_deg": yaw}, "fast")
        m2 = twin.movel({"xyz_mm": top, "rpy": rpy, "yaw_offset_deg": yaw}, "slow")
        twin.hold(PROBE_DWELL_S)
        actual = twin.datum_world_mm(d["id"])
        measured = [float(v) + rnd.gauss(0, args.noise_mm) for v in actual]
        m3 = twin.movel({"xyz_mm": above, "rpy": rpy, "yaw_offset_deg": yaw}, "fast")
        ok = all(m["arrived"] for m in (m1, m2, m3))
        probe_ok &= ok
        probes.append({"datum": d["id"], "nominal_in_fixture_mm": d["xyz"], "nominal_in_base_mm": [round(v, 4) for v in F_nom.apply(d["xyz"])],
                       "actual_in_base_mm": [round(float(v), 6) for v in actual], "measured_in_base_mm": [round(v, 6) for v in measured],
                       "nominal_vs_actual_mm": round(norm(sub(F_nom.apply(d["xyz"]), list(actual))), 4),
                       "arm_arrived": ok, "touch_err_mm": m2.get("err_mm"), "duration_s": round(twin.t - t0, 3)})
        print(f"CAL probe {d['id']} arrived={ok} nominal_vs_actual={probes[-1]['nominal_vs_actual_mm']}mm")
    probe_collisions = list(twin.collisions)

    # ---------- 2. 解算 ----------
    F_cal = calibrate_fixture([d["xyz"] for d in fx["datums"]], [p["measured_in_base_mm"] for p in probes])
    before, after = pose_error(F_nom, F_true), pose_error(F_cal, F_true)
    # 不标定直接用名义位姿时，各紧固点（engage 位）的孔位偏差（解析计算，不另跑一遍）
    uncal = {s["point"]: round(norm(sub(F_nom.apply(s["engage"]["xyz"]), F_true.apply(s["engage"]["xyz"]))), 4) for s in flow["sequence"]}
    print(f"CAL before={before} after={after}")
    twin.caption(f"标定完成：治具位姿误差 {before['translation_mm']:.3f} mm → {after['translation_mm']:.3f} mm；"
                 f"不标定时孔位最多偏 {max(uncal.values()):.2f} mm。用标定后的位姿重新生成机械臂程序，开始拧整件")
    twin.gateway.append({"t_s": round(twin.t, 3), "source": station["station_id"], "via": modules[twin.by_role["feeder"]]["mes"]["via"],
                         "event": "fixture_calibrated", "fixture": station["fixture_frame"],
                         "offset_from_nominal_mm": [round(a - b, 4) for a, b in zip(F_cal.t, F_nom.t)],
                         "residual_translation_mm": round(after["translation_mm"], 4)})

    # ---------- 3. 用标定后的位姿重跑整件（同一次仿真） ----------
    twin.ops = codegen.neutral(flow, F_cal)
    rerun = twin.run_unit()
    twin.stop()
    if args.record:
        twin.set_view("product")
        twin.caption(f"标定后重跑：{rerun['points_arrived']}/{rerun['points_total']} 点到位，孔位误差最大 {rerun['hole_err_max_mm']} mm，"
                     f"整件{'合格' if rerun['unit_result'] == 'OK' else '不合格'}")
        twin.hold_frames(4.0)
        rerun["video"] = twin.finish_recording("calibration")
        print(f"CAL video {rerun['video']}")
    rerun["fixture_pose_source"] = "calibrate_demo.py（同一次仿真内标定）"
    TWIN_DIR.mkdir(parents=True, exist_ok=True)
    (TWIN_DIR / "twin_report_calibrated.json").write_text(json.dumps(rerun, ensure_ascii=False, indent=2), encoding="utf-8")
    with open(TWIN_DIR / "gateway_calibrated.jsonl", "w", encoding="utf-8", newline="\n") as f:
        for rec in twin.gateway:
            f.write(json.dumps(rec, ensure_ascii=False) + "\n")

    cal_ok = after["translation_mm"] < CAL_TRANS_TOL_MM and probe_ok and not probe_collisions
    report = {
        "station": station["station_id"], "fixture": station["fixture_frame"], "generated": datetime.now().isoformat(timespec="seconds"),
        "probe_model": "机械臂下探到名义基准球顶；实测 = 场景中基准球实际中心 + N(0, noise_mm)；未模拟触碰传感过程",
        "true_offset": {"xyz_mm": args.offset_xyz, "rpy_deg": args.offset_rpy}, "noise_mm": args.noise_mm, "seed": args.seed,
        "nominal_pose": pose_json(F_nom), "target_pose": pose_json(F_target), "true_pose": pose_json(F_true),
        "true_pose_source": "场景读回：三个基准球实际中心无噪声拟合",
        "calibrated_pose": pose_json(F_cal),
        "scene_readback_datum_err_mm": {k: round(v, 6) for k, v in readback.items()}, "scene_readback_tol_mm": SCENE_READBACK_TOL_MM,
        "probes": probes, "probe_collisions": probe_collisions,
        "before": err_json(before), "after": err_json(after), "translation_tol_mm": CAL_TRANS_TOL_MM,
        "uncalibrated_hole_err_mm": uncal, "uncalibrated_hole_err_max_mm": max(uncal.values()),
        "note_orientation": "arm_program 只携带治具偏航（yaw_offset_deg），标定出的微小横滚/俯仰不改变工具姿态，只体现在孔位坐标上",
        "rerun_dfm_fix": dfm_fix,
        "rerun": {"report": "build/twin/twin_report_calibrated.json", "ok": rerun["ok"], "points_arrived": rerun["points_arrived"],
                  "points_total": rerun["points_total"], "hole_err_max_mm": rerun["hole_err_max_mm"],
                  "collisions": len(rerun["collisions"]), "takt_measured_s": rerun["takt_measured_s"]},
        "calibration_ok": cal_ok,
        "ok": cal_ok and rerun["ok"],
    }
    (TWIN_DIR / "calibration_report.json").write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8")
    print(f"CAL before_t={before['translation_mm']:.4f}mm after_t={after['translation_mm']:.4f}mm after_r={after['rotation_deg']:.5f}deg "
          f"uncal_hole_max={report['uncalibrated_hole_err_max_mm']}mm rerun_ok={rerun['ok']} rerun_hole_max={rerun['hole_err_max_mm']}mm "
          f"ok={report['ok']}")
    exit_code = 0 if report["ok"] else 1
except Exception:
    import traceback

    traceback.print_exc()
finally:
    for _ in range(5):
        app.update()
    app.close(exit_code=exit_code)
sys.exit(exit_code)
