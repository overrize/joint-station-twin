"""任务 M3.2 / M3.4：在数字孪生里跑完一整件（运行核心在 twin.py）。

输入：build/twin/JS-01.usd、build/station/arm_program.json、build/station/curves/*.csv
输出（<tag> 由选项决定，基线运行为空）：
  build/twin/twin_report<tag>.json —— 每段动作实测时间、整件节拍、碰撞、到位误差、每点判定、异常处置、供钉器余量
  build/twin/gateway<tag>.jsonl    —— 网关 GW-1 的旁路数据流：供钉器 MES 数据点 + 每点紧固结果 + 复拧/问题事件
  build/twin/issues<tag>.jsonl     —— 问题记录（每个最终不合格点一条，状态「待复判」）

运行（先 python -m jsdemo run --yes 与 build_scene.py）：
    F:\\isaacsim\\python.bat sim\\isaac\\run_flow.py [--headless]
    ... --dfm-fix                                     # 按 DFM-01 建议外移障碍物（只改场景）验证整改效果，tag 含 _dfmfix
    ... --fault-rate 0.01 --seed 3                    # 每颗螺钉每次拧 1% 异常（滑牙/浮高），tag 含 _demo
    ... --inject D2:slip --inject E3:float,ok          # 指定点位第 1、2 次拧的结果（演示用），tag 含 _demo
    ... --snap                                        # 关键时刻近景截图到 build/twin/media/

--fixture-pose build/twin/calibration_report.json（M3.3）：场景里的治具放到报告中的实际位姿（模拟现场放偏），
机械臂程序用报告中标定出的位姿重新生成（jsdemo.codegen.neutral），tag 含 _calibrated。
"""
from __future__ import annotations

import argparse
import json
import sys

from _common import STATION_DIR, TWIN_DIR, load_outputs

NL = chr(10)
ap = argparse.ArgumentParser()
ap.add_argument("--headless", action="store_true")
ap.add_argument("--keep-open", action="store_true", help="有界面时跑完不退出，关窗口才结束（现场演示用）")
ap.add_argument("--fixture-pose", help="自标定报告 JSON（calibrate_demo.py 产出）：用其中标定后的治具位姿重跑整件")
ap.add_argument("--sn", default="TWIN-0001")
ap.add_argument("--dfm-fix", action="store_true", help="按 DFM-01 建议外移障碍物（只改场景，不改产品数据）")
ap.add_argument("--fault-rate", type=float, default=0.0, help="每颗螺钉每次拧的异常概率（演示用，如 0.01）")
ap.add_argument("--seed", type=int, default=0)
ap.add_argument("--inject", action="append", default=[], metavar="POINT:MODE[,MODE]", help="指定点位各次拧的结果：ok/slip/float")
ap.add_argument("--snap", action="store_true", help="近景截图到 build/twin/media/")
ap.add_argument("--record", action="store_true", help="录屏：mp4 + 字幕 srt 到 build/twin/media/<tag>/（M4.1）")
ap.add_argument("--label", default="", help="追加到输出文件名的标签，区分多次演示运行（如 d2slip）")
ap.add_argument("--record-every", type=int, default=2, help="录屏每几个物理步截一帧（2 → 30 fps；4 → 15 fps，截图减半、录得快，播放仍为正常速度）")
ap.add_argument("--serial", action="store_true", help="两臂轮流（不并行）；默认两臂并行（用户 2026-10-26：节拍加速）")
ap.add_argument("--units", type=int, default=1, help="流水线：连续装几台（M4 ③，需要放件臂；>1 时报告写 twin_report<tag>_line.json）")
ap.add_argument("--collision-selftest", action="store_true", help="自检：把执行器压向桌面，确认碰撞检测能报出来")
args, _ = ap.parse_known_args()

inject = {}
for item in args.inject:
    pt, modes = item.split(":")
    inject[pt] = modes.split(",")
    bad = [m for m in inject[pt] if m not in ("ok", "slip", "float")]
    if bad:
        sys.exit(f"--inject {item}：不支持的结果 {bad}（只支持 ok / slip / float）")

from isaacsim import SimulationApp  # noqa: E402

app = SimulationApp({"headless": args.headless})

from isaacsim.core.rendering_manager import ViewportManager  # noqa: E402

from jsdemo import codegen  # noqa: E402
from jsdemo.procmodel import MODE_CN  # noqa: E402
from jsdemo.geometry import Pose  # noqa: E402
from twin import Twin  # noqa: E402

CAMERA_MAIN = {"eye": [0.15, -1.85, 1.15], "target": [0.45, -0.12, 0.18]}


def pose_from_json(d: dict) -> Pose:
    return Pose([list(map(float, r)) for r in d["R"]], list(map(float, d["xyz_mm"])))


exit_code = 1
try:
    station, flow, ops, modules = load_outputs()
    tag, cal = "", None
    if args.fixture_pose:
        cal = json.loads(open(args.fixture_pose, encoding="utf-8").read())
        ops = codegen.neutral(flow, pose_from_json(cal["calibrated_pose"]))
        tag += "_calibrated"
    if args.dfm_fix:
        tag += "_dfmfix"
    if args.fault_rate > 0 or inject:
        tag += "_demo"
    if args.label:
        tag += "_" + args.label
    media = TWIN_DIR / "media" / (tag.strip("_") or "baseline")
    twin = Twin(app, station, ops, modules, headless=args.headless, sn=args.sn, fault_rate=args.fault_rate, seed=args.seed,
                inject=inject, snap_dir=media if args.snap else None, tag=tag, record_dir=media if args.record else None,
                record_every=args.record_every)
    twin.parallel = not args.serial
    twin.kanban_overlay = args.record and args.units > 1   # 流水线录屏：画面右上角叠加线体看板
    if cal:
        twin.place_fixture(pose_from_json(cal["true_pose"]))
    if args.dfm_fix:
        dfm = json.loads((STATION_DIR / "report.json").read_text(encoding="utf-8"))["stages"]["dfm"]["issues"]
        print(f"TWIN dfm-fix {twin.apply_dfm_fix(dfm)}")
    if not args.headless or args.record:
        twin.camera_main = CAMERA_MAIN
        ViewportManager.set_camera_view("/OmniverseKit_Persp", **CAMERA_MAIN)
    if args.collision_selftest:
        r = twin.collision_selftest()
        print(f"TWIN selftest detected={r['detected']} {r['collisions'][:3]}")
        exit_code = 0 if r["detected"] else 1
    else:
        twin.caption(f"{station['product']} · 序列号 {args.sn} · 工站 {station['station_id']}"
                     + (" · 异常注入 " + "，".join(f"{k} 第{i + 1}次{MODE_CN.get(m, m)}" for k, ms in inject.items() for i, m in enumerate(ms))
                        if inject else "")
                     + (f" · 异常概率 {args.fault_rate:.0%}" if args.fault_rate > 0 else ""))
        if args.units > 1:   # 流水线（M4 ③）
            units = twin.run_line(args.units)
            if args.record:
                twin.set_view("main")
                twin.caption("　".join(f"第 {r['line']['unit']} 台 {r['line']['sn']}：{'合格' if r['unit_result'] == 'OK' else '不合格'}，"
                                      f"周期 {r['line']['cycle_s']} s" for r in units))
                twin.hold_frames(4.0)
                v = twin.finish_recording(f"{args.sn}_line")
                print(f"TWIN video {v}")
            line = {"units": units, "summary": {"n": len(units), "ok": [r["unit_result"] for r in units],
                                                "cycle_s": [r["line"]["cycle_s"] for r in units],
                                                "total_s": round(sum(r["line"]["cycle_s"] for r in units), 2),
                                                "twin_ok": all(r["twin_ok"] for r in units)}}
            TWIN_DIR.mkdir(parents=True, exist_ok=True)
            (TWIN_DIR / f"twin_report{tag}_line.json").write_text(json.dumps(line, ensure_ascii=False, indent=2), encoding="utf-8")
            with open(TWIN_DIR / f"gateway{tag}_line.jsonl", "w", encoding="utf-8", newline=NL) as f:
                for rec in twin.gateway:
                    f.write(json.dumps(rec, ensure_ascii=False) + NL)
            for r in units:
                print(f"TWIN line unit={r['line']['unit']} sn={r['line']['sn']} interval={r['line'].get('interval_s')}s "
                      f"points={r['points_arrived']}/{r['points_total']} "
                      f"collisions={len(r['collisions'])} unit={r['unit_result']} placed={r['handler']['placed'] if r.get('handler') else None} "
                      f"cycle={r['line']['cycle_s']}s twin_ok={r['twin_ok']}")
            print(f"TWIN line total={line['summary']['total_s']}s twin_ok={line['summary']['twin_ok']}")
            exit_code = 0 if line["summary"]["twin_ok"] else 1
            raise SystemExit
        report = twin.run()
        if args.record:
            twin.set_view("product")
            twin.caption(f"整件{'合格' if report['unit_result'] == 'OK' else '不合格'}：{sum(r['result'] == 'OK' for r in report['results'])}/"
                         f"{report['points_total']} 点合格，问题 {len(report['issues'])} 条，仿真节拍 {report['takt']['measured_s']} s")
            twin.hold_frames(4.0)
            report["video"] = twin.finish_recording(f"{args.sn}")
            print(f"TWIN video {report['video']}")
        if cal:
            report["fixture_pose_source"] = args.fixture_pose
        TWIN_DIR.mkdir(parents=True, exist_ok=True)
        (TWIN_DIR / f"twin_report{tag}.json").write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8")
        for name, rows in (("gateway", twin.gateway), ("issues", twin.issues)):
            with open(TWIN_DIR / f"{name}{tag}.jsonl", "w", encoding="utf-8", newline="\n") as f:
                for rec in rows:
                    f.write(json.dumps(rec, ensure_ascii=False) + "\n")
        print(f"TWIN tag={tag or 'baseline'} points={report['points_arrived']}/{report['points_total']} collisions={len(report['collisions'])} "
              f"obstacle_hits={report['obstacle_collisions']} dfm_open={report['dfm_open_issues']} "
              f"results_ok={sum(r['result'] == 'OK' for r in report['results'])}/{len(report['results'])} "
              f"unit={report['unit_result']} issues={len(report['issues'])} handling_ok={report['handling']['ok']} "
              f"hole_err_max={report['hole_err_max_mm']}mm takt={report['takt']['measured_s']}s ok={report['ok']} twin_ok={report['twin_ok']}")
        # 返回码：孪生本身是否按预期工作（到位、无碰撞、异常处置正确）；整件是否合格看 unit_result
        exit_code = 0 if report["twin_ok"] else 1
except SystemExit:
    pass
except Exception:
    import traceback

    traceback.print_exc()
finally:
    if args.keep_open and not args.headless:
        print("TWIN 跑完，关闭 Isaac 窗口结束")
        while app.is_running():
            app.update()
    for _ in range(5):
        app.update()
    app.close(exit_code=exit_code)
sys.exit(exit_code)
