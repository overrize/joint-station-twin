"""从同一个源（产品数据 + 工艺窗口 + 业务流）生成参数包和整套文件。

所有文件都从同一个源派生，改源就全部重新生成，不手改派生文件。
"""
from __future__ import annotations

import csv
import io
from pathlib import Path

from . import procmodel
from .io import dump_yaml, groups_by_id, points, write_text

MODE_ROWS = [  # (失效模式, S, O, D, 原因)
    ("滑牙", 7, 4, 3, "窗口上限偏高或底孔偏大"),
    ("浮高", 6, 3, 3, "未贴合即达到扭矩"),
    ("扭矩超差", 6, 3, 2, "参数错误或批头磨损"),
    ("漏打", 8, 2, 2, "供钉失败未检出"),
]


def _ctq_groups(product):
    gs, ps = set(), set()
    for c in product["ctq"]:
        gs.update(c.get("groups", []))
        ps.update(c.get("points", []))
    return gs, ps


def _ap(s, o, d):
    if s >= 8 and o * d >= 4:
        return "H"
    if s >= 7 and o * d >= 9:
        return "H"
    return "M" if o * d >= 6 else "L"


def generate(product: dict, windows: dict, flow: dict, out: Path, inject_gap: str | None = None) -> list[str]:
    """inject_gap：演示用，故意漏掉某个点位的控制计划条目，让审查环节能真实地发现它。"""
    groups = groups_by_id(product)
    ctq_g, ctq_p = _ctq_groups(product)
    files = []

    # 参数包：每个点位一份
    for s in flow["sequence"]:
        w = windows[s["group"]]
        dump_yaml({"point": s["point"], "params_id": w["id"], "window_nm": w["window_nm"], "target_nm": w["target_nm"],
                   "stages": w["stages"], "source": w["status"], "confidence": w["confidence"], "version": 1,
                   **({"retry": s["executor_retry"]} if s.get("executor_retry") else {})},
                  out / "params" / f"{s['point']}.yaml")
        files.append(f"params/{s['point']}.yaml")

    def _retry_cn(s):
        r = s.get("executor_retry")
        if not r:
            return ""
        modes = "、".join(procmodel.MODE_CN.get(m, m) for m in r["on_modes"])
        demo = "（反转参数为 demo 占位）" if r.get("reverse", {}).get("source") == "demo_placeholder" else ""
        return f"；{modes}时执行器自动复拧 {r['max']} 次{demo}"

    # 作业指导书
    lines = ["# 作业指导书 · 工站 JS-01（示例）", "", "| 步骤 | 点位 | 螺钉 | 参数 | 要点 |", "|---|---|---|---|---|"]
    for i, s in enumerate(flow["sequence"], 1):
        g = groups[s["group"]]
        lines.append(f"| {i} | {s['point']} | {g['screw']} | {s['executor_params']} | {g['name']}；异常处理：{'整件停止' if 'stop_unit' in s['on_ng'] else '标记 NG 继续'}{_retry_cn(s)} |")
    write_text("\n".join(lines) + "\n", out / "docs" / "wi.md")
    files.append("docs/wi.md")

    # PFMEA：按组
    pf_rows = []
    for g in groups.values():
        for mode, s, o, d, cause in MODE_ROWS:
            sev = 9 if g["id"] in ctq_g else s
            pf_rows.append([g["id"], g["name"], mode, cause, sev, o, d, _ap(sev, o, d)])
    buf = io.StringIO()
    w = csv.writer(buf)
    w.writerow(["group", "name", "failure_mode", "cause", "S", "O", "D", "AP"])
    w.writerows(pf_rows)
    write_text(buf.getvalue(), out / "docs" / "pfmea.csv")
    files.append("docs/pfmea.csv")

    # 控制计划：每个点位一行；CTQ 点位追加曲线全量存档
    buf = io.StringIO()
    w = csv.writer(buf)
    w.writerow(["point", "characteristic", "spec", "method", "frequency", "reaction"])
    for s in flow["sequence"]:
        if inject_gap and s["point"] == inject_gap:
            continue
        win = windows[s["group"]]["window_nm"]
        react = "整件停止并隔离" if "stop_unit" in s["on_ng"] else "标记 NG，下线复判"
        react = _retry_cn(s).lstrip("；") + "；" + react if s.get("executor_retry") else react
        w.writerow([s["point"], "终止扭矩与角度", f"{win[0]}–{win[1]} N·m", "执行器在线曲线判定", "100%", react])
        if s["group"] in ctq_g or s["point"] in ctq_p:
            w.writerow([s["point"], "曲线存档", "全量", "网关存档", "100%", "缺失即停线"])
    write_text(buf.getvalue(), out / "docs" / "control_plan.csv")
    files.append("docs/control_plan.csv")

    # 点检表、追溯映射、备件
    write_text("# 点检表（每班）\n\n- [ ] 批头磨损检查\n- [ ] 供钉器三通道余量\n- [ ] 治具基准点清洁\n- [ ] 浮动批头回中\n- [ ] 执行器自检通过\n",
               out / "docs" / "checklist.md")
    dump_yaml({"fields": ["sn", "station", "point", "params_id", "params_version", "final_nm", "total_angle_deg", "result", "mode", "curve_file", "timestamp"],
               "mes": "旁路模式：先由网关本地存储，有议价权后再对接客户 MES"}, out / "docs" / "trace_map.yaml")
    write_text("# 备件清单（示例）\n\n- 批头 ×10\n- 浮动批头弹簧组 ×2\n- 供钉器通道导轨 ×1\n", out / "docs" / "spares.md")
    files += ["docs/checklist.md", "docs/trace_map.yaml", "docs/spares.md"]
    return files
