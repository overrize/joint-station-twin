"""业务流：设备无关的工站动作序列。agent 主要写这一层，状态机和机械臂程序都从它生成。"""
from __future__ import annotations

from .io import groups_by_id, points

APPROACH_MM = 30.0
NG_POLICY = {"D": ["retract", "mark_ng", "stop_unit"]}  # 自攻滑牙后整件停止
DEFAULT_NG = ["retract", "mark_ng", "continue"]
# 执行器侧复拧（DEC-NG-001，用户 2026-10-24 签字）：机螺钉组浮高时执行器自行反转后重拧一次，最后只回一次完成/异常；
# 机械臂程序不变（仍是保持-等待）。自攻螺钉组 D 滑牙不复拧
EXECUTOR_RETRY = {"C": {"max": 1, "on_modes": ["float"]}, "E": {"max": 1, "on_modes": ["float"]}}
# 复拧的反转参数属于执行器，供应商尚未给出（DEC-NG-001 assumptions）。演示阶段用占位值并标 demo（用户 2026-10-25 同意），
# 不作为工艺数据：反转退到起点 → 停顿 → 用同一参数包重拧
EXECUTOR_REVERSE_DEMO = {"reverse": "full_backout", "reverse_rpm": 300, "dwell_s": 0.3, "redrive": "same_params",
                         "source": "demo_placeholder"}


def _retry(gid, policy):
    r = policy[gid].get("executor_retry") if policy else EXECUTOR_RETRY.get(gid)
    return {"executor_retry": {**r, "reverse": r.get("reverse", EXECUTOR_REVERSE_DEMO)}} if r else {}


def build(product: dict, station: dict, windows: dict, policy: dict | None = None) -> dict:
    """policy：来自工站 SOP 的每组不合格与复拧策略（jsdemo.sop.to_station_policy）；没有时用本文件的默认值。"""
    groups = groups_by_id(product)
    seq = []
    for p in points(product):
        g = groups[p["group"]]
        x, y, z = p["xyz"]
        seq.append({
            "point": p["id"],
            "group": g["id"],
            "approach": {"frame": station["fixture_frame"], "xyz": [x, y, round(z + APPROACH_MM, 3)], "rpy": [180, 0, 0]},
            "engage": {"frame": station["fixture_frame"], "xyz": [x, y, z], "rpy": [180, 0, 0]},
            "handoff": "tier0",
            "executor_params": windows[g["id"]]["id"],
            "on_ng": policy[g["id"]]["on_ng"] if policy else NG_POLICY.get(g["id"], DEFAULT_NG),
            **_retry(g["id"], policy),
        })
    return {"station": station["station_id"], "fixture": station["fixture_frame"],
            "takt_target_s": product["bom"]["target_takt_s"], "sequence": seq}
