"""孪生运行报告（M3.2 / M3.4）：报告不存在时跳过。

M3.4 起产品带碰撞、障碍物按产品数据放置（用户 2026-10-24：如实建模）：
  - 基线 twin_report.json：孪生查出的障碍物干涉必须与 DFM 未关闭的问题一一对应，干涉点必须记问题、整件判 NG；
  - 按 DFM 建议外移障碍物后的 twin_report_dfmfix.json：整件 ok 为 true（M3.2 原验收在这份报告上继续成立）。
"""
import json
from pathlib import Path

import pytest

TWIN = Path(__file__).resolve().parents[1] / "build" / "twin"
FIELDS = ["station", "sn", "segments", "points", "points_arrived", "points_total", "takt", "takt_measured_s",
          "collisions", "results", "feeder", "ok", "unit_result", "handling", "twin_ok", "issues",
          "obstacle_collisions", "dfm_open_issues"]


def _load(name):
    p = TWIN / name
    if not p.exists():
        pytest.skip(f"还没有 {name}：先运行 sim/isaac/run_flow.py（见 docs/03-plan.md M3.4）")
    return json.loads(p.read_text(encoding="utf-8"))


@pytest.fixture(scope="module")
def report():
    return _load("twin_report.json")


@pytest.fixture(scope="module")
def fixed():
    return _load("twin_report_dfmfix.json")


def test_fields_complete(report):
    missing = [k for k in FIELDS if k not in report]
    assert not missing, f"缺少字段 {missing}"
    for k in ("measured_s", "estimated_s", "deviation_pct", "flag"):
        assert k in report["takt"]


def test_baseline_interference_matches_dfm(report):
    """孪生与 DFM 结论一致：撞到的障碍物 = DFM-01 未关闭的 (点位, 障碍物)；没有别的碰撞。"""
    assert report["obstacle_collisions"] == report["dfm_open_issues"]
    assert all("/components/" in c["with"] for c in report["collisions"])   # 障碍物 = SOP 零件上的元件（M3.5b）


def test_baseline_interference_recorded(report):
    hit_points = {p for p, _ in report["obstacle_collisions"]}
    issue_points = {i["point"] for i in report["issues"] if i["mode"] == "interference"}
    assert hit_points == issue_points
    if hit_points:
        assert report["unit_result"] == "NG" and report["ok"] is False
    else:
        assert report["ok"] is True


def test_results_cover_every_point(report):
    assert sorted(r["point"] for r in report["results"]) == sorted(p["point"] for p in report["points"])


def test_takt_flag_consistent(report):
    t = report["takt"]
    assert t["flag"] == (abs(t["deviation_pct"]) > 20.0)


def test_dfm_fix_unit_ok(fixed):
    """M3.2 验收：13 个点全部到达、无碰撞、全部合格、ok 为 true。"""
    assert fixed["ok"] is True and fixed["twin_ok"] is True
    assert fixed["points_arrived"] == fixed["points_total"] == len(fixed["points"])
    assert fixed["collisions"] == [] and fixed["issues"] == []
    assert all(r["result"] == "OK" for r in fixed["results"])
    assert fixed["dfm_fix"], "外移记录为空"
