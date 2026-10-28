"""孪生自标定报告（M3.3）：build/twin/calibration_report.json 不存在时跳过；存在时检查验收项。"""
import json
from pathlib import Path

import pytest

TWIN = Path(__file__).resolve().parents[1] / "build" / "twin"


@pytest.fixture(scope="module")
def report():
    p = TWIN / "calibration_report.json"
    if not p.exists():
        pytest.skip("还没有自标定报告：先运行 sim/isaac/calibrate_demo.py")
    return json.loads(p.read_text(encoding="utf-8"))


def test_fields_complete(report):
    for k in ("true_offset", "nominal_pose", "true_pose", "calibrated_pose", "probes", "before", "after", "rerun", "ok"):
        assert k in report, k
    assert len(report["probes"]) == 3


def test_offset_was_real(report):
    assert report["before"]["translation_mm"] > 0.5


def test_calibrated_within_tolerance(report):
    assert report["after"]["translation_mm"] < 0.1
    assert all(p["arm_arrived"] for p in report["probes"])
    assert report["probe_collisions"] == []


def test_rerun_ok(report):
    rerun = json.loads((TWIN / "twin_report_calibrated.json").read_text(encoding="utf-8"))
    assert rerun["ok"] is True and report["rerun"]["ok"] is True
    assert rerun["points_arrived"] == rerun["points_total"]
    assert rerun["collisions"] == []


def test_ok(report):
    assert report["ok"] is True
