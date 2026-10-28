"""孪生异常演示（M3.4）：注入异常后，复拧、问题记录、整件停止/继续都按业务流执行。报告不存在时跳过。

对应的运行（见 docs/03-plan.md M3.4）：
  run_flow.py --dfm-fix --inject D2:slip --label d2slip
  run_flow.py --dfm-fix --inject E3:float,ok --label e3retryok
  run_flow.py --dfm-fix --inject C2:float,float --label c2retryng
"""
import json
from pathlib import Path

import pytest

TWIN = Path(__file__).resolve().parents[1] / "build" / "twin"


def _load(label):
    p = TWIN / f"twin_report_dfmfix_demo_{label}.json"
    if not p.exists():
        pytest.skip(f"还没有 {p.name}")
    rep = json.loads(p.read_text(encoding="utf-8"))
    issues = [json.loads(x) for x in (TWIN / f"issues_dfmfix_demo_{label}.jsonl").read_text(encoding="utf-8").splitlines() if x.strip()]
    gateway = [json.loads(x) for x in (TWIN / f"gateway_dfmfix_demo_{label}.jsonl").read_text(encoding="utf-8").splitlines() if x.strip()]
    return rep, issues, gateway


def _res(rep, point):
    return next(r for r in rep["results"] if r["point"] == point)


def test_d2_slip_stops_unit():
    rep, issues, gw = _load("d2slip")
    assert rep["twin_ok"] is True and rep["handling"]["ok"] is True
    assert rep["unit_result"] == "NG"
    assert rep["results"][-1]["point"] == "D2" and rep["results"][-1]["mode"] == "slip"
    assert _res(rep, "D2")["attempts"] == 1           # 自攻螺钉不复拧（DEC-NG-001）
    assert [i["point"] for i in issues] == ["D2"] and issues[0]["unit_disposition"] == "整件停止"
    assert issues[0]["status"] == "待复判" and issues[0]["attempts"][0]["curve_file"]
    assert any(g.get("event") == "issue_opened" and g["point"] == "D2" for g in gw)


def test_e3_float_retry_recovers():
    rep, issues, gw = _load("e3retryok")
    assert rep["twin_ok"] is True and rep["unit_result"] == "OK" and rep["ok"] is True
    r = _res(rep, "E3")
    assert r["attempts"] == 2 and r["attempt_log"][0]["mode"] == "float" and r["result"] == "OK"
    assert issues == []
    assert any(g.get("event") == "exec_retry" and g["point"] == "E3" for g in gw)


def test_c2_float_retry_fails_marks_and_continues():
    rep, issues, gw = _load("c2retryng")
    assert rep["twin_ok"] is True and rep["unit_result"] == "NG"
    assert len(rep["results"]) == rep["points_total"]   # 机螺钉 NG 继续后续点位
    r = _res(rep, "C2")
    assert r["attempts"] == 2 and r["result"] == "NG" and r["mode"] == "float"
    assert [i["point"] for i in issues] == ["C2"]
    assert issues[0]["actions"][0] == "exec_retry" and "CTQ-2" in issues[0]["ctq"]
    assert all(x["result"] == "OK" for x in rep["results"] if x["point"] != "C2")
