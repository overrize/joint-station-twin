"""演示页接入真实数据（M4.2）：数值来自注入的 demo-data，页面脚本里不再写死结果。"""
import json
import re
from pathlib import Path

import pytest

from jsdemo import presenter

ROOT = Path(__file__).resolve().parents[1]
DATA_RE = re.compile(r'<script id="demo-data" type="application/json">(.*?)</script>', re.S)


@pytest.fixture(scope="module")
def built(run, tmp_path_factory):
    out = tmp_path_factory.mktemp("presenter")
    station = run["out"]
    page = presenter.build(station_dir=station, twin_dir=out / "no_twin", out=out / "page.html")
    rep = json.loads((station / "report.json").read_text(encoding="utf-8"))
    text = page.read_text(encoding="utf-8")
    return text, json.loads(DATA_RE.search(text).group(1).replace("<\\/", "</")), rep


def test_data_matches_report(built):
    _, d, rep = built
    st = rep["stages"]
    assert d["source"] == "build"
    assert d["points"] == st["intake"]["points"]
    assert d["dfm"]["rules_checked"] == st["dfm"]["rules_checked"]
    assert d["compose"]["modules"] == len(st["compose"]["station"]["modules"])
    assert d["compose"]["takt"]["total_s"] == st["compose"]["takt"]["total_s"]
    assert d["fai"]["ok"] == sum(r["result"] == "OK" for r in st["fai"]["results"])
    assert {g["id"]: g["window_nm"] for g in d["groups"]} == {k: w["window_nm"] for k, w in st["process"]["windows"].items()}
    assert d["example_data"] is True


def test_log_from_run(built):
    _, d, _ = built
    assert all(d["log"][k] for k in ("intake", "dfm", "process", "compose", "generate", "review", "verify", "fai"))


def test_missing_twin_is_explicit(built):
    _, d, _ = built
    assert d["twin"] is None   # 孪生没跑就是没跑，页面显示「未运行」，不拿示例数顶上


def test_no_hardcoded_results_in_script():
    src = (ROOT / "presenter" / "joint-station-demo.html").read_text(encoding="utf-8")
    script = DATA_RE.sub("", src)
    script = script[script.index("var D=JSON.parse"):]
    for stale in ("41.1", "0.185", "0.178", "12 条", "13 / 13", ">52<", "0.19–0.25"):
        assert stale not in script, stale
