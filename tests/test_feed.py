"""取钉（DEC-FEED-001）：每颗螺钉先去对应料道的出钉位取，再去孔位。孪生报告不存在时跳过孪生部分。"""
import json
from pathlib import Path

import pytest

from jsdemo.io import load_yaml
from jsdemo.layout import feed_pick_tcp

ROOT = Path(__file__).resolve().parents[1]


def test_each_point_feeds_from_its_lane(run):
    out = run["out"]
    ops = json.loads((out / "arm_program.json").read_text(encoding="utf-8"))
    st = load_yaml(out / "station.yaml")
    kinds = st["checks"]["screw_kinds"]
    fp = load_yaml(ROOT / "kb" / "product" / "joint-v1" / "fastening_points.yaml")
    screw = {p["id"]: next(g["screw"] for g in fp["groups"] if g["id"] == p["group"]) for p in fp["points"]}
    for pt, sid in screw.items():
        seq = [o for o in ops if o.get("point") == pt]
        feed = [o for o in seq if o.get("purpose") == "feed"]
        first_hole = next(k for k, o in enumerate(seq) if o["op"] == "movel" and o.get("purpose") != "feed")
        assert all(seq.index(o) < first_hole for o in feed)          # 先取钉，再去孔位
        lane = kinds.index(sid)
        pick = next(o for o in feed if o["speed"] == "slow")["pose"]["xyz_mm"]
        assert pick == pytest.approx(feed_pick_tcp(lane, 3, float(sid.split("-")[0].split("x")[1])), abs=1e-3)
        assert any(o.get("until") == "feed_ok" and o["point"] == pt for o in seq)


def test_urscript_waits_for_feed(run):
    src = (run["out"] / "arm_program.urscript").read_text(encoding="utf-8")
    assert src.count("get_standard_digital_in(4)") == 13


def test_twin_picked_every_screw():
    p = ROOT / "build" / "twin" / "twin_report_dfmfix.json"
    if not p.exists():
        pytest.skip("还没有孪生报告")
    rep = json.loads(p.read_text(encoding="utf-8"))
    feed = [s for s in rep["segments"] if s.get("purpose") == "feed"]
    if not feed:
        pytest.skip("孪生报告是取钉之前的版本")
    picked = [s for s in feed if "feed_ok" in s]
    assert len(picked) == 13 and all(s["feed_ok"] for s in picked)
    assert all(s["arrived"] for s in feed) and rep["collisions"] == []
