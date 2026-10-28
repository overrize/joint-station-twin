"""可达性：每个接近和到位姿态都在机械臂工作范围内。"""
import json

from sim.kinematic import reach_check


def test_all_poses_reachable(run, modules):
    ops = json.loads((run["out"] / "arm_program.json").read_text(encoding="utf-8"))
    r = reach_check(ops, modules["ARM-UR5E"])
    assert r["ok"], r["violations"]
    feed = [o for o in ops if o["op"] == "movel" and o.get("purpose") == "feed"]
    assert len(feed) == 13 * 4                      # DEC-FEED-001：每颗去出钉位 接近 / 快降 / 慢降 / 抬起
    near = [o for o in ops if o["op"] == "movel" and o.get("purpose") == "near"]
    assert len(near) == 13                          # 节拍加速：每点快降到孔位上方 5 mm 的一个位姿
    assert r["checked"] == 13 * 3 + len(near) + len(feed)   # 孔位的接近、到位、退回，加上快降点与取钉位姿，全部查可达
