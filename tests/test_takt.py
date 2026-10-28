"""节拍：估算不超过目标。"""


def test_takt_within_target(run):
    t = run["report"]["stages"]["compose"]["takt"]
    assert t["ok"] and t["total_s"] <= t["target_s"]
