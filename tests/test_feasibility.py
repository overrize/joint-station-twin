"""方案合法性：用到的模块都在库里，参数都在模块能力范围内。"""


def test_pipeline_accepted(run):
    assert run["report"]["status"] == "accepted"


def test_modules_from_library(run, modules):
    st = run["report"]["stages"]["compose"]["station"]
    assert all(m["id"] in modules for m in st["modules"])


def test_executor_covers_all_windows(run, modules):
    st = run["report"]["stages"]["compose"]["station"]
    ex = modules[next(m["id"] for m in st["modules"] if m["role"] == "executor")]
    for w in run["report"]["stages"]["process"]["windows"].values():
        assert ex["torque_range_nm"][0] <= w["window_nm"][0]
        assert w["window_nm"][1] <= 0.9 * ex["torque_range_nm"][1]


def test_feeder_lanes(run):
    c = run["report"]["stages"]["compose"]["station"]["checks"]
    assert c["feeder_lanes"] >= len(c["screw_kinds"])


def test_error_budget_within_float_capture(run):
    c = run["report"]["stages"]["compose"]["station"]["checks"]
    assert c["use_vision"] or c["error_budget_mm"] <= c["float_capture_mm"]


def test_dfm_finds_d3(run):
    issues = run["report"]["stages"]["dfm"]["issues"]
    assert any(i.get("point") == "D3" and i.get("obstacle") == "C12" for i in issues)
