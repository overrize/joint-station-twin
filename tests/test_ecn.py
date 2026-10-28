"""设计变更：只影响 D 组 4 个点，其余 9 个不动。"""


def test_ecn_impact(run):
    imp = run["report"]["stages"]["ecn"]
    assert imp["affected_points"] == ["D1", "D2", "D3", "D4"]
    assert len(imp["unaffected_points"]) == 9
    assert imp["new_windows"]["D"]["window_nm"] == [0.19, 0.25]
    assert len(imp["files_to_update"]) == 12
    assert imp["blocked"] == []
