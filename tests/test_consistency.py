"""一致性：审查能发现故意注入的缺口，修正后为零。"""
from jsdemo import docsgen, review
from jsdemo.io import load_yaml


def test_review_found_injected_gap(run):
    found = run["report"]["stages"]["review"]["found"]
    assert any(i["point"] == "C2" for i in found)


def test_no_remaining_issues(run):
    assert run["report"]["stages"]["review"]["remaining"] == []


def test_review_detects_gap_independently(run, product, tmp_path):
    flow = load_yaml(run["out"] / "flow.yaml")
    windows = run["report"]["stages"]["process"]["windows"]
    docsgen.generate(product, windows, flow, tmp_path, inject_gap="E4")
    issues = review.check(tmp_path, flow)
    assert {i["point"] for i in issues} == {"E4"}
