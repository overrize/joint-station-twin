"""职责边界：机械臂程序里没有紧固逻辑；golden 约束没有被改动。"""
import hashlib
from pathlib import Path

from jsdemo.io import ROOT


def test_arm_program_has_no_fastening_logic(run, rules):
    text = (run["out"] / "arm_program.urscript").read_text(encoding="utf-8")
    text += (run["out"] / "arm_program.json").read_text(encoding="utf-8")
    found = [t for t in rules["golden"]["arm_program_forbidden_tokens"] if t in text]
    assert not found, f"机械臂程序出现了紧固相关内容：{found}"


def test_golden_unchanged():
    expect = (Path(__file__).parent / "golden.sha256").read_text().strip()
    raw = (ROOT / "kb" / "rules" / "golden.yaml").read_bytes().replace(b"\r\n", b"\n")  # Windows 换行不影响判断
    actual = hashlib.sha256(raw).hexdigest()
    assert actual == expect, "golden.yaml 被改动了。这个文件只能由人签字修改，并同步更新 tests/golden.sha256"


def test_points_in_fixture_frame(run):
    from jsdemo.io import load_yaml
    flow = load_yaml(run["out"] / "flow.yaml")
    assert all(s["approach"]["frame"] == flow["fixture"] and s["engage"]["frame"] == flow["fixture"] for s in flow["sequence"])
