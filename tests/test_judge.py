"""曲线判定：合格、滑牙、浮高能正确区分。"""
import pytest

from jsdemo.procmodel import judge, synth_curve


@pytest.mark.parametrize("kind,mode", [("ok", "ok"), ("slip", "slip"), ("float", "float")])
def test_judge_modes(lib, kind, mode):
    spec = lib["screws"]["M2x6-tapping"]
    rows = synth_curve(kind, [0.15, 0.21], 0.185, spec, 5.0)
    assert judge(rows, [0.15, 0.21], spec, 5.0)["mode"] == mode
