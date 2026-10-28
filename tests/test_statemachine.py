"""状态机：正常、滑牙、浮高、急停四条路径。"""
import pytest

from jsdemo.io import load_yaml
from jsdemo.statemachine import EStop, StationSM


class Arm:
    def move_to(self, t): pass
    def lock(self): pass
    def retract(self): pass


class Exec:
    def __init__(self, faults=None, estop_at=None):
        self.faults, self.estop_at = faults or {}, estop_at
    def run(self, params_id, point):
        if point == self.estop_at:
            raise EStop()
        mode = self.faults.get(point, "ok")
        return {"result": "OK" if mode == "ok" else "NG", "mode": mode}


@pytest.fixture
def flow(run):
    return load_yaml(run["out"] / "flow.yaml")


def test_normal(flow):
    r = StationSM(flow, Arm(), Exec()).run_unit()
    assert r["status"] == "OK" and len(r["results"]) == 13
    assert r["trace"][-1] == "UNLOAD"
    # 每个点位都走完 tier0 交接
    assert all(f"{s['point']}:DONE" in r["trace"] for s in flow["sequence"])


def test_slip_on_driver_board_stops_unit(flow):
    r = StationSM(flow, Arm(), Exec({"D2": "slip"})).run_unit()
    assert r["status"] == "NG"
    assert "D2:STOP_UNIT" in r["trace"]
    assert r["results"][-1]["point"] == "D2"


def test_float_on_end_cap_marks_ng_and_continues(flow):
    r = StationSM(flow, Arm(), Exec({"E3": "float"})).run_unit()
    assert r["status"] == "NG" and len(r["results"]) == 13
    assert r["trace"][-1] == "UNLOAD_NG"


def test_estop(flow):
    r = StationSM(flow, Arm(), Exec(estop_at="C2")).run_unit()
    assert r["status"] == "ESTOP" and r["trace"][-1] == "ESTOP"


def test_executor_retry_policy_in_flow(flow):
    """DEC-NG-001：机螺钉组 C/E 浮高时执行器复拧一次；自攻组 D 不复拧。"""
    for s in flow["sequence"]:
        if s["group"] in ("C", "E"):
            r = s["executor_retry"]
            assert {k: r[k] for k in ("max", "on_modes")} == {"max": 1, "on_modes": ["float"]}
            # 反转参数供应商未给：只能是标了 demo 的占位值，不能冒充工艺数据
            assert r["reverse"]["source"] == "demo_placeholder" and r["reverse"]["reverse_rpm"] > 0
        else:
            assert "executor_retry" not in s


def test_executor_retry_recovers(flow):
    """执行器复拧成功：该点最终合格，整件合格，轨迹里留下复拧记录。"""
    class RetryExec(Exec):
        def run(self, params_id, point):
            if point == "E3":
                return {"result": "OK", "mode": "ok", "attempts": 2, "first_mode": "float"}
            return super().run(params_id, point)
    r = StationSM(flow, Arm(), RetryExec()).run_unit()
    assert r["status"] == "OK" and "E3:EXEC_RETRY" in r["trace"]


def test_arm_program_has_no_retry_logic(run):
    """复拧完全在执行器侧：机械臂程序与无复拧时一致，只有保持-等待。"""
    import json
    ops = json.loads((run["out"] / "arm_program.json").read_text(encoding="utf-8"))
    assert {o["op"] for o in ops} == {"movej", "movel", "hold", "signal", "wait", "branch_on_fault"}
    assert not any("retry" in json.dumps(o) for o in ops)


def test_stop_unit_retracts_before_halt(run):
    """整件停止：URScript 在 halt 之前先退回接近位并回原位，不把批头留在螺钉上。"""
    src = (run["out"] / "arm_program.urscript").read_text(encoding="utf-8")
    lines = src.splitlines()
    i = next(k for k, ln in enumerate(lines) if "stop unit" in ln)
    block = lines[i + 1:lines.index("    halt", i)]
    assert len(block) == 2 and block[0].strip().startswith("movel(") and block[1].strip().startswith("movej(")
