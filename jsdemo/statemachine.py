"""工站状态机：按业务流调度机械臂和执行器。机械臂只负责到位与锁定，紧固全部交给执行器。

tier0 交接：控制器给执行器「就绪」→ 执行器回「接管」→ 结束时回「完成」或「异常」。
"""
from __future__ import annotations


class EStop(Exception):
    pass


class StationSM:
    def __init__(self, flow: dict, arm, executor):
        self.flow, self.arm, self.executor = flow, arm, executor
        self.trace: list[str] = []

    def _t(self, s):
        self.trace.append(s)

    def run_unit(self, sn: str = "SN-0001") -> dict:
        results, status = [], "OK"
        self._t("LOAD")
        self._t("LOCATE")
        try:
            for step in self.flow["sequence"]:
                pid = step["point"]
                self.arm.move_to(step["approach"])
                self._t(f"{pid}:APPROACH")
                self.arm.move_to(step["engage"])
                self.arm.lock()
                self._t(f"{pid}:LOCK")
                self._t(f"{pid}:READY")
                res = self.executor.run(step["executor_params"], pid)
                self._t(f"{pid}:TAKE")
                if res.get("attempts", 1) > 1:  # 执行器侧复拧（executor_retry），机械臂只是多等一会
                    self._t(f"{pid}:EXEC_RETRY")
                self._t(f"{pid}:{'DONE' if res['result'] == 'OK' else 'FAULT'}")
                self.arm.retract()
                results.append({"point": pid, **res})
                if res["result"] != "OK":
                    status = "NG"
                    if "stop_unit" in step["on_ng"]:
                        self._t(f"{pid}:STOP_UNIT")
                        break
            self._t("UNLOAD_NG" if status == "NG" else "UNLOAD")
        except EStop:
            self._t("ESTOP")
            status = "ESTOP"
        return {"sn": sn, "status": status, "results": results, "trace": list(self.trace)}
