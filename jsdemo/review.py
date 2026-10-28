"""独立审查：只看产出和规则，不看生成过程。

核对：每个点位都有参数包、作业指导书步骤、控制计划条目；PFMEA 高风险项都有对应控制；业务流引用的参数都存在。
"""
from __future__ import annotations

import csv
from pathlib import Path


def check(out: Path, flow: dict) -> list[dict]:
    issues = []
    wi = (out / "docs" / "wi.md").read_text(encoding="utf-8")
    with open(out / "docs" / "control_plan.csv", encoding="utf-8") as f:
        cp = list(csv.DictReader(f))
    with open(out / "docs" / "pfmea.csv", encoding="utf-8") as f:
        pf = list(csv.DictReader(f))
    cp_points = {r["point"] for r in cp}
    for s in flow["sequence"]:
        p = s["point"]
        if not (out / "params" / f"{p}.yaml").exists():
            issues.append({"point": p, "kind": "missing_params", "message": f"{p} 缺少参数包"})
        if f"| {p} |" not in wi:
            issues.append({"point": p, "kind": "missing_wi", "message": f"{p} 缺少作业指导书步骤"})
        if p not in cp_points:
            issues.append({"point": p, "kind": "missing_control", "message": f"{p} 缺少控制计划条目"})
    high_groups = {r["group"] for r in pf if r["AP"] == "H"}
    for g in sorted(high_groups):
        gpts = [s["point"] for s in flow["sequence"] if s["group"] == g]
        miss = [p for p in gpts if p not in cp_points]
        for p in miss:
            issues.append({"point": p, "kind": "pfmea_uncovered", "message": f"{p} 属于 PFMEA 高风险组 {g}，但控制计划未覆盖"})
    return issues
