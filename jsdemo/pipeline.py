"""演示流水线：九个阶段，四个签字关口。

python -m jsdemo run            交互运行，关口处等你输入 y
python -m jsdemo run --yes      自动通过关口（用于测试和录屏前预跑）
python -m jsdemo run --ecn ecn/ECN-001.yaml   跑完后再演示一次设计变更
"""
from __future__ import annotations

import json
import time
from datetime import datetime
from pathlib import Path

from . import clearance, codegen, composer, dfm, handling, docsgen, ecn as ecnmod, flow as flowmod, layout, procmodel, review, sop as sopmod, station as stationmod, takt
from .io import ROOT, dump_yaml, groups_by_id, load_library, load_modules, load_product, load_rules, load_yaml, points, write_text
from .statemachine import StationSM

GATES = {"dfm": "研发", "process": "工艺工程师", "compose": "项目负责人", "fai": "质量"}


class GateRejected(Exception):
    pass


class SopInvalid(Exception):
    pass


class StationInvalid(Exception):
    pass


class Runner:
    def __init__(self, out: Path, yes: bool = False, root: Path = ROOT, quiet: bool = False, sop: str | None = None,
                 station_spec: str | None = None):
        self.out, self.yes, self.root, self.quiet = Path(out), yes, root, quiet
        self.sop_path, self.station_spec = sop, station_spec
        self.policy, self.fixture_id = None, None
        self.report: dict = {"started": datetime.now().isoformat(timespec="seconds"), "stages": {}}
        self.log_lines: list[str] = []
        self.gates: list[dict] = []
        self.decisions: list[dict] = []

    # ---- 基础 ----
    def log(self, who: str, msg: str):
        line = f"[{who}] {msg}"
        self.log_lines.append(line)
        if not self.quiet:
            print(line)

    def gate(self, stage: str, summary: str):
        who = GATES[stage]
        if self.yes:
            ok = True
        else:
            ans = input(f"\n── 签字关口 · {who} ──\n{summary}\n确认签字？[y/N] ").strip().lower()
            ok = ans in ("y", "yes")
        self.gates.append({"stage": stage, "signer_role": who, "approved": ok,
                           "time": datetime.now().isoformat(timespec="seconds"), "auto": self.yes})
        self.log("PM", f"关口 {stage}：{who} {'已签字' if ok else '未签字'}")
        if not ok:
            raise GateRejected(stage)

    def decision(self, d: dict, confirmed_by: str):
        d = {**d, "confirmed_by": confirmed_by, "status": "draft"}
        self.decisions.append(d)
        dump_yaml(d, self.out / "decisions" / f"{d['id']}.yaml")
        self.log("秘书", f"记录决策草稿 {d['id']}：{d['decision']}")

    # ---- 阶段 ----
    def run(self, stop_after: str | None = None, ecn_path: str | None = None) -> dict:
        self.out.mkdir(parents=True, exist_ok=True)
        stages = ["intake", "dfm", "process", "compose", "generate", "review", "verify", "fai"]
        try:
            for s in stages:
                getattr(self, f"s_{s}")()
                if stop_after == s:
                    break
            if ecn_path and stop_after is None:
                self.s_ecn(ecn_path)
            self.report["status"] = "accepted" if self.report["stages"].get("verify", {}).get("ok") else "incomplete"
        except GateRejected as e:
            self.report["status"] = f"stopped_at_gate:{e}"
        except StationInvalid as e:
            self.report["status"] = "stopped:station_invalid"
            self.log("PM", f"工站设计校验有 {e} 处错误，打回上游 agent（见 station_validation.json）")
        except SopInvalid as e:
            self.report["status"] = "stopped:sop_invalid"
            self.log("PM", f"SOP 校验有 {e} 处错误，打回上游 agent（见 sop_validation.json）")
        except composer.CompositionError as e:
            self.report["status"] = "stopped:composition"
            self.log("PM", f"模块库无法满足：{e}。按规则停下来问人，不发明模块。")
        finally:
            self._flush()
        return self.report

    def s_intake(self):
        self.modules = load_modules(self.root)
        self._intake_station()
        if self.sop_path:
            self._intake_sop()
        else:
            self.product = load_product(root=self.root)
        self.rules = load_rules(self.root)
        self.lib = load_library(self.root)
        pts = points(self.product)
        g = groups_by_id(self.product)
        self.log("调研", f"读取 {self.product['name']}：{len(pts)} 个紧固点，{len(g)} 组，{len(self.product['ctq'])} 项关键特性")
        self.report["stages"]["intake"] = {"points": len(pts), "groups": list(g), "ctq": len(self.product["ctq"]),
                                           "target_takt_s": self.product["bom"]["target_takt_s"],
                                           "product": self.product["name"], **({"sop": self.sop_summary} if self.sop_path else {}),
                                           "station_spec": self.station_summary}

    def _intake_station(self):
        """工站设计（上游 agent 输出，jsdemo.station/1）：布局、治具位姿、原位都从它读。给了就先校验，有 error 打回。
        没给时布局取示例工站设计（demo），不校验。"""
        L = layout.use(self.station_spec)
        self.station_summary = {"path": L["SPEC_PATH"], "explicit": bool(self.station_spec)}
        if not self.station_spec:
            self.log("调研", f"布局取示例工站设计 {L['SPEC_PATH']}（demo）")
            return
        doc = load_yaml(self.station_spec)
        res = stationmod.validate(doc, modules=self.modules)
        write_text(json.dumps({"station_spec": str(self.station_spec), **res}, ensure_ascii=False, indent=2), self.out / "station_validation.json")
        c = res["counts"]
        self.log("调研", f"工站设计 {self.station_spec}：error {c['error']}、缺失 {c['blocker']}、待审 {c['review']}、警告 {c['warning']}"
                 f"（{'可签字' if res['signable'] else '不可签字'}）")
        self.station_summary.update({"counts": c, "buildable": res["buildable"], "signable": res["signable"]})
        if not res["buildable"]:
            raise StationInvalid(c["error"])
        ref = doc["meta"]["sop"]
        if self.sop_path and Path(self.sop_path).resolve() != (self.root / ref).resolve():
            self.log("调研", f"--sop {self.sop_path} 与工站设计引用的 {ref} 不一致，按 --sop")
        self.sop_path = self.sop_path or str(ref)

    def _intake_sop(self):
        """工站 SOP（上游 agent 输出）→ 产品数据、每组策略、零件与工步。有 error 就打回，不往下走。"""
        doc = load_yaml(self.sop_path)
        res = sopmod.validate(doc, modules=self.modules)
        write_text(json.dumps({"sop": str(self.sop_path), **res}, ensure_ascii=False, indent=2), self.out / "sop_validation.json")
        c = res["counts"]
        self.log("调研", f"SOP {self.sop_path}：error {c['error']}、缺失 {c['blocker']}、待审 {c['review']}、警告 {c['warning']}"
                 f"（{'可签字' if res['signable'] else '不可签字，缺失项需人补'}）")
        self.sop_summary = {"path": str(self.sop_path), "counts": c, "buildable": res["buildable"], "signable": res["signable"]}
        if not res["buildable"]:
            raise SopInvalid(c["error"])
        self.product = sopmod.to_product(doc)
        self.policy = sopmod.to_station_policy(doc)
        self.fixture_id = doc["frame"]["mounting_in_fixture"]["fixture"]
        self.assembly = sopmod.to_assembly(doc)
        for name, data in (("bom.yaml", self.product["bom"]), ("fastening_points.yaml", self.product["fastening"]),
                           ("ctq.yaml", {**({"example_data": True} if doc.get("example_data") else {}), "ctq": self.product["ctq"]})):
            dump_yaml(data, self.out / "product" / name)
        dump_yaml(self.assembly, self.out / "assembly.yaml")

    def s_dfm(self):
        r = dfm.check(self.product, self.rules["dfm"], self.lib["screws"])
        for i in r["issues"]:
            self.log("调研", i["message"])
        self.report["stages"]["dfm"] = r
        if r["issues"]:
            i = r["issues"][0]
            self.decision({"id": "DEC-001", "decision": i["suggestion"],
                           "rejected": "保持原布局：批头无法到位，会导致漏打或刮伤元件",
                           "assumptions": f"{i['obstacle']} 高于紧固面，批头外径加浮动行程需要 {i['limit_mm']} mm 间隙",
                           "invalidation": "更换批头规格或浮动批头型号后重新检查", "trigger": "跨边界"}, "研发")
        self.gate("dfm", f"{r['points_passed']}/{r['points_total']} 个点通过，问题 {len(r['issues'])} 个")

    def s_process(self):
        groups = groups_by_id(self.product)
        self.windows = {}
        for gid, g in groups.items():
            w = procmodel.recommend(g["material"], g["screw"], g.get("hole_d_mm"), self.lib)
            self.windows[gid] = w
            if w["status"] == "unknown":
                self.log("实现", f"{g['name']}：工艺模型返回未知（{w['reason']}），按规则停下来")
                raise GateRejected("process_unknown")
            self.log("实现", f"{g['name']} {g['material']} {g['screw']}：{w['window_nm'][0]}–{w['window_nm'][1]} N·m（{w['status']}，置信度 {w['confidence']}）")
        probe = procmodel.recommend("PEEK", "M2x6-tapping", 1.6, self.lib)
        self.log("实现", f"擦除演示：PEEK + M2 自攻 → {probe['status']}（{probe.get('reason', '')}）")
        self.report["stages"]["process"] = {"windows": self.windows, "erasure_probe": probe}
        self.gate("process", "\n".join(f"{gid}: {w['window_nm']} N·m ({w['status']})" for gid, w in self.windows.items()))

    def s_compose(self):
        n_place = (sum(1 for s in self.assembly["steps"] if s["type"] == "place" and s["actor"] in handling.ACTORS_FOR_HANDLER)
                   if getattr(self, "assembly", None) else 0)
        self.station = composer.compose(self.product, self.modules, self.windows, self.lib["screws"], fixture_id=self.fixture_id,
                                        place_steps=n_place)
        self.station["layout_source"] = {"station_spec": layout.current()["SPEC_PATH"]}   # 孪生按同一份工站设计搭场景
        self.station["product_source"] = ({"sop": str(self.sop_path), "product_dir": "product", "assembly": "assembly.yaml"}
                                          if self.sop_path else {"kb": f"kb/product/{self.product['name']}"})
        n = len(points(self.product))
        self.takt = takt.estimate(self.station, self.modules, n, self.product["bom"]["target_takt_s"])
        for d in self.station["decisions"]:
            self.decision(d, "项目负责人")
        self.log("实现", f"组合 {len(self.station['modules'])} 个模块，全部来自模块库")
        self.log("验证", f"节拍 {self.takt['total_s']} s，目标 {self.takt['target_s']} s")
        self.report["stages"]["compose"] = {"station": self.station, "takt": self.takt}
        dump_yaml(self.station, self.out / "station.yaml")
        self.gate("compose", f"模块 {len(self.station['modules'])} 个；节拍 {self.takt['total_s']} s / {self.takt['target_s']} s；误差预算 {self.station['checks']['error_budget_mm']} mm")

    def s_generate(self, inject_gap: str | None = "C2"):
        self.flow = flowmod.build(self.product, self.station, self.windows, policy=self.policy)
        pids = [s["point"] for s in self.flow["sequence"]]
        if inject_gap not in pids:   # 审查演示：故意漏掉一个点位的控制计划行，审查要能查出来
            inject_gap = pids[min(1, len(pids) - 1)]
        dump_yaml(self.flow, self.out / "flow.yaml")
        F = composer.fixture_pose(self.modules, self.station["fixture_frame"])
        # 放件（M4 ②）：有放件臂时先做放件规划，拧紧臂程序在需要新零件的点之前插入握手
        self.handling = handling.plan(self.assembly, self.modules, self.station, F) if getattr(self, "assembly", None) else None
        self.handler_ops = []
        if self.handling:
            self.handler_ops = handling.handler_ops(self.handling, self.modules)
            dump_yaml({k: v for k, v in self.handling.items() if k != "H"}, self.out / "handling.yaml")
            write_text(json.dumps(self.handler_ops, ensure_ascii=False, indent=1), self.out / "handler_program.json")
            write_text(codegen.urscript_handler(self.handler_ops), self.out / "handler_program.urscript")
            for it in self.handling["items"]:
                self.log("实现", f"放件 {it['step']} {it['name']}：料盘第 {it['slot']} 穴 → 治具，{it['tool']} {it['grasp']['rule']}"
                                 f"（{'执行者缺失，演示交给放件臂' if it['actor_assumed'] else '放件臂'}）")
        g = groups_by_id(self.product)
        screw_of = {p["id"]: g[p["group"]]["screw"] for p in points(self.product)}
        feeder = self.modules[[m["id"] for m in self.station["modules"] if m["role"] == "feeder"][0]]
        self.feed = codegen.feed_points(self.flow, self.station["checks"]["screw_kinds"], screw_of, feeder["lanes"])
        self.ops = codegen.neutral(self.flow, F, handoff=self.handling["handoff_before_point"] if self.handling else None,
                                   park_j=layout.PARK_J, feed=self.feed,
                                   window=self.handling.get("tool_change_window_before_point") if self.handling else None)
        write_text(json.dumps(self.ops, ensure_ascii=False, indent=1), self.out / "arm_program.json")
        write_text(codegen.urscript(self.ops), self.out / "arm_program.urscript")
        files = docsgen.generate(self.product, self.windows, self.flow, self.out, inject_gap=inject_gap)
        if not self.sop_path:
            self.decision({"id": "DEC-003", "decision": "驱动板组 NG 后整件停止",
                           "rejected": "标记 NG 继续：自攻滑牙后塑料孔已损坏，继续拧后续点位没有意义",
                           "assumptions": "支架为塑料件，滑牙不可返修", "invalidation": "支架改为金属嵌件后重新评估",
                           "trigger": "偏离默认"}, "工艺工程师")
        else:   # 策略来自 SOP：整件停止的组照记决策草稿，理由写「缺失」，不替上游补
            g = groups_by_id(self.product)
            for s in self.flow["sequence"]:
                if "stop_unit" in s["on_ng"] and not any(d["id"] == f"DEC-003-{s['group']}" for d in self.decisions):
                    self.decision({"id": f"DEC-003-{s['group']}", "decision": f"{g[s['group']]['name']}组 NG 后整件停止（来自 SOP）",
                                   "rejected": "缺失", "assumptions": "缺失", "invalidation": "缺失", "trigger": "偏离默认"}, "工艺工程师")
        self.log("实现", f"生成业务流、机械臂程序、{len(files)} 个文件")
        hfiles = ["handling.yaml", "handler_program.json", "handler_program.urscript"] if self.handling else []
        self.report["stages"]["generate"] = {"files": ["flow.yaml", "arm_program.json", "arm_program.urscript", "station.yaml"] + hfiles + files}

    def s_review(self):
        issues = review.check(self.out, self.flow)
        for i in issues:
            self.log("审查", i["message"])
        fixed = False
        if issues:
            docsgen.generate(self.product, self.windows, self.flow, self.out, inject_gap=None)
            self.log("实现", "按审查意见重新生成文件")
            fixed = True
        after = review.check(self.out, self.flow)
        self.log("审查", f"复核：{len(after)} 处不一致")
        self.report["stages"]["review"] = {"found": issues, "fixed": fixed, "remaining": after}

    def s_verify(self):
        from sim.kinematic import reach_check
        golden = self.rules["golden"]
        arm = self.modules[[m["id"] for m in self.station["modules"] if m["role"] == "arm"][0]]
        prog = (self.out / "arm_program.urscript").read_text(encoding="utf-8") + json.dumps(self.ops, ensure_ascii=False)
        if self.handler_ops:   # 放件臂程序同样不许出现紧固逻辑
            prog += (self.out / "handler_program.urscript").read_text(encoding="utf-8") + json.dumps(self.handler_ops, ensure_ascii=False)
        forbidden = [t for t in golden["arm_program_forbidden_tokens"] if t in prog]
        checks = {
            "feasibility": all(m["id"] in self.modules for m in self.station["modules"]),
            "reach": reach_check(self.ops, arm)["ok"] and (not self.handler_ops or reach_check(self.handler_ops, arm)["ok"]),
            "takt": self.takt["ok"],
            "boundary": not forbidden,
            "consistency": not self.report["stages"]["review"]["remaining"],
            "erasure": self.report["stages"]["process"]["erasure_probe"]["status"] == "unknown",
        }
        # 离线干涉检查：需要零件外形，只有走 SOP（有 assembly）时才有；没有就如实记跳过，不算通过
        clash = None
        if getattr(self, "assembly", None):
            env = self.modules[[m["id"] for m in self.station["modules"] if m["role"] == "executor"][0]]["envelope"]
            clash = clearance.check(self.flow, self.assembly, env, self.lib["screws"], self.report["stages"]["dfm"]["issues"],
                                    feed=self.feed, F=composer.fixture_pose(self.modules, self.station["fixture_frame"]))
            checks["clearance"] = clash["ok"]
            for h in clash["known_hits"]:
                self.log("验证", f"干涉 {h['point']}：{h['tool']} 碰 {h['obstacle']}（深 {h['depth_mm']} mm），DFM 已记录，待布局修改")
            for h in clash["new_hits"]:
                self.log("验证", f"新干涉 {h['point']}：{h['tool']} 碰 {h['obstacle']}（深 {h['depth_mm']} mm，{'/'.join(h['legs'])}）")
            if self.handling:   # 放件臂（M4 ②）
                F = composer.fixture_pose(self.modules, self.station["fixture_frame"])
                clash["handler"] = clearance.check_handler(self.handling, self.assembly, self.modules, F)
                checks["clearance"] = checks["clearance"] and clash["handler"]["ok"]
                for h in clash["handler"]["new_hits"]:
                    self.log("验证", f"放件干涉 {h['step']}：{h['tool']} 碰 {h['obstacle']}（深 {h['depth_mm']} mm）")
            dump_yaml(clash, self.out / "clearance.yaml")
        else:
            self.log("验证", "clearance: 跳过（没有零件外形，走 --sop 时才检查）")
        for k, v in checks.items():
            self.log("验证", f"{k}: {'通过' if v else '失败'}")
        self.report["stages"]["verify"] = {"checks": checks, "ok": all(checks.values()), "forbidden_found": forbidden,
                                           "clearance": clash,
                                           "note": "完整验收以 pytest 为准；Isaac 孪生运行是第 8 项，见 docs/03-plan.md"}

    def s_fai(self):
        groups = groups_by_id(self.product)

        class Exec:
            def __init__(s2, runner):
                s2.r = runner
            def run(s2, params_id, point):
                g = groups[next(p for p in points(self.product) if p["id"] == point)["group"]]
                w = next(w for w in self.windows.values() if w["id"] == params_id)
                spec = self.lib["screws"][g["screw"]]
                rows = procmodel.synth_curve("ok", w["window_nm"], w["target_nm"], spec, g["depth_mm"])
                procmodel.write_curve(rows, self.out / "curves" / f"{point}.csv")
                return procmodel.judge(rows, w["window_nm"], spec, g["depth_mm"])

        class Arm:
            def move_to(s2, t): pass
            def lock(s2): pass
            def retract(s2): pass

        res = StationSM(self.flow, Arm(), Exec(self)).run_unit("FAI-0001")
        ok = sum(1 for r in res["results"] if r["result"] == "OK")
        self.log("验证", f"首件 {ok}/{len(res['results'])} 合格（演示曲线；真实部署时由执行器上传）")
        self.report["stages"]["fai"] = res
        self.gate("fai", f"首件 {ok}/{len(res['results'])} 合格")

    def s_ecn(self, path: str):
        e = load_yaml(self.root / path if not Path(path).is_absolute() else path)
        imp = ecnmod.impact(self.product, e, self.lib)
        self.log("PM", f"收到变更 {imp['ecn']}：{imp['title']}")
        for gid, w in imp["new_windows"].items():
            if w["status"] == "unknown":
                self.log("实现", f"{gid} 组新窗口未知，变更被阻塞，需要打样")
            else:
                self.log("实现", f"{gid} 组重推窗口：{w['window_nm'][0]}–{w['window_nm'][1]} N·m（{w['status']}）")
        self.log("审查", f"受影响 {len(imp['affected_points'])} 点，不受影响 {len(imp['unaffected_points'])} 点，需更新 {len(imp['files_to_update'])} 处")
        dump_yaml(imp, self.out / "ecn_impact.yaml")
        self.report["stages"]["ecn"] = imp

    def _flush(self):
        self.report["finished"] = datetime.now().isoformat(timespec="seconds")
        self.report["gates"] = self.gates
        self.report["decisions"] = [d["id"] for d in self.decisions]
        write_text(json.dumps(self.report, ensure_ascii=False, indent=2), self.out / "report.json")
        write_text("\n".join(self.log_lines) + "\n", self.out / "run_log.txt")
