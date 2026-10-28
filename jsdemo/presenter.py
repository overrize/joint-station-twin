"""演示页接入真实数据（M4.2）：读流水线与孪生的产出，注入 presenter/joint-station-demo.html，写到 build/presenter/。

页面里有一段 <script id="demo-data" type="application/json">，源文件里是示例值（source = "example"），
这里整段替换成本次运行的结果。页面不联网，构建时注入即可。

    python -m jsdemo.presenter [--station build/station] [--twin build/twin]
"""
from __future__ import annotations

import argparse
import csv
import json
import re
from pathlib import Path

from .io import ROOT, load_yaml, write_text

PAGE = ROOT / "presenter" / "joint-station-demo.html"
OUT = ROOT / "build" / "presenter" / "joint-station-demo.html"
DATA_RE = re.compile(r'(<script id="demo-data" type="application/json">)(.*?)(</script>)', re.S)
ROLE_CN = {"调研", "实现", "审查", "验证", "秘书", "PM"}
TWIN_REPORT = "twin_report_dfmfix.json"   # 按 DFM 建议修正 C12 后的整件运行（基线 D3 干涉，见 M3.4）


def _split_log(lines: list[str]) -> dict[str, list[str]]:
    """run_log.txt 按出现顺序切到演示的各步：关口行收尾一步，生成 / 审查 / 验证 / 首件各自开头。"""
    steps = {k: [] for k in ("intake", "dfm", "process", "compose", "generate", "review", "verify", "fai")}
    order = list(steps)
    cur = "intake"
    for ln in lines:
        m = re.match(r"\[(.+?)\] (.*)", ln.strip())
        if not m or m.group(1) not in ROLE_CN:
            continue
        role, text = m.groups()
        if cur == "intake" and steps["intake"] and steps["intake"][-1][1].startswith("读取"):
            cur = "dfm"
        if cur == "compose" and text.startswith("生成"):
            cur = "generate"
        if cur == "generate" and role == "审查":
            cur = "review"
        if cur == "review" and role == "验证":
            cur = "verify"
        if cur == "verify" and text.startswith("首件"):
            cur = "fai"
        steps[cur].append([role, text])
        g = re.match(r"关口 ([a-z]+)", text) if role == "PM" else None
        if g:
            gate = g.group(1)
            if gate in steps and gate != "fai":
                cur = order[order.index(gate) + 1]
    return steps


def _curve(path: Path, n: int = 120) -> list[list[float]]:
    rows = [[float(a), float(t)] for a, t in list(csv.reader(path.open(encoding="utf-8")))[1:]]
    step = max(1, len(rows) // n)
    keep = rows[::step]
    if keep[-1] != rows[-1]:
        keep.append(rows[-1])
    return keep


def collect(station_dir: Path, twin_dir: Path) -> dict:
    rep = json.loads((station_dir / "report.json").read_text(encoding="utf-8"))
    st = rep["stages"]
    # 走 SOP 时产品数据随工站写在 station/product/；走 kb 时就在 kb/product/<产品>/
    pdir = station_dir / "product"
    if not pdir.exists():
        pdir = ROOT / "kb" / "product" / st["intake"]["product"]
    bom = load_yaml(pdir / "bom.yaml")
    fp = load_yaml(pdir / "fastening_points.yaml")
    ctq = load_yaml(pdir / "ctq.yaml")
    parts = {p["id"]: p for p in bom["parts"]}
    pts = fp["points"]

    def rng(ids):
        return f"{ids[0]}–{ids[-1]}" if len(ids) > 1 else ids[0]

    groups = []
    for g in fp["groups"]:
        ids = [p["id"] for p in pts if p["group"] == g["id"]]
        w = st["process"]["windows"][g["id"]]
        groups.append({"id": g["id"], "name": g["name"], "screw": g["screw"], "material": g["material"],
                       "part": parts.get(g.get("part"), {}).get("name", g.get("part")), "count": len(ids), "points": ids, "range": rng(ids),
                       "window_nm": w["window_nm"], "stages": w["stages"], "status": w["status"], "confidence": w["confidence"],
                       "window_id": w["id"]})

    flow = load_yaml(station_dir / "flow.yaml")
    d1 = next(s for s in flow["sequence"] if s["point"] == "D1")
    fai = st["fai"]["results"]
    d_final = [r["final_nm"] for r in fai if r["point"].startswith("D")]

    twin = None
    tp = twin_dir / TWIN_REPORT
    if tp.exists():
        t = json.loads(tp.read_text(encoding="utf-8"))
        twin = {"report": f"{twin_dir.name}/{TWIN_REPORT}", "generated": t.get("generated"), "twin_ok": t["twin_ok"],
                "unit_result": t["unit_result"], "points_arrived": t["points_arrived"], "points_total": t["points_total"],
                "collisions": len(t["collisions"]), "hole_err_max_mm": t["hole_err_max_mm"],
                "results_ok": sum(r["result"] == "OK" for r in t["results"]),
                "takt": {k: t["takt"].get(k) for k in ("arm_sim_s", "load_unload_s", "measured_s", "estimated_s", "deviation_pct", "target_s")}}

    ecn = load_yaml(station_dir / "ecn_impact.yaml") if (station_dir / "ecn_impact.yaml").exists() else None
    if ecn:
        nw = next(iter(ecn["new_windows"].values()))
        ecn = {"id": ecn["ecn"], "title": ecn["title"], "affected": ecn["affected_points"], "unaffected": ecn["unaffected_points"],
               "files": len(ecn["files_to_update"]), "refai": ecn["refai_points"], "window_nm": nw["window_nm"],
               "status": nw["status"], "confidence": nw["confidence"]}

    line = None
    lp = twin_dir / "twin_report_dfmfix_line_line.json"
    gp = twin_dir / "gateway_dfmfix_line_line.jsonl"
    if lp.exists() and gp.exists():   # 流水线（连续流动）：每台摘要 + 看板回放要用的事件
        L = json.loads(lp.read_text(encoding="utf-8"))
        keep = []
        for ln in gp.read_text(encoding="utf-8").splitlines():
            e = json.loads(ln)
            ev = e.get("event")
            if ev in ("unit_start", "unit_unloaded", "unit_result", "unit_delivered") or (ev is None and "result" in e and "point" in e):
                keep.append({k: e[k] for k in ("t_s", "event", "unit", "sn", "point", "result") if k in e})
        line = {"report": f"{twin_dir.name}/{lp.name}", "points_total": L["units"][0]["points_total"],
                "units": [{"unit": u["line"]["unit"], "sn": u["line"]["sn"], "result": u["unit_result"], "cycle_s": u["line"]["cycle_s"],
                           "interval_s": u["line"].get("interval_s"), "collisions": len(u["collisions"])} for u in L["units"]],
                "total_s": L["summary"]["total_s"], "events": keep}
    log = _split_log((station_dir / "run_log.txt").read_text(encoding="utf-8").splitlines())
    return {
        "source": "build",
        "example_data": bool(bom.get("example_data")),
        "generated": rep.get("finished"),
        "status": rep["status"],
        "product": {"id": bom["product"], "name": bom["name"], "target_takt_s": st["intake"]["target_takt_s"]},
        "points": len(pts), "point_ids": [p["id"] for p in pts],
        "groups": groups,
        "materials": sorted({g["material"] for g in groups}),
        "ctq": [c["name"] for c in ctq["ctq"]],
        "sop": st["intake"].get("sop"),
        "dfm": {"rules_checked": st["dfm"]["rules_checked"], "points_passed": st["dfm"]["points_passed"],
                "points_total": st["dfm"]["points_total"], "issues": st["dfm"]["issues"]},
        "erasure": st["process"].get("erasure_probe"),
        "compose": {"modules": len(st["compose"]["station"]["modules"]), "takt": st["compose"]["takt"]},
        "generate": {"files": st["generate"]["files"], "params": sum(f.startswith("params/") for f in st["generate"]["files"]),
                     "decisions": rep.get("decisions", []), "flow_d1": d1, "station_id": flow["station"], "fixture": flow["fixture"]},
        "review": st["review"],
        "verify": st["verify"]["checks"],
        "verify_clearance": st["verify"].get("clearance"),
        "fai": {"sn": st["fai"]["sn"], "ok": sum(r["result"] == "OK" for r in fai), "total": len(fai),
                "d_final_nm": [min(d_final), max(d_final)], "d1_final_nm": next(r["final_nm"] for r in fai if r["point"] == "D1"),
                "d1_curve": _curve(station_dir / "curves" / "D1.csv")},
        "ecn": ecn,
        "twin": twin,
        "line": line,
        "gates": rep.get("gates", []),
        "log": log,
    }


def build(station_dir: Path = ROOT / "build" / "station", twin_dir: Path = ROOT / "build" / "twin", out: Path = OUT) -> Path:
    data = collect(station_dir, twin_dir)
    page = PAGE.read_text(encoding="utf-8")
    blob = json.dumps(data, ensure_ascii=False).replace("</", "<\\/")
    page, n = DATA_RE.subn(lambda m: m.group(1) + blob + m.group(3), page)
    if n != 1:
        raise RuntimeError("presenter 页面里找不到 demo-data 数据块")
    write_text(page, out)
    return out


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--station", default=str(ROOT / "build" / "station"))
    ap.add_argument("--twin", default=str(ROOT / "build" / "twin"))
    ap.add_argument("--out", default=str(OUT))
    a = ap.parse_args(argv)
    print(build(Path(a.station), Path(a.twin), Path(a.out)))


if __name__ == "__main__":
    main()
