"""工站 SOP（上游 agent 的输出）：校验与转换。标准见 docs/06-sop-contract.md，格式定义见 kb/contracts/sop-v1.schema.json。

校验结果分四级：
  error    —— 结构错误或几何上做不到，SOP 不能用来搭工站，打回上游 agent；
  blocker  —— 信息缺失（「缺失」、工艺库无样本、决策未确认等），可以先搭孪生排练，但不能签字；
  review   —— 可制造性问题（如批头间隙不足），交给流水线 DFM 关口由人判断；
  warning  —— 占位/估算数据、几何上可疑的地方，提示但不阻断。

转换：把 SOP 写成流水线现有的产品数据格式（bom / fastening_points / ctq），点位从产品坐标系换到治具坐标系；
另输出 station_policy（不合格与复拧策略）和 assembly（零件外形与工步，供孪生使用）。生成物只写 build/。

    python -m jsdemo.sop kb/sop/joint-v1.sop.yaml                 # 校验并转换到 build/sop/<product>/
"""
from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path

from . import procmodel
from .flow import APPROACH_MM
from .geometry import Pose
from .io import ROOT, dump_yaml, load_library, load_modules, load_rules, load_yaml

SCHEMA_ID = "jsdemo.sop/1"
MISSING = "缺失"
SOURCES = {"drawing", "cad", "measured", "experiment", "customer", "supplier", "estimate", "placeholder", "example"}
SOFT_SOURCES = {"estimate", "placeholder"}
ACTORS = {"operator", "robot", "executor", "upstream"}
NG_ACTIONS = {"continue": ["retract", "mark_ng", "continue"], "stop_unit": ["retract", "mark_ng", "stop_unit"]}
RETRY_MODES = {"float", "slip", "under", "over"}
SURFACE_TOL_MM = 0.05
EPS = 0.01


class Findings:
    def __init__(self):
        self.items: list[dict] = []

    def add(self, level: str, code: str, path: str, message: str, fix: str = ""):
        self.items.append({"level": level, "code": code, "path": path, "message": message, **({"fix": fix} if fix else {})})

    def of(self, level):
        return [i for i in self.items if i["level"] == level]


def is_missing(v) -> bool:
    return v == MISSING


def _num(v) -> bool:
    return isinstance(v, (int, float)) and not isinstance(v, bool)


def _vec(v, n=3) -> bool:
    return isinstance(v, list) and len(v) == n and all(_num(x) for x in v)


# ---------- 简化体几何（v1：零件姿态只允许绕 z 旋转，简化体轴线沿 z） ----------
def _shape_to_product(shape: dict, pose: dict) -> dict:
    """零件坐标系下的简化体 → 产品坐标系。返回带 xy（中心）、z（区间）、yaw 的副本。"""
    yaw = math.radians(pose["rpy"][2])
    c, s = math.cos(yaw), math.sin(yaw)
    t = pose["xyz"]
    out = dict(shape)
    if shape["type"] == "box":
        cx, cy, cz = shape["center"]
        sx, sy, sz = shape["size"]
        out["xy"] = [c * cx - s * cy + t[0], s * cx + c * cy + t[1]]
        out["z"] = [cz - sz / 2 + t[2], cz + sz / 2 + t[2]]
        out["yaw"] = math.degrees(yaw) + shape.get("yaw", 0.0)
    else:
        x, y = shape.get("xy", [0.0, 0.0])
        out["xy"] = [c * x - s * y + t[0], s * x + c * y + t[1]]
        out["z"] = [shape["z"][0] + t[2], shape["z"][1] + t[2]]
    return out


def _xy_distance(shape: dict, p) -> float:
    """点 p(x, y) 到简化体投影的距离（在内部为 0）。"""
    dx, dy = p[0] - shape["xy"][0], p[1] - shape["xy"][1]
    if shape["type"] == "cylinder":
        return max(0.0, math.hypot(dx, dy) - shape["r"])
    if shape["type"] == "ring":
        d = math.hypot(dx, dy)
        return shape["r_in"] - d if d < shape["r_in"] else max(0.0, d - shape["r_out"])
    a = -math.radians(shape["yaw"])
    lx, ly = math.cos(a) * dx - math.sin(a) * dy, math.sin(a) * dx + math.cos(a) * dy
    hx, hy = shape["size"][0] / 2, shape["size"][1] / 2
    return math.hypot(max(0.0, abs(lx) - hx), max(0.0, abs(ly) - hy))


def _contains_xy(shape, p) -> bool:
    return _xy_distance(shape, p) <= 1e-9


def screw_length_mm(screw_id: str) -> float | None:
    """'M3x8-machine' → 8.0；型号里没有长度时返回 None。"""
    try:
        return float(screw_id.split("-")[0].split("x")[1])
    except (IndexError, ValueError):
        return None


# ---------- 结构检查 ----------
def _check_structure(sop: dict, F: Findings) -> bool:
    ok = True

    def need(obj, key, path, check=None, what=""):
        nonlocal ok
        if not isinstance(obj, dict) or key not in obj:
            F.add("error", "E-SCHEMA", f"{path}.{key}", "必填字段缺少", "按 docs/06-sop-contract.md 补上该字段；不知道的值写「缺失」")
            ok = False
            return None
        v = obj[key]
        if check and not is_missing(v) and not check(v):
            F.add("error", "E-SCHEMA", f"{path}.{key}", f"类型不对：应为 {what}，实际 {v!r}")
            ok = False
        return v

    if sop.get("schema") != SCHEMA_ID:
        F.add("error", "E-SCHEMA", "schema", f"schema 应为 {SCHEMA_ID!r}，实际 {sop.get('schema')!r}")
        return False
    meta = need(sop, "meta", "", lambda v: isinstance(v, dict), "对象") or {}
    for k in ("product", "name", "revision", "status"):
        need(meta, k, "meta", lambda v: isinstance(v, str), "字符串")
    gb = need(meta, "generated_by", "meta", lambda v: isinstance(v, dict), "对象") or {}
    for k in ("agent", "version", "at"):
        need(gb, k, "meta.generated_by")
    units = need(sop, "units", "", lambda v: isinstance(v, dict), "对象") or {}
    for k, want in (("length", "mm"), ("angle", "deg"), ("time", "s")):
        if units.get(k) != want:
            F.add("error", "E-UNIT", f"units.{k}", f"单位必须是 {want}，实际 {units.get(k)!r}")
            ok = False
    frame = need(sop, "frame", "", lambda v: isinstance(v, dict), "对象") or {}
    mnt = need(frame, "mounting_in_fixture", "frame", lambda v: isinstance(v, dict), "对象") or {}
    need(mnt, "fixture", "frame.mounting_in_fixture", lambda v: isinstance(v, str), "字符串")
    need(mnt, "xyz", "frame.mounting_in_fixture", _vec, "3 个数")
    need(mnt, "rpy", "frame.mounting_in_fixture", _vec, "3 个数")
    need(mnt, "source", "frame.mounting_in_fixture")
    parts = need(sop, "parts", "", lambda v: isinstance(v, list) and v, "非空列表") or []
    for i, p in enumerate(parts):
        path = f"parts[{i}]"
        for k in ("id", "name", "material"):
            need(p, k, path, lambda v: isinstance(v, str), "字符串")
        pose = need(p, "pose", path, lambda v: isinstance(v, dict), "对象") or {}
        need(pose, "xyz", f"{path}.pose", _vec, "3 个数")
        need(pose, "rpy", f"{path}.pose", _vec, "3 个数")
        geo = need(p, "geometry", path, lambda v: isinstance(v, dict), "对象") or {}
        need(geo, "source", f"{path}.geometry")
        shapes = need(geo, "shapes", f"{path}.geometry", lambda v: isinstance(v, list) and v, "非空列表") or []
        comps = p.get("components", []) or []
        for j, sh in enumerate(list(shapes) + [c.get("shape", {}) for c in comps]):
            sp = f"{path}.geometry.shapes[{j}]" if j < len(shapes) else f"{path}.components[{j - len(shapes)}].shape"
            ok &= _check_shape(sh, sp, F)
        for j, c in enumerate(comps):
            for k in ("id", "name", "shape", "source"):
                need(c, k, f"{path}.components[{j}]")
    need(sop, "screws", "", lambda v: isinstance(v, list) and v, "非空列表")
    for i, s in enumerate(sop.get("screws") or []):
        need(s, "id", f"screws[{i}]", lambda v: isinstance(v, str), "字符串")
    steps = need(sop, "steps", "", lambda v: isinstance(v, list) and v, "非空列表") or []
    for i, st in enumerate(steps):
        path = f"steps[{i}]"
        need(st, "id", path, lambda v: isinstance(v, str), "字符串")
        typ = need(st, "type", path, lambda v: v in ("place", "fasten", "inspect"), "place / fasten / inspect")
        need(st, "actor", path)
        if typ == "place":
            need(st, "part", path, lambda v: isinstance(v, str), "字符串")
            need(st, "duration_s", path, _num, "数")
        elif typ == "fasten":
            for k in ("group", "name", "fastens", "into", "screw"):
                need(st, k, path, lambda v: isinstance(v, str), "字符串")
            hole = need(st, "hole", path, lambda v: isinstance(v, dict), "对象") or {}
            need(hole, "material", f"{path}.hole", lambda v: isinstance(v, str), "字符串")
            need(hole, "depth_mm", f"{path}.hole", _num, "数")
            need(hole, "source", f"{path}.hole")
            if "spec" not in hole and "d_mm" not in hole:
                F.add("error", "E-SCHEMA", f"{path}.hole", "孔要给 spec（如 M2-thread）或 d_mm（自攻底孔直径）之一")
                ok = False
            need(st, "approach", path, _vec, "3 个数")
            ng = need(st, "on_ng", path, lambda v: isinstance(v, dict), "对象") or {}
            need(ng, "action", f"{path}.on_ng", lambda v: v in NG_ACTIONS, " / ".join(NG_ACTIONS))
            pts = need(st, "points", path, lambda v: isinstance(v, list) and v, "非空列表") or []
            for j, pt in enumerate(pts):
                need(pt, "id", f"{path}.points[{j}]", lambda v: isinstance(v, str), "字符串")
                need(pt, "xyz", f"{path}.points[{j}]", _vec, "3 个数")
    need(sop, "open_items", "", lambda v: isinstance(v, list), "列表（没有就写 []）")
    return ok


def _check_shape(sh, path, F) -> bool:
    t = sh.get("type") if isinstance(sh, dict) else None
    req = {"cylinder": ("r", "z"), "ring": ("r_in", "r_out", "z"), "box": ("center", "size")}
    if t not in req:
        F.add("error", "E-SCHEMA", f"{path}.type", f"简化体类型应为 cylinder / ring / box，实际 {t!r}")
        return False
    ok = True
    for k in req[t]:
        v = sh.get(k)
        if is_missing(v):
            F.add("error", "E-MISSING", f"{path}.{k}", "外形尺寸不能缺失（孪生和干涉检查都要用）", "给出尺寸；不确定时给估计值并把 geometry.source 标为 estimate")
            ok = False
        elif k == "z" and not (_vec(v, 2) and v[1] > v[0]):
            F.add("error", "E-SCHEMA", f"{path}.z", f"z 应为 [下, 上] 且上 > 下，实际 {v!r}")
            ok = False
        elif k in ("center", "size") and not _vec(v):
            F.add("error", "E-SCHEMA", f"{path}.{k}", f"应为 3 个数，实际 {v!r}")
            ok = False
        elif k in ("r", "r_in", "r_out") and not (_num(v) and v > 0):
            F.add("error", "E-SCHEMA", f"{path}.{k}", f"应为正数，实际 {v!r}")
            ok = False
    if t == "ring" and ok and sh["r_in"] >= sh["r_out"]:
        F.add("error", "E-SCHEMA", path, "环的 r_in 必须小于 r_out")
        ok = False
    return ok


# ---------- 语义检查 ----------
def _source(F, v, path, example):
    if is_missing(v):
        F.add("blocker", "B-MISSING", path, "数据来源缺失")
    elif v not in SOURCES:
        F.add("error", "E-SCHEMA", path, f"来源应为 {sorted(SOURCES)} 之一，实际 {v!r}")
    elif v == "example" and not example:
        F.add("error", "E-SOURCE", path, "source: example 只允许出现在 example_data: true 的示例文件里")
    elif v in SOFT_SOURCES:
        F.add("warning", "W-SOFT", path, f"数据为 {v}（非图纸/实测），签字前需确认")


def validate(sop: dict, *, library=None, rules=None, modules=None, decisions_dir: Path | None = None) -> dict:
    F = Findings()
    if not _check_structure(sop, F):
        return _result(F)
    lib = library or load_library()
    rules = rules or load_rules()
    modules = modules or load_modules()
    decisions_dir = decisions_dir or ROOT / "kb" / "decisions"
    example = sop.get("example_data") is True
    dfm01 = next((r for r in rules["dfm"] if r["check"] == "bit_clearance"), None)
    clear_mm = dfm01["min_mm"] if dfm01 else 0.0

    # 文件头
    meta = sop["meta"]
    if meta["status"] not in ("draft", "signed"):
        F.add("error", "E-SCHEMA", "meta.status", "status 应为 draft 或 signed")
    mnt = sop["frame"]["mounting_in_fixture"]
    if mnt["fixture"] not in modules:
        F.add("blocker", "B-FIXTURE", "frame.mounting_in_fixture.fixture", f"模块库里没有治具 {mnt['fixture']}，需要新设计治具")
    elif modules[mnt["fixture"]].get("product") != meta["product"]:
        F.add("blocker", "B-FIXTURE", "frame.mounting_in_fixture.fixture",
              f"治具 {mnt['fixture']} 是为 {modules[mnt['fixture']].get('product')} 设计的，借用到 {meta['product']} 需验证定位与夹持")
    if any(abs(a) > 1e-9 for a in mnt["rpy"][:2]):
        F.add("error", "E-UNSUPPORTED", "frame.mounting_in_fixture.rpy", "v1 只支持产品绕 z 旋转装夹（横滚、俯仰必须为 0）")
    _source(F, mnt["source"], "frame.mounting_in_fixture.source", example)
    tt = sop.get("takt_target_s")
    if not tt:
        F.add("blocker", "B-MISSING", "takt_target_s", "目标节拍缺失")
    else:
        if is_missing(tt.get("value")):
            F.add("blocker", "B-MISSING", "takt_target_s.value", "目标节拍缺失")
        _source(F, tt.get("source", MISSING), "takt_target_s.source", example)

    # 零件
    parts, ids = {}, set()
    for i, p in enumerate(sop["parts"]):
        if p["id"] in parts:
            F.add("error", "E-DUP", f"parts[{i}].id", f"零件 id 重复：{p['id']}")
        parts[p["id"]] = p
        if any(abs(a) > 1e-9 for a in p["pose"]["rpy"][:2]):
            F.add("error", "E-UNSUPPORTED", f"parts[{i}].pose.rpy", "v1 只支持零件绕 z 旋转（横滚、俯仰必须为 0）")
        _source(F, p["geometry"]["source"], f"parts[{i}].geometry.source", example)
        for j, c in enumerate(p.get("components", []) or []):
            _source(F, c["source"], f"parts[{i}].components[{j}].source", example)
            if c["id"] in ids:
                F.add("error", "E-DUP", f"parts[{i}].components[{j}].id", f"元件 id 重复：{c['id']}")
            ids.add(c["id"])
        if p["geometry"].get("mesh"):
            F.add("warning", "W-MESH-IGNORED", f"parts[{i}].geometry.mesh", "v1 只用简化体，网格附件暂不使用")
    screws = {s["id"]: s for s in sop["screws"]}
    for i, s in enumerate(sop["screws"]):
        if s["id"] not in lib["screws"]:
            F.add("error", "E-REF", f"screws[{i}].id", f"工艺库没有螺钉型号 {s['id']}", "先在 process/library 补该型号（需人签字），或改用库内型号")
        if screw_length_mm(s["id"]) is None:
            F.add("warning", "W-SCREW-LEN", f"screws[{i}].id", "型号里读不出螺钉长度（应形如 M3x8-...），无法检查螺钉够不够长")
    ctq_ids = {c["id"] for c in sop.get("ctq", []) or []}

    # 工步
    placed: list[str] = []
    step_ids, point_ids, groups = set(), set(), set()
    for i, st in enumerate(sop["steps"]):
        path = f"steps[{i}]"
        if st["id"] in step_ids:
            F.add("error", "E-DUP", f"{path}.id", f"工步 id 重复：{st['id']}")
        step_ids.add(st["id"])
        if is_missing(st["actor"]):
            F.add("blocker", "B-MISSING", f"{path}.actor", "执行者缺失（operator / robot / executor / upstream）", "写明谁做这一步")
        elif st["actor"] not in ACTORS:
            F.add("error", "E-SCHEMA", f"{path}.actor", f"执行者应为 {sorted(ACTORS)} 之一，实际 {st['actor']!r}")
        if st["type"] == "place":
            if st["part"] not in parts:
                F.add("error", "E-REF", f"{path}.part", f"引用了不存在的零件 {st['part']}")
            elif st["part"] in placed:
                F.add("error", "E-ORDER", f"{path}.part", f"零件 {st['part']} 被放了两次")
            else:
                placed.append(st["part"])
            if is_missing(st["duration_s"]):
                F.add("blocker", "B-MISSING", f"{path}.duration_s", "放件时长缺失，节拍里无法计入")
        elif st["type"] == "fasten":
            _check_fasten(st, path, parts, screws, placed, lib, ctq_ids, clear_mm, F, example, decisions_dir)
            if st["group"] in groups:
                F.add("error", "E-DUP", f"{path}.group", f"拧紧组 {st['group']} 重复")
            groups.add(st["group"])
            for j, pt in enumerate(st["points"]):
                if pt["id"] in point_ids:
                    F.add("error", "E-DUP", f"{path}.points[{j}].id", f"点位 id 重复：{pt['id']}")
                point_ids.add(pt["id"])
    for pid in parts:
        if pid not in placed:
            F.add("warning", "W-UNPLACED", "parts", f"零件 {pid} 没有放件工步")
    if meta["status"] == "signed":
        if not meta.get("signed_by") or is_missing(meta.get("signed_by")):
            F.add("error", "E-SIGN", "meta.signed_by", "status 为 signed 时必须写签字人")
        if F.of("blocker"):
            F.add("error", "E-SIGN", "meta.status", f"还有 {len(F.of('blocker'))} 项缺失信息，不能标为 signed")
    for i, oi in enumerate(sop["open_items"]):
        F.add("blocker", "B-OPEN", f"open_items[{i}]", f"上游 agent 声明的未决问题：{oi.get('text', oi)}")
    return _result(F)


def _shapes_of(part: dict, with_components=True) -> list[dict]:
    out = [_shape_to_product(s, part["pose"]) for s in part["geometry"]["shapes"]]
    if with_components:
        out += [{**_shape_to_product(c["shape"], part["pose"]), "component": c["id"]} for c in part.get("components", []) or []]
    return out


def _check_fasten(st, path, parts, screws, placed, lib, ctq_ids, clear_mm, F, example, decisions_dir):
    for k in ("fastens", "into"):
        if st[k] not in parts:
            F.add("error", "E-REF", f"{path}.{k}", f"引用了不存在的零件 {st[k]}")
        elif st[k] not in placed:
            F.add("error", "E-ORDER", f"{path}.{k}", f"拧紧时零件 {st[k]} 还没放（放件工步必须排在前面）")
    if st["screw"] not in screws:
        F.add("error", "E-REF", f"{path}.screw", f"螺钉 {st['screw']} 不在 screws 列表里")
    if st["actor"] not in ("robot", MISSING):
        F.add("error", "E-UNSUPPORTED", f"{path}.actor", "本工站拧紧由机械臂 + 执行器完成，fasten 的 actor 应为 robot")
    if [round(v, 9) for v in st["approach"]] != [0.0, 0.0, -1.0]:
        F.add("error", "E-UNSUPPORTED", f"{path}.approach", "v1 只支持竖直向下接近 [0, 0, -1]（DFM-04）")
    hole = st["hole"]
    _source(F, hole["source"], f"{path}.hole.source", example)
    for c in st.get("ctq", []) or []:
        if c not in ctq_ids:
            F.add("error", "E-REF", f"{path}.ctq", f"引用了不存在的 CTQ {c}")
    for j, pt in enumerate(st["points"]):
        for c in pt.get("ctq", []) or []:
            if c not in ctq_ids:
                F.add("error", "E-REF", f"{path}.points[{j}].ctq", f"引用了不存在的 CTQ {c}")
    ng = st["on_ng"]
    retry = ng.get("retry")
    if retry:
        if not (isinstance(retry.get("max"), int) and retry["max"] >= 1) or not set(retry.get("on_modes", [])) <= RETRY_MODES:
            F.add("error", "E-SCHEMA", f"{path}.on_ng.retry", f"retry 应为 {{max: ≥1 的整数, on_modes: {sorted(RETRY_MODES)} 的子集, decision}}")
        dec = retry.get("decision")
        dp = decisions_dir / f"{dec}.yaml" if dec and not is_missing(dec) else None
        if not dp or not dp.exists() or (load_yaml(dp) or {}).get("status") != "confirmed":
            F.add("blocker", "B-DECISION", f"{path}.on_ng.retry.decision", "复拧是工艺策略，要引用已确认的决策记录（kb/decisions）", "请人确认后写入决策 id")
    if any(is_missing(v) for v in (hole["material"], hole["depth_mm"], hole.get("d_mm"), hole.get("spec"))):
        F.add("blocker", "B-MISSING", f"{path}.hole", "孔的材料 / 规格 / 深度缺失，工艺窗口无法确定")
    elif st["screw"] in lib["screws"]:
        rec = procmodel.recommend(hole["material"], st["screw"], hole.get("d_mm"), lib)
        if rec["status"] == "unknown":
            F.add("blocker", "B-PROCESS", f"{path}.hole", f"工艺库给不出窗口：{rec['reason']}（流水线会停下来问人）")
    if st["fastens"] not in parts or st["into"] not in parts:
        return

    # 几何：点位在被紧固件顶面上；下方有承接件；接近通道畅通；螺钉长度够
    own = _shapes_of(parts[st["fastens"]], with_components=False)
    into = _shapes_of(parts[st["into"]], with_components=False)
    env = [s for pid in placed for s in _shapes_of(parts[pid])]
    corridor = APPROACH_MM
    L = screw_length_mm(st["screw"])
    for j, pt in enumerate(st["points"]):
        pp = f"{path}.points[{j}]"
        x, y, z = pt["xyz"]
        under = [s for s in own if _contains_xy(s, (x, y))]
        if not under:
            F.add("error", "E-SURFACE", pp, f"{pt['id']} 不在零件 {st['fastens']} 的投影范围内", "检查点位坐标或零件外形")
            continue
        top = max(s["z"][1] for s in under)
        if abs(top - z) > SURFACE_TOL_MM:
            F.add("error", "E-SURFACE", pp, f"{pt['id']} 的 z={z} 不在 {st['fastens']} 顶面（该处顶面 z={round(top, 3)}）",
                  "点位应取螺钉头落座的表面")
            continue
        bottom = min(s["z"][0] for s in under)
        if not any(_contains_xy(s, (x, y)) and abs(s["z"][1] - bottom) <= SURFACE_TOL_MM for s in into):
            F.add("error", "E-INTO", pp, f"{pt['id']} 下方没有承接件 {st['into']}（{st['fastens']} 底面 z={round(bottom, 3)} 处）",
                  "检查 into 零件或外形")
        if L is not None and not is_missing(hole["depth_mm"]) and L + 1e-9 < (z - bottom) + hole["depth_mm"]:
            F.add("warning", "W-STACK", pp, f"{pt['id']}：螺钉长 {L} mm < 被紧固件厚 {round(z - bottom, 3)} + 拧入深度 {hole['depth_mm']} mm",
                  "核对螺钉长度、零件厚度（是否有沉孔）或拧入深度")
        for s in env:
            if s["z"][1] <= z + EPS or s["z"][0] >= z + corridor:
                continue
            d = _xy_distance(s, (x, y))
            if d <= 1e-9:
                F.add("error", "E-BLOCKED", pp, f"{pt['id']} 正上方 {corridor} mm 内被 {s.get('component') or _owner(s, placed, parts)} 挡住，批头下不去",
                      "调整放件顺序（先拧后放）或孔位")
            elif d < clear_mm:
                F.add("review", "DFM-01", pp, f"{pt['id']} 批头间隙不足：距 {s.get('component') or _owner(s, placed, parts)} 外缘 {round(d, 2)} mm，下限 {clear_mm} mm")


def _owner(shape, placed, parts) -> str:
    for pid in placed:
        if any(_shape_to_product(s, parts[pid]["pose"]) == {k: v for k, v in shape.items() if k != "component"}
               for s in parts[pid]["geometry"]["shapes"]):
            return pid
    return "?"


def _result(F: Findings) -> dict:
    errors, blockers = F.of("error"), F.of("blocker")
    return {"buildable": not errors, "signable": not errors and not blockers,
            "counts": {lv: len(F.of(lv)) for lv in ("error", "blocker", "review", "warning")}, "findings": F.items}


# ---------- 转换 ----------
def _mount(sop) -> Pose:
    m = sop["frame"]["mounting_in_fixture"]
    return Pose.from_xyz_rpy(m["xyz"], (0.0, 0.0, m["rpy"][2]))


def to_product(sop: dict) -> dict:
    """SOP → 流水线现有产品数据（点位换到治具坐标系）。"""
    M = _mount(sop)
    fx = sop["frame"]["mounting_in_fixture"]["fixture"]
    fasten = [s for s in sop["steps"] if s["type"] == "fasten"]
    r3 = lambda v: [round(a, 3) for a in v]  # noqa: E731
    groups, points, ctq_groups, ctq_points = [], [], {}, {}
    for st in fasten:
        g = {"id": st["group"], "name": st["name"], "screw": st["screw"], "material": st["hole"]["material"]}
        if "spec" in st["hole"]:
            g["hole"] = st["hole"]["spec"]
        if "d_mm" in st["hole"]:
            g["hole_d_mm"] = st["hole"]["d_mm"]
        g["depth_mm"] = st["hole"]["depth_mm"]
        g["part"] = st["fastens"]
        groups.append(g)
        for c in st.get("ctq", []) or []:
            ctq_groups.setdefault(c, []).append(st["group"])
        for pt in st["points"]:
            points.append({"id": pt["id"], "group": st["group"], "xyz": r3(M.apply(pt["xyz"]))})
            for c in pt.get("ctq", []) or []:
                ctq_points.setdefault(c, []).append(pt["id"])
    obstacles = []
    for p in sop["parts"]:
        for c in p.get("components", []) or []:
            s = _shape_to_product(c["shape"], p["pose"])
            xy = M.apply([s["xy"][0], s["xy"][1], 0.0])
            obstacles.append({"id": c["id"], "name": c["name"], "xy": r3(xy[:2]), "z_top": round(s["z"][1] + M.t[2], 3)})
    count = {}
    for st in fasten:
        count[st["screw"]] = count.get(st["screw"], 0) + len(st["points"])
    bom_parts = [{k: p[k] for k in ("id", "name", "material", "note") if k in p} for p in sop["parts"]]
    bom_parts += [{"id": s.get("bom_id", s["id"]), "name": s.get("name", s["id"]), "qty": count.get(s["id"], 0)} for s in sop["screws"]]
    ctq = []
    for c in sop.get("ctq", []) or []:
        row = {"id": c["id"], "name": c["name"]}
        if c["id"] in ctq_groups:
            row["groups"] = ctq_groups[c["id"]]
        if c["id"] in ctq_points:
            row["points"] = ctq_points[c["id"]]
        ctq.append(row)
    tt = sop.get("takt_target_s", {}).get("value", MISSING)
    flag = {"example_data": True} if sop.get("example_data") else {}
    return {
        "name": sop["meta"]["product"],
        "bom": {**flag, "product": sop["meta"]["product"], "name": sop["meta"]["name"], "parts": bom_parts,
                "assembly_order": [st["group"] for st in fasten], "target_takt_s": tt},
        "fastening": {**flag, "frame": fx, "units": "mm", "groups": groups, "points": points, "obstacles": obstacles},
        "ctq": ctq,
    }


def to_station_policy(sop: dict) -> dict:
    """每个拧紧组的不合格策略与执行器复拧（对应 jsdemo.flow 的 NG_POLICY / EXECUTOR_RETRY）。"""
    out = {}
    for st in sop["steps"]:
        if st["type"] != "fasten":
            continue
        row = {"on_ng": NG_ACTIONS[st["on_ng"]["action"]]}
        if st["on_ng"].get("retry"):
            r = st["on_ng"]["retry"]
            row["executor_retry"] = {"max": r["max"], "on_modes": list(r["on_modes"])}
            row["decision"] = r.get("decision")
        out[st["group"]] = row
    return out


def to_assembly(sop: dict) -> dict:
    """零件外形（治具坐标系）与工步序列，供孪生按 SOP 安装（M3.5b）。"""
    M = _mount(sop)
    myaw = sop["frame"]["mounting_in_fixture"]["rpy"][2]
    parts = []
    for p in sop["parts"]:
        pose = {"xyz": [round(v, 4) for v in M.apply(p["pose"]["xyz"])], "rpy": [0.0, 0.0, round(p["pose"]["rpy"][2] + myaw, 6)]}
        parts.append({"id": p["id"], "name": p["name"], "material": p["material"], "pose_in_fixture": pose,
                      "shapes": p["geometry"]["shapes"], "components": p.get("components", []) or [],
                      "geometry_source": p["geometry"]["source"]})
    steps = []
    for st in sop["steps"]:
        row = {"id": st["id"], "type": st["type"], "actor": st["actor"]}
        if st["type"] == "place":
            row.update(part=st["part"], duration_s=st["duration_s"])
        elif st["type"] == "fasten":
            row.update(group=st["group"], fastens=st["fastens"], into=st["into"], screw=st["screw"], points=[pt["id"] for pt in st["points"]])
        steps.append(row)
    return {"product": sop["meta"]["product"], "fixture": sop["frame"]["mounting_in_fixture"]["fixture"], "parts": parts, "steps": steps}


def run(path: Path, out_root: Path | None = None) -> dict:
    sop = load_yaml(path)
    res = validate(sop)
    product = (sop.get("meta") or {}).get("product", Path(path).stem)
    out = (out_root or ROOT / "build" / "sop") / product
    out.mkdir(parents=True, exist_ok=True)
    res = {"sop": str(path), **res}
    (out / "validation.json").write_text(json.dumps(res, ensure_ascii=False, indent=2), encoding="utf-8")
    if res["buildable"]:
        prod = to_product(sop)
        dump_yaml(prod["bom"], out / "product" / "bom.yaml")
        dump_yaml(prod["fastening"], out / "product" / "fastening_points.yaml")
        dump_yaml({"example_data": True, "ctq": prod["ctq"]} if sop.get("example_data") else {"ctq": prod["ctq"]}, out / "product" / "ctq.yaml")
        dump_yaml(to_station_policy(sop), out / "station_policy.yaml")
        dump_yaml(to_assembly(sop), out / "assembly.yaml")
    res["out"] = str(out)
    return res


def _main():
    ap = argparse.ArgumentParser(prog="jsdemo.sop", description="校验工站 SOP 并转换到 build/sop/<product>/")
    ap.add_argument("sop")
    a = ap.parse_args()
    res = run(Path(a.sop))
    for f in res["findings"]:
        print(f"[{f['level']:<7}] {f['code']:<14} {f['path']}: {f['message']}" + (f"  → {f['fix']}" if f.get("fix") else ""))
    c = res["counts"]
    print(f"\nerror {c['error']}  blocker {c['blocker']}  review {c['review']}  warning {c['warning']}  "
          f"可搭建={'是' if res['buildable'] else '否'}  可签字={'是' if res['signable'] else '否'}  输出 {res['out']}")
    sys.exit(0 if res["buildable"] else 1)


if __name__ == "__main__":
    _main()
