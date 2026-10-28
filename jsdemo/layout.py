"""工站布局：从工站设计（jsdemo.station/1，上游方案 agent 输出）读出，流水线生成程序与孪生搭场景共用这一份。

代码里不写布局数值（用户 2026-10-26：布局坐标由上游 agent 给，工具链只校验与生成）。模块外形取模块库。
用哪份工站设计：use(path) 显式指定（流水线 --station）；否则取环境变量 JSDEMO_STATION_SPEC；都没有时用示例
kb/station/joint-v1.station.yaml（由 demo 工站反写，demo 占位，不是现场实测布局）。

读法：layout.current()["TRAY_STOP"]，或 layout.TRAY_STOP（模块属性按当前工站设计动态取值）。
不要 `from jsdemo.layout import TRAY_STOP`：那样取到的是 import 时的值，换工站设计后不会跟着变。
"""
from __future__ import annotations

import os
from pathlib import Path

from .io import ROOT, load_modules, load_yaml

DEFAULT_SPEC = ROOT / "kb" / "station" / "joint-v1.station.yaml"

# 取钉接近动作（运动参数，不是布局）
FEED_CLEAR_MM = 25.0     # 取钉：出钉位上方这么高处为接近点
FEED_NEAR_MM = 3.0       # 快速降到这里，最后一段慢速

_cur: dict | None = None


def _f(v):
    return [float(x) for x in v] if isinstance(v, (list, tuple)) else float(v)


def _pose(xyz, yaw=0.0) -> dict:
    return {"xyz_mm": _f(xyz), "yaw_deg": float(yaw)}


def from_spec(spec: dict, modules: dict | None = None, path: str | None = None) -> dict:
    """工站设计 → 工具链用的布局数据。模块外形（供钉器尺寸、出钉位）取模块库。"""
    modules = modules or load_modules()
    by_role: dict[str, list[dict]] = {}
    for m in spec["modules"]:
        by_role.setdefault(m["role"], []).append(m)
    one = lambda role: (by_role.get(role) or [None])[0]   # noqa: E731
    fx = spec["fixture"]
    L = {"SPEC_PATH": path, "STATION": spec["meta"]["station"],
         "FIXTURE": {"id": fx.get("use") or (fx.get("suggestion") or {}).get("id"),
                     **_pose(fx["pose"]["xyz"], fx["pose"].get("yaw", 0.0))}}
    fr = spec["frame"]
    L["FLOOR_Z"] = float(fr["floor_z"])
    L["ENVELOPE"] = {"x": _f(fr["envelope"]["x"]), "y": _f(fr["envelope"]["y"]), "z_max": float(fr["envelope"]["z_max"])}
    fa = one("fastening_arm")
    if fa:
        L["RISER_MM"] = float(fa.get("riser_mm", 0.0))
        L["HOME_J"], L["PARK_J"] = _f(fa["home_joints_deg"]), _f(fa.get("park_joints_deg") or fa["home_joints_deg"])
    h = one("handling_arm")
    if h:
        L["HANDLER_MOUNT"] = _pose(h["pose"]["xyz"], h["pose"].get("yaw", 0.0))
        L["H_HOME_J"] = _f(h["home_joints_deg"])
        L["H_RISER_MM"] = float(h.get("riser_mm", 0.0))
    cv = one("conveyor")
    if cv:
        st = cv.get("stops", {})
        L["CONVEYOR"] = {"x": float(cv["path"]["x"]), "y": _f(cv["path"]["y"]), "width": float(cv["path"]["width"]),
                         "top_z": float(cv["path"]["top_z"])}
        for key in ("tray_stop", "out_stop", "tray_prestop", "tray_station"):
            if key in st:
                L[key.upper()] = _pose(st[key])
        L["BUFFER"] = [_f(b) for b in st.get("buffer", [])]
    for tc in by_role.get("tool_changer", []):
        slot_xy = {s["slot"]: s["xy"] for s in tc.get("slots", [])}
        L["TOOL_DOCKS"] = {g["module"]: _f(slot_xy[g["dock"]["slot"]]) for g in by_role.get("gripper", [])
                           if g.get("dock", {}).get("changer") == tc["id"]}
        L["TOOL_VIA"] = _f(tc["via"]) if tc.get("via") else None
    fd = one("feeder")
    if fd:
        geo = modules[fd["module"]]["geometry"]
        L["FEEDER"] = {"xyz_mm": _f(fd["pose"]["xyz"]), "yaw_deg": float(fd["pose"].get("yaw", 0.0)),
                       "size_mm": _f(geo["size_mm"]), "lane_pitch_mm": float(geo["lane_pitch_mm"]),
                       "outlet_len_mm": float(geo["outlet_len_mm"]), "nest_size_mm": _f(geo["nest_size_mm"]),
                       "lanes": {ln["lane"]: ln["screw"] for ln in fd.get("lanes", [])}, "source": geo.get("source")}
    # 其余落地模块（基板、控制器、接口盒、网关）：只给位置，孪生按它摆外形
    L["PLACED"] = {m["role"]: _f(m["pose"]["xyz"]) for m in spec["modules"]
                   if m["role"] in ("baseplate", "controller", "interface_box", "gateway", "vision") and "pose" in m}
    return L


def use(path: str | Path | None = None) -> dict:
    """切换当前工站设计。返回布局数据。"""
    global _cur
    p = Path(path or os.environ.get("JSDEMO_STATION_SPEC") or DEFAULT_SPEC)
    if not p.is_absolute():
        p = (ROOT / p) if (ROOT / p).exists() else p.resolve()
    _cur = from_spec(load_yaml(p), path=str(p.relative_to(ROOT) if p.is_relative_to(ROOT) else p).replace("\\", "/"))
    return _cur


def current() -> dict:
    return _cur if _cur is not None else use()


def __getattr__(name: str):
    if name.isupper():
        L = current()
        if name in L:
            return L[name]
    raise AttributeError(f"module 'jsdemo.layout' has no attribute {name!r}")


def lane_x(i: int, lanes: int) -> float:
    return (i - (lanes - 1) / 2) * current()["FEEDER"]["lane_pitch_mm"]


def nest_top(i: int, lanes: int) -> list[float]:
    """第 i 路（从 0 起）出钉位顶面中心，工站坐标系（v1 供钉器只支持 yaw 0）。"""
    F = current()["FEEDER"]
    x, y, z = F["xyz_mm"]
    sx, sy, sz = F["size_mm"]
    return [x + lane_x(i, lanes), y - sy / 2 - F["outlet_len_mm"] - F["nest_size_mm"][1] / 2, z + sz / 2]


def feed_pick_tcp(i: int, lanes: int, screw_len_mm: float) -> list[float]:
    """取钉时的 TCP：拿着螺钉时 TCP = 螺钉尖（与 engage 位约定一致），螺钉立在出钉位里，尖端在顶面下一个杆长。"""
    x, y, z = nest_top(i, lanes)
    return [x, y, z - screw_len_mm]
