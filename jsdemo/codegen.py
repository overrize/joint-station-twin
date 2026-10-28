"""从业务流生成机械臂程序。机械臂程序里不允许出现任何紧固逻辑，那是执行器的事。

先生成设备无关的中间指令（arm_program.json，给仿真用），再翻译成具体品牌的语言。
这里给出 URScript 生成器作为第一个目标；以后自研臂只需要新增一个生成器。
"""
from __future__ import annotations

import math

from . import layout   # 原位 / 停靠位、出钉位取当前工站设计
from .geometry import Pose

# 运动参数（URScript 与孪生共用）：movel 线速度 m/s 与加速度 m/s²；movej 关节速度 rad/s 与加速度 rad/s²
# 节拍加速（用户 2026-10-26）：快移 0.25 → 0.8 m/s、加速度 0.5 → 2 m/s²、关节 0.8 → 2 rad/s、3 rad/s²（UR5e 允许范围内，demo 值）；
# 慢速只留最后 ENGAGE_NEAR_MM
MOVEL_V = {"fast": 0.8, "slow": 0.03}
MOVEL_A = 2.0
MOVEJ_V = 2.0
MOVEJ_A = 3.0
ENGAGE_NEAR_MM = 5.0


def to_base(target: dict, F: Pose) -> dict:
    p = F.apply(target["xyz"])
    return {"xyz_mm": [round(v, 3) for v in p], "rpy": target["rpy"], "yaw_offset_deg": round(F.yaw_deg(), 4)}


def feed_points(flow: dict, screw_kinds: list[str], screw_of: dict, lanes: int) -> dict:
    """每个点位去哪一路出钉位取钉（M4：用户 2026-10-25 定「每颗都去供钉器出钉口取」）。返回 {点位: 取钉 TCP（基座系）}。
    第 i 路 = screw_kinds 第 i 种（模块库 lane_assignment）。"""

    out = {}
    for s in flow["sequence"]:
        sid = screw_of[s["point"]]
        out[s["point"]] = {"lane": screw_kinds.index(sid) + 1,
                           "pick": [round(v, 3) for v in layout.feed_pick_tcp(screw_kinds.index(sid), lanes, float(sid.split("-")[0].split("x")[1]))]}
    return out


def neutral(flow: dict, F: Pose, handoff: dict | None = None, park_j: list | None = None, feed: dict | None = None,
            window: list | None = None) -> list[dict]:
    """handoff：{点位: 放件臂须已放完的件数}（jsdemo.handling.plan）。在这些点之前退到停靠位、给「区域空闲」、
    等放件计数，再收回区域空闲继续。拧紧臂自己不放件。
    feed：{点位: {lane, pick}}（feed_points）。每个点先去该路出钉位取钉：接近 → 快降 → 慢降 → 等取钉确认 → 抬起。
    window：这些握手点之前放件臂中途要换工具（handling.plan 的 tool_change_window_before_point）。放件臂到区域外给「已离开」，
    拧紧臂趁这段收回区域空闲、去取该点的螺钉、回停靠位再交出区域，然后才等放件计数（用户 2026-10-26：不干等换手）。"""

    ops = [{"op": "movej", "joints_deg": layout.HOME_J}]
    for s in flow["sequence"]:
        feed_ops = []
        if feed:
            fp = feed[s["point"]]
            x, y, z = fp["pick"]

            def at(dz, x=x, y=y, z=z):
                return {"xyz_mm": [x, y, round(z + dz, 3)], "rpy": [180, 0, 0], "yaw_offset_deg": round(F.yaw_deg(), 4)}

            feed_ops = [{"op": "movel", "point": s["point"], "pose": at(layout.FEED_CLEAR_MM), "speed": "fast", "purpose": "feed", "lane": fp["lane"]},
                    {"op": "movel", "point": s["point"], "pose": at(layout.FEED_NEAR_MM), "speed": "fast", "purpose": "feed", "lane": fp["lane"]},
                    {"op": "movel", "point": s["point"], "pose": at(0.0), "speed": "slow", "purpose": "feed", "lane": fp["lane"]},
                    {"op": "wait", "until": "feed_ok", "point": s["point"], "lane": fp["lane"]},
                    {"op": "movel", "point": s["point"], "pose": at(layout.FEED_CLEAR_MM), "speed": "fast", "purpose": "feed", "lane": fp["lane"]}]
        if handoff and s["point"] in handoff:
            # 取钉要先收回区域空闲：供钉器在治具远侧，拧紧臂伸过去取钉时小臂横在治具上方。
            # 试过放件臂还在治具上时先取钉，两臂小臂相撞（孪生实测，2026-10-26），所以只在放件臂离开去换手时取
            park = {"op": "movej", "joints_deg": park_j or layout.HOME_J, "purpose": "park"}
            ops += [park, {"op": "signal", "name": "zone_free", "value": True}]
            if window and s["point"] in window and feed_ops:
                ops += [{"op": "wait", "until": "zone_left"}, {"op": "signal", "name": "zone_free", "value": False}]
                ops += feed_ops + [dict(park, purpose="park", prefetch=s["point"]), {"op": "signal", "name": "zone_free", "value": True}]
                feed_ops = []
            ops += [{"op": "wait", "until": "placed_count", "value": handoff[s["point"]]},
                    {"op": "signal", "name": "zone_free", "value": False}]
        ops += feed_ops
        ops += [
            {"op": "movel", "point": s["point"], "pose": to_base(s["approach"], F), "speed": "fast"},
            # 快速降到孔位上方 ENGAGE_NEAR_MM，最后一段慢速下压（之前接近位 30 mm 起就慢速，每点 1 s）
            {"op": "movel", "point": s["point"], "pose": to_base({**s["engage"], "xyz": [s["engage"]["xyz"][0], s["engage"]["xyz"][1],
                                                                                    s["engage"]["xyz"][2] + ENGAGE_NEAR_MM]}, F),
             "speed": "fast", "purpose": "near"},
            {"op": "movel", "point": s["point"], "pose": to_base(s["engage"], F), "speed": "slow"},
            {"op": "hold", "point": s["point"]},
            {"op": "signal", "name": "ready", "value": True},
            {"op": "wait", "until": "done_or_fault"},
            {"op": "signal", "name": "ready", "value": False},
            # 整件停止：先退回接近位、回原位再停（不停在下压位，批头不留在螺钉上）
            {"op": "branch_on_fault", "point": s["point"], "policy": s["on_ng"],
             "on_stop": [{"op": "movel", "point": s["point"], "pose": to_base(s["approach"], F), "speed": "fast"},
                         {"op": "movej", "joints_deg": layout.HOME_J}] if "stop_unit" in s["on_ng"] else []},
            {"op": "movel", "point": s["point"], "pose": to_base(s["approach"], F), "speed": "fast"},
        ]
    ops.append({"op": "movej", "joints_deg": layout.HOME_J})
    if handoff:   # 有放件臂（M4 ③）：整件拧完退到停靠位，给「整件完成」，放件臂来取走整件
        ops += [{"op": "movej", "joints_deg": park_j or layout.HOME_J, "purpose": "park"},
                {"op": "signal", "name": "unit_done", "value": True}]
    return ops


def _rv(rpy):
    return [math.pi, 0.0, 0.0] if list(rpy) == [180, 0, 0] else [math.radians(a) for a in rpy]


def _motion(o: dict, ind: str = "  ") -> str:
    if o["op"] == "movej":
        j = ", ".join(f"d2r({a})" for a in o["joints_deg"])
        return f"{ind}movej([{j}], a={MOVEJ_A}, v={MOVEJ_V})"
    x, y, z = (v / 1000.0 for v in o["pose"]["xyz_mm"])
    r = _rv(o["pose"]["rpy"])
    return f"{ind}movel(p[{x:.5f}, {y:.5f}, {z:.5f}, {r[0]:.5f}, {r[1]:.5f}, {r[2]:.5f}], a={MOVEL_A}, v={MOVEL_V[o['speed']]})  # {o.get('point') or o.get('part')}"


def urscript(ops: list[dict]) -> str:
    """URScript 生成器。tier0：DO0=就绪，DI0=完成，DI1=异常，DI4=取钉确认（批头真空）；与放件臂：DO1=区域空闲，DO2=整件完成，
    DI5=放件臂已离开治具区域（换手），整数寄存器 0=放件计数。"""
    L = ["def station_js01():", "  # generated from station/flow.yaml; arm handles positioning and holding only"]
    for o in ops:
        if o["op"] in ("movej", "movel"):
            L.append(_motion(o))
        elif o["op"] == "hold":
            L.append(f"  stopl(1.0)  # {o['point']} hold position")
        elif o["op"] == "signal" and o["name"] == "unit_done":
            L.append("  set_standard_digital_out(2, True)  # unit done: handler arm unloads")
        elif o["op"] == "signal" and o["name"] == "zone_free":
            L.append(f"  set_standard_digital_out(1, {'True' if o['value'] else 'False'})  # zone free for handler arm")
        elif o["op"] == "signal":
            L.append(f"  set_standard_digital_out(0, {'True' if o['value'] else 'False'})")
        elif o["op"] == "wait" and o["until"] == "feed_ok":
            L.append(f"  while not get_standard_digital_in(4):  # {o['point']} feed nest lane {o['lane']}: bit vacuum ok")
            L.append("    sync()")
            L.append("  end")
        elif o["op"] == "wait" and o["until"] == "zone_left":
            L.append("  while not get_standard_digital_in(5):  # wait handler arm left the fixture zone (tool change): visit feed nest meanwhile")
            L.append("    sync()")
            L.append("  end")
        elif o["op"] == "wait" and o["until"] == "placed_count":
            L.append(f"  while read_input_integer_register(0) < {o['value']}:  # wait handler arm: {o['value']} parts placed")
            L.append("    sync()")
            L.append("  end")
        elif o["op"] == "wait":
            L.append("  while not (get_standard_digital_in(0) or get_standard_digital_in(1)):")
            L.append("    sync()")
            L.append("  end")
        elif o["op"] == "branch_on_fault":
            if "stop_unit" in o["policy"]:
                L.append(f"  if get_standard_digital_in(1):  # {o['point']} fault -> retract, home, stop unit")
                L += [_motion(m, "    ") for m in o.get("on_stop", [])]
                L.append("    halt")
                L.append("  end")
    L.append("end")
    L.append("station_js01()")
    return "\n".join(L) + "\n"


def urscript_handler(ops: list[dict]) -> str:
    """放件臂 URScript。DI3=料盘到位，DI2=区域空闲（来自拧紧臂），DI5=整件完成，DO0=已离开治具区域（换手），工具 DO0=吸盘真空，
    整数寄存器 0=放件计数。"""
    L = ["def station_js01_handler():", "  # generated from assembly steps; handler arm picks parts from tray and places them on the fixture"]
    for o in ops:
        if o["op"] in ("movej", "movel"):
            L.append(_motion(o))
        elif o["op"] == "grip":
            L.append(f"  set_tool_digital_out(0, {'True' if o['value'] else 'False'})  # vacuum {'on' if o['value'] else 'off'}: {o['part']}")
            L.append("  sleep(0.3)")
        elif o["op"] == "wait":
            di = {"tray_ready": 3, "zone_free": 2, "unit_done": 5}[o["until"]]
            L.append(f"  while not get_standard_digital_in({di}):  # wait {o['until']}")
            L.append("    sync()")
            L.append("  end")
        elif o["op"] == "signal" and o["name"] == "zone_left":
            L.append(f"  set_standard_digital_out(0, {'True' if o['value'] else 'False'})  # {'left' if o['value'] else 're-entering'} fixture zone")
        elif o["op"] == "signal" and o["name"] == "unloaded":
            L.append("  write_output_integer_register(1, 1)  # unit unloaded to downstream pallet")
        elif o["op"] == "signal":
            L.append(f"  write_output_integer_register(0, {o['value']})  # placed count")
    L.append("end")
    L.append("station_js01_handler()")
    return "\n".join(L) + "\n"
