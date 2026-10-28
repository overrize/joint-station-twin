"""从模块库组合工站。只组合库里已有的模块，不做非标设计；库里没有就明确说没有。"""
from __future__ import annotations

from .geometry import Pose, norm
from .io import groups_by_id


class CompositionError(Exception):
    """库内没有满足条件的模块。这时应该停下来问人，而不是发明一个模块。"""


def fixture_pose(modules: dict, fixture_id: str) -> Pose:
    """治具在工站里的位姿：工站设计 fixture.pose 给了就用它（布局由上游 agent 定），否则取治具模块的名义位姿。"""
    from . import layout
    fx = layout.current().get("FIXTURE")
    nom = fx if fx and fx["id"] == fixture_id else modules[fixture_id]["nominal_pose_in_base"]
    return Pose.from_xyz_rpy(nom["xyz_mm"], (0.0, 0.0, nom["yaw_deg"]))


def _of(modules, typ):
    return [m for m in modules.values() if m["type"] == typ]


def compose(product: dict, modules: dict, windows: dict, screws: dict, fixture_id: str | None = None, place_steps: int = 0) -> dict:
    """windows: 组 id -> 工艺模型输出。fixture_id：工站 SOP 里声明的治具（frame.mounting_in_fixture.fixture）。
    place_steps：归放件臂的放件工步数（jsdemo.handling），> 0 时加放件臂、吸盘、料盘（M4 ②）。"""
    decisions = []
    pts = product["fastening"]["points"]
    groups = groups_by_id(product)

    if fixture_id:
        if fixture_id not in modules or modules[fixture_id]["type"] != "fixture":
            raise CompositionError(f"SOP 声明的治具 {fixture_id} 不在模块库里")
        fixture = modules[fixture_id]
        if fixture.get("product") != product["name"]:
            decisions.append({
                "id": "DEC-004",
                "decision": f"{product['name']} 借用治具 {fixture_id}（该治具为 {fixture.get('product')} 设计）",
                "rejected": "新设计专用治具：模块库里没有，按规则不发明模块",
                "assumptions": "缺失（借用治具的定位、夹持是否适配该产品未验证）",
                "invalidation": "定位或夹持验证不通过；为该产品设计专用治具后",
                "trigger": "偏离默认",
            })
    else:
        fixtures = [m for m in _of(modules, "fixture") if m.get("product") == product["name"]]
        if not fixtures:
            raise CompositionError(f"模块库里没有 {product['name']} 的定位治具")
        fixture = fixtures[0]
    F = fixture_pose(modules, fixture["id"])

    # 执行器：扭矩范围覆盖所有窗口，上限留 10% 余量
    need_lo = min(w["window_nm"][0] for w in windows.values())
    need_hi = max(w["window_nm"][1] for w in windows.values())
    execs = [m for m in _of(modules, "executor")
             if m["torque_range_nm"][0] <= need_lo and need_hi <= 0.9 * m["torque_range_nm"][1]]
    if not execs:
        raise CompositionError(f"没有执行器覆盖 {need_lo}–{need_hi} N·m")
    executor = execs[0]

    # 机械臂：所有点位在可达范围内，负载够
    dists = [norm(F.apply(p["xyz"])) for p in pts]
    arms = sorted([m for m in _of(modules, "arm")
                   if m["min_reach_mm"] <= min(dists) and max(dists) <= m["reach_mm"] and m["payload_kg"] >= executor["mass_kg"]],
                  key=lambda m: m["reach_mm"])
    if not arms:
        raise CompositionError("没有机械臂同时满足可达范围和负载")
    arm = arms[0]

    # 供钉：通道数不少于螺钉种类
    screw_kinds = sorted({g["screw"] for g in groups.values()})
    feeders = [m for m in _of(modules, "feeder") if m["lanes"] >= len(screw_kinds)]
    if not feeders:
        raise CompositionError(f"没有供钉器能同时供 {len(screw_kinds)} 种螺钉")
    feeder = feeders[0]

    # 误差预算：臂绝对精度 + 治具 + 基板，必须小于浮动批头吸收范围，否则加视觉找正
    baseplate = _of(modules, "baseplate")[0]
    float_head = _of(modules, "float_head")[0]
    budget = arm["absolute_accuracy_mm"] + fixture["placement_tolerance_mm"] + baseplate["placement_tolerance_mm"]
    use_vision = budget > float_head["capture_mm"]
    chosen = [
        ("conveyor", _of(modules, "conveyor")[0]), ("baseplate", baseplate), ("fixture", fixture),
        ("arm", arm), ("executor", executor), ("float_head", float_head), ("feeder", feeder),
        ("controller", _of(modules, "controller")[0]), ("interface_box", _of(modules, "interface_box")[0]),
        ("gateway", _of(modules, "gateway")[0]),
    ]
    if use_vision:
        chosen.append(("vision_locator", _of(modules, "vision_locator")[0]))
    if place_steps:
        # 放件臂：与拧紧臂同型号（IK 配置已在本机验证，风险最小）；吸盘与料盘取库里第一个满足条件的
        # 末端：库里的夹爪与吸盘都装上快换，由放件规划按零件外形逐件选（用户 2026-10-25）
        grippers = sorted(_of(modules, "gripper"), key=lambda m: m["principle"] != "parallel_2f")
        trays = [m for m in _of(modules, "tray") if m["slots"] >= place_steps]
        changers = [m for m in _of(modules, "tool_changer") if m["slots"] >= len(grippers)]
        if not grippers or not trays or (len(grippers) > 1 and not changers):
            raise CompositionError("模块库里没有放件用的末端、穴数够的料盘或工位够的快换架")
        chosen += [("handler_arm", arm)] + [("gripper", g) for g in grippers] + ([("tool_changer", changers[0])] if len(grippers) > 1 else []) + [("tray", trays[0])]
        decisions.append({
            "id": "DEC-005",
            "decision": f"输送线送配套料盘，第二台 {arm['name']} 放件（{place_steps} 个放件工步），末端 {'、'.join(g['name'] for g in grippers)} 经快换按零件外形选用",
            "rejected": "人工放件：节拍与一致性差；在拧紧臂上加换手：拧紧臂程序里会混进放件动作",
            "assumptions": "零件质量缺失，吸盘负载未核；布局、吸盘外形、料盘穴位为 demo 占位；两臂不同时进治具上方，按握手串行",
            "invalidation": "零件有不能吸的表面或超出吸盘负载；节拍要求两臂并行",
            "trigger": "用户提出（2026-10-25：输送线送件 + 另一台机械臂放件）",
        })
    decisions.append({
        "id": "DEC-002",
        "decision": "加视觉找正" if use_vision else "不加视觉找正",
        "rejected": "不加视觉找正：误差预算超出浮动吸收范围" if use_vision else "加视觉找正：每点多 0.4 s，误差预算已满足，没有必要",
        "assumptions": f"臂绝对精度 {arm['absolute_accuracy_mm']} mm + 治具 {fixture['placement_tolerance_mm']} mm + 基板 {baseplate['placement_tolerance_mm']} mm = {budget:.2f} mm，浮动吸收 {float_head['capture_mm']} mm",
        "invalidation": "更换机械臂型号、治具或浮动批头后重新计算误差预算",
        "trigger": "偏离默认",
    })

    return {
        "station_id": "JS-01",
        "product": product["name"],
        "fixture_frame": fixture["id"],
        "modules": [{"role": r, "id": m["id"], "name": m["name"]} for r, m in chosen],
        "checks": {
            "torque_need_nm": [need_lo, need_hi],
            "executor_range_nm": executor["torque_range_nm"],
            "reach_mm": [round(min(dists), 1), round(max(dists), 1)],
            "arm_reach_mm": [arm["min_reach_mm"], arm["reach_mm"]],
            "screw_kinds": screw_kinds,
            "feeder_lanes": feeder["lanes"],
            "error_budget_mm": round(budget, 3),
            "float_capture_mm": float_head["capture_mm"],
            "use_vision": use_vision,
        },
        "decisions": decisions,
    }
