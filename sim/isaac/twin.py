"""数字孪生运行核心（M3.2 实现，M3.3 抽出供 run_flow.py 与 calibrate_demo.py 共用）。

必须在 SimulationApp 创建之后再 import 本模块。

原则：
  - 机械臂只按 arm_program 的 movel/movej 运动，到位后保持；运动参数与 URScript 共用 jsdemo.codegen 的常量；
  - 'hold' 期间停留 executor.fasten_s_base 秒的仿真时间，然后读取对应曲线做判定；
  - 判定用 jsdemo.procmodel.judge，不在这里重写判定逻辑；
  - 发现不可达或碰撞，记录并继续下一个点，最后整体判失败，不要自动修改业务流；
  - 节拍实测与估算偏差超过 20% 只记录（takt.flag），不改模块库。
IK 用 Lula UR5e（DEC-ARM-001）；TCP = tool0 沿 z 偏移执行器长度；实测 TCP 取物理引擎里 wrist_3_link 的位姿（与 tool0 重合）。
"""
from __future__ import annotations

import json
import os
import math
import random
from datetime import datetime

import carb
import numpy as np
import omni.kit.app

import isaacsim.core.experimental.utils.app as app_utils
import isaacsim.core.experimental.utils.stage as stage_utils
from isaacsim.core.experimental.materials import PreviewSurfaceMaterial
from isaacsim.core.experimental.prims import Articulation, GeomPrim, RigidPrim, XformPrim
from isaacsim.core.rendering_manager import RenderingManager
from isaacsim.core.rendering_manager import ViewportManager
from isaacsim.core.simulation_manager import SimulationManager
from omni.kit.viewport.utility import capture_viewport_to_file, get_active_viewport
from pxr import UsdGeom

omni.kit.app.get_app().get_extension_manager().set_extension_enabled_immediate(
    "isaacsim.robot_motion.motion_generation", True
)
from isaacsim.robot_motion.motion_generation import LulaKinematicsSolver, interface_config_loader  # noqa: E402

from _common import (SCENE, STATION_DIR, TWIN_DIR, USD_PATH, load_assembly, load_handler_ops, load_handling,  # noqa: E402
                     load_station_product, prim_name, screw_length_mm)
from jsdemo import composer, procmodel  # noqa: E402
from jsdemo.codegen import MOVEJ_A, MOVEJ_V, MOVEL_A, MOVEL_V  # noqa: E402
from jsdemo.geometry import Pose, calibrate_fixture  # noqa: E402
from jsdemo.io import groups_by_id, load_library, load_yaml  # noqa: E402

DT = 1.0 / 60.0
ARRIVE_TOL_MM = 0.1        # 孪生的到位判据（仿真验收用，不是工艺参数）
FEED_TOL_MM = 0.3          # 取钉位到位判据（出钉位有导向、批头浮动，demo）：供钉器在臂展 660–710 mm 处，仿真关节驱动静差约 0.125 mm
JOINT_V_FRAC = 0.9          # 规划时关节速度不超过资产上限的这个比例
SETTLE_MAX_S = 2.0         # 轨迹结束后最多等多久到位
CONTACT_N = 0.01           # 接触力超过它记为碰撞
TAKT_FLAG_PCT = 20.0       # docs/03-plan.md M3.2
MOVEL_KNOT_MM = 5.0        # 直线运动逆解节点间距
REFINE_MAX_RAD = math.radians(1.0)  # 细化解相对粗解的最大关节变化
IK_ORI_TOL_RAD = 3e-5      # × 0.15 m TCP ≈ 0.005 mm
IK_POS_TOL_M = 1e-6
ROOT = "/World/" + prim_name("JS-01")
FAULT_MODES = ["slip", "float"]   # 只注入 procmodel.synth_curve 能生成的异常
UR_LINKS = ["shoulder_link", "upper_arm_link", "forearm_link", "wrist_1_link", "wrist_2_link", "wrist_3_link"]
G2F_LINKS = ["base_link", "left_outer_knuckle", "left_outer_finger", "left_inner_finger", "left_inner_knuckle",
             "right_outer_knuckle", "right_outer_finger", "right_inner_finger", "right_inner_knuckle"]   # 本机核实的 2F-85 刚体
FINGER_STALL_DEG = 0.02    # 手指每步变化小于它、连续 8 步，视为堵转（接触）
FINGER_MAX_S = 1.5


# ---------- 运动学小工具 ----------
def trapezoid(d: float, v: float, a: float) -> float:
    """梯形速度曲线走完距离 d 的时间。"""
    if d <= 0:
        return 0.0
    return d / v + v / a if d >= v * v / a else 2.0 * math.sqrt(d / a)


def trapezoid_s(t: float, d: float, v: float, a: float) -> float:
    """梯形曲线在时刻 t 走过的距离。"""
    T = trapezoid(d, v, a)
    if t >= T:
        return d
    if d < v * v / a:  # 三角形
        if t <= T / 2:
            return 0.5 * a * t * t
        return d - 0.5 * a * (T - t) ** 2
    ta = v / a
    if t <= ta:
        return 0.5 * a * t * t
    if t <= T - ta:
        return 0.5 * a * ta * ta + v * (t - ta)
    return d - 0.5 * a * (T - t) ** 2


def rot_to_quat(R) -> np.ndarray:
    R = np.asarray(R, dtype=float)
    w = math.sqrt(max(0.0, 1.0 + R[0, 0] + R[1, 1] + R[2, 2])) / 2.0
    x = math.copysign(math.sqrt(max(0.0, 1.0 + R[0, 0] - R[1, 1] - R[2, 2])) / 2.0, R[2, 1] - R[1, 2])
    y = math.copysign(math.sqrt(max(0.0, 1.0 - R[0, 0] + R[1, 1] - R[2, 2])) / 2.0, R[0, 2] - R[2, 0])
    z = math.copysign(math.sqrt(max(0.0, 1.0 - R[0, 0] - R[1, 1] + R[2, 2])) / 2.0, R[1, 0] - R[0, 1])
    q = np.array([w, x, y, z])
    return q / np.linalg.norm(q)


def quat_to_rot(q) -> np.ndarray:
    w, x, y, z = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


def same_branch(qa, qb) -> bool:
    """UR 构型分支：肘（elbow 符号）与腕（wrist_2 的 sin 符号）不变，且各关节变化不超过半圈。"""
    qa, qb = np.asarray(qa), np.asarray(qb)
    return (np.sign(qa[2]) == np.sign(qb[2]) and np.sign(math.sin(qa[4])) == np.sign(math.sin(qb[4]))
            and float(np.max(np.abs(qb - qa))) < math.pi)


def pose_rot(pose: dict) -> np.ndarray:
    """arm_program 的姿态：先 rpy，再绕基座 z 转 yaw_offset（治具偏航），与 codegen.to_base 一致。"""
    R = Pose.from_xyz_rpy([0, 0, 0], pose["rpy"]).R
    Rz = Pose.from_xyz_rpy([0, 0, 0], [0, 0, pose.get("yaw_offset_deg", 0.0)]).R
    return np.asarray(Rz) @ np.asarray(R)


def all_arrived_exec(arrived: dict, results: list) -> bool:
    """已执行的点位全部到位（整件停止时只看停止前执行过的点）。"""
    done = {r["point"] for r in results}
    return all(a["arrived"] for p, a in arrived.items() if p in done) and done <= set(arrived)


def pose_mat(R, t) -> np.ndarray:
    M = np.eye(4)
    M[:3, :3], M[:3, 3] = np.asarray(R, dtype=float), np.asarray(t, dtype=float)
    return M


class Rig:
    """一台机械臂：关节、连杆接触、Lula 逆解、TCP 偏移、当前关节指令。Lula 的解算在世界坐标系里（set_robot_base_pose，本机核实）。"""

    def __init__(self, name, path, ik_cfg, tcp_m, filters, base_pos_m=None, base_quat=None):
        self.name, self.path, self.tcp_m, self.filters = name, path, tcp_m, filters
        self.arm = Articulation(path)
        self.links = RigidPrim([f"{path}/{n}" for n in UR_LINKS], contact_filter_paths=filters)
        self.links.set_enabled_contact_tracking([True])  # 给连杆挂 PhysxContactReportAPI，play 前必须做
        self.ik = LulaKinematicsSolver(**interface_config_loader.load_supported_lula_kinematics_solver_config(ik_cfg["robot"]))
        self.ik_frame = ik_cfg["frame"]
        # Lula 默认容差（位置 1 mm、姿态 0.01 rad）乘上 TCP 偏移后末端偏 0.6~0.9 mm；只收紧容差会解不出，
        # 还要收紧 BFGS 终止条件，并先用默认容差粗解再细化（M3.2 实测：39 个位姿全部解出，TCP 误差 ≤ 0.001 mm）
        self.ik.bfgs_gradient_norm_termination = 1e-12
        self.ik.bfgs_max_iterations = 500
        self.ik.ccd_descent_termination_delta = 1e-12
        if base_pos_m is not None:
            self.ik.set_robot_base_pose(np.asarray(base_pos_m, dtype=float), np.asarray(base_quat, dtype=float))
        self.cmd = None
        self.vmax = None   # 各关节最大速度 rad/s（play 之后从资产读）


class Twin:
    # 运动相关的方法都作用在「当前臂」self.rig 上（默认拧紧臂；放件时临时切到放件臂）
    arm = property(lambda s: s.rig.arm)
    links = property(lambda s: s.rig.links)
    ik = property(lambda s: s.rig.ik)
    ik_frame = property(lambda s: s.rig.ik_frame)
    tcp_m = property(lambda s: s.rig.tcp_m)
    cmd = property(lambda s: s.rig.cmd, lambda s, v: setattr(s.rig, "cmd", v))

    def __init__(self, app, station, ops, modules, *, headless: bool, sn: str, fault_rate: float = 0.0, seed: int = 0,
                 inject: dict | None = None, snap_dir=None, tag: str = "", record_dir=None, record_every: int = 2):
        self.app, self.headless, self.sn = app, headless, sn
        # 录屏（M4.1）：每 record_every 个物理步截一帧（DT=1/60 → 30 fps，视频时长 = 仿真时长），结束时编码成 mp4 + 字幕
        self.record_dir, self.record_every = record_dir, record_every
        self._nstep, self._nframe, self.captions, self.view = 0, 0, [], None
        self._closeup_frame = None   # 第一次切到孔位近景时的帧号，结束时留一张近景预览
        self.fault_rate, self.seed, self.inject, self.snap_dir, self.tag = fault_rate, seed, inject or {}, snap_dir, tag
        self.product = load_station_product(station)
        self.assembly = load_assembly(station)   # M3.5b：零件外形与工步来自工站 SOP
        self.station, self.ops, self.modules = station, ops, modules
        by_role = {m["role"]: m["id"] for m in station["modules"]}
        self.by_role = by_role
        self.arm_mod = modules[by_role["arm"]]
        self.exec_mod = modules[by_role["executor"]]
        self.feeder_mod = modules[by_role["feeder"]]
        self.t = 0.0
        self.op_index = -1
        self.collisions: list[dict] = []
        self.gateway: list[dict] = []
        self.snaps: list[str] = []
        self.recorded_groups: set[str] = set()
        self.recorded_lanes: set = set()
        self.snapped_groups: set[str] = set()
        self.pending_snap = None
        self.camera_main = None   # 有界面时截完近景回到的主机位 {"eye", "target"}
        # 治具在场景里的实际位姿（真值）；默认 = 名义位姿。孔位误差 hole_err_mm 以它为准
        self.fixture_true = composer.fixture_pose(modules, station["fixture_frame"])

        ok, _ = stage_utils.open_stage(str(USD_PATH))
        if not ok:
            raise RuntimeError(f"打不开 {USD_PATH}，先运行 build_scene.py")
        arm_path = f"{ROOT}/{prim_name(by_role['arm'])}"
        self.fixture_path = f"{ROOT}/{prim_name(station['fixture_frame'])}"
        self.product_path = f"{self.fixture_path}/{prim_name(station['product'])}"
        self.unit, self.screws_root = 1, f"{ROOT}/screws"   # 流水线（M4 ③）：第 k 台用 {产品}_u<k> 与 screws_u<k>
        env = self._env_paths()
        self.rigs = {"fastener": Rig("fastener", arm_path, self.arm_mod["isaac_ik"], SCENE["executor"]["length"] / 1000.0, env)}
        self.rig = self.rigs["fastener"]
        # 放件臂（M4 ②）：与拧紧臂同型号；接触过滤再加上拧紧臂的连杆，两臂相碰也能报出来
        self.hd = load_handling()
        self.h_ops = load_handler_ops() if self.hd else []
        if self.hd:
            hm = self.hd["handler_mount"]
            self.H = Pose.from_xyz_rpy(hm["xyz_mm"], (0.0, 0.0, hm["yaw_deg"]))
            hpath = f"{ROOT}/{prim_name(by_role['handler_arm'])}_handler"
            self.rigs["handler"] = Rig("handler", hpath, modules[by_role["handler_arm"]]["isaac_ik"], 0.0,
                                       env + [f"{arm_path}/{n}" for n in UR_LINKS],
                                       base_pos_m=[v / 1000.0 for v in hm["xyz_mm"]], base_quat=rot_to_quat(self.H.R))
            # 末端工具（M4 ①）：各自独立；接上的那个每步跟随法兰，没接的停在快换架上
            dz = modules[self.hd["tool_changer"]]["dock_z_mm"] if self.hd.get("tool_changer") else 150.0
            self.tools = {}
            for tid, xy in self.hd["tool_docks"].items():
                g = modules[tid]
                tp = f"{ROOT}/{prim_name(tid)}"
                t = {"id": tid, "mod": g, "path": tp, "kind": "2f" if g["principle"] == "parallel_2f" else "vac",
                     "dock": pose_mat(quat_to_rot([0.0, 1.0, 0.0, 0.0]), [xy[0] / 1000.0, xy[1] / 1000.0, dz / 1000.0])}
                if t["kind"] == "2f":   # 运动学夹爪：各连杆按姿态表（record_gripper.py）摆，不参与物理
                    tab = json.loads((STATION_DIR.parents[1] / "build" / "assets" / g["finger_table"]).read_text(encoding="utf-8"))
                    ap = f"{tp}/{tab['root']}"
                    t["table"], t["link_names"] = tab["table"], tab["links"]
                    t["links_x"] = XformPrim([f"{ap}/{n}" for n in tab["links"]], reset_xform_op_properties=True)
                    t["angle"] = 0.0
                self.tools[tid] = t
            self.tool_on = None   # 开机时工具都在快换架上，放件臂程序先去接
            self.grip_ignore: set[str] = set()   # 正在夹 / 刚松开的零件：手指与它的接触是夹持，不算碰撞
            self.tray_mod = modules[by_role["tray"]]
            self.tray_path = f"{ROOT}/{prim_name(by_role['tray'])}"
        self.handled_steps = {it["step"] for it in self.hd["items"]} if self.hd else set()
        self.h_i, self.h_count, self.h_time = 0, 0, 0.0
        self.h_segments: list[dict] = []
        self.carry = None             # {"part", "rel"}：吸盘上的零件（相对 TCP 的位姿）
        self.pending_collide = []     # 刚放下的零件：吸盘抬起后再开碰撞（贴合接触不算碰撞）
        self.tray_arrived = False
        self._feeder_init()
        self._fastening_init()

    def _env_paths(self) -> list[str]:
        r = self.by_role
        fd = f"{ROOT}/{prim_name(r['feeder'])}"
        paths = [f"{ROOT}/table", f"{ROOT}/riser", f"{ROOT}/{prim_name(r['conveyor'])}", f"{ROOT}/{prim_name(r['baseplate'])}",
                 f"{ROOT}/{prim_name(r['fixture'])}/body", f"{fd}/body"]
        paths += [f"{fd}/lane_{i + 1}/hopper" for i in range(self.feeder_mod["lanes"])]
        paths += [f"{ROOT}/{prim_name(r[k])}" for k in ("controller", "interface_box", "gateway")]
        # M4 ①：快换架（托轨、横梁、立柱）
        hd = load_handling()
        if hd and hd.get("tool_changer"):
            tc = f"{ROOT}/{prim_name(hd['tool_changer'])}"
            for tid in hd["tool_docks"]:
                paths += [f"{tc}/dock_{prim_name(tid)}_{n}" for n in ("railL", "railR", "beam", "post")]
        # M3.5b：SOP 零件的每个简化体与板上元件（build_scene.build_product 的命名）
        units = SCENE["line_units"] if load_handling() else 1
        for k in range(1, units + 1):
            pp = f"{ROOT}/{prim_name(r['fixture'])}/{prim_name(self.station['product'])}{'' if k == 1 else f'_u{k}'}/parts"
            for part in self.assembly["parts"]:
                xp = f"{pp}/{prim_name(part['id'])}"
                paths += [f"{xp}/s{i}" for i in range(len(part["shapes"]))]
                paths += [f"{xp}/components/{prim_name(c['id'])}" for c in part["components"]]
        self.env_paths = paths
        return paths

    # ---------- 治具实际位姿（M3.3：模拟现场治具放偏） ----------
    def place_fixture(self, F: Pose) -> dict:
        """把场景里的治具（含基准球、产品）放到基座系位姿 F（mm）。play 之前调用。
        真值 fixture_true 取场景读回（三个基准球实际中心无噪声拟合），不取写入目标：M3.3 实测写入后
        世界矩阵偏航与目标差约 2e-5 rad（基准球偏约 1.5 µm）。返回各基准球 写入目标 vs 读回 的差（mm）。"""
        XformPrim(self.fixture_path).set_world_poses(positions=[[v / 1000.0 for v in F.t]],
                                                     orientations=[rot_to_quat(F.R).tolist()])
        datums = self.modules[self.station["fixture_frame"]]["datums"]
        actual = [self.datum_world_mm(d["id"]).tolist() for d in datums]
        self.fixture_true = calibrate_fixture([d["xyz"] for d in datums], actual)
        return {d["id"]: float(np.linalg.norm(np.asarray(a) - np.asarray(F.apply(d["xyz"])))) for d, a in zip(datums, actual)}

    def fixture_world_pose(self) -> Pose:
        """从场景读回治具位姿（mm），用于核对真值确实写进去了。"""
        p, o = XformPrim(self.fixture_path).get_world_poses()
        return Pose(quat_to_rot(o.numpy()[0]).tolist(), [float(v) * 1000.0 for v in p.numpy()[0]])

    def datum_world_mm(self, datum_id: str) -> np.ndarray:
        p, _ = XformPrim(f"{self.fixture_path}/{datum_id}").get_world_poses()
        return p.numpy()[0].astype(float) * 1000.0

    # ---------- 供钉器（MES 数据点见 kb/modules/feeder-fd3l.yaml） ----------
    def _feeder_init(self):
        kinds = self.station["checks"]["screw_kinds"]
        cap = self.feeder_mod["lane_capacity_pcs"]
        fd = f"{ROOT}/{prim_name(self.by_role['feeder'])}"
        self.fd_path = fd
        self.lanes = [{"lane": i + 1, "screw": kinds[i] if i < len(kinds) else None, "capacity": cap, "level": cap,
                       "feed_count": 0, "jam": False} for i in range(self.feeder_mod["lanes"])]
        self.look_ok = PreviewSurfaceMaterial("/World/Looks/lamp_ok")
        self.look_feed = PreviewSurfaceMaterial("/World/Looks/lamp_feed")
        self.look_feed.set_input_values("diffuseColor", [1.0, 0.75, 0.0])
        self.look_feed.set_input_values("emissiveColor", [1.0, 0.6, 0.0])
        self.hopper_h = SCENE["feeder"]["hopper"]["height"] / 1000.0
        self.body_h = SCENE["feeder"]["size"][2] / 1000.0
        product = self.product
        groups = groups_by_id(product)
        self.point_screw = {p["id"]: groups[p["group"]]["screw"] for p in product["fastening"]["points"]}
        self.point_group = {p["id"]: groups[p["group"]] for p in product["fastening"]["points"]}
        for ln in self.lanes:
            self._emit_lane(ln, "idle")

    def _emit_lane(self, ln, state, event=None):
        rec = {"t_s": round(self.t, 3), "source": self.by_role["feeder"], "via": self.feeder_mod["mes"]["via"], "state": state,
               "lane": ln["lane"], "lane_screw": ln["screw"], "lane_level_pct": round(100.0 * ln["level"] / ln["capacity"], 1),
               "lane_empty": ln["level"] <= 0, "lane_jam": ln["jam"], "feed_count": ln["feed_count"]}
        if event:
            rec["event"] = event
        self.gateway.append(rec)

    def _lane_of(self, point):
        screw = self.point_screw[point]
        return next(ln for ln in self.lanes if ln["screw"] == screw)

    def feed_start(self, point):
        ln = self._lane_of(point)
        XformPrim(f"{self.fd_path}/lane_{ln['lane']}/lamp").apply_visual_materials(self.look_feed)
        self._emit_lane(ln, "feeding")

    def feed_done(self, point):
        ln = self._lane_of(point)
        if ln["level"] <= 0:
            self._emit_lane(ln, "fault")
            return False
        ln["level"] -= 1
        ln["feed_count"] += 1
        k = max(ln["level"] / ln["capacity"], 0.02)
        hopper = XformPrim(f"{self.fd_path}/lane_{ln['lane']}/hopper")
        hopper.set_local_scales([[1.0, 1.0, k]])
        hopper.set_local_poses(translations=[[0.0, 0.0, self.body_h / 2 + self.hopper_h * k / 2]])
        XformPrim(f"{self.fd_path}/lane_{ln['lane']}/lamp").apply_visual_materials(self.look_ok)
        self._emit_lane(ln, "idle", event="feed_done")
        self._screw_on_bit(point)
        return True

    # ---------- 螺钉、异常注入、复拧、问题记录（M3.4） ----------
    def _fastening_init(self):
        lib = load_library()
        self.lib = lib
        groups = groups_by_id(self.product)
        sh = SCENE["screw_head"]
        self.screw_geo = {}
        for p in self.product["fastening"]["points"]:
            sid = groups[p["group"]]["screw"]
            d = lib["screws"][sid]["d_mm"]
            self.screw_geo[p["id"]] = {"L": screw_length_mm(sid), "hh": sh["h_ratio"] * d}
        self.default_lift = min(g["L"] + g["hh"] for g in self.screw_geo.values())
        self.parts = {p["id"]: p for p in self.assembly["parts"]}
        self.fastens_of = {pid: st["fastens"] for st in self.assembly["steps"] if st["type"] == "fasten" for pid in st["points"]}
        self.placed: list[dict] = []          # 已执行的放件工步
        self.point_xyz = {p["id"]: p["xyz"] for p in self.product["fastening"]["points"]}
        self.screw_prims = {pid: XformPrim(f"{self.screws_root}/{pid}") for pid in self.screw_geo}
        self.bit_visual = XformPrim(f"{ROOT}/{prim_name(self.by_role['arm'])}/wrist_3_link/{prim_name(self.by_role['executor'])}/bit_visual")
        self.active = None            # {"point", "insert_mm", "spin_deg"}：批头上的螺钉
        self.issues: list[dict] = []
        self.looks = {}
        for name, rgb, emit in (("hole_ok", [0.1, 0.85, 0.2], [0.0, 0.5, 0.1]), ("hole_ng", [0.95, 0.1, 0.1], [0.8, 0.0, 0.0]),
                                ("hole_work", [1.0, 0.75, 0.0], [0.6, 0.4, 0.0]), ("andon_ok", [0.1, 0.9, 0.2], [0.0, 0.6, 0.1]),
                                ("andon_retry", [1.0, 0.75, 0.0], [1.0, 0.6, 0.0]), ("andon_ng", [0.95, 0.1, 0.1], [1.0, 0.0, 0.0])):
            m = PreviewSurfaceMaterial(f"/World/Looks/{name}")
            m.set_input_values("diffuseColor", rgb)
            m.set_input_values("emissiveColor", emit)
            self.looks[name] = m
        self._set_bit(self.default_lift, 0.0)
        self.fault_plan = self._draw_fault_plan()

    def _draw_fault_plan(self) -> dict:
        """每颗螺钉每次拧（含复拧）独立按 fault_rate 抽签；inject 覆盖前几次。固定种子，结果可复现。"""
        rng = random.Random(f"{self.seed}:{self.sn}")   # 种子 + 序列号：同一件可复现，连续多件各自独立
        steps = load_yaml(STATION_DIR / "flow.yaml")["sequence"]
        plan = {}
        for s in steps:
            n = 1 + s.get("executor_retry", {}).get("max", 0)
            modes = []
            for _ in range(n):
                r, m = rng.random(), rng.choice(FAULT_MODES)
                modes.append(m if r < self.fault_rate else "ok")
            for i, m in enumerate(self.inject.get(s["point"], [])[:n]):
                modes[i] = m
            plan[s["point"]] = modes
        return plan

    def _visible(self, path: str, on: bool):
        img = UsdGeom.Imageable(stage_utils.get_current_stage(backend="usd").GetPrimAtPath(path))
        img.MakeVisible() if on else img.MakeInvisible()

    def _part_path(self, part_id) -> str:
        return f"{self.product_path}/parts/{prim_name(part_id)}"

    def _hole_path(self, point) -> str:
        return f"{self._part_path(self.fastens_of[point])}/holes/{point}"

    def _mark(self, point, look):
        XformPrim(self._hole_path(point)).apply_visual_materials(self.looks[look])

    def _andon(self, look):
        XformPrim(f"{ROOT}/andon").apply_visual_materials(self.looks[look])

    def _collide(self, part_id, on: bool):
        part = self.parts[part_id]
        xp = self._part_path(part_id)
        paths = [f"{xp}/s{i}" for i in range(len(part["shapes"]))] + [f"{xp}/components/{prim_name(c['id'])}" for c in part["components"]]
        GeomPrim(paths=paths).set_enabled_collisions([on])

    def place_g(self, step: dict, animate: bool = True):
        """执行 SOP 放件工步：零件从上方 place_drop_mm 落到装配位姿，落到位后才打开碰撞。
        时长取 SOP 的 duration_s；为「缺失」时用 place_default_s 演示并标 assumed。upstream 的零件开始时直接在位。
        运行中切换碰撞开关、移动静态碰撞体均已在本机验证（docs/isaac-api-notes.md）。"""
        pid = step["part"]
        part = self.parts[pid]
        dur = step.get("duration_s")
        assumed = not isinstance(dur, (int, float)) or isinstance(dur, bool)
        seconds = SCENE["place_default_s"] if assumed else float(dur)
        xp = XformPrim(self._part_path(pid))
        x, y, z = (v / 1000.0 for v in part["pose_in_fixture"]["xyz"])
        t0 = self.t
        self._visible(self._part_path(pid), True)
        if animate and seconds > 0:
            drop = SCENE["place_drop_mm"] / 1000.0
            n = max(1, round(seconds / DT))
            for k in range(1, n + 1):
                xp.set_local_poses(translations=[[x, y, z + drop * (1 - k / n)]])
                yield
        xp.set_local_poses(translations=[[x, y, z]])
        self._collide(pid, True)
        actor_cn = {"operator": "人工", "robot": "机械臂", "executor": "执行器", "upstream": "上游"}.get(step["actor"], "执行者未定")
        if animate:
            self.caption(f"放件 {part['name']}（{actor_cn}）")
        rec = {"step": step["id"], "part": pid, "actor": step["actor"], "t_s": round(t0, 3), "duration_s": round(self.t - t0, 3),
               "duration_source": "assumed" if assumed else "sop"}
        self.placed.append(rec)
        self.gateway.append({"t_s": round(self.t, 3), "source": self.station["station_id"], "via": self.feeder_mod["mes"]["via"],
                             "event": "part_placed", **{k: rec[k] for k in ("step", "part", "actor", "duration_source")}})

    def place(self, step: dict, animate: bool = True):
        return self._run_g(self.place_g(step=step, animate=animate))

    def _place_until_g(self, point):
        """拧 point 之前，把 SOP 里排在它所在拧紧工步前面、还没执行的放件工步做完（机械臂原地等待）。"""
        done = {r["step"] for r in self.placed}
        for st in self.assembly["steps"]:
            if st["type"] == "fasten" and point in st["points"]:
                return
            if st["type"] == "place" and st["id"] not in done and st["id"] not in self.handled_steps:
                (yield from self.place_g(st, animate=st["actor"] != "upstream"))

    def _place_until(self, point):
        return self._run_g(self._place_until_g(point=point))

    def _set_bit(self, lift_mm: float, spin_deg: float):
        """批头可见部分：尖端在 TCP 上方 lift_mm（浮动头压缩），绕工具轴转 spin_deg。"""
        h = math.radians(spin_deg) / 2
        self.bit_visual.set_local_poses(translations=[[0.0, 0.0, -lift_mm / 1000.0]], orientations=[[math.cos(h), 0.0, 0.0, math.sin(h)]])

    def _present_at_nest(self, point, op_i):
        """出钉位上立起这颗螺钉（头朝上），批头抬到这颗螺钉的高度准备吸取。位置取程序里的取钉位姿（TCP = 螺钉尖）。"""
        pick = next(o for o in self.ops[op_i:] if o.get("purpose") == "feed" and o["speed"] == "slow" and o["point"] == point)
        g = self.screw_geo[point]
        x, y, z = (v / 1000.0 for v in pick["pose"]["xyz_mm"])
        self.screw_prims[point].set_world_poses(positions=[[x, y, z + (g["L"] + g["hh"]) / 1000.0]], orientations=[[1.0, 0.0, 0.0, 0.0]])
        self._visible(f"{self.screws_root}/{point}", True)
        self._set_bit(g["L"] + g["hh"], 0.0)

    def _screw_on_bit(self, point):
        self.active = {"point": point, "insert_mm": 0.0, "spin_deg": 0.0}
        self._visible(f"{self.screws_root}/{point}", True)
        self._mark(point, "hole_work")
        self._update_screw()

    def _update_screw(self):
        """批头上的螺钉跟着 TCP：螺钉头顶 = 批头尖端；螺钉尖在 engage 位刚好到孔口，拧入 insert_mm 后头部落座。"""
        if not self.active:
            return
        g = self.screw_geo[self.active["point"]]
        lift = g["L"] + g["hh"] - self.active["insert_mm"]
        self._set_bit(lift, self.active["spin_deg"])
        fr = self.rigs["fastener"]
        p, o = fr.links.get_world_poses(indices=[UR_LINKS.index("wrist_3_link")])
        R = quat_to_rot(o.numpy()[0])
        tip = p.numpy()[0] + R[:, 2] * (fr.tcp_m - lift / 1000.0)
        h = math.radians(self.active["spin_deg"]) / 2
        Rs = R @ np.array([[1.0, 0, 0], [0, -1.0, 0], [0, 0, -1.0]]) @ quat_to_rot([math.cos(h), 0.0, 0.0, math.sin(h)])
        self.screw_prims[self.active["point"]].set_world_poses(positions=[tip.tolist()], orientations=[rot_to_quat(Rs).tolist()])

    def _release_screw(self):
        self.active = None
        self._set_bit(self.default_lift, 0.0)

    def _attempt_curve(self, point, k, mode, step):
        """第 k 次拧的曲线：合格用流水线产出的曲线；异常用 procmodel.synth_curve 生成（演示曲线）。"""
        if mode == "ok":
            path = STATION_DIR / "curves" / f"{point}.csv"
            return procmodel.read_curve(path), path
        g = self.point_group[point]
        w = self.windows[step["executor_params"]]
        rows = procmodel.synth_curve(mode, w["window_nm"], w["target_nm"], self.lib["screws"][g["screw"]], g["depth_mm"])
        path = TWIN_DIR / f"curves{self.tag}" / f"{point}_a{k}_{mode}.csv"
        path.parent.mkdir(parents=True, exist_ok=True)
        procmodel.write_curve(rows, path)
        return rows, path

    def _drive_g(self, point, rows, seconds, reverse=False):
        """拧入（或反转退出）动画：螺钉深度按曲线角度 / 理论角度，转角按曲线角度，在 seconds 内走完。"""
        g = self.point_group[point]
        exp = procmodel.expected_angle_deg(self.lib["screws"][g["screw"]], g["depth_mm"])
        total = rows[-1][0] if rows else 0.0
        L = self.screw_geo[point]["L"]
        a0, i0 = self.active["spin_deg"], self.active["insert_mm"]
        i1 = 0.0 if reverse else L * min(1.0, total / exp)
        n = max(1, round(seconds / DT))
        for k in range(1, n + 1):
            f = k / n
            self.active["insert_mm"] = i0 + (i1 - i0) * f
            self.active["spin_deg"] = a0 + (-1 if reverse else 1) * total * f
            yield

    def _drive(self, point, rows, seconds, reverse=False):
        return self._run_g(self._drive_g(point=point, rows=rows, seconds=seconds, reverse=reverse))

    def snap(self, name: str, point: str | None = None, place_view: str | None = None, at_mm=None):
        """近景截图到 snap_dir（只用 Isaac 视口截图）。给 point 时对准该孔位（从斜上方较陡的角度，避开壳体侧壁），否则拍整个产品。"""
        if not self.snap_dir:
            return
        self.snap_dir.mkdir(parents=True, exist_ok=True)
        path = self.snap_dir / f"{name}.png"
        if path.exists():
            path.unlink()
        if at_mm is not None:   # 任意位置（基座系 mm）的近景，例如快换工位
            tgt = np.asarray(at_mm) / 1000.0
            eye = tgt + np.asarray([0.22, -0.32, 0.20])
        elif place_view:   # 放件：从侧上方看治具上的零件与工具
            tgt = np.asarray(self.fixture_true.apply([0.0, 0.0, 30.0])) / 1000.0
            eye = tgt + np.asarray([0.16, -0.30, 0.22])
        elif point:
            tgt = np.asarray(self.fixture_true.apply(self.point_xyz[point])) / 1000.0
            eye = tgt + np.asarray(SCENE["camera_point_offset"])
        else:
            tgt = np.asarray(self.fixture_true.apply([0.0, 0.0, 20.0])) / 1000.0
            eye = tgt + np.asarray(SCENE["camera_product_offset"])
        ViewportManager.set_camera_view("/OmniverseKit_Persp", eye=eye.tolist(), target=tgt.tolist())
        self._capture(path, settle=4)
        self.set_view(self.view)
        self.snaps.append(str(path))

    def _capture(self, path, settle=1):
        """Isaac 视口截图（不用屏幕截图）。返回后文件可能尚未落盘，等到出现为止。
        等待用 RenderingManager.render()（只渲染）：play 状态下直接 app.update() 会推进物理（本机实测 2026-10-26），
        截图期间关节按最后的指令继续走，录屏时放件臂冲过目标撞上快换架托轨。"""
        for _ in range(settle):
            RenderingManager.render()

        async def capture():
            await capture_viewport_to_file(get_active_viewport(), file_path=str(path), is_hdr=False).wait_for_result()

        # run_coroutine 等截图时内部也会 app.update()：期间关掉物理步进（同 RenderingManager.render 的做法），截完恢复。
        # 不关时每截一帧物理多走几步，录屏与不录屏结果不同（本机实测：第 4 步起关节就有差异，录屏版第 2 台擦到 C12）
        settings = carb.settings.get_settings()
        play = settings.get_as_bool("/app/player/playSimulations")
        settings.set_bool("/app/player/playSimulations", False)
        try:
            self.app.run_coroutine(capture())
            frames = 0
            while not path.exists() and frames < 600:
                RenderingManager.render()
                frames += 1
        finally:
            settings.set_bool("/app/player/playSimulations", play)

    # ---------- 录屏（M4.1） ----------
    def set_view(self, view):
        """机位：None/"main" 全景；("point", 点位) 孔位近景；"product" 整件近景。"""
        self.view = view
        if view in (None, "main"):
            if self.camera_main:
                ViewportManager.set_camera_view("/OmniverseKit_Persp", **self.camera_main)
            return
        if view == "product":
            tgt = np.asarray(self.fixture_true.apply([0.0, 0.0, 20.0])) / 1000.0
            eye = tgt + np.asarray(SCENE["camera_product_offset"])
        elif view == "place":   # 放件：侧上方看治具上的零件与工具
            tgt = np.asarray(self.fixture_true.apply([0.0, 0.0, 30.0])) / 1000.0
            eye = tgt + np.asarray([0.16, -0.30, 0.22])
        elif isinstance(view, tuple) and view[0] == "nest":   # 取钉：出钉位近景
            tgt = np.asarray(view[1]) / 1000.0
            eye = tgt + np.asarray([0.10, -0.14, 0.10])
        else:
            tgt = np.asarray(self.fixture_true.apply(self.point_xyz[view[1]])) / 1000.0
            eye = tgt + np.asarray(SCENE["camera_record_point_offset"])
        ViewportManager.set_camera_view("/OmniverseKit_Persp", eye=eye.tolist(), target=tgt.tolist())

    def _record_frame(self):
        d = self.record_dir / "frames"
        d.mkdir(parents=True, exist_ok=True)
        if isinstance(self.view, tuple) and self._closeup_frame is None:
            self._closeup_frame = self._nframe
        self._capture(d / f"f{self._nframe:05d}.png")
        if getattr(self, "kanban_overlay", False):
            self._draw_kanban(d / f"f{self._nframe:05d}.png")
        self._nframe += 1

    def kanban_state(self) -> dict:
        """线体看板的状态，全部由网关事件推出（与演示页回放用同一套规则）。"""
        n = getattr(self, "line_n", 1)
        units = {k: {"unit": k, "sn": None, "state": "等待", "points": 0, "ng": 0, "t_start": None, "t_end": None} for k in range(1, n + 1)}
        sn_unit = {}
        for e in self.gateway:
            ev = e.get("event")
            if ev == "unit_start":
                u = units[e["unit"]]
                u.update(sn=e["sn"], state="装配中", t_start=e["t_s"])
                sn_unit[e["sn"]] = e["unit"]
            elif "result" in e and "point" in e and e.get("sn") in sn_unit and ev is None:
                u = units[sn_unit[e["sn"]]]
                u["points"] += 1
                u["ng"] += e["result"] != "OK"
            elif ev == "unit_unloaded":
                units[e["unit"]]["state"] = "下线中"
            elif ev == "unit_result":
                u = units[e["unit"]]
                u["result"] = e["result"]
                u["t_end"] = e["t_s"]
            elif ev == "unit_delivered":
                u = units[e["unit"]]
                u["state"] = "已送达下游" + ("（合格）" if u.get("result") == "OK" else "（不合格）")
        done = [u for u in units.values() if u.get("result")]
        starts = [u["t_start"] for u in units.values() if u["t_start"] is not None]
        interval = round((starts[-1] - starts[0]) / (len(starts) - 1), 1) if len(starts) > 1 else None
        return {"t": round(self.t, 1), "units": list(units.values()), "delivered": sum(u["state"].startswith("已送达") for u in units.values()),
                "ok": sum(u.get("result") == "OK" for u in done), "done": len(done), "interval_s": interval,
                "current": self.unit, "points_total": len(self.point_xyz)}

    def _draw_kanban(self, path):
        """在已截下的帧右上角叠加看板面板。"""
        from PIL import Image

        img = Image.open(path).convert("RGBA")
        W, H = img.size
        panel = self._kanban_image(int(W * 0.34), int(H * 0.36), alpha=215)
        img.alpha_composite(panel, (W - panel.width - 16, 16))
        img.convert("RGB").save(path)

    def _kanban_image(self, pw, ph, alpha=255, show_t=True):
        """看板面板（PIL RGBA）：标题、送达 / 良率 / 节拍、每台一行状态。"""
        from PIL import Image, ImageDraw, ImageFont

        k = self.kanban_state()
        H = ph / 0.36
        layer = Image.new("RGBA", (pw, ph), (0, 0, 0, 0))
        d = ImageDraw.Draw(layer)
        x0, y0 = 0, 0
        d.rounded_rectangle([0, 0, pw - 1, ph - 1], radius=14, fill=(14, 28, 48, alpha))
        fb = ImageFont.truetype("C:/Windows/Fonts/msyhbd.ttc", int(H * 0.030))
        fr = ImageFont.truetype("C:/Windows/Fonts/msyh.ttc", int(H * 0.024))
        lh = int(H * 0.036)
        d.text((x0 + 18, y0 + 12), f"线体看板 · {self.station['station_id']} · {self.station['product']}", font=fb, fill=(255, 255, 255))
        y = y0 + 12 + lh + 6
        rate = f"{100.0 * k['ok'] / k['done']:.0f}%" if k["done"] else "—"
        d.text((x0 + 18, y), f"送达 {k['delivered']} 台   良率 {rate}   节拍 {k['interval_s'] if k['interval_s'] else '—'} s" + (f"   T+{k['t']:.0f} s" if show_t else ""),
               font=fr, fill=(170, 220, 255))
        y += lh + 4
        colors = {"等待": (150, 160, 175), "装配中": (255, 200, 60), "下线中": (120, 200, 255)}
        for u in k["units"]:
            st = u["state"]
            col = colors.get(st, (90, 220, 120) if "合格）" in st and "不" not in st else (255, 90, 90))
            sn = u["sn"] or f"第 {u['unit']} 台"
            prog = f"{u['points']}/{k['points_total']}" + (f"  NG {u['ng']}" if u["ng"] else "")
            mark = "● " if u["unit"] == k["current"] and st in ("装配中", "下线中") else "   "
            d.text((x0 + 18, y), f"{mark}{sn}", font=fr, fill=(255, 255, 255))
            d.text((x0 + int(pw * 0.42), y), st, font=fr, fill=col)
            d.text((x0 + int(pw * 0.78), y), prog, font=fr, fill=(220, 225, 235))
            y += lh
        return layer

    def _update_screen(self):
        """场景大屏：把看板画成图，改大屏材质贴图的 file（运行中改能刷新，本机实测）。"""
        if not getattr(self, "screen_tex", None):
            return
        k = self.kanban_state()
        key = json.dumps({x: k[x] for x in k if x != "t"}, ensure_ascii=False, sort_keys=True, default=str)
        if key == getattr(self, "_screen_key", None):   # 状态没变就不重写（频繁换贴图会让渲染器一直显示占位色，孪生实测）
            return
        self._screen_key = key
        self._screen_flip = 1 - getattr(self, "_screen_flip", 1)   # 两个文件名交替写，避免读到写了一半的图
        png = self.screen_png.with_name(f"line_screen_live_{self._screen_flip}.png")
        self._kanban_image(1024, 584, show_t=False).convert("RGB").save(png)
        self.screen_tex.Set(png.as_posix())

    def caption(self, text: str):
        """字幕：记在当前视频时间上（帧号 / fps）。"""
        if self.record_dir:
            self.captions.append({"t": self._nframe * self.record_every * DT, "text": text})

    def hold_frames(self, seconds: float):
        """不推进物理，只多录几秒当前画面（片头、片尾）。"""
        if not self.record_dir:
            return
        for _ in range(round(seconds / (DT * self.record_every))):
            self._record_frame()

    def finish_recording(self, name: str) -> dict | None:
        """帧序列 → mp4（omni.videoencoding，本机核实）；字幕写 .srt；删掉帧序列。"""
        if not self.record_dir or not self._nframe:
            return None
        import shutil

        omni.kit.app.get_app().get_extension_manager().set_extension_enabled_immediate("omni.videoencoding", True)
        from video_encoding import get_video_encoding_interface

        fps = round(1.0 / (DT * self.record_every))
        frames = sorted((self.record_dir / "frames").glob("f*.png"))
        mp4 = self.record_dir / f"{name}.mp4"
        api = get_video_encoding_interface()
        if not api.start_encoding(str(mp4), fps, len(frames), True):
            raise RuntimeError("视频编码启动失败（omni.videoencoding）")
        for f in frames:
            api.encode_next_frame_from_file(str(f))
        api.finalize_encoding()
        end = len(frames) / fps

        def ts(x):
            ms = int(round(x * 1000))
            return f"{ms // 3600000:02d}:{ms // 60000 % 60:02d}:{ms // 1000 % 60:02d},{ms % 1000:03d}"

        rows = []
        for i, c in enumerate(self.captions):
            t1 = min(end, self.captions[i + 1]["t"] if i + 1 < len(self.captions) else end, c["t"] + 4.0)
            rows.append(f"{i + 1}\n{ts(c['t'])} --> {ts(max(t1, c['t'] + 0.8))}\n{c['text']}\n")
        (self.record_dir / f"{name}.srt").write_text("\n".join(rows), encoding="utf-8")
        for k, q in enumerate((0.2, 0.45, 0.7, 0.95)):   # 留 4 张预览帧，便于不开视频就核对画面
            shutil.copyfile(frames[min(len(frames) - 1, int(q * len(frames)))], self.record_dir / f"{name}_preview{k + 1}.png")
        if self._closeup_frame is not None:   # 近景切入后约 1 s（批头已下到孔口）
            shutil.copyfile(frames[min(len(frames) - 1, self._closeup_frame + fps)], self.record_dir / f"{name}_closeup.png")
        shutil.rmtree(self.record_dir / "frames", ignore_errors=True)
        return {"mp4": str(mp4), "srt": str(self.record_dir / f"{name}.srt"), "frames": len(frames), "fps": fps,
                "seconds": round(end, 2), "bytes": mp4.stat().st_size}

    def apply_dfm_fix(self, dfm_issues: list[dict]) -> list[dict]:
        """按 DFM-01 建议把障碍物沿「孔位 → 障碍物」方向外移到 下限 + 0.3 mm（与 dfm.check 的建议同一算法）。只改场景，不改产品数据。play 之前调用。"""
        obs = {o["id"]: o for o in self.product["fastening"].get("obstacles", [])}
        moved = []
        for it in dfm_issues:
            if it.get("rule") != "DFM-01":
                continue
            ob, pt = obs[it["obstacle"]], self.point_xyz[it["point"]]
            v = np.asarray(ob["xy"], dtype=float) - np.asarray(pt[:2], dtype=float)
            new_xy = np.asarray(pt[:2]) + v / np.linalg.norm(v) * (it["limit_mm"] + 0.3)
            owner = next(p for p in self.assembly["parts"] if any(c["id"] == ob["id"] for c in p["components"]))
            prim = XformPrim(f"{self._part_path(owner['id'])}/components/{prim_name(ob['id'])}", reset_xform_op_properties=True)
            yaw = math.radians(owner["pose_in_fixture"]["rpy"][2])
            dx, dy = new_xy - np.asarray(ob["xy"], dtype=float)
            lx, ly = math.cos(-yaw) * dx - math.sin(-yaw) * dy, math.sin(-yaw) * dx + math.cos(-yaw) * dy   # 换到零件坐标系
            t, _ = prim.get_local_poses()
            t = t.numpy()[0]
            prim.set_local_poses(translations=[[t[0] + lx / 1000.0, t[1] + ly / 1000.0, t[2]]])
            moved.append({"obstacle": ob["id"], "point": it["point"], "from_xy": ob["xy"], "to_xy": [round(float(v), 3) for v in new_xy],
                          "shift_mm": round(float(np.linalg.norm(new_xy - np.asarray(ob["xy"]))), 3)})
        self.dfm_fix = moved
        self._dfm_issues = dfm_issues   # 流水线换下一台时对新零件同样外移（M4 ③）
        return moved

    def _issue(self, point, attempts, step, stopped):
        g = self.point_group[point]
        ctq = [c["id"] for c in self.product["ctq"] if g["id"] in c.get("groups", []) or point in c.get("points", [])]
        final = attempts[-1]
        rec = {"issue_id": f"{self.sn}-{point}-{len(self.issues) + 1:02d}", "t_s": round(self.t, 3), "sn": self.sn,
               "station": self.station["station_id"], "point": point, "group": g["id"], "part": g.get("part"), "screw": g["screw"],
               "params_id": step["executor_params"], "ctq": ctq, "mode": final["mode"], "mode_cn": procmodel.MODE_CN.get(final["mode"], final["mode"]),
               "attempts": attempts, "actions": (["exec_retry"] if len(attempts) > 1 else []) + list(step["on_ng"]),
               "unit_disposition": "整件停止" if stopped else "继续后续点位，整件判 NG",
               "status": "待复判"}
        if stopped:
            rec["arm_state"] = "整件停止：机械臂退回接近位、回原位后停止（branch_on_fault.on_stop）"
        self.issues.append(rec)
        return rec

    # ---------- 仿真推进 ----------
    def step(self):
        SimulationManager.step()
        if self.hd:
            self._update_tools()
            self._update_carry()
            if getattr(self, "line_n", 1) > 1 and self._nstep % 15 == 0:
                self._update_screen()
        # 只渲染、不再推进物理：play 状态下 app.update() 本身也走一步物理（本机实测 2026-10-26），
        # 以前这里 SimulationManager.step() 之后再 app.update()，每个控制步物理实际走了两步
        RenderingManager.render()
        self.t += DT
        self._nstep += 1
        self._update_screw()
        if os.environ.get("JSDEMO_TRACE"):   # 诊断：逐步记录两臂关节实测 / 指令、法兰位置（排查抖动用）
            self._trace_row()
        if self.record_dir and self._nstep % self.record_every == 0:
            self._record_frame()
        for r in self.rigs.values():
            f = r.links.get_contact_force_matrix(dt=DT).numpy()  # (links, filters, 3)
            mag = np.linalg.norm(f, axis=2)
            for i, j in zip(*np.nonzero(mag > CONTACT_N)):
                key = (r.name, UR_LINKS[i], r.filters[j])
                if not any((c.get("arm", "fastener"), c["link"], c["with"]) == key for c in self.collisions):
                    self.collisions.append({"t_s": round(self.t, 3), "arm": r.name, "link": key[1], "with": key[2],
                                            "force_n": round(float(mag[i, j]), 4), "op_index": self.op_index})
                    if self.snap_dir and len(self.collisions) <= 2:   # 碰撞那一刻的全景，便于看是谁撞了谁
                        self.snap(f"{self.sn}_collision_{len(self.collisions)}", at_mm=[560.0, -60.0, 250.0])
        t = self.tools.get(self.tool_on) if self.hd and self.tool_on else None
        if t and t["kind"] == "2f" and "links" in t:   # 夹爪手指与环境的接触；与正在夹的零件接触不算
            f = t["links"].get_contact_force_matrix(dt=DT).numpy()
            mag = np.linalg.norm(f, axis=2)
            for i, j in zip(*np.nonzero(mag > CONTACT_N)):
                w = self.env_paths[j]
                if any(f"/parts/{prim_name(pid)}/" in w for pid in self.grip_ignore):
                    continue
                key = ("gripper", G2F_LINKS[i], w)
                if not any((c.get("arm"), c["link"], c["with"]) == key for c in self.collisions):
                    self.collisions.append({"t_s": round(self.t, 3), "arm": "gripper", "link": key[1], "with": w,
                                            "force_n": round(float(mag[i, j]), 4), "op_index": self.op_index})

    def _trace_row(self):
        if not hasattr(self, "_trace_f"):
            self._trace_f = open(os.environ["JSDEMO_TRACE"], "w", encoding="utf-8")
            for n, r in self.rigs.items():
                kp, kd = r.arm.get_dof_gains()
                self._trace_f.write(json.dumps({"gains": n, "kp": kp.numpy()[0].tolist(), "kd": kd.numpy()[0].tolist()}) + chr(10))
        row = {"t": round(self.t, 4), "h_i": getattr(self, "h_i", None), "tool": getattr(self, "tool_on", None),
               "carry": (self.carry or {}).get("part") if self.hd else None}
        for n, r in self.rigs.items():
            if r.cmd is None:
                continue
            row[n] = {"q": r.arm.get_dof_positions().numpy()[0].round(6).tolist(), "cmd": np.round(r.cmd, 6).tolist(),
                      "v": r.arm.get_dof_velocities().numpy()[0].round(5).tolist()}
        if self.hd:
            row["flange"] = np.round(self._flange_world()[:3, 3], 6).tolist()
        self._trace_f.write(json.dumps(row) + chr(10))
        self._trace_f.flush()

    def q(self) -> np.ndarray:
        return self.arm.get_dof_positions().numpy()[0].astype(float)

    def tcp_measured(self) -> np.ndarray:
        """物理引擎里 wrist_3_link（= tool0）位姿推出的 TCP，单位 m。"""
        p, o = self.links.get_world_poses(indices=[UR_LINKS.index("wrist_3_link")])
        p, o = p.numpy()[0], o.numpy()[0]
        return p + quat_to_rot(o)[:, 2] * self.tcp_m

    def ik_for(self, tcp_pos, R, warm, need_fine=True):
        """默认容差粗解，再以粗解为初值用紧容差细化。need_fine=False（直线中间节点）时细化失败可退回粗解。"""
        tool0, quat = tcp_pos - R[:, 2] * self.tcp_m, rot_to_quat(R)
        coarse, ok = self.ik.compute_inverse_kinematics(self.ik_frame, tool0, quat, warm_start=warm)
        if not ok:
            return np.asarray(coarse, dtype=float), False
        joints, ok = self.ik.compute_inverse_kinematics(self.ik_frame, tool0, quat, warm_start=coarse,
                                                        orientation_tolerance=IK_ORI_TOL_RAD, position_tolerance=IK_POS_TOL_M)
        # 细化收敛不了时 Lula 会随机换初值，可能落到绕工具轴翻转的等效解（M3.2 实测 wrist_3 跳 179°）：离粗解太远即视为失败
        ok = ok and float(np.max(np.abs(np.asarray(joints) - np.asarray(coarse)))) < REFINE_MAX_RAD
        if not ok and not need_fine:
            return np.asarray(coarse, dtype=float), True
        return np.asarray(joints, dtype=float), bool(ok)

    def settle_g(self, target_tcp=None, target_q=None, tol_mm=ARRIVE_TOL_MM):
        """轨迹发完后等到位；返回 (到位, 等待秒数, 误差 mm)。"""
        t0, err = self.t, float("inf")
        while self.t - t0 < SETTLE_MAX_S:
            yield
            if target_tcp is not None:
                err = float(np.linalg.norm(self.tcp_measured() - target_tcp)) * 1000.0
            else:
                err = float(np.max(np.abs(self.q() - target_q))) * 180.0 / math.pi  # deg
            if err <= tol_mm if target_tcp is not None else err <= 0.05:
                return True, self.t - t0, err
        return False, self.t - t0, err

    def settle(self, target_tcp=None, target_q=None, tol_mm=ARRIVE_TOL_MM):
        return self._run_g(self.settle_g(target_tcp=target_tcp, target_q=target_q, tol_mm=tol_mm))

    def _command(self, q, final=False):
        """发关节指令：位置目标 + 速度前馈（指令曲线的差分）。只给位置时速度目标为 0，资产自带的阻尼（wrist_1 kd 90）
        全程把关节往回拽、顶到力矩上限，wrist_1 最多落后 34°，停下后再以 170°/s 追上——放件臂到位时的「抖」就是它
        （孪生实测 2026-10-26）。最后一步速度目标归零。"""
        q = np.asarray(q, dtype=float)
        qd = np.zeros_like(q) if final or self.cmd is None else (q - self.cmd) / DT
        self.cmd = q
        self.arm.set_dof_position_targets(q[None, :])
        self.arm.set_dof_velocity_targets(qd[None, :])

    def movej_g(self, joints_deg):
        q0, q1 = self.cmd.copy(), np.radians(joints_deg)
        d = float(np.max(np.abs(q1 - q0)))
        T = trapezoid(d, MOVEJ_V, MOVEJ_A)
        t0 = self.t
        while self.t - t0 < T:
            s = trapezoid_s(self.t - t0 + DT, d, MOVEJ_V, MOVEJ_A) / d if d > 0 else 1.0
            self._command(q0 + (q1 - q0) * s)
            yield
        self._command(q1, final=True)
        ok, ts, err = (yield from self.settle_g(target_q=q1))
        return {"planned_s": round(T, 3), "settle_s": round(ts, 3), "arrived": ok, "err_deg": round(err, 4), "ik_ok": True}

    def movej(self, joints_deg):
        return self._run_g(self.movej_g(joints_deg=joints_deg))

    def tcp_fk(self, q) -> np.ndarray:
        pos, rot = self.ik.compute_forward_kinematics(self.ik_frame, q)
        return np.asarray(pos) + np.asarray(rot)[:, 2] * self.tcp_m

    def movel_g(self, pose, speed, tol_mm=ARRIVE_TOL_MM, fine=True):
        """直线运动：每 MOVEL_KNOT_MM 取一个节点做精确逆解（初值 = 上一节点，构型分支不变），节点间关节插补；
        时间按直线梯形速度律分配。用正解抽样记录实际路径偏离直线的最大值 path_dev_mm。"""
        R1 = pose_rot(pose)
        p1 = np.asarray(pose["xyz_mm"], dtype=float) / 1000.0
        q0 = self.cmd.copy()
        pos0, rot0 = self.ik.compute_forward_kinematics(self.ik_frame, q0)
        R0 = np.asarray(rot0)
        p0 = np.asarray(pos0) + R0[:, 2] * self.tcp_m
        d = float(np.linalg.norm(p1 - p0))
        v = MOVEL_V[speed]
        T = trapezoid(d, v, MOVEL_A)
        # 节点：位置线性，姿态从起点到终点线性过渡（四元数归一化插值，本程序各段姿态相同或接近）
        n = max(1, math.ceil(d * 1000.0 / MOVEL_KNOT_MM))
        qa, qb = rot_to_quat(R0), rot_to_quat(R1)
        if np.dot(qa, qb) < 0:
            qb = -qb
        knots, warm = [q0], q0
        for k in range(1, n + 1):
            f = k / n
            qq = qa + (qb - qa) * f
            q, ok = self.ik_for(p0 + (p1 - p0) * f, quat_to_rot(qq / np.linalg.norm(qq)), warm, need_fine=fine and k == n)
            if not ok or not same_branch(warm, q):
                return {"planned_s": round(T, 3), "settle_s": 0.0, "arrived": False, "err_mm": None, "ik_ok": False,
                        "reason": ("ik_failed" if not ok else "branch_changed") + f" at knot {k}/{n}",
                        "q_prev_deg": [round(float(x), 2) for x in np.degrees(warm)], "q_deg": [round(float(x), 2) for x in np.degrees(q)]}
            knots.append(q)
            warm = q
        # 关节限速：按巡航速度走直线时各关节要的角速度不能超过资产的最大关节速度（留 10%），超了就整段降速
        # （UR 控制器同样会降速）。不降速时 wrist_3 被要到 300°/s、资产上限 180°/s，转工具时落后 30°（孪生实测 2026-10-26）
        if d > 0:
            jv = max(float(np.max(np.abs(knots[k + 1] - knots[k]) / self.rig.vmax)) for k in range(n)) * n / d * v
            if jv > JOINT_V_FRAC:
                v = v * JOINT_V_FRAC / jv
                T = trapezoid(d, v, MOVEL_A)
        u = (p1 - p0) / d if d > 0 else np.zeros(3)
        dev, t0 = 0.0, self.t
        while self.t - t0 < T:
            f = (trapezoid_s(self.t - t0 + DT, d, v, MOVEL_A) / d if d > 0 else 1.0) * n
            k = min(int(f), n - 1)
            self._command(knots[k] + (knots[k + 1] - knots[k]) * (f - k))
            r = self.tcp_fk(self.cmd) - p0
            dev = max(dev, float(np.linalg.norm(r - np.dot(r, u) * u)))
            yield
        self._command(knots[-1], final=True)
        ok, ts, err = (yield from self.settle_g(target_tcp=p1, tol_mm=tol_mm))
        return {"planned_s": round(T, 3), "settle_s": round(ts, 3), "arrived": ok, "err_mm": round(err, 4), "ik_ok": True, "tol_mm": tol_mm,
                "knots": n, "path_dev_mm": round(dev * 1000, 3), **self.diag(p1)}

    def movel(self, pose, speed, tol_mm=ARRIVE_TOL_MM, fine=True):
        return self._run_g(self.movel_g(pose=pose, speed=speed, tol_mm=tol_mm, fine=fine))

    def diag(self, p_target) -> dict:
        """区分误差来源：关节跟踪误差（实测 q 与指令 q）、模型误差（Lula 正解 vs 物理位姿）。"""
        q = self.q()
        pos, rot = self.ik.compute_forward_kinematics(self.ik_frame, q)
        tcp_fk = np.asarray(pos) + np.asarray(rot)[:, 2] * self.tcp_m
        pos_c, rot_c = self.ik.compute_forward_kinematics(self.ik_frame, self.cmd)
        tcp_cmd = np.asarray(pos_c) + np.asarray(rot_c)[:, 2] * self.tcp_m
        return {"q_track_err_deg": [round(float(v), 4) for v in np.degrees(q - self.cmd)],
                "fk_measured_vs_target_mm": round(float(np.linalg.norm(tcp_fk - p_target)) * 1000, 4),
                "fk_cmd_vs_target_mm": round(float(np.linalg.norm(tcp_cmd - p_target)) * 1000, 4),
                "physics_vs_fk_mm": round(float(np.linalg.norm(self.tcp_measured() - tcp_fk)) * 1000, 4)}

    def hold_g(self, seconds):
        t0 = self.t
        while self.t - t0 < seconds - 1e-9:
            yield

    def hold(self, seconds):
        return self._run_g(self.hold_g(seconds=seconds))

    # ---------- 放件臂（M4 ②） ----------
    def _hpose(self, pose: dict) -> dict:
        """放件臂程序位姿（放件臂基座系）→ 世界系，供 movel 使用。"""
        return {"xyz_mm": self.H.apply(pose["xyz_mm"]), "rpy": pose["rpy"],
                "yaw_offset_deg": self.H.yaw_deg() + pose.get("yaw_offset_deg", 0.0)}

    def _part_local(self, pid, W: np.ndarray):
        """零件世界位姿 W（4×4，m）→ 治具下的局部位姿（零件只绕 z 转）。"""
        L = np.linalg.inv(self.FXW) @ W
        yaw = math.atan2(L[1, 0], L[0, 0])
        XformPrim(self._part_path(pid)).set_local_poses(translations=[L[:3, 3].tolist()],
                                                        orientations=[[math.cos(yaw / 2), 0.0, 0.0, math.sin(yaw / 2)]])

    def _tray_world(self, dy_mm: float) -> np.ndarray:
        st = self.hd["tray_stop"]
        T = Pose.from_xyz_rpy([st["xyz_mm"][0], st["xyz_mm"][1] + dy_mm, st["xyz_mm"][2]], (0.0, 0.0, st["yaw_deg"]))
        return pose_mat(T.R, [v / 1000.0 for v in T.t])

    def _unit_paths(self, k):
        pp = f"{self.fixture_path}/{prim_name(self.station['product'])}" + ("" if k == 1 else f"_u{k}")
        return pp, (f"{ROOT}/screws" if k == 1 else f"{ROOT}/screws_u{k}")

    def _tray_of(self, k) -> str:
        """第 k 台用哪个料盘：A / B 轮换。"""
        return f"{ROOT}/{prim_name(self.by_role['tray'])}" + ("" if k % 2 == 1 else "_B")

    def _tray_place(self, k, dy_mm, dz_mm=0.0, skip=()):
        """第 k 台的料盘与盘上零件一起放到 停止位 + dy（沿输送方向）+ dz。"""
        TW = self._tray_world(dy_mm)
        TW[2, 3] += dz_mm / 1000.0
        XformPrim(self._tray_of(k)).set_world_poses(positions=[TW[:3, 3].tolist()], orientations=[rot_to_quat(TW[:3, :3]).tolist()])
        pp, _ = self._unit_paths(k)
        Fi = np.linalg.inv(self.FXW)
        for it in self.hd["items"]:
            if it["part"] in skip:
                continue
            P = Pose.from_xyz_rpy(it["part_in_tray"]["xyz"], (0.0, 0.0, it["part_in_tray"]["yaw_deg"]))
            L = Fi @ TW @ pose_mat(P.R, [v / 1000.0 for v in P.t])
            yaw = math.atan2(L[1, 0], L[0, 0])
            XformPrim(f"{pp}/parts/{prim_name(it['part'])}").set_local_poses(
                translations=[L[:3, 3].tolist()], orientations=[[math.cos(yaw / 2), 0.0, 0.0, math.sin(yaw / 2)]])

    def _tray_at(self, dy_mm: float):
        """当前这台的料盘与盘上还没被取走的零件一起放到 停止位 + dy。"""
        taken = {r["part"] for r in self.placed} | ({self.carry["part"]} if self.carry else set())
        self._tray_place(self.unit, dy_mm, skip=taken)

    def _tray_arrive_g(self):
        """输送线把料盘从上游送到放件位（travel_s，demo 占位）。"""
        if self.tray_arrived:
            return
        self.caption(f"输送线送来配套料盘：{len(self.hd['items'])} 个零件（{self.tray_mod['name']}）")
        dy0, n = SCENE["tray_entry_dy"], max(1, round(self.tray_mod["travel_s"] / DT))
        lp = SCENE["line"]
        nxt = self.unit + 1 if self.unit + 1 <= getattr(self, "line_n", 1) else None
        for k in range(1, n + 1):
            f = k / n
            sm = 3 * f * f - 2 * f ** 3   # 起停平滑
            self._tray_at(dy0 * (1 - sm))
            if nxt:   # 同时下一盘从配套工站出来，到排队位等
                self._tray_place(nxt, lp["tray_station_dy"] + (lp["tray_prestop_dy"] - lp["tray_station_dy"]) * sm)
            yield
        self.tray_arrived = True
        self.gateway.append({"t_s": round(self.t, 3), "source": self.by_role["conveyor"], "via": self.feeder_mod["mes"]["via"],
                             "event": "tray_arrived", "tray": self.by_role["tray"], "parts": [it["part"] for it in self.hd["items"]]})

    def _tray_arrive(self):
        return self._run_g(self._tray_arrive_g())

    def _tcp_world(self, rig) -> np.ndarray:
        p, o = rig.links.get_world_poses(indices=[UR_LINKS.index("wrist_3_link")])
        R = quat_to_rot(o.numpy()[0])
        return pose_mat(R, p.numpy()[0] + R[:, 2] * rig.tcp_m)

    def _update_carry(self):
        if not self.carry:
            return
        T = self._tcp_world(self.rigs["handler"])
        if self.carry["part"] == "UNIT":
            for path, rel in self.carry["group"]:
                W = T @ rel
                XformPrim(path).set_world_poses(positions=[W[:3, 3].tolist()], orientations=[rot_to_quat(W[:3, :3]).tolist()])
        else:
            self._part_local(self.carry["part"], T @ self.carry["rel"])

    def _unit_group(self):
        return [self.product_path, self.screws_root]

    def _world(self, path) -> np.ndarray:
        p, q = XformPrim(path).get_world_poses()
        return pose_mat(quat_to_rot(q.numpy()[0]), p.numpy()[0])

    def _flange_world(self) -> np.ndarray:
        p, o = self.rigs["handler"].links.get_world_poses(indices=[UR_LINKS.index("wrist_3_link")])
        return pose_mat(quat_to_rot(o.numpy()[0]), p.numpy()[0])

    def _update_tools(self):
        """接上的工具贴着法兰（工具坐标系 = 对接面，与 wrist_3_link 重合）；其余停在快换架上。"""
        F = self._flange_world()
        for tid, t in self.tools.items():
            W = F if tid == self.tool_on else t["dock"]
            XformPrim(t["path"]).set_world_poses(positions=[W[:3, 3].tolist()], orientations=[rot_to_quat(W[:3, :3]).tolist()])

    def _set_angle(self, t, deg):
        """按姿态表插值摆各连杆（表为空场景里物理驱动录下的，每 1°一行）。"""
        deg = min(45.0, max(0.0, deg))
        i = min(44, int(deg))
        f = deg - i
        a, b = t["table"][i]["links"], t["table"][i + 1]["links"]
        tr, qs = [], []
        for n in t["link_names"]:
            pa, pb = np.asarray(a[n]["t"]), np.asarray(b[n]["t"])
            qa, qb = np.asarray(a[n]["q"]), np.asarray(b[n]["q"])
            if np.dot(qa, qb) < 0:
                qb = -qb
            q = qa + (qb - qa) * f
            tr.append((pa + (pb - pa) * f).tolist())
            qs.append((q / np.linalg.norm(q)).tolist())
        t["links_x"].set_local_poses(translations=tr, orientations=qs)
        t["angle"] = deg

    def _finger_to_g(self, deg, stop_at=None):
        """手指以 45° / cycle_s 的速度转到 deg；stop_at 为几何接触角（外夹：两指内侧间距 = 外径；内撑：外侧跨度 = 内孔），
        越过它就停在接触角（运动学夹爪，接触为几何判定，用户 2026-10-25）。"""
        if not self.tool_on or self.tools[self.tool_on]["kind"] != "2f":
            return {"finger_deg": None, "contact": None, "arrived": False, "reason": "夹爪没有接上"}
        t = self.tools[self.tool_on]
        a0, t0 = t["angle"], self.t
        rate = 45.0 / t["mod"]["cycle_s"]
        end = deg
        if stop_at is not None and (a0 - stop_at) * (deg - stop_at) <= 0:
            end = stop_at
        n = max(1, math.ceil(abs(end - a0) / (rate * DT)))
        for k in range(1, n + 1):
            self._set_angle(t, a0 + (end - a0) * k / n)
            yield
        return {"finger_deg": round(t["angle"], 3), "contact": stop_at is not None and end == stop_at, "s": round(self.t - t0, 3)}

    def _finger_to(self, deg, stop_at=None):
        return self._run_g(self._finger_to_g(deg=deg, stop_at=stop_at))

    def _grip_g(self, pid: str, on: bool, o: dict | None = None):
        o = o or {}
        if not self.tool_on:
            return {"arrived": False, "reason": f"没有接上工具，取放 {pid} 跳过"}
        t = self.tools[self.tool_on]
        if pid == "UNIT":   # 下线：吸起整件（产品与已拧的螺钉一起）
            (yield from self.hold_g(t["mod"]["cycle_s"]))
            T = self._tcp_world(self.rigs["handler"])
            if on:
                self.carry = {"part": "UNIT", "group": [(pth, np.linalg.inv(T) @ self._world(pth)) for pth in self._unit_group()]}
                self.caption(f"放件臂吸起第 {self.unit} 台整件（{self.sn}）")
                return {"unit": self.unit}
            self.carry = None
            self.gateway.append({"t_s": round(self.t, 3), "source": self.station["station_id"], "via": self.feeder_mod["mes"]["via"],
                                 "sn": self.sn, "event": "unit_unloaded", "unit": self.unit})
            return {"unit": self.unit}
        if on:
            res = {}
            if t["kind"] == "2f":
                # 外夹合拢、内撑张开，到几何接触角停下并锁定零件
                r = (yield from self._finger_to_g(45.0 if o.get("mode") == "external" else 0.0, stop_at=o.get("grip_deg")))
                res = {"grip_mode": o.get("mode"), "finger_deg": r["finger_deg"], "plan_deg": o.get("grip_deg"),
                       "contact": bool(r["contact"]), "contact_source": "geometry"}
            else:
                (yield from self.hold_g(t["mod"]["cycle_s"]))
            p, q = XformPrim(self._part_path(pid)).get_world_poses()
            W = pose_mat(quat_to_rot(q.numpy()[0]), p.numpy()[0])
            self.carry = {"part": pid, "rel": np.linalg.inv(self._tcp_world(self.rigs["handler"])) @ W}
            self.picked = getattr(self, "picked", 0) + 1
            return res
        self.snap(f"{self.sn}_place_{pid}", place_view=pid)   # 放到位、松开前的近景（--snap）
        if t["kind"] == "2f":   # 松开：手指回到预置角，再解除锁定
            (yield from self._finger_to_g(o.get("preset_deg", 0.0)))
        else:
            (yield from self.hold_g(t["mod"]["cycle_s"]))
        # 松开：记下放到位的偏差，再把零件落到装配位姿（偏差在报告里，不掩盖）
        part = self.parts[pid]
        pose = part["pose_in_fixture"]
        p, _ = XformPrim(self._part_path(pid)).get_world_poses()
        target = (self.FXW @ np.array([*[v / 1000.0 for v in pose["xyz"]], 1.0]))[:3]
        err = float(np.linalg.norm(p.numpy()[0] - target)) * 1000.0
        self.carry = None
        self.grip_ignore = {pid}   # 抬起离开前手指可能还挨着它
        x, y, z = (v / 1000.0 for v in pose["xyz"])
        h = math.radians(pose["rpy"][2]) / 2
        XformPrim(self._part_path(pid)).set_local_poses(translations=[[x, y, z]], orientations=[[math.cos(h), 0.0, 0.0, math.sin(h)]])
        self.pending_collide.append(pid)   # 工具抬起后再开碰撞
        it = next(i for i in self.hd["items"] if i["part"] == pid)
        st = next(x for x in self.assembly["steps"] if x["id"] == it["step"])
        rec = {"step": st["id"], "part": pid, "actor": st["actor"], "by": "handler_arm", "actor_assumed": it["actor_assumed"],
               "tool": self.tool_on, "grasp": it["grasp"]["mode"],
               "t_s": round(self.t, 3), "duration_s": None, "duration_source": "measured", "place_err_mm": round(err, 4)}
        self.placed.append(rec)
        self.caption(f"放件臂放 {part['name']}（{it['grasp']['rule'] if t['kind'] == '2f' else '吸盘'}）"
                     + ("，SOP 执行者缺失，demo 交给放件臂" if it["actor_assumed"] else ""))
        self.gateway.append({"t_s": round(self.t, 3), "source": self.station["station_id"], "via": self.feeder_mod["mes"]["via"],
                             "event": "part_placed", **{k: rec[k] for k in ("step", "part", "actor", "by", "duration_source")}})
        return {"place_err_mm": round(err, 4)}

    def _grip(self, pid: str, on: bool, o: dict | None = None):
        return self._run_g(self._grip_g(pid=pid, on=on, o=o))

    def _run_handler_g(self, need: int, to_end: bool = False):
        """拧紧臂已在停靠位并给出区域空闲：执行放件臂程序，直到放件计数 ≥ need。两臂不并行。"""
        t0 = self.t
        self.h_need = max(getattr(self, "h_need", 0), need)
        self.rig = self.rigs["handler"]
        try:
            while (to_end or self.h_count < need) and self.h_i < len(self.h_ops):
                o = self.h_ops[self.h_i]
                if o.get("phase") == "unload" and not to_end:
                    break
                self.h_i += 1
                s0 = self.t
                sg = {"i": self.h_i - 1, "op": o["op"], "part": o.get("part")}
                if o["op"] == "movej":
                    sg.update((yield from self.movej_g(o["joints_deg"])))
                elif o["op"] == "movel":
                    sg["speed"] = o["speed"]
                    if "tcp_mm" in o:
                        self.rig.tcp_m = o["tcp_mm"] / 1000.0
                    if self.record_dir and o.get("leg") == "place" and self.view != "place":
                        self.set_view("place")
                    dock = "tool_change" in o
                    sg.update({k: v for k, v in (yield from self.movel_g(self._hpose(o["pose"]), o["speed"], tol_mm=1.0 if dock else ARRIVE_TOL_MM,
                                                           fine=not dock)).items()
                               if k in ("arrived", "err_mm", "ik_ok", "reason", "path_dev_mm")})
                    if o["speed"] == "fast" and not self.carry:
                        for pid in self.pending_collide:
                            self._collide(pid, True)
                        self.pending_collide = []
                        if o.get("part"):   # 松开后抬起：不再忽略与它的接触
                            self.grip_ignore = set()
                        if self.record_dir and self.view == "place":
                            self.set_view("main")
                elif o["op"] == "grip":
                    sg.update((yield from self._grip_g(o["part"], o["value"], o)) or {})
                elif o["op"] == "gripper":
                    sg.update((yield from self._finger_to_g(o["angle_deg"])))
                elif o["op"] == "tool":
                    t = self.tools[o["tool"]]
                    # 只有法兰正对快换工位（< 2 mm）才切换约束；否则不接，记失败（避免约束把夹爪硬拽过去）
                    d = t["dock"] @ np.eye(4)
                    gap = float(np.linalg.norm(self._flange_world()[:3, 3] - d[:3, 3])) * 1000.0
                    sg["dock_gap_mm"] = round(gap, 3)
                    if gap > 2.0:
                        sg["arrived"] = False
                        sg["reason"] = f"法兰离快换工位 {gap:.1f} mm，未{'放下' if o['action'] == 'release' else '接上'}"
                        self.h_segments.append(sg)
                        continue
                    (yield from self.hold_g(self.modules[self.hd["tool_changer"]]["change_s"]))
                    self.snap(f"{self.sn}_tool_{o['action']}_{o['tool']}", at_mm=(d[:3, 3] * 1000.0 - np.array([0.0, 0.0, 60.0])).tolist())
                    if o["action"] == "release":
                        self.tool_on = None
                    else:
                        self.tool_on = o["tool"]
                        self.caption(f"快换：接上 {self.tools[o['tool']]['mod']['name']}")
                    sg["tool"] = o["tool"]
                elif o["op"] == "wait" and o["until"] == "zone_free":
                    while getattr(self, "parallel", False) and not self.zone_free:
                        yield
                elif o["op"] == "wait" and o["until"] == "tray_ready":
                    if getattr(self, "parallel", False):   # 输送线任务在送料盘，这里只等到位
                        while not self.tray_arrived:
                            yield
                    else:
                        (yield from self._tray_arrive_g())
                elif o["op"] == "signal" and o["name"] == "placed_count":
                    self.h_count = o["value"]
                elif o["op"] == "signal" and o["name"] == "zone_left":   # 中途换手：已到治具区域外 / 重新进入
                    self.zone_left = bool(o["value"])
                elif o["op"] == "wait" and o["until"] == "unit_done":
                    pass   # 拧紧臂已在停靠位并给出整件完成
                sg["measured_s"] = round(self.t - s0, 3)
                self.h_segments.append(sg)
                print(f"TWIN h_op={sg['i']:3d} {o['op']:<7} {str(o.get('part') or o.get('tool') or ''):<8} {sg['measured_s']:6.3f}s "
                      f"{'' if 'arrived' not in sg else ('OK' if sg['arrived'] else 'NOT-ARRIVED ' + str(sg.get('reason', '')))} "
                      f"{sg.get('err_mm', '')}{' finger=' + str(sg['finger_deg']) if 'finger_deg' in sg else ''}", flush=True)
                if sg.get("arrived") is False:
                    print(f"TWIN handler op={sg['i']} {o['op']} {o.get('part')} NOT-ARRIVED {sg.get('reason', '')}")
        finally:
            self.rig = self.rigs["fastener"]
        self.h_time += self.t - t0
        return {"handler_s": round(self.t - t0, 3), "placed_count": self.h_count}

    def _run_handler(self, need: int, to_end: bool = False):
        return self._run_g(self._run_handler_g(need=need, to_end=to_end))

    # ---------- 生成器调度（两臂并行，用户 2026-10-26） ----------
    def _run_g(self, g):
        """一口气跑完一个生成器：每次 yield 推进一个物理步。"""
        while True:
            try:
                next(g)
            except StopIteration as e:
                return e.value
            self.step()

    def _run_parallel(self):
        """两台臂各一条生成器，每个物理步各推进一步；靠 zone_free / placed_count / unit_done 协调。"""
        self.zone_free, self.unit_done_flag, self.zone_left = False, False, False
        tasks = [["fastener", self.run_unit_g()], ["handler", self._run_handler_g(len(self.hd["items"]))]]
        if not self.tray_arrived:   # 料盘由输送线自己送进来，与放件臂接工具同时进行
            tasks.append(["conveyor", self._tray_arrive_g()])
        tasks += getattr(self, "bg_tasks", [])   # 连续流动：换盘、送出上一台
        self.bg_tasks = []
        result = None
        while tasks:
            for task in list(tasks):
                self.rig = self.rigs.get(task[0], self.rigs["fastener"])
                try:
                    next(task[1])
                except StopIteration as e:
                    tasks.remove(task)
                    if task[0] == "fastener":
                        result = e.value
            self.rig = self.rigs["fastener"]
            if tasks:
                self.step()
        return result

    # ---------- 开始 / 结束 ----------
    def start(self):
        """play，瞬移到 home，清掉瞬移产生的接触。"""
        home = next(o["joints_deg"] for o in self.ops if o["op"] == "movej")
        self.op_index = -1
        SimulationManager.set_physics_dt(DT)
        # 显式用 CPU 物理（本机实测 2026-10-25）：默认状态设备报 cpu、GPU 动力学却开着，场景里同时有 UR5e 与带 mimic 关节的
        # 2F-85 时 Kit update 随机卡死；设成 cpu 后完整场景开合 600 步无卡顿，每步还更快
        SimulationManager.set_device("cpu")
        app_utils.play()
        self.app.update()
        for r in self.rigs.values():
            r.vmax = r.arm.get_dof_max_velocities().numpy()[0].astype(float)   # rad/s
        print(f"TWIN joint vmax deg/s {np.round(np.degrees(self.rig.vmax), 1).tolist()}", flush=True)
        self.arm.set_dof_positions(np.radians(home)[None, :])
        self.cmd = np.radians(home)
        self.arm.set_dof_position_targets(self.cmd[None, :])
        if self.hd:
            hr = self.rigs["handler"]
            hh = next(o["joints_deg"] for o in self.h_ops if o["op"] == "movej")
            hr.arm.set_dof_positions(np.radians(hh)[None, :])
            hr.cmd = np.radians(hh)
            hr.arm.set_dof_position_targets(hr.cmd[None, :])
            p, o = XformPrim(self.fixture_path).get_world_poses()
            self.FXW = pose_mat(quat_to_rot(o.numpy()[0]), p.numpy()[0])
            self._tray_at(SCENE["tray_entry_dy"])
            if getattr(self, "line_n", 1) >= 2:   # 第 2 台的配件还在配套工站里
                self._tray_place(2, SCENE["line"]["tray_station_dy"])
            st_ = stage_utils.get_current_stage(backend="usd")
            tex = st_.GetPrimAtPath("/World/Looks/label_line_screen/tex")
            if tex and getattr(self, "line_n", 1) > 1:   # 场景大屏看板
                from pxr import UsdShade
                self.screen_tex = UsdShade.Shader(tex).GetInput("file")
                self.screen_png = TWIN_DIR / "labels" / "line_screen_live.png"
                self._update_screen()
            for t in self.tools.values():
                if t["kind"] == "2f":
                    self._set_angle(t, 0.0)
            self._update_tools()
        for st in self.assembly["steps"]:   # 上游工站已装好的零件开始时就在位
            if st["type"] == "place" and st["actor"] == "upstream" and st["id"] not in {r["step"] for r in self.placed}:
                self.place(st, animate=False)
        for _ in range(10):
            self.step()
        self.collisions.clear()  # 起始姿态瞬移产生的接触不计

    def stop(self):
        app_utils.stop()

    # ---------- 一整件 ----------
    def run(self) -> dict:
        self.start()
        report = self.run_unit()
        self.stop()
        return report

    def run_line(self, units: int) -> list[dict]:
        """流水线（用户 2026-10-25 / 26）：连续流动。第 k 台：放件 → 拧紧 → 吸起整件放到下游托盘；
        同时：第 k 台的料盘一取空就下沉回流、排队的第 k+1 盘补进来、回流的料盘装上第 k+2 台配件再出来排队；
        第 k 台的成品沿输送线送进下游缓存位，与第 k+1 台的装配同时进行。每台单独出报告（序列号 <sn>-<k>）。"""
        base_sn = self.sn
        self.line_n = units
        self.start()
        reports, bg = [], []
        for k in range(1, units + 1):
            if k > 1:
                self._switch_unit(k, f"{base_sn}-{k}")
            else:
                self.sn = f"{base_sn}-1"
                self.fault_plan = self._draw_fault_plan()
                self.picked = 0
            self.caption(f"第 {k} 台开工 · 序列号 {self.sn}")
            self.gateway.append({"t_s": round(self.t, 3), "source": self.station["station_id"], "event": "unit_start", "sn": self.sn, "unit": k})
            self.bg_tasks = bg + ([["conveyor_exchange", self._exchange_g(k)]] if k < units else [])
            t0 = self.t
            rep_ = self.run_unit()
            u0 = self.t
            self._run_handler(0, to_end=True)   # 下线：吸起整件放到下游托盘、工具放回快换架
            self.gateway.append({"t_s": round(self.t, 3), "source": self.station["station_id"], "event": "unit_result", "sn": self.sn,
                                 "unit": k, "result": rep_["unit_result"]})
            bg = [["conveyor_out", self._send_out_g(k)]]
            rep_["line"] = {"unit": k, "sn": self.sn, "t_start_s": round(t0, 2), "unit_s": round(u0 - t0, 2),
                            "unload_s": round(self.t - u0, 2), "cycle_s": round(self.t - t0, 2)}
            reports.append(rep_)
        for _, g in bg:   # 最后一台送进下游缓存
            self._run_g(g)
        for i, r in enumerate(reports):   # 连续流动下的节拍：相邻两台开工时间差
            r["line"]["interval_s"] = round(reports[i + 1]["line"]["t_start_s"] - r["line"]["t_start_s"], 2) if i + 1 < len(reports) else None
        self.stop()
        return reports

    def _exchange_g(self, k):
        """第 k 台的料盘取空后：下沉到回流线；排队的第 k+1 盘送到放件位；回流的料盘装上第 k+2 台配件，从配套工站出来到排队位。"""
        lp = SCENE["line"]
        while getattr(self, "picked", 0) < len(self.hd["items"]):
            yield
        self.caption("料盘取空：下沉到回流线送回配套工站；排队的下一盘补进来")
        n = max(1, round(lp["exchange_s"] / DT))
        for i in range(1, n // 2 + 1):
            self._tray_place(k, 0.0, dz_mm=-lp["return_drop_mm"] * i / (n // 2), skip=[it["part"] for it in self.hd["items"]])
            yield
        self._visible(self._tray_of(k), False)
        for i in range(1, n + 1):
            f = i / n
            self._tray_place(k + 1, lp["tray_prestop_dy"] * (1 - (3 * f * f - 2 * f ** 3)))
            yield
        self.next_tray_ready = True
        self.gateway.append({"t_s": round(self.t, 3), "source": self.by_role["conveyor"], "event": "tray_arrived", "unit": k + 1})
        if k + 2 <= getattr(self, "line_n", 1):
            self._visible(self._tray_of(k), True)
            pp, _ = self._unit_paths(k + 2)
            for it in self.hd["items"]:
                self._visible(f"{pp}/parts/{prim_name(it['part'])}", True)
            for i in range(1, n + 1):
                f = i / n
                self._tray_place(k + 2, lp["tray_station_dy"] + (lp["tray_prestop_dy"] - lp["tray_station_dy"]) * (3 * f * f - 2 * f ** 3))
                yield

    def _send_out_g(self, k):
        """第 k 台的成品连同下线托盘沿输送线送进下游缓存位并留在那里；托盘离开后，第 k+1 个下线托盘出现在出件位。"""
        from jsdemo.layout import current
        OUT_STOP = current()["OUT_STOP"]

        lp = SCENE["line"]
        pal = f"{ROOT}/out_pallet_{k}"
        pp, sr = self._unit_paths(k)
        GeomPrim(paths=[f"{pp}/parts/{prim_name(p_['id'])}/s{i}" for p_ in self.assembly["parts"] for i in range(len(p_["shapes"]))]
                 ).set_enabled_collisions([False])
        starts = {pth: self._world(pth) for pth in (pal, pp, sr)}
        dy = (lp["buffer_y"][(k - 1) % len(lp["buffer_y"])] - OUT_STOP["xyz_mm"][1]) / 1000.0
        n = max(1, round(max(1.5, abs(dy) / 0.25) / DT))
        self.caption(f"输送线把第 {k} 台送进下游缓存")
        for i in range(1, n + 1):
            f = i / n
            for pth, W in starts.items():
                W2 = W.copy()
                W2[1, 3] += dy * (3 * f * f - 2 * f ** 3)
                XformPrim(pth).set_world_poses(positions=[W2[:3, 3].tolist()], orientations=[rot_to_quat(W2[:3, :3]).tolist()])
            yield
        self.gateway.append({"t_s": round(self.t, 3), "source": self.by_role["conveyor"], "event": "unit_delivered", "unit": k})
        if k + 1 <= getattr(self, "line_n", 1):
            self._visible(f"{ROOT}/out_pallet_{k + 1}", True)

    def _switch_unit(self, k: int, sn: str):
        """换到第 k 台（料盘已由 _exchange_g 送到放件位）：各类每台状态清零。"""
        self.unit, self.sn = k, sn
        self.product_path, self.screws_root = self._unit_paths(k)
        self.screw_prims = {pid: XformPrim(f"{self.screws_root}/{pid}") for pid in self.screw_geo}
        self.placed, self.issues = [], []
        self.h_i, self.h_count, self.picked = 0, 0, 0
        self.tray_arrived = True
        self.h_segments, self.h_time = [], 0.0
        self.recorded_groups, self.pending_snap = set(), None
        self.fault_plan = self._draw_fault_plan()
        if getattr(self, "_dfm_issues", None):
            self.apply_dfm_fix(self._dfm_issues)

    def run_unit_g(self):
        """按 self.ops 跑一整件（仿真已在运行）。"""
        process = json.loads((STATION_DIR / "report.json").read_text(encoding="utf-8"))
        windows = {w["id"]: w for w in process["stages"]["process"]["windows"].values() if "id" in w}
        self.windows = windows
        self.dfm_open = process["stages"]["dfm"]["issues"]
        flow_steps = {s["point"]: s for s in load_yaml(STATION_DIR / "flow.yaml")["sequence"]}

        self.collisions.clear()
        pending = None
        t_start = self.t
        segments, results, arrived, fed = [], [], {}, set()
        self.feeding = set()
        self.has_feed = any(o.get("purpose") == "feed" for o in self.ops)
        stop_unit = False
        for i, o in enumerate(self.ops):
            self.op_index = i
            seg_t0 = self.t
            seg = {"i": i, "op": o["op"], "point": o.get("point")}
            if o["op"] == "movej":
                seg.update((yield from self.movej_g(o["joints_deg"])))
            elif o["op"] == "movel":
                pt = o["point"]
                feed_op = o.get("purpose") == "feed"
                if pt not in fed:
                    (yield from self._place_until_g(pt))
                if pt not in fed and feed_op and pt not in self.feeding:   # 取钉（DEC-FEED-001）：出钉位上出现螺钉
                    self.feeding.add(pt)
                    self.feed_start(pt)
                    self._present_at_nest(pt, i)
                    lane = o.get("lane")
                    if self.record_dir and lane not in self.recorded_lanes:   # 每路第一次取钉切近景
                        self.recorded_lanes.add(lane)
                        pick = next(x for x in self.ops[i:] if x.get("purpose") == "feed" and x["speed"] == "slow" and x["point"] == pt)
                        self.set_view(("nest", pick["pose"]["xyz_mm"]))
                        self.caption(f"{pt} 去第 {lane} 路出钉位取钉")
                elif pt not in fed and not feed_op and not self.has_feed:   # 旧程序（吹气送钉）：去该点的路上供钉
                    self.feed_start(pt)
                seg["speed"] = o["speed"]
                if feed_op:
                    seg["purpose"] = "feed"
                seg.update((yield from self.movel_g(o["pose"], o["speed"], tol_mm=FEED_TOL_MM if feed_op else ARRIVE_TOL_MM)))
                if self.pending_snap and o["speed"] == "fast":
                    self.snap(*self.pending_snap)
                    self.pending_snap = None
                if pt not in fed and (not self.has_feed or (feed_op and o["speed"] == "slow")):
                    fed.add(pt)   # 批头落到出钉位上吸住螺钉（或旧程序到了接近位）
                    seg["feed_ok"] = self.feed_done(pt)
                if o["speed"] == "slow" and not feed_op:  # 下压到 engage 位
                    # 孔位误差：实测 TCP 与「治具实际位姿下的孔位」之差（治具放偏而未标定时它会大于到位误差）
                    hole = np.asarray(self.fixture_true.apply(flow_steps[pt]["engage"]["xyz"])) / 1000.0
                    seg["hole_err_mm"] = round(float(np.linalg.norm(self.tcp_measured() - hole)) * 1000.0, 4)
                    arrived[pt] = {"arrived": seg["arrived"], "err_mm": seg.get("err_mm"), "hole_err_mm": seg["hole_err_mm"]}
            elif o["op"] == "hold":
                # 执行器第 1 次拧：hold 时长仍是 fasten_s_base，期间播放拧入动画
                pt = o["point"]
                mode = self.fault_plan[pt][0]
                gid = self.point_group[pt]["id"]
                if self.record_dir and (gid not in self.recorded_groups or mode != "ok"):   # 每组第一颗与异常点切近景
                    self.recorded_groups.add(gid)
                    self.set_view(("point", pt))
                rows, curve = self._attempt_curve(pt, 1, mode, flow_steps[pt])
                pending = {"point": pt, "rows": rows, "curve": curve, "mode_planned": mode}
                if self.active and self.active["point"] == pt:
                    (yield from self._drive_g(pt, rows, self.exec_mod["fasten_s_base"]))
                else:
                    (yield from self.hold_g(self.exec_mod["fasten_s_base"]))
                seg["hold_s"] = self.exec_mod["fasten_s_base"]
            elif o["op"] == "wait" and o.get("until") == "feed_ok":
                pass   # 取钉确认：feed_done 已在批头到出钉位时完成
            elif o["op"] == "wait" and o.get("until") == "placed_count":
                if getattr(self, "parallel", False):   # 两臂并行：放件臂自己在跑，这里只等计数
                    while self.h_count < o["value"]:
                        yield
                    seg["waited_s"] = round(self.t - seg_t0, 3)
                else:
                    seg.update((yield from self._run_handler_g(o["value"])))
            elif o["op"] == "signal" and o.get("name") == "zone_free":
                self.zone_free = bool(o["value"])
            elif o["op"] == "wait" and o.get("until") == "zone_left":
                # 放件臂中途换手、离开治具区域：拧紧臂趁这段去取下一颗螺钉（用户 2026-10-26）。串行时放件臂不动，直接取
                while getattr(self, "parallel", False) and not getattr(self, "zone_left", False):
                    yield
                seg["waited_s"] = round(self.t - seg_t0, 3)
            elif o["op"] == "signal" and o.get("name") == "unit_done":
                self.unit_done_flag = True
            elif o["op"] == "wait":
                # 执行器判定；DEC-NG-001：首次不合格且模式在 executor_retry.on_modes 内时执行器自行反转、重拧一次，机械臂继续等
                pt = pending["point"]
                step = flow_steps[pt]
                g = self.point_group[pt]
                w = windows[step["executor_params"]]
                spec = self.lib["screws"][g["screw"]]
                attempts = []
                rows, curve = pending["rows"], pending["curve"]
                retry = step.get("executor_retry", {"max": 0, "on_modes": []})
                for k in range(1, 2 + retry["max"]):
                    if k > 1:
                        mode = self.fault_plan[pt][k - 1]
                        self._andon("andon_retry")
                        self.caption(f"{pt} {procmodel.MODE_CN.get(attempts[-1]['mode'], attempts[-1]['mode'])} → 执行器自动反转复拧")
                        self.gateway.append({"t_s": round(self.t, 3), "source": self.by_role["executor"], "via": self.feeder_mod["mes"]["via"],
                                             "sn": self.sn, "point": pt, "event": "exec_retry", "attempt": k, "previous_mode": attempts[-1]["mode"]})
                        self.snap(f"{self.sn}_{pt}_a{k - 1}_{attempts[-1]['mode']}", pt)
                        # 反转退出：时长 = 已拧入角度 / 反转转速 + 停顿（executor_retry.reverse，demo 占位参数）
                        rv = retry.get("reverse", {})
                        rpm = rv.get("reverse_rpm", 300)
                        (yield from self._drive_g(pt, pending["rows"], pending["rows"][-1][0] / (rpm * 6.0), reverse=True))
                        (yield from self.hold_g(rv.get("dwell_s", 0.0)))
                        rows, curve = self._attempt_curve(pt, k, mode, step)
                        (yield from self._drive_g(pt, rows, self.exec_mod["fasten_s_base"]))
                        pending["rows"] = rows
                    r = procmodel.judge(rows, w["window_nm"], spec, g["depth_mm"])
                    attempts.append({"attempt": k, "planned": self.fault_plan[pt][k - 1], "result": r["result"], "mode": r["mode"],
                                     "final_nm": r.get("final_nm"), "total_angle_deg": r.get("total_angle_deg"),
                                     "curve_file": curve.relative_to(STATION_DIR.parent.parent).as_posix()})
                    if r["result"] == "OK" or r["mode"] not in retry["on_modes"] or k > retry["max"]:
                        break
                res = {"point": pt, "params_id": step["executor_params"], **r, "curve_file": attempts[-1]["curve_file"],
                       "attempts": len(attempts), "attempt_log": attempts}
                results.append(res)
                self._mark(pt, "hole_ok" if r["result"] == "OK" else "hole_ng")
                self._andon("andon_ok" if r["result"] == "OK" and all(x["result"] == "OK" for x in results) else "andon_ng")
                self.gateway.append({"t_s": round(self.t, 3), "source": self.by_role["executor"], "via": self.feeder_mod["mes"]["via"],
                                     "sn": self.sn, "station": self.station["station_id"], "point": pt,
                                     "params_id": res["params_id"], "final_nm": res.get("final_nm"),
                                     "total_angle_deg": res.get("total_angle_deg"), "result": res["result"], "mode": res["mode"],
                                     "attempts": len(attempts), "curve_file": res["curve_file"]})
                first_of_group = g["id"] not in self.snapped_groups
                if r["result"] != "OK" or len(attempts) > 1 or first_of_group:
                    self.snapped_groups.add(g["id"])
                    # 等机械臂退回后再拍，否则螺钉被批头挡住
                    self.pending_snap = (f"{self.sn}_{pt}_{'OK' if r['result'] == 'OK' else r['mode']}", pt)
                self._release_screw()
                g_name = self.point_group[pt].get("name", self.point_group[pt]["id"])
                self.caption(f"{pt}（{g_name}）{'合格' if r['result'] == 'OK' else '不合格：' + procmodel.MODE_CN.get(r['mode'], r['mode'])}"
                             f"  {r.get('final_nm')} N·m" + ("  复拧后" if len(attempts) > 1 else ""))
                if self.view not in (None, "main"):
                    self.set_view("main")
            elif o["op"] == "branch_on_fault":
                last = results[-1] if results else None
                if last and last["result"] != "OK":
                    stop_unit = "stop_unit" in o["policy"]
                    issue = self._issue(last["point"], last["attempt_log"], flow_steps[last["point"]], stop_unit)
                    self.caption(f"记录问题 {issue['issue_id']}：{issue['mode_cn']}，{issue['unit_disposition']}")
                    self.gateway.append({"t_s": round(self.t, 3), "source": self.station["station_id"], "via": self.feeder_mod["mes"]["via"],
                                         "sn": self.sn, "event": "issue_opened", "issue_id": issue["issue_id"], "point": last["point"],
                                         "mode": issue["mode"], "unit_disposition": issue["unit_disposition"]})
            seg["measured_s"] = round(self.t - seg_t0, 3)
            segments.append(seg)
            print(f"TWIN op={i:3d} {o['op']:<15} {str(o.get('point') or ''):<3} {seg['measured_s']:6.3f}s "
                  f"{'' if 'arrived' not in seg else ('OK' if seg['arrived'] else 'NOT-ARRIVED')} {seg.get('err_mm', '')}"
                  f"{'' if 'hole_err_mm' not in seg else ' hole=' + str(seg['hole_err_mm'])}")
            if stop_unit:
                for m in o.get("on_stop", []):   # 退回、回原位，再停
                    self.op_index = i
                    r = (yield from self.movel_g(m["pose"], m["speed"])) if m["op"] == "movel" else (yield from self.movej_g(m["joints_deg"]))
                    if self.pending_snap and m["op"] == "movel":
                        self.snap(*self.pending_snap)
                        self.pending_snap = None
                    seg.setdefault("on_stop", []).append({"op": m["op"], **{k: r.get(k) for k in ("arrived", "err_mm") if k in r}})
                seg["measured_s"] = round(self.t - seg_t0, 3)
                self.stop_motion = seg.get("on_stop", [])
                break
        arm_time = self.t - t_start
        if self.pending_snap:
            self.snap(*self.pending_snap)
            self.pending_snap = None
        self.snap(f"{self.sn}_overview")
        # 干涉 / 未到位：曲线判定之外单独记问题（批头没到孔位，曲线合格也不能算合格）
        seg_point = {sg["i"]: sg.get("point") for sg in segments}
        dfm_by_point = {}
        for it in self.dfm_open:
            dfm_by_point.setdefault(it.get("point"), []).append(it)
        for pt in [r["point"] for r in results]:
            hits = [c for c in self.collisions if seg_point.get(c["op_index"]) == pt]
            if not hits and arrived.get(pt, {}).get("arrived", False):
                continue
            g = self.point_group[pt]
            self._mark(pt, "hole_ng")
            self._andon("andon_ng")
            rec = {"issue_id": f"{self.sn}-{pt}-{len(self.issues) + 1:02d}", "t_s": round(self.t, 3), "sn": self.sn,
                   "station": self.station["station_id"], "point": pt, "group": g["id"], "part": g.get("part"), "screw": g["screw"],
                   "mode": "interference", "mode_cn": "干涉/未到位",
                   "detail": {"arrived": arrived.get(pt, {}).get("arrived"), "hole_err_mm": arrived.get(pt, {}).get("hole_err_mm"),
                              "collisions": [{"link": c["link"], "with": c["with"].rsplit("/", 1)[-1], "force_n": c["force_n"]} for c in hits]},
                   "dfm_match": [it["rule"] + " " + it.get("obstacle", "") for it in dfm_by_point.get(pt, [])],
                   "actions": ["mark_ng"], "unit_disposition": "整件判 NG（离线干涉检查可提前标出；运行中未停止）", "status": "待复判"}
            self.issues.append(rec)
            self.gateway.append({"t_s": round(self.t, 3), "source": self.station["station_id"], "via": self.feeder_mod["mes"]["via"],
                                 "sn": self.sn, "event": "issue_opened", "issue_id": rec["issue_id"], "point": pt, "mode": "interference"})
        interfered = {i["point"] for i in self.issues if i["mode"] == "interference"}

        est = process["stages"]["compose"]["takt"]
        measured = round(arm_time + est["load_unload_s"], 2)
        dev = round(100.0 * (measured - est["total_s"]) / est["total_s"], 1)
        n_points = len(flow_steps)
        all_arrived = len(arrived) == n_points and all(a["arrived"] for a in arrived.values())
        all_ok = len(results) == n_points and all(r["result"] == "OK" for r in results)
        handling = self._check_handling(results, flow_steps, stop_unit)
        # 放件臂放够了已到达的交接点所要求的件数（整件停止后不再放后续零件）
        h_need = getattr(self, "h_need", 0)
        h_ok = (not self.hd) or (self.h_count >= h_need and all(sg.get("arrived", True) for sg in self.h_segments))
        handling["checks"].append({"check": "handler_placed_required", "placed": self.h_count, "expect": h_need, "ok": h_ok})
        handling["ok"] = handling["ok"] and h_ok
        seg_point = {sg["i"]: sg.get("point") for sg in segments}
        for c in self.collisions:
            c["point"] = seg_point.get(c["op_index"])
        obstacle_hits = sorted({(c["point"], c["with"].rsplit("/", 1)[-1]) for c in self.collisions if "/components/" in c["with"]})
        dfm_pairs = sorted({(it["point"], it["obstacle"]) for it in self.dfm_open if it.get("rule") == "DFM-01"})
        return {
            "tag": self.tag,
            "fault": {"rate": self.fault_rate, "seed": self.seed, "inject": self.inject, "plan": self.fault_plan},
            "dfm_fix": getattr(self, "dfm_fix", []),
            "dfm_open_issues": [list(x) for x in dfm_pairs],
            "obstacle_collisions": [list(x) for x in obstacle_hits],
            "unit_result": "OK" if all_ok and not stop_unit and not interfered else "NG",
            "handling": handling,
            "twin_ok": all_arrived_exec(arrived, results) and not self.collisions and handling["ok"],
            "issues": list(self.issues),
            "place_steps": self.placed,
            "product": self.station["product"],
            "snapshots": self.snaps,
            "station": self.station["station_id"], "sn": self.sn, "generated": datetime.now().isoformat(timespec="seconds"),
            "usd": str(USD_PATH), "sim": {"dt_s": DT, "tcp_offset_mm": self.tcp_m * 1000, "arrive_tol_mm": ARRIVE_TOL_MM,
                                         "contact_threshold_n": CONTACT_N, "ik": self.arm_mod["isaac_ik"]},
            "fixture_true_pose": {"xyz_mm": [round(v, 4) for v in self.fixture_true.t], "R": [[round(v, 9) for v in r] for r in self.fixture_true.R]},
            "segments": segments,
            "points": [{"point": p, **a} for p, a in arrived.items()],
            "points_arrived": sum(1 for a in arrived.values() if a["arrived"]), "points_total": n_points,
            "hole_err_max_mm": max((a["hole_err_mm"] for a in arrived.values()), default=None),
            "handler": ({"items": len(self.hd["items"]), "placed": self.h_count, "handler_s": round(self.h_time, 2),
                         "segments": self.h_segments, "source": self.hd["source"]} if self.hd else None),
            "takt": {"arm_sim_s": round(arm_time, 2), "load_unload_s": est["load_unload_s"], "measured_s": measured,
                     "handler_s": round(self.h_time, 2), "fasten_arm_s": round(arm_time - self.h_time, 2),
                     "estimated_s": est["total_s"], "deviation_pct": dev, "flag": abs(dev) > TAKT_FLAG_PCT,
                     "target_s": est["target_s"]},
            "takt_measured_s": measured,
            "collisions": list(self.collisions),
            "results": results,
            "feeder": {"id": self.by_role["feeder"], "lanes": self.lanes},
            "ok": all_arrived and not self.collisions and all_ok and not stop_unit,
        }

    def run_unit(self):
        if getattr(self, "parallel", False) and self.hd:
            return self._run_parallel()
        return self._run_g(self.run_unit_g())

    def _check_handling(self, results, flow_steps, stop_unit) -> dict:
        """孪生对异常的处理是否符合业务流：复拧次数、问题记录、整件停止/继续。"""
        checks = []
        order = [s for s in flow_steps]
        for r in results:
            st = flow_steps[r["point"]]
            retry = st.get("executor_retry", {"max": 0, "on_modes": []})
            first = r["attempt_log"][0]
            expect = 1 + (1 if first["result"] != "OK" and first["mode"] in retry["on_modes"] and retry["max"] >= 1 else 0)
            checks.append({"point": r["point"], "check": "attempts", "expect": expect, "actual": r["attempts"], "ok": expect == r["attempts"]})
            if r["result"] != "OK":
                has = any(i["point"] == r["point"] for i in self.issues)
                checks.append({"point": r["point"], "check": "issue_recorded", "ok": has})
        ng_stop = next((r["point"] for r in results if r["result"] != "OK" and "stop_unit" in flow_steps[r["point"]]["on_ng"]), None)
        if ng_stop:
            ok = results[-1]["point"] == ng_stop and stop_unit
            checks.append({"point": ng_stop, "check": "stop_unit", "ok": ok})
            sm = getattr(self, "stop_motion", [])
            checks.append({"point": ng_stop, "check": "retract_home_before_stop", "ops": [m["op"] for m in sm],
                           "ok": [m["op"] for m in sm] == ["movel", "movej"] and all(m.get("arrived", True) for m in sm)})
        else:
            checks.append({"check": "all_points_executed", "ok": [r["point"] for r in results] == order})
        return {"ok": all(c["ok"] for c in checks), "checks": checks}

    def collision_selftest(self) -> dict:
        """把 TCP 压到桌面以下，确认检测器能报出接触。"""
        self.start()
        probe = dict(next(o for o in self.ops if o["op"] == "movel")["pose"])
        x, y, _ = probe["xyz_mm"]
        self.movel({**probe, "xyz_mm": [x + 150.0, y, SCENE["table_top_z"] - 30.0]}, "fast")
        self.stop()
        return {"collisions": self.collisions, "detected": bool(self.collisions)}
