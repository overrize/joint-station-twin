"""任务 M3.1：从 station.yaml 生成工站的数字孪生（USD），保存到 build/twin/JS-01.usd，并截图 build/twin/scene.png。

运行（Windows，Isaac Sim 6.2 装在 F:\\isaacsim）：
    python -m jsdemo run --yes                       # 先生成 build/station
    F:\\isaacsim\\python.bat sim\\isaac\\build_scene.py
    F:\\isaacsim\\python.bat sim\\isaac\\build_scene.py --headless

只用 docs/isaac-api-notes.md 里已在本机核实的写法（isaacsim.core.experimental）。
尺寸和位置全部来自 _common.SCENE 与模块库；世界坐标系 = 机械臂基座坐标系，单位换算 mm -> m。
每个模块一个 prim，名字 = prim_name(模块 id)，customData["jsdemo:module_id"] = 原 id。
"""
from __future__ import annotations

import argparse
import json
import math
import sys

from _common import (SCENE, STATION_DIR, TWIN_DIR, USD_PATH, load_assembly, load_handler_ops, load_handling, load_outputs,
                     load_station_product, mm, prim_name, screw_length_mm)

ap = argparse.ArgumentParser()
ap.add_argument("--headless", action="store_true")
ap.add_argument("--settle-frames", type=int, default=90, help="截图前仿真帧数，让机械臂到达 home 姿态")
args, _ = ap.parse_known_args()

from isaacsim import SimulationApp  # noqa: E402

app = SimulationApp({"headless": args.headless})

# ---- 以下 import 必须在 SimulationApp 创建之后 ----
import isaacsim.core.experimental.utils.app as app_utils  # noqa: E402
import isaacsim.core.experimental.utils.stage as stage_utils  # noqa: E402
from isaacsim.core.experimental.materials import PreviewSurfaceMaterial  # noqa: E402
from isaacsim.core.experimental.objects import Cube, Cylinder, DistantLight, DomeLight, Sphere  # noqa: E402
from isaacsim.core.experimental.prims import GeomPrim, XformPrim  # noqa: E402
from isaacsim.core.rendering_manager import RenderingManager, ViewportManager  # noqa: E402
from isaacsim.core.simulation_manager import SimulationManager  # noqa: E402
from isaacsim.storage.native import get_assets_root_path  # noqa: E402
from omni.kit.viewport.utility import capture_viewport_to_file, get_active_viewport  # noqa: E402
from pxr import Gf, Usd, UsdGeom, UsdPhysics, Vt  # noqa: E402

from jsdemo import composer, layout  # noqa: E402
from jsdemo.geometry import Pose  # noqa: E402
from jsdemo.io import groups_by_id, load_library  # noqa: E402

ROOT = "/World/" + prim_name("JS-01")
MODULE_KEY = "jsdemo:module_id"
UR_JOINTS = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]
SCREENSHOT = TWIN_DIR / "scene.png"


def yaw_quat(deg: float) -> list[float]:
    h = math.radians(deg) / 2
    return [math.cos(h), 0.0, 0.0, math.sin(h)]  # wxyz


class Builder:
    def __init__(self, stage):
        self.stage = stage
        self.materials: dict[str, PreviewSurfaceMaterial] = {}

    def material(self, name: str, rgb, roughness: float = 0.5, metallic: float = 0.0):
        if name not in self.materials:
            m = PreviewSurfaceMaterial(f"/World/Looks/{name}")
            m.set_input_values("diffuseColor", rgb)
            m.set_input_values("roughness", [roughness])
            m.set_input_values("metallic", [metallic])
            self.materials[name] = m
        return self.materials[name]

    def tag(self, path: str, module_id: str) -> None:
        self.stage.GetPrimAtPath(path).SetCustomDataByKey(MODULE_KEY, module_id)

    def box(self, path, size_mm, xyz_mm, look, *, local=False, collide=True):
        pose = {"translations" if local else "positions": [mm(xyz_mm)]}
        # Cube 的 size 是边长；非立方体用 scale 拉伸
        Cube(paths=path, sizes=1.0, scales=[mm(size_mm)], **pose).apply_visual_materials(look)
        if collide:
            GeomPrim(paths=path, apply_collision_apis=True)

    def ring(self, path, r_in, r_out, z0, z1, look, *, collide=True, segments=96, xy=(0.0, 0.0)):
        """环形柱（USD Mesh，入参 mm），静态碰撞用三角网格。"""
        pts, faces, counts = [], [], []
        for i in range(segments):
            a = 2 * math.pi * i / segments
            c, s_ = math.cos(a), math.sin(a)
            for r in (r_in, r_out):
                for z in (z0, z1):
                    pts.append(Gf.Vec3f(*mm([xy[0] + r * c, xy[1] + r * s_, z])))

        def idx(i, ro, zi):
            return (i % segments) * 4 + ro * 2 + zi

        for i in range(segments):
            j = i + 1
            for quad in ((idx(i, 1, 0), idx(j, 1, 0), idx(j, 1, 1), idx(i, 1, 1)),   # 外壁
                         (idx(i, 0, 0), idx(i, 0, 1), idx(j, 0, 1), idx(j, 0, 0)),   # 内壁
                         (idx(i, 0, 1), idx(i, 1, 1), idx(j, 1, 1), idx(j, 0, 1)),   # 顶面
                         (idx(i, 0, 0), idx(j, 0, 0), idx(j, 1, 0), idx(i, 1, 0))):  # 底面
                faces += quad
                counts.append(4)
        mesh = UsdGeom.Mesh.Define(self.stage, path)
        mesh.CreatePointsAttr(Vt.Vec3fArray(pts))
        mesh.CreateFaceVertexIndicesAttr(Vt.IntArray(faces))
        mesh.CreateFaceVertexCountsAttr(Vt.IntArray(counts))
        mesh.CreateSubdivisionSchemeAttr("none")
        GeomPrim(paths=path).apply_visual_materials(look)
        if collide:
            GeomPrim(paths=path, apply_collision_apis=True).set_collision_approximations("none")

    def solid(self, path, r, z0, z1, look, *, xy=(0.0, 0.0), collide=True, segments=96):
        """实心圆柱（USD Mesh，入参 mm，坐标为父节点局部系）。Cylinder 图元在 RTX 下只画成十边形（M3.4 截图），
        产品与螺钉这类近景物体改用网格；静态碰撞用三角网格。挂在关节连杆上的执行器仍用 Cylinder（动态刚体不能用三角网格碰撞）。"""
        x0, y0 = xy
        pts = [Gf.Vec3f(*mm([x0, y0, z0])), Gf.Vec3f(*mm([x0, y0, z1]))]
        for i in range(segments):
            a = 2 * math.pi * i / segments
            for z in (z0, z1):
                pts.append(Gf.Vec3f(*mm([x0 + r * math.cos(a), y0 + r * math.sin(a), z])))

        # 顶/底面另用一圈重复的边缘顶点，避免与侧面共用顶点导致法线被平均（M3.4 截图：编码器座看起来像圆顶）
        cap0 = len(pts)
        for i in range(segments):
            a = 2 * math.pi * i / segments
            for z in (z0, z1):
                pts.append(Gf.Vec3f(*mm([x0 + r * math.cos(a), y0 + r * math.sin(a), z])))

        def rim(i, zi, base=2):
            return base + (i % segments) * 2 + zi

        faces, counts = [], []
        for i in range(segments):
            j = i + 1
            faces += [rim(i, 0), rim(j, 0), rim(j, 1), rim(i, 1)]
            counts.append(4)
            faces += [1, rim(i, 1, cap0), rim(j, 1, cap0)]
            counts.append(3)
            faces += [0, rim(j, 0, cap0), rim(i, 0, cap0)]
            counts.append(3)
        mesh = UsdGeom.Mesh.Define(self.stage, path)
        mesh.CreatePointsAttr(Vt.Vec3fArray(pts))
        mesh.CreateFaceVertexIndicesAttr(Vt.IntArray(faces))
        mesh.CreateFaceVertexCountsAttr(Vt.IntArray(counts))
        mesh.CreateSubdivisionSchemeAttr("none")
        GeomPrim(paths=path).apply_visual_materials(look)
        if collide:
            GeomPrim(paths=path, apply_collision_apis=True).set_collision_approximations("none")

    def cylinder(self, path, radius_mm, height_mm, xyz_mm, look, *, local=True, collide=True):
        pose = {"translations" if local else "positions": [mm(xyz_mm)]}
        Cylinder(paths=path, radii=mm(radius_mm), heights=mm(height_mm), axes="Z", **pose).apply_visual_materials(look)
        if collide:
            GeomPrim(paths=path, apply_collision_apis=True)


# 材料 → 外观（同一材料出现多次时轮换色调，便于分清相邻零件）
MATERIAL_LOOKS = {
    "AL6061": [([0.72, 0.74, 0.78], 0.35, 0.8), ([0.55, 0.6, 0.68], 0.3, 0.9), ([0.15, 0.35, 0.65], 0.35, 0.4)],
    "PA66": [([0.12, 0.12, 0.13], 0.7, 0.0)],
    "FR4": [([0.02, 0.16, 0.08], 0.5, 0.0)],
    "PC": [([0.82, 0.85, 0.9], 0.4, 0.0), ([0.3, 0.32, 0.36], 0.5, 0.0)],
}
DEFAULT_LOOK = [([0.6, 0.6, 0.62], 0.5, 0.2)]


def _looks(b):
    return {"obstacle": b.material("obstacle", [0.95, 0.85, 0.2], 0.4),
            "hole": b.material("hole_idle", [0.35, 0.35, 0.38], 0.6),
            "screw": b.material("screw", [0.82, 0.83, 0.86], 0.25, 1.0),
            "slot": b.material("screw_slot", [0.05, 0.05, 0.05], 0.8)}


def _shape(b, path, sh, look, collide=True):
    """SOP 简化体（零件坐标系，mm）→ USD。圆柱/环用网格，长方体用 Cube（缩放 + 偏航）。"""
    if sh["type"] == "cylinder":
        b.solid(path, sh["r"], sh["z"][0], sh["z"][1], look, xy=tuple(sh.get("xy", (0.0, 0.0))), collide=collide)
    elif sh["type"] == "ring":
        b.ring(path, sh["r_in"], sh["r_out"], sh["z"][0], sh["z"][1], look, xy=tuple(sh.get("xy", (0.0, 0.0))), collide=collide)
    else:
        Cube(paths=path, sizes=1.0, scales=[mm(sh["size"])], translations=[mm(sh["center"])],
             orientations=[yaw_quat(sh.get("yaw", 0.0))]).apply_visual_materials(look)
        if collide:
            GeomPrim(paths=path, apply_collision_apis=True)


def build_product(b, pp, station):
    """产品（M3.5b）：按工站 SOP 的零件简化体生成，每个零件一个 Xform（装配位姿，治具坐标系），
    下有 s<i>（外形）、components/<id>（板上元件，也是障碍物）、holes/<点位>（状态环）。
    全部零件初始隐藏、碰撞关闭，由 twin.py 按 SOP 放件工步依次放入并打开碰撞。"""
    asm = load_assembly(station)
    product = load_station_product(station)
    groups = groups_by_id(product)
    lib = load_library()
    lk, hm = _looks(b), SCENE["hole_marker"]
    stage_utils.define_prim(pp, "Xform")
    XformPrim(pp, translations=[[0.0, 0.0, 0.0]], reset_xform_op_properties=True)   # 下线时整件随吸盘走（M4 ③）
    stage_utils.define_prim(f"{pp}/parts", "Xform")
    fastens_of = {pid: st["fastens"] for st in asm["steps"] if st["type"] == "fasten" for pid in st["points"]}
    used = {}
    for part in asm["parts"]:
        mat = part["material"]
        k = used.get(mat, 0)
        used[mat] = k + 1
        rgb, rough, metal = MATERIAL_LOOKS.get(mat, DEFAULT_LOOK)[k % len(MATERIAL_LOOKS.get(mat, DEFAULT_LOOK))]
        look = b.material(f"part_{prim_name(part['id'])}", rgb, rough, metal)
        xp = f"{pp}/parts/{prim_name(part['id'])}"
        stage_utils.define_prim(xp, "Xform")
        pose = part["pose_in_fixture"]
        XformPrim(xp, translations=[mm(pose["xyz"])], orientations=[yaw_quat(pose["rpy"][2])], reset_xform_op_properties=True)
        prim = b.stage.GetPrimAtPath(xp)
        prim.SetCustomDataByKey("jsdemo:part", part["id"])
        prim.SetCustomDataByKey("jsdemo:material", mat)
        prim.SetCustomDataByKey("jsdemo:geometry_source", part["geometry_source"])
        for i, sh in enumerate(part["shapes"]):
            _shape(b, f"{xp}/s{i}", sh, look)
        if part["components"]:
            stage_utils.define_prim(f"{xp}/components", "Xform")
        for c in part["components"]:
            cp = f"{xp}/components/{prim_name(c['id'])}"
            _shape(b, cp, c["shape"], lk["obstacle"])
            b.stage.GetPrimAtPath(cp).SetCustomDataByKey("jsdemo:name", c["name"])
        # 状态环：点位换到零件坐标系，放在被紧固件下，随零件一起出现
        P = Pose.from_xyz_rpy(pose["xyz"], (0.0, 0.0, pose["rpy"][2])).inverse()
        for pt in product["fastening"]["points"]:
            if fastens_of.get(pt["id"]) != part["id"]:
                continue
            if not b.stage.GetPrimAtPath(f"{xp}/holes"):
                stage_utils.define_prim(f"{xp}/holes", "Xform")
            d = lib["screws"][groups[pt["group"]]["screw"]]["d_mm"]
            x, y, z = P.apply(pt["xyz"])
            b.ring(f"{xp}/holes/{pt['id']}", SCENE["screw_head"]["d_ratio"] * d / 2 * 1.05, d * hm["r_ratio"], z, z + hm["thickness"], lk["hole"],
                   collide=False, segments=48, xy=(x, y))
            b.solid(f"{xp}/holes/{pt['id']}_bore", d / 2, z, z + hm["thickness"] * 0.5, lk["slot"], collide=False, segments=32, xy=(x, y))
        UsdGeom.Imageable(prim).MakeInvisible()
        collide_paths = [f"{xp}/s{i}" for i in range(len(part["shapes"]))] + [f"{xp}/components/{prim_name(c['id'])}" for c in part["components"]]
        GeomPrim(paths=collide_paths).set_enabled_collisions([False])


def build_executor(b, ep, look_tool):
    """执行器（M3.4）：本体 + 批头杆。批头分两份：bit_visual（无碰撞，运行时随拧入上下移动、转动）与
    bit_collider（隐藏、带碰撞、固定在最短螺钉的起始位置）；浮动头是单独的模块 prim。"""
    ex, parts = SCENE["executor"], SCENE["executor_parts"]
    stage_utils.define_prim(ep, "Xform")
    body = parts["body"]
    b.cylinder(f"{ep}/body", body["radius"], body["length"], [0.0, 0.0, body["length"] / 2], look_tool)
    bit = parts["bit"]
    lk = _looks(b)
    stripe = b.material("bit_stripe", [0.95, 0.3, 0.1], 0.4)
    stage_utils.define_prim(f"{ep}/bit_visual", "Xform")
    XformPrim(f"{ep}/bit_visual", translations=[[0.0, 0.0, 0.0]], reset_xform_op_properties=True)
    b.cylinder(f"{ep}/bit_visual/shaft", bit["radius"], bit["length"], [0.0, 0.0, ex["length"] - bit["length"] / 2], lk["screw"], collide=False)
    Cube(paths=f"{ep}/bit_visual/stripe", sizes=1.0, scales=[mm([1.2, 2.0 * bit["radius"] + 0.6, 12.0])],
         translations=[mm([0.0, 0.0, ex["length"] - 20.0])]).apply_visual_materials(stripe)
    # 碰撞用批头：尖端停在 TCP 上方「最短螺钉的 杆长 + 头高」处（螺钉尖刚到孔口时批头的位置）
    lib = load_library()
    sh = SCENE["screw_head"]
    lift = min(screw_length_mm(k) + sh["h_ratio"] * v["d_mm"] for k, v in lib["screws"].items())
    b.cylinder(f"{ep}/bit_collider", bit["radius"], bit["length"], [0.0, 0.0, ex["length"] - lift - bit["length"] / 2], look_tool)
    UsdGeom.Imageable(b.stage.GetPrimAtPath(f"{ep}/bit_collider")).MakeInvisible()


def build_screws(b, station, root_name="screws"):
    """每个点位一颗螺钉（世界坐标，无碰撞，初始隐藏）。原点在螺钉头顶面中心，沿 -z 依次是头、杆。"""
    product = load_station_product(station)
    groups = groups_by_id(product)
    lib = load_library()
    sh, lk = SCENE["screw_head"], _looks(b)
    stage_utils.define_prim(f"{ROOT}/{root_name}", "Xform")
    XformPrim(f"{ROOT}/{root_name}", translations=[[0.0, 0.0, 0.0]], reset_xform_op_properties=True)
    for pt in product["fastening"]["points"]:
        sid = groups[pt["group"]]["screw"]
        d, length = lib["screws"][sid]["d_mm"], screw_length_mm(sid)
        hr, hh = sh["d_ratio"] * d / 2, sh["h_ratio"] * d
        sp = f"{ROOT}/{root_name}/{pt['id']}"
        stage_utils.define_prim(sp, "Xform")
        XformPrim(sp, positions=[[0.0, 0.0, 0.0]], reset_xform_op_properties=True)
        b.solid(f"{sp}/head", hr, -hh, 0.0, lk["screw"], collide=False, segments=48)
        Cube(paths=f"{sp}/slot", sizes=1.0, scales=[mm([2 * hr * 0.95, 0.3 * d, 0.35 * hh])],
             translations=[mm([0.0, 0.0, -0.1 * hh])]).apply_visual_materials(lk["slot"])
        b.solid(f"{sp}/shank", d / 2, -hh - length, -hh, lk["screw"], collide=False, segments=24)
        b.stage.GetPrimAtPath(sp).SetCustomDataByKey("jsdemo:screw", sid)
        UsdGeom.Imageable(b.stage.GetPrimAtPath(sp)).MakeInvisible()


def build(station, ops, modules) -> dict[str, str]:
    """搭场景，返回 {模块 id: prim 路径}。"""
    stage_utils.create_new_stage()
    stage = stage_utils.get_current_stage(backend="usd")
    b = Builder(stage)
    by_role = {m["role"]: m["id"] for m in station["modules"]}
    paths: dict[str, str] = {}

    stage_utils.define_prim("/World", "Xform")
    stage_utils.define_prim(ROOT, "Xform")
    stage.GetPrimAtPath(ROOT).SetCustomDataByKey(MODULE_KEY, station["station_id"])

    look_table = b.material("table", [0.55, 0.57, 0.6], 0.7)
    look_dark = b.material("dark", [0.12, 0.13, 0.15], 0.6)
    look_alu = b.material("aluminium", [0.78, 0.8, 0.83], 0.3, 0.8)
    look_plate = b.material("plate", [0.3, 0.33, 0.38], 0.4, 0.5)
    look_datum = b.material("datum", [0.9, 0.15, 0.1], 0.3)
    look_tool = b.material("tool", [0.2, 0.2, 0.22], 0.4, 0.6)
    look_float = b.material("float_head", [0.95, 0.6, 0.1], 0.4, 0.3)
    look_box = b.material("cabinet", [0.85, 0.86, 0.88], 0.6)
    look_lanes = [b.material("lane_1", [0.85, 0.55, 0.15], 0.4, 0.3), b.material("lane_2", [0.2, 0.6, 0.35], 0.4, 0.3),
                  b.material("lane_3", [0.55, 0.3, 0.75], 0.4, 0.3)]
    look_lamp = b.material("lamp_ok", [0.1, 0.9, 0.2], 0.2)

    # 桌面与垫高（非模块）
    tz = SCENE["table_top_z"]
    ts = SCENE["table_size"]
    b.box(f"{ROOT}/table", ts, [*SCENE["table_center_xy"], tz - ts[2] / 2], look_table)
    rh = SCENE["riser_height"]
    b.box(f"{ROOT}/riser", [*SCENE["riser_size_xy"], rh], [0.0, 0.0, -rh / 2], look_dark)

    def module_path(role: str) -> tuple[str, str]:
        mid = by_role[role]
        p = f"{ROOT}/{prim_name(mid)}"
        paths[mid] = p
        return mid, p

    # 输送段：铺在桌面上，沿 y 方向
    mid, p = module_path("conveyor")
    cv = SCENE["conveyor"]
    y0, y1 = cv["y_span"]
    b.box(p, [cv["width"], y1 - y0, cv["height"]], [cv["x"], (y0 + y1) / 2, tz + cv["height"] / 2], look_dark)
    b.tag(p, mid)

    # 定位销基板：铺在桌面上，治具下方
    fx_mod = modules[by_role["fixture"]]
    fx_pose = composer.fixture_pose(modules, by_role["fixture"])   # 工站设计 fixture.pose
    fx_xyz = fx_pose.t
    mid, p = module_path("baseplate")
    bs = SCENE["baseplate_size"]
    bp = SCENE.get("baseplate_xyz") or [fx_xyz[0], fx_xyz[1], tz + bs[2] / 2]
    b.box(p, bs, bp, look_plate)
    b.tag(p, mid)

    # 治具：Xform 放在治具名义位姿（= 治具坐标系），治具块顶面中心在原点
    mid, p = module_path("fixture")
    stage_utils.define_prim(p, "Xform")
    XformPrim(p, positions=[mm(fx_xyz)], orientations=[yaw_quat(fx_pose.yaw_deg())],
              reset_xform_op_properties=True)
    b.tag(p, mid)
    fs = SCENE["fixture_size"]
    b.box(f"{p}/body", fs, [0.0, 0.0, -fs[2] / 2], look_alu, local=True)
    for d in fx_mod["datums"]:
        Sphere(paths=f"{p}/{d['id']}", radii=mm(SCENE["datum_radius"]), translations=[mm(d["xyz"])]).apply_visual_materials(look_datum)
    # 产品（M3.5b）：按工站 SOP 的零件生成；见 build_product
    build_product(b, f"{p}/{prim_name(station['product'])}", station)
    if load_handling():   # 流水线（M4 ③）：后续各台的零件（初始隐藏），名字带 _u<k>
        for k in range(2, SCENE["line_units"] + 1):
            build_product(b, f"{p}/{prim_name(station['product'])}_u{k}", station)

    # 机械臂：从资产库引用，路径来自模块库 isaac_asset
    mid, p = module_path("arm")
    arm_mod = modules[mid]
    stage_utils.add_reference_to_stage(_asset(arm_mod), p)
    b.tag(p, mid)
    # home 姿态写进 USD 的关节初始状态与驱动目标（取 arm_program 第一条 movej，单位 deg）
    home = next(o["joints_deg"] for o in ops if o["op"] == "movej")
    for name, deg in zip(UR_JOINTS, home):
        j = stage.GetPrimAtPath(f"{p}/joints/{name}")
        j.GetAttribute("state:angular:physics:position").Set(float(deg))
        j.GetAttribute("drive:angular:physics:targetPosition").Set(float(deg))

    # 执行器与浮动批头座：挂在 wrist_3_link（与 Lula 的 tool0 坐标系重合，本机核实），沿其 z 轴
    flange = f"{p}/wrist_3_link"
    mid = by_role["executor"]
    ep = f"{flange}/{prim_name(mid)}"
    paths[mid] = ep
    build_executor(b, ep, look_tool)
    b.tag(ep, mid)
    fh = SCENE["float_head"]
    mid = by_role["float_head"]
    fp = f"{flange}/{prim_name(mid)}"
    paths[mid] = fp
    b.cylinder(fp, fh["radius"], fh["length"], [0.0, 0.0, fh["center_from_flange"]], look_float)
    b.tag(fp, mid)

    # 放件（M4 ②）：配套料盘（随输送线进出）、放件臂 + 真空吸盘。布局取 handling.yaml（demo 占位）
    hd = load_handling()
    if hd:
        build_handling(b, hd, modules, by_role, paths, look_dark, look_tool, station["product"])

    # 供钉器：机身 + 每路料斗、出钉口、状态灯；第 i 路 = screw_kinds 第 i 种（模块库 lane_assignment）
    mid, p = module_path("feeder")
    fd, fd_mod = SCENE["feeder"], modules[mid]
    kinds = station["checks"]["screw_kinds"]
    if len(kinds) > fd_mod["lanes"]:
        raise RuntimeError(f"{mid} 只有 {fd_mod['lanes']} 路，供不了 {len(kinds)} 种螺钉")
    stage_utils.define_prim(p, "Xform")
    XformPrim(p, positions=[mm(fd["xyz"])], reset_xform_op_properties=True)
    b.tag(p, mid)
    w, d, h = fd["size"]
    b.box(f"{p}/body", fd["size"], [0.0, 0.0, 0.0], look_box, local=True)
    hp, ol = fd["hopper"], fd["outlet"]
    for i in range(fd_mod["lanes"]):
        lane = f"{p}/lane_{i + 1}"
        x = (i - (fd_mod["lanes"] - 1) / 2) * hp["pitch"]
        stage_utils.define_prim(lane, "Xform")
        XformPrim(lane, translations=[mm([x, 0.0, 0.0])], reset_xform_op_properties=True)
        screw = kinds[i] if i < len(kinds) else None
        lane_prim = stage.GetPrimAtPath(lane)
        lane_prim.SetCustomDataByKey("jsdemo:lane", i + 1)
        lane_prim.SetCustomDataByKey("jsdemo:screw_kind", screw or "空")
        look = look_lanes[i % len(look_lanes)] if screw else look_dark
        b.cylinder(f"{lane}/hopper", hp["radius"], hp["height"], [0.0, 0.0, h / 2 + hp["height"] / 2], look)
        Cylinder(paths=f"{lane}/outlet", radii=mm(ol["radius"]), heights=mm(ol["length"]), axes="Y",
                 translations=[mm([0.0, -d / 2 - ol["length"] / 2, h / 4])]).apply_visual_materials(look_tool)
        Sphere(paths=f"{lane}/lamp", radii=mm(fd["lamp_radius"]),
               translations=[mm([0.0, -d / 2, h / 2])]).apply_visual_materials(look_lamp if screw else look_dark)
        # 出钉位（M4 取钉，DEC-FEED-001）：出钉口末端立在桌面上的小块，顶面与机身顶面齐平；只做外形，不参与碰撞
        # （批头碰撞体固定在最短螺钉的位置，取长螺钉时会和出钉位顶面重叠，属建模简化，不是真实干涉）
        ns = fd["nest_size"]
        b.box(f"{lane}/nest", ns, [0.0, -d / 2 - ol["length"] - ns[1] / 2, h / 2 - ns[2] / 2], look_box, local=True, collide=False)
        b.solid(f"{lane}/nest_bore", 2.6, h / 2 - 0.2, h / 2 + 0.05, look_dark, collide=False, segments=24,
                xy=(0.0, -d / 2 - ol["length"] - ns[1] / 2))

    # 控制器、接口盒、网关：外形占位
    for role, spec in SCENE["boxes"].items():
        mid, p = module_path(role)
        b.box(p, spec["size"], spec["xyz"], look_box)
        b.tag(p, mid)

    missing = [m["id"] for m in station["modules"] if m["id"] not in paths]
    if missing:
        raise RuntimeError(f"station.yaml 中这些模块没有放进场景：{missing}")

    # 工站状态灯（M3.4，非模块）与每个点位的螺钉（运行时由 twin.py 显示并驱动）
    an = SCENE["andon"]
    Sphere(paths=f"{ROOT}/andon", radii=mm(an["radius"]), positions=[mm(an["xyz"])]).apply_visual_materials(
        b.material("andon_ok", [0.1, 0.9, 0.2], 0.2))
    build_screws(b, station)
    if load_handling():
        for k in range(2, SCENE["line_units"] + 1):
            build_screws(b, station, f"screws_u{k}")
        # 下线托盘：停在治具下游的输送线上，顶面在 OUT_STOP
        OUT_STOP = layout.OUT_STOP
        op_ = SCENE["out_pallet"]
        look_op = b.material("out_pallet", [0.2, 0.55, 0.35], 0.5)
        for k in range(1, SCENE["line_units"] + 1):   # 每台一个下线托盘；第 1 个在出件位，其余等前一个送走后出现
            opp = f"{ROOT}/out_pallet_{k}"
            stage_utils.define_prim(opp, "Xform")
            XformPrim(opp, positions=[mm(OUT_STOP["xyz_mm"])], reset_xform_op_properties=True)
            b.box(f"{opp}/body", op_["size"], [0.0, 0.0, -op_["size"][2] / 2], look_op, local=True, collide=False)
            if k > 1:
                UsdGeom.Imageable(stage.GetPrimAtPath(opp)).MakeInvisible()
        # 线体看板大屏（用户 2026-10-26）：拧紧臂后方、朝主机位；运行中由 twin.py 刷新贴图
        sc = SCENE["line"]["screen"]
        b.box(f"{ROOT}/line_screen_frame", [sc["size"][0] + 30.0, 20.0, sc["size"][1] + 30.0],
              [sc["center"][0], sc["center"][1] + 12.0, sc["center"][2]], b.material("screen_frame", [0.1, 0.1, 0.12], 0.5), collide=False)
        b.box(f"{ROOT}/line_screen_post", [40.0, 40.0, sc["center"][2] - sc["size"][1] / 2 - SCENE["table_top_z"]],
              [sc["center"][0], sc["center"][1] + 12.0, (sc["center"][2] - sc["size"][1] / 2 + SCENE["table_top_z"]) / 2],
              b.material("screen_frame", [0.1, 0.1, 0.12], 0.5), collide=False)
        label_quad(b, f"{ROOT}/line_screen", "线体看板", sc["center"], sc["size"])
        # 上下游工站（外形 + 标牌）：只表现物料进出接口，不模拟其内部动作
        look_st = b.material("line_station", [0.78, 0.8, 0.84], 0.6)
        for key in ("upstream_station", "downstream_station"):
            spec = SCENE["line"][key]
            cx, cy = spec["center"]
            sx, sy, sz_ = spec["size"]
            b.box(f"{ROOT}/{key}", spec["size"], [cx, cy, SCENE["table_top_z"] + sz_ / 2], look_st, collide=False)
            # 上游工站标牌贴在它朝工站（+y）的一面、下游贴在朝工站（-y）的一面，都要从主机位能看到：统一放在 -y 面前方
            label_quad(b, f"{ROOT}/{key}_label", spec["label"], [cx, cy - sy / 2 - 2.0, SCENE["table_top_z"] + sz_ * 0.66], [sx * 0.92, 80.0])

    DomeLight("/World/DomeLight").set_intensities(600)
    DistantLight("/World/DistantLight", orientations=[[0.9239, 0.2706, 0.2706, 0.0]]).set_intensities(1500)
    return paths


def label_quad(b, path, text, center_mm, size_mm):
    """竖直标牌：PIL 把文字画成 PNG，贴到一个面向 -y（主机位一侧）的平面上，平面沿 x 展开。"""
    from PIL import Image, ImageDraw, ImageFont
    from pxr import Sdf, UsdShade

    w, h = 1024, int(1024 * size_mm[1] / size_mm[0])
    img = Image.new("RGB", (w, h), (24, 48, 82))
    d = ImageDraw.Draw(img)
    font = ImageFont.truetype("C:/Windows/Fonts/msyhbd.ttc", int(h * 0.42))
    tw = d.textlength(text, font=font)
    d.text(((w - tw) / 2, h * 0.24), text, font=font, fill=(255, 255, 255))
    png = TWIN_DIR / "labels" / f"{path.rsplit('/', 1)[-1]}.png"
    png.parent.mkdir(parents=True, exist_ok=True)
    img.save(png)
    stage = b.stage
    cx, cy, cz = center_mm
    hw, hh = size_mm[0] / 2, size_mm[1] / 2
    mesh = UsdGeom.Mesh.Define(stage, path)
    pts = [(cx - hw, cy, cz - hh), (cx + hw, cy, cz - hh), (cx + hw, cy, cz + hh), (cx - hw, cy, cz + hh)]
    mesh.CreatePointsAttr(Vt.Vec3fArray([Gf.Vec3f(*mm(list(p))) for p in pts]))
    mesh.CreateFaceVertexCountsAttr(Vt.IntArray([4]))
    mesh.CreateFaceVertexIndicesAttr(Vt.IntArray([0, 1, 2, 3]))
    mesh.CreateSubdivisionSchemeAttr("none")
    mesh.CreateDoubleSidedAttr(True)
    st_ = UsdGeom.PrimvarsAPI(mesh).CreatePrimvar("st", Sdf.ValueTypeNames.TexCoord2fArray, UsdGeom.Tokens.faceVarying)
    st_.Set(Vt.Vec2fArray([Gf.Vec2f(0, 0), Gf.Vec2f(1, 0), Gf.Vec2f(1, 1), Gf.Vec2f(0, 1)]))
    mp = f"/World/Looks/label_{path.rsplit('/', 1)[-1]}"
    mat = UsdShade.Material.Define(stage, mp)
    sh = UsdShade.Shader.Define(stage, f"{mp}/pbr")
    sh.CreateIdAttr("UsdPreviewSurface")
    tex = UsdShade.Shader.Define(stage, f"{mp}/tex")
    tex.CreateIdAttr("UsdUVTexture")
    tex.CreateInput("file", Sdf.ValueTypeNames.Asset).Set(png.as_posix())
    rd = UsdShade.Shader.Define(stage, f"{mp}/st")
    rd.CreateIdAttr("UsdPrimvarReader_float2")
    rd.CreateInput("varname", Sdf.ValueTypeNames.Token).Set("st")
    tex.CreateInput("st", Sdf.ValueTypeNames.Float2).ConnectToSource(rd.ConnectableAPI(), "result")
    tex.CreateOutput("rgb", Sdf.ValueTypeNames.Float3)
    sh.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).ConnectToSource(tex.ConnectableAPI(), "rgb")
    sh.CreateInput("emissiveColor", Sdf.ValueTypeNames.Color3f).ConnectToSource(tex.ConnectableAPI(), "rgb")
    sh.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.8)
    mat.CreateSurfaceOutput().ConnectToSource(sh.ConnectableAPI(), "surface")
    UsdShade.MaterialBindingAPI.Apply(mesh.GetPrim()).Bind(mat)


def _asset(mod) -> str:
    """模块的 Isaac 资产：有本机合成的单文件（sim/isaac/fetch_assets.py）就用它，否则引用资产库。"""
    if mod.get("local_asset"):
        local = STATION_DIR.parents[1] / "build" / "assets" / mod["local_asset"]
        if local.exists():
            return str(local)
    return get_assets_root_path() + mod["isaac_asset"]


def handler_path(by_role) -> str:
    """放件臂与拧紧臂同型号（同一模块 id），prim 名加后缀区分。"""
    return f"{ROOT}/{prim_name(by_role['handler_arm'])}_handler"


def build_handling(b, hd, modules, by_role, paths, look_dark, look_tool, product):
    stage = b.stage
    look_tray = b.material("tray", [0.16, 0.36, 0.62], 0.55)
    look_slot = b.material("tray_slot", [0.85, 0.88, 0.92], 0.5)
    look_cup = b.material("vacuum_cup", [0.1, 0.1, 0.1], 0.8)
    look_grip = b.material("gripper", [0.92, 0.55, 0.12], 0.4, 0.3)
    # 料盘：Xform 原点在盘面中心，放在停止位；运行时 twin.py 让它从上游进场
    tid = by_role["tray"]
    tr = modules[tid]
    st = hd["tray_stop"]
    lp = SCENE["line"]
    for name, dy in ((prim_name(tid), 0.0), (f"{prim_name(tid)}_B", lp["tray_prestop_dy"])):   # 料盘 A / B 轮换（连续流动）
        tp = f"{ROOT}/{name}"
        stage_utils.define_prim(tp, "Xform")
        XformPrim(tp, positions=[mm([st["xyz_mm"][0], st["xyz_mm"][1] + dy, st["xyz_mm"][2]])], orientations=[yaw_quat(st["yaw_deg"])],
                  reset_xform_op_properties=True)
        sz = tr["size_mm"]
        b.box(f"{tp}/body", sz, [0.0, 0.0, -sz[2] / 2], look_tray, local=True, collide=False)
        r = tr["slot_max_diameter_mm"] / 2
        for k in range(tr["slots"]):
            nx, ny = tr["slot_grid"]
            px, py = tr["slot_pitch_mm"]
            xy = ((k % nx - (nx - 1) / 2) * px, (k // nx - (ny - 1) / 2) * py)
            b.ring(f"{tp}/slot_{k + 1}", r - 3.0, r, 0.0, 0.6, look_slot, collide=False, segments=64, xy=xy)
    tp = f"{ROOT}/{prim_name(tid)}"
    paths[tid] = tp
    b.tag(tp, tid)
    # 放件臂：垫高块 + 资产引用，基座位姿取 handler_mount；home 写进关节初始状态
    hm = hd["handler_mount"]
    rh = SCENE["riser_height"]
    b.box(f"{ROOT}/riser_handler", [*SCENE["riser_size_xy"], rh], [hm["xyz_mm"][0], hm["xyz_mm"][1], -rh / 2], look_dark)
    aid = by_role["handler_arm"]
    hp = handler_path(by_role)
    stage_utils.add_reference_to_stage(_asset(modules[aid]), hp)
    XformPrim(hp, positions=[mm(hm["xyz_mm"])], orientations=[yaw_quat(hm["yaw_deg"])], reset_xform_op_properties=True)
    b.tag(hp, aid)
    stage.GetPrimAtPath(hp).SetCustomDataByKey("jsdemo:role", "handler_arm")
    home = next(o["joints_deg"] for o in load_handler_ops() if o["op"] == "movej")
    for name, deg in zip(UR_JOINTS, home):
        j = stage.GetPrimAtPath(f"{hp}/joints/{name}")
        j.GetAttribute("state:angular:physics:position").Set(float(deg))
        j.GetAttribute("drive:angular:physics:targetPosition").Set(float(deg))
    # 末端工具（M4 ①）：各自独立，开始时都在快换架上；运行中接上的那个每步跟随法兰（twin.py）。
    # 工具坐标系 = 与法兰对接的面，+z 为工具伸出方向；停放时工具朝下（绕 x 转 180°），法兰面在 dock_z
    dock_z = modules[hd["tool_changer"]]["dock_z_mm"] if hd.get("tool_changer") else 150.0
    down = [0.0, 1.0, 0.0, 0.0]   # wxyz：绕 x 转 180°
    tc = hd.get("tool_changer")
    tcp_ = f"{ROOT}/{prim_name(tc)}" if tc else ROOT
    if tc:
        stage_utils.define_prim(tcp_, "Xform")
        paths[tc] = tcp_
        b.tag(tcp_, tc)
    for tid, xy in hd["tool_docks"].items():
        g = modules[tid]
        tp_ = f"{ROOT}/{prim_name(tid)}"
        paths[tid] = tp_
        if g["principle"] == "parallel_2f":
            # 运动学夹爪（用户 2026-10-25）：引用去掉物理的显示版（sim/isaac/record_gripper.py），手指按姿态表播放
            local = STATION_DIR.parents[1] / "build" / "assets" / g["visual_asset"]
            if not local.exists():
                raise RuntimeError(f"缺少 {local}，先运行 sim/isaac/fetch_assets.py 与 sim/isaac/record_gripper.py")
            stage_utils.add_reference_to_stage(str(local), tp_)
        else:
            stage_utils.define_prim(tp_, "Xform")
            tcp = g["envelope"]["segments"][-1]["z"][1]
            for seg in g["envelope"]["segments"]:
                z0, z1 = seg["z"]
                look = look_cup if seg["name"] == "cup" else (look_tool if seg["name"] == "stem" else look_grip)
                b.cylinder(f"{tp_}/{seg['name']}", seg["r"], z1 - z0, [0.0, 0.0, tcp - (z0 + z1) / 2], look, collide=False)
        XformPrim(tp_, positions=[mm([xy[0], xy[1], dock_z])], orientations=[down], reset_xform_op_properties=True)
        b.tag(tp_, tid)
        # 快换架工位（叉形托架）：两条托轨从两侧托住工具法兰盘下沿，横梁与立柱在工具后方（+x），都不进入工具外形
        # （之前的整块托板横穿工具本体，穿模；用户 2026-10-25 指出）。带碰撞，并纳入机械臂接触监测
        half = SCENE["tool_dock"]["half_width_mm"]
        rz = dock_z - SCENE["tool_dock"]["rail_drop_mm"]
        for side in (1, -1):
            b.box(f"{tcp_}/dock_{prim_name(tid)}_rail{'L' if side > 0 else 'R'}", [100.0, 10.0, 8.0], [xy[0] + 10.0, xy[1] + side * (half + 5.0), rz], look_dark)
        b.box(f"{tcp_}/dock_{prim_name(tid)}_beam", [16.0, 2 * half + 20.0, 8.0], [xy[0] + 68.0, xy[1], rz], look_dark)
        b.box(f"{tcp_}/dock_{prim_name(tid)}_post", [30.0, 30.0, rz - 4.0 - SCENE["table_top_z"]],
              [xy[0] + 68.0, xy[1], (rz - 4.0 + SCENE["table_top_z"]) / 2], look_dark)

    # 归放件臂的零件开始时在料盘里（可见、碰撞仍关闭）：零件在治具坐标系下的位姿 = F⁻¹ · 料盘 · 穴内位姿
    F = composer.fixture_pose(modules, by_role["fixture"])
    T = Pose.from_xyz_rpy(st["xyz_mm"], (0.0, 0.0, st["yaw_deg"]))
    for k in range(1, SCENE["line_units"] + 1):   # 第 1 台在料盘 A（停止位）、第 2 台在料盘 B（排队位）可见；第 3 台以后随回流的料盘再来
        pp = f"{ROOT}/{prim_name(by_role['fixture'])}/{prim_name(product)}{'' if k == 1 else f'_u{k}'}/parts"
        Tk = Pose.from_xyz_rpy([st["xyz_mm"][0], st["xyz_mm"][1] + (lp["tray_prestop_dy"] if k == 2 else 0.0), st["xyz_mm"][2]],
                               (0.0, 0.0, st["yaw_deg"]))
        for it in hd["items"]:
            L = F.inverse().compose(Tk).compose(Pose.from_xyz_rpy(it["part_in_tray"]["xyz"], (0.0, 0.0, it["part_in_tray"]["yaw_deg"])))
            xp = f"{pp}/{prim_name(it['part'])}"
            XformPrim(xp).set_local_poses(translations=[mm(L.t)], orientations=[yaw_quat(L.yaw_deg())])
            if k <= 2:
                UsdGeom.Imageable(stage.GetPrimAtPath(xp)).MakeVisible()


def check_reopened(station) -> dict:
    """重新打开 USD，核对每个模块都有 prim，且名字 = prim_name(id)、customData 记着原 id。"""
    ok, stage = stage_utils.open_stage(str(USD_PATH))
    if not ok:
        raise RuntimeError(f"无法重新打开 {USD_PATH}")
    found = {}
    for prim in Usd.PrimRange(stage.GetPseudoRoot()):
        mid = prim.GetCustomDataByKey(MODULE_KEY)
        if mid:
            role = prim.GetCustomDataByKey("jsdemo:role") or ""
            found[(mid, role == "handler_arm")] = str(prim.GetPath())
    rows = []
    for m in station["modules"]:
        h = m["role"] == "handler_arm"   # 放件臂与拧紧臂同型号，prim 名带 _handler 后缀
        path = found.get((m["id"], h))
        rows.append({"id": m["id"], "role": m["role"], "prim": path,
                     "name_ok": bool(path) and path.rsplit("/", 1)[-1] == prim_name(m["id"]) + ("_handler" if h else "")})
    return {"usd": str(USD_PATH), "station_id": station["station_id"], "modules": rows,
            "ok": all(r["name_ok"] for r in rows)}


def screenshot() -> bool:
    ViewportManager.set_camera_view("/OmniverseKit_Persp", eye=[2.0, -1.6, 1.15], target=[0.25, 0.0, 0.2])
    SimulationManager.set_physics_dt(1.0 / 60.0)
    SimulationManager.set_device("cpu")   # 见 twin.start：带 mimic 关节的夹爪在默认（GPU 动力学开着）状态下会卡死
    app_utils.play()
    for _ in range(args.settle_frames):
        SimulationManager.step()
        RenderingManager.render()
        app.update()
    if SCREENSHOT.exists():
        SCREENSHOT.unlink()

    async def capture():
        await capture_viewport_to_file(get_active_viewport(), file_path=str(SCREENSHOT), is_hdr=False).wait_for_result()

    app.run_coroutine(capture())
    frames = 0
    while not SCREENSHOT.exists() and frames < 600:  # 返回后文件可能尚未落盘（M0.2 实测）
        app.update()
        frames += 1
    app_utils.stop()
    return SCREENSHOT.exists()


exit_code = 1
try:
    if not (STATION_DIR / "station.yaml").exists():
        raise RuntimeError("缺少 build/station/station.yaml，先运行 python -m jsdemo run --yes")
    station, flow, ops, modules = load_outputs()
    build(station, ops, modules)
    TWIN_DIR.mkdir(parents=True, exist_ok=True)
    if not stage_utils.save_stage(str(USD_PATH)):
        raise RuntimeError(f"保存失败：{USD_PATH}")
    result = check_reopened(station)
    result["screenshot"] = str(SCREENSHOT) if screenshot() else None
    (TWIN_DIR / "scene_check.json").write_text(json.dumps(result, ensure_ascii=False, indent=2), encoding="utf-8")
    for r in result["modules"]:
        print(f"SCENE {r['id']:<12} -> {r['prim']}  {'OK' if r['name_ok'] else 'MISSING'}")
    print(f"SCENE usd={USD_PATH} screenshot={result['screenshot']} ok={result['ok']}")
    exit_code = 0 if result["ok"] and result["screenshot"] else 1
except Exception:
    import traceback

    traceback.print_exc()
finally:
    for _ in range(5):  # docs/04: 关闭前多 update 几次
        app.update()
    app.close(exit_code=exit_code)
sys.exit(exit_code)
