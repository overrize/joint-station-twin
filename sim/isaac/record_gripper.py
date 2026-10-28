"""运动学夹爪（M4 ①，用户 2026-10-25 选定）：在单独的空场景里用物理驱动 2F-85 手指，录下 0–45° 每 1° 各连杆相对夹爪根的位姿；
再生成去掉物理的显示用资产。工站孪生里夹爪只做显示，按这张表播放开合。

为什么不在工站里直接用物理：本机实测，工站场景里每步改写夹爪根位姿会让 Kit update 卡死，play 中切换 / 新建固定关节不生效，
物理合拢与快换无法同时做；空场景里单独驱动手指正常（本脚本即在空场景里录）。

    F:\\isaacsim\\python.bat sim\\isaac\\record_gripper.py     （先运行 fetch_assets.py）
输出：build/assets/robotiq_2f85_fingers.json、build/assets/robotiq_2f85_visual.usd
"""
from __future__ import annotations

import json
import math
import sys
from pathlib import Path

import numpy as np
from isaacsim import SimulationApp

app = SimulationApp({"headless": True})
import isaacsim.core.experimental.utils.app as app_utils  # noqa: E402
import isaacsim.core.experimental.utils.stage as stage_utils  # noqa: E402
from isaacsim.core.experimental.prims import Articulation, XformPrim  # noqa: E402
from isaacsim.core.simulation_manager import SimulationManager  # noqa: E402
from pxr import Gf, Usd, UsdGeom, UsdPhysics  # noqa: E402

ROOT = Path(__file__).resolve().parents[2]
A = ROOT / "build" / "assets"
SRC = A / "robotiq_2f85.usd"
LINKS = ["base_link", "left_outer_knuckle", "left_outer_finger", "left_inner_finger", "left_inner_knuckle",
         "right_outer_knuckle", "right_outer_finger", "right_inner_finger", "right_inner_knuckle"]
if not SRC.exists():
    print("RECORD 缺少 build/assets/robotiq_2f85.usd，先运行 fetch_assets.py")
    sys.exit(1)

# ---- 1) 空场景里用物理录姿态：夹爪根用固定关节钉在空中 ----
stage_utils.create_new_stage()
stage = stage_utils.get_current_stage(backend="usd")
stage_utils.define_prim("/World", "Xform")
stage_utils.add_reference_to_stage(str(SRC), "/World/G")
G = "/World/G/Robotiq_2F_85"
dj = UsdPhysics.FixedJoint.Define(stage, "/World/G/hold")
dj.CreateBody1Rel().SetTargets([f"{G}/base_link"])
dj.CreateLocalPos0Attr().Set(Gf.Vec3f(0.0, 0.0, 0.5))
dj.CreateLocalRot0Attr().Set(Gf.Quatf(1, 0, 0, 0))
XformPrim("/World/G", positions=[[0.0, 0.0, 0.5]], reset_xform_op_properties=True)
art = Articulation(G)
SimulationManager.set_physics_dt(1 / 60)
SimulationManager.set_device("cpu")
app_utils.play()
app.update()
fj = art.dof_names.index("finger_joint")
links = {n: XformPrim(f"{G}/{n}") for n in LINKS}
root = XformPrim(G)
table = []
for deg in range(0, 46):
    tgt = np.zeros((1, art.num_dofs))
    tgt[0, fj] = math.radians(deg)
    art.set_dof_position_targets(tgt)
    for _ in range(40):
        SimulationManager.step()
        app.update()
    rp, rq = root.get_world_poses()
    R0 = Gf.Matrix4d().SetTranslateOnly(Gf.Vec3d(*rp.numpy()[0].tolist()))
    R0.SetRotateOnly(Gf.Quatd(*rq.numpy()[0].tolist()))
    inv = R0.GetInverse()
    row = {"deg": deg, "q_deg": round(math.degrees(float(art.get_dof_positions().numpy()[0][fj])), 4), "links": {}}
    for n, x in links.items():
        p, q = x.get_world_poses()
        M = Gf.Matrix4d().SetTranslateOnly(Gf.Vec3d(*p.numpy()[0].tolist()))
        M.SetRotateOnly(Gf.Quatd(*q.numpy()[0].tolist()))
        L = M * inv   # 行向量约定：world = local * root
        t = L.ExtractTranslation()
        qq = L.ExtractRotationQuat()
        row["links"][n] = {"t": [round(v, 7) for v in t], "q": [round(qq.GetReal(), 7), *[round(v, 7) for v in qq.GetImaginary()]]}
    table.append(row)
app_utils.stop()
(A / "robotiq_2f85_fingers.json").write_text(json.dumps({"source": "record_gripper.py（空场景物理驱动，本机 Isaac 6.2）",
                                                         "root": "Robotiq_2F_85", "links": LINKS, "table": table}, indent=1), encoding="utf-8")
print(f"RECORD table {len(table)} 行，45° 时 q={table[-1]['q_deg']}", flush=True)

# ---- 2) 显示用资产：去掉 articulation、刚体、关节、碰撞（运行时只改各连杆的局部位姿） ----
st = Usd.Stage.Open(str(SRC))
# 网格在 instance 原型里，普通遍历进不去：先取消 instance，物理属性才能删干净（否则残留静态碰撞体，法兰对接时顶住夹爪）
for _ in range(5):
    inst = [p for p in st.TraverseAll() if p.IsInstanceable()]
    if not inst:
        break
    for p in inst:
        p.SetInstanceable(False)
drop = []
for p in st.TraverseAll():
    if p.IsA(UsdPhysics.Joint):
        drop.append(p.GetPath())
        continue
    for api in list(p.GetAppliedSchemas()):
        if api.startswith(("Physics", "Physx")):
            p.RemoveAppliedSchema(api)
for path in drop:
    st.RemovePrim(path)
out = A / "robotiq_2f85_visual.usd"
st.GetRootLayer().Export(str(out))
left = sum(1 for p in st.TraverseAll() if any(a.startswith(("Physics", "Physx")) for a in p.GetAppliedSchemas()))
print(f"RECORD visual -> {out}，删掉关节 {len(drop)} 个，剩余物理属性 prim {left} 个", flush=True)
app.close()
sys.exit(0)
