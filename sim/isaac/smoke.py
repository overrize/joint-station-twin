"""任务 M2.1 冒烟测试：建场景、放一个方块、仿真 60 帧、正常退出（返回码 0）。

运行（Isaac Sim 6.2 装在 F:\\isaacsim）：
    F:\\isaacsim\\python.bat sim\\isaac\\smoke.py
    F:\\isaacsim\\python.bat sim\\isaac\\smoke.py --headless

只用 docs/isaac-api-notes.md 里已在本机核实的写法（官方 tutorials/getting_started/getting_started.py）。
"""
from __future__ import annotations

import argparse
import sys

ap = argparse.ArgumentParser()
ap.add_argument("--headless", action="store_true")
ap.add_argument("--frames", type=int, default=60)
args, _ = ap.parse_known_args()

from isaacsim import SimulationApp  # noqa: E402

app = SimulationApp({"headless": args.headless})

import isaacsim.core.experimental.utils.app as app_utils  # noqa: E402
import isaacsim.core.experimental.utils.stage as stage_utils  # noqa: E402
from isaacsim.core.experimental.objects import Cube, DistantLight, GroundPlane  # noqa: E402
from isaacsim.core.experimental.prims import GeomPrim, RigidPrim  # noqa: E402
from isaacsim.core.rendering_manager import RenderingManager  # noqa: E402
from isaacsim.core.simulation_manager import SimulationManager  # noqa: E402

exit_code = 1
try:
    stage_utils.create_new_stage()
    GroundPlane("/World/GroundPlane", positions=[0, 0, 0])
    DistantLight("/World/DistantLight").set_intensities(300)
    Cube(paths="/World/smoke_cube", positions=[0.0, 0.0, 0.5], sizes=0.1)
    cube = RigidPrim(paths="/World/smoke_cube")
    GeomPrim(paths="/World/smoke_cube", apply_collision_apis=True)

    SimulationManager.set_physics_dt(1.0 / 60.0)
    app_utils.play()
    app.update()
    for _ in range(args.frames):
        SimulationManager.step()
        RenderingManager.render()
        app.update()

    z = float(cube.get_world_poses()[0].numpy()[0][2])
    print(f"SMOKE frames={args.frames} cube_z={z:.3f} (started at 0.500, should have fallen)")
    exit_code = 0 if z < 0.5 else 1
except Exception:
    import traceback

    traceback.print_exc()
finally:
    app_utils.stop()
    for _ in range(5):  # docs/04: 关闭前多 update 几次，避免线程清理不干净
        app.update()
    app.close(exit_code=exit_code)
sys.exit(exit_code)
