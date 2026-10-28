"""把运行中会用到的外部资产拉到本机，合成单个 USD（含网格与物理），场景改为引用本地文件。

原因（M4 ①，2026-10-25 本机排查）：2F-85 资产带 payload，场景保存后再打开时 payload 在 play 中按需从 S3 拉取，
经代理时 Kit update 随机卡死（手指不动也卡；同一资产在新建场景里直接引用不卡）。

    F:\\isaacsim\\python.bat sim\\isaac\\fetch_assets.py
输出：build/assets/<名字>.usd
"""
from __future__ import annotations

import sys
from pathlib import Path

from isaacsim import SimulationApp

app = SimulationApp({"headless": True})
import isaacsim.core.experimental.utils.stage as stage_utils  # noqa: E402
from isaacsim.storage.native import get_assets_root_path  # noqa: E402

ROOT = Path(__file__).resolve().parents[2]
OUT = ROOT / "build" / "assets"
ASSETS = {"robotiq_2f85": "/Isaac/Robots/Robotiq/2F-85/Robotiq_2F_85_edit.usd", "ur5e": "/Isaac/Robots/UniversalRobots/ur5e/ur5e.usd"}

OUT.mkdir(parents=True, exist_ok=True)
code = 0
for name, rel in ASSETS.items():
    stage_utils.open_stage(get_assets_root_path() + rel)
    stage = stage_utils.get_current_stage(backend="usd")
    stage.Load()   # 载入全部 payload
    for _ in range(20):
        app.update()
    dst = OUT / f"{name}.usd"
    flat = stage.Flatten()
    ok = flat.Export(str(dst))
    print(f"ASSET {name} -> {dst} ok={ok} prims={len(list(stage.Traverse()))} default={stage.GetDefaultPrim().GetPath()}", flush=True)
    code |= 0 if ok else 1
app.close()
sys.exit(code)
