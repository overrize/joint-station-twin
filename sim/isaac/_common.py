"""Isaac 脚本共用：路径、读取流水线产出、场景尺寸常量。

本文件不 import 任何 Isaac 模块，可以在普通 Python 里被测试。
"""
from __future__ import annotations

import json
import os
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from jsdemo.io import load_modules, load_product, load_yaml  # noqa: E402
from jsdemo import layout  # noqa: E402

# 工站目录：默认 build/station（python -m jsdemo run --yes 的输出）。环境变量 JSDEMO_STATION 可换成别的产出目录
# （如 build/station_hub），孪生产出相应放到 build/twin_<目录名>，互不覆盖（M3.5b）
_station = os.environ.get("JSDEMO_STATION", "build/station")
STATION_DIR = (ROOT / _station).resolve()
# 布局：与流水线用同一份工站设计（station.yaml 的 layout_source 记着是哪份），场景按它搭
_st_yaml = STATION_DIR / "station.yaml"
LAYOUT = layout.use(((load_yaml(_st_yaml).get("layout_source") or {}).get("station_spec")) if _st_yaml.exists() else None)
FEEDER = LAYOUT["FEEDER"]
TWIN_DIR = ROOT / "build" / ("twin" if STATION_DIR == (ROOT / "build" / "station").resolve() else f"twin_{STATION_DIR.name}")
USD_PATH = TWIN_DIR / "JS-01.usd"

# 场景约定（单位 mm，Isaac 里换算成 m）。世界坐标系 = 工站坐标系（拧紧臂基座）。
# 位置类数值（桌面、垫高、输送线与停止位、供钉器、控制器等的位置）由 _apply_layout 按工站设计覆盖；这里留的是外形占位
SCENE = {
    "riser_height": 60.0,            # 机械臂基座下的垫高，治具顶面与基座同高
    "table_top_z": -60.0,
    "table_size": [1400.0, 1845.0, 40.0],     # M4 ②：加宽，放下料盘入口与放件臂；流水线（2026-10-26）：上下游延长
    "fixture_size": [160.0, 160.0, 60.0],    # 治具块，顶面中心即治具坐标系原点
    "product_radius": 45.0, "product_height": 40.0,
    "conveyor": {"x": 450.0, "y_span": [-1000.0, 760.0], "width": 240.0, "height": 20.0},   # M4 ②：上游延长、加宽，承载配套料盘；M4 ③：下游延长，送走整件
    "out_pallet": {"size": [130.0, 130.0, 12.0], "travel_mm": 170.0, "travel_s": 1.5},   # 下线托盘（M4 ③，demo 占位）
    "line_units": 3,                 # 流水线演示：场景里备几套零件（用户 2026-10-26：做三台）
    # 连续流动（用户 2026-10-26）：下一盘配件在排队位等；取空的料盘下沉到回流线回配套工站；成品进下游缓存位
    "line": {"tray_prestop_dy": -360.0, "tray_station_dy": -560.0, "return_drop_mm": 70.0, "exchange_s": 1.5,
             "buffer_y": [645.0, 505.0, 365.0],
             "screen": {"center": [-170.0, 420.0, 300.0], "size": [520.0, 296.0]},   # 线体看板大屏（中心 xyz、宽高 mm），朝 -y
             "upstream_station": {"center": [450.0, -1020.0], "size": [420.0, 60.0, 260.0], "label": "配套拣选工站（上游）"},
             "downstream_station": {"center": [450.0, 770.0], "size": [420.0, 40.0, 260.0], "label": "检测 / 下线工站（下游）"}},
    "tray_entry_dy": -360.0,          # 第一盘从排队位送进来
    # 快换架托轨（M4 ①）：轨道内侧离工具轴线 half_width（2F-85 本体在开合方向半宽 42.4 mm，实测），在法兰面下方 rail_drop 处
    "tool_dock": {"half_width_mm": 50.0, "rail_drop_mm": 10.0},         # 料盘从停止位往上游这么远处进场（mm，沿 y）
    "executor": {"radius": 20.0, "length": 150.0},   # 挂在机械臂法兰上；length = TCP 偏移（批头尖端）
    # M3.4 执行器分段外形（占位数，用户 2026-10-24 确认，非实测）：法兰起依次 本体 → 浮动头 → 批头杆，总长 = executor.length
    "executor_parts": {"body": {"radius": 20.0, "length": 70.0}, "bit": {"radius": 2.5, "length": 60.0}},
    # 产品外形（M3.5b 起）：不再写死，按工站 SOP 的零件简化体生成（build/station/assembly.yaml）
    "place_drop_mm": 40.0,          # 放件动画：零件从上方这个高度落到位
    "place_default_s": 1.0,         # SOP 里放件时长为「缺失」时演示用的时长（报告里标 assumed）
    # 螺钉外形（占位比例）：头径 2d、头高 0.7d；杆长取型号里的长度（如 M3x8 → 8 mm）
    "screw_head": {"d_ratio": 2.0, "h_ratio": 0.7},
    "hole_marker": {"r_ratio": 1.6, "thickness": 0.2},   # 孔位状态环：未拧灰、合格绿、不合格红
    "andon": {"xyz": [-280.0, 260.0, 152.0], "radius": 12.0},   # 控制器顶上的状态灯（M3.4）：绿=正常 黄=复拧 红=不合格
    "camera_point_offset": [0.06, -0.05, 0.09],     # 近景：相对孔位（m）
    "camera_product_offset": [0.09, -0.12, 0.17],   # 整件：相对产品中心（m）
    "camera_record_point_offset": [0.10, -0.09, 0.13],   # 录屏近景：比截图稍远，能看到批头下降（m）
    "datum_radius": 3.0,
    # 供钉器（M3.1b）：放在桌面上、输送段旁边，不挡输送段。xyz 为机身中心
    "feeder": {
        # 位置取工站设计，尺寸、路距、出钉口长度、出钉位取模块库外形（jsdemo.layout.FEEDER，与程序生成共用一份）
        "xyz": FEEDER["xyz_mm"], "size": FEEDER["size_mm"],
        "hopper": {"radius": 22.0, "height": 35.0, "pitch": FEEDER["lane_pitch_mm"]},   # 每路一个料斗，沿 x 排列在机身顶面
        "outlet": {"radius": 6.0, "length": FEEDER["outlet_len_mm"]},                   # 每路出钉口，朝 -y（机械臂一侧）
        "nest_size": FEEDER["nest_size_mm"],                                            # 出钉位（M4 取钉）
        "lamp_radius": 6.0,                                          # 每路状态灯
    },
    # 以下只用于外形占位（M3.1 补充），不参与任何工艺或节拍计算。xyz 为盒子中心
    "table_center_xy": [300.0, -132.5],
    "riser_size_xy": [180.0, 180.0],         # 机械臂垫高块，高度为 riser_height
    "baseplate_size": [300.0, 300.0, 10.0],  # 定位销基板，铺在桌面上，治具压在其上
    "float_head": {"radius": 24.0, "length": 20.0, "center_from_flange": 80.0},  # 本体与批头杆之间（M3.4），不改变 TCP
    "boxes": {
        "controller": {"xyz": [-280.0, 260.0, 40.0], "size": [120.0, 160.0, 200.0]},
        "interface_box": {"xyz": [-280.0, 100.0, 0.0], "size": [80.0, 100.0, 120.0]},
        "gateway": {"xyz": [-280.0, -260.0, -30.0], "size": [80.0, 100.0, 60.0]},
    },
}



def _apply_layout(S: dict, L: dict) -> None:
    """场景里由工站设计决定的量（位置、输送线、停止位、桌面）从布局写进 SCENE；其余是外形占位。"""
    env, fz = L["ENVELOPE"], L["FLOOR_Z"]
    S["table_top_z"] = fz
    S["table_size"] = [env["x"][1] - env["x"][0], env["y"][1] - env["y"][0], S["table_size"][2]]   # 桌面 = 工站可用空间
    S["table_center_xy"] = [(env["x"][0] + env["x"][1]) / 2, (env["y"][0] + env["y"][1]) / 2]
    S["riser_height"] = L["RISER_MM"]
    cv = L["CONVEYOR"]
    S["conveyor"].update({"x": cv["x"], "y_span": cv["y"], "width": cv["width"], "height": cv["top_z"] - fz})
    ty = L["TRAY_STOP"]["xyz_mm"][1]
    ln = S["line"]
    ln["tray_prestop_dy"] = L["TRAY_PRESTOP"]["xyz_mm"][1] - ty
    ln["tray_station_dy"] = L["TRAY_STATION"]["xyz_mm"][1] - ty
    ln["buffer_y"] = [b[1] for b in L["BUFFER"]]
    S["tray_entry_dy"] = ln["tray_prestop_dy"]
    ln["upstream_station"]["center"] = [cv["x"], cv["y"][0] - 20.0]     # 上下游工站外形：贴着输送线两端
    ln["downstream_station"]["center"] = [cv["x"], cv["y"][1] + 10.0]
    for role, box in S["boxes"].items():
        if role in L["PLACED"]:
            box["xyz"] = L["PLACED"][role]
    if "controller" in L["PLACED"]:   # 灯塔在控制器顶上
        c = L["PLACED"]["controller"]
        S["andon"]["xyz"] = [c[0], c[1], c[2] + S["boxes"]["controller"]["size"][2] / 2 + S["andon"]["radius"]]
    if "baseplate" in L["PLACED"]:
        S["baseplate_xyz"] = L["PLACED"]["baseplate"]


_apply_layout(SCENE, LAYOUT)

def prim_name(module_id: str) -> str:
    """USD prim 名只允许字母、数字、下划线：模块 id 中的 '-' 换成 '_'。原 id 另存在 customData。"""
    return module_id.replace("-", "_")


def mm(v):
    return [x / 1000.0 for x in v] if isinstance(v, (list, tuple)) else v / 1000.0


def load_station_product(station: dict) -> dict:
    """工站用的产品数据：SOP 路径下取工站目录里转换好的 product/，否则取 kb/product/<名>。"""
    src = station.get("product_source", {})
    if "product_dir" in src:
        d = STATION_DIR / src["product_dir"]
        return {"name": station["product"], "bom": load_yaml(d / "bom.yaml"), "fastening": load_yaml(d / "fastening_points.yaml"),
                "ctq": load_yaml(d / "ctq.yaml")["ctq"]}
    return load_product(station["product"])


def load_assembly(station: dict) -> dict:
    """零件外形与工步（jsdemo.sop.to_assembly）。孪生按 SOP 安装，没有就停下来说明怎么生成。"""
    src = station.get("product_source", {})
    if "assembly" not in src:
        raise RuntimeError("工站不是从 SOP 生成的，孪生无法按 SOP 安装。先运行：python -m jsdemo run --yes --sop kb/sop/joint-v1.sop.yaml")
    return load_yaml(STATION_DIR / src["assembly"])


def load_handling() -> dict | None:
    """放件规划（M4 ②，jsdemo.handling）：没有放件臂的工站返回 None。"""
    p = STATION_DIR / "handling.yaml"
    return load_yaml(p) if p.exists() else None


def load_handler_ops() -> list[dict]:
    p = STATION_DIR / "handler_program.json"
    return json.loads(p.read_text(encoding="utf-8")) if p.exists() else []


def load_outputs():
    st = load_yaml(STATION_DIR / "station.yaml")
    flow = load_yaml(STATION_DIR / "flow.yaml")
    ops = json.loads((STATION_DIR / "arm_program.json").read_text(encoding="utf-8"))
    return st, flow, ops, load_modules()


def screw_length_mm(screw_id: str) -> float:
    """螺钉型号里的长度：'M3x8-machine' -> 8.0（外形用）。"""
    return float(screw_id.split("-")[0].split("x")[1])
