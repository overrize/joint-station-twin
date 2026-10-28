# Isaac Sim 6.2 API 笔记（本机 F:\isaacsim 核实）

状态：M2.1 完成（2026-10-23）。只记录已在本机源码/示例中核实的写法。冒烟测试：`sim/isaac/smoke.py`（有界面/无界面均返回码 0）。

## 已核实

| 项 | 写法 | 本机出处 |
|---|---|---|
| 入口 | `from isaacsim import SimulationApp`；`SimulationApp({"headless": False})` | standalone_examples/tutorials/getting_started/getting_started.py:18-20 |
| 场景/时间推进（替代 World） | `isaacsim.core.experimental.utils.stage.create_new_stage()`；`app_utils.play()`；`SimulationManager.set_physics_dt()` / `.step()`；`RenderingManager.render()`；`simulation_app.update()` | getting_started.py:22-102 |
| 立方体、地面、灯 | `from isaacsim.core.experimental.objects import Cube, DistantLight, GroundPlane`；`Cube(paths=..., positions=..., sizes=...)` | getting_started.py:25,51 |
| 圆柱 | `isaacsim.core.experimental.objects.Cylinder(paths, radii=, heights=, axes=, colors=, positions=/translations=)` | exts/isaacsim.core.experimental.objects/.../shapes/cylinder.py |
| 刚体/碰撞 | `RigidPrim(paths=...)`；`GeomPrim(paths=..., apply_collision_apis=True)` | getting_started.py:74-75 |
| 相机视角 | `from isaacsim.core.rendering_manager import ViewportManager`；`ViewportManager.set_camera_view("/OmniverseKit_Persp", eye=, target=)` | tutorials/state_machine/franka_stacking.py:397 |
| 视口截图 | `await capture_viewport_to_file(get_active_viewport(), file_path=..., is_hdr=False).wait_for_result()`（`omni.kit.viewport.utility`），用 `simulation_app.run_coroutine()` 执行（阻塞到完成）。**返回后文件可能尚未落盘，需轮询等待** | exts/isaacsim.gui.menu/.../edit_menu.py:1284；M0.2 实测 |
| 资产根路径 | `isaacsim.storage.native.get_assets_root_path()`；本机走国内镜像（DEC-ENV-001） | exts/isaacsim.storage.native/.../nucleus.py |
| 球 | `Sphere(paths, radii=, colors=, positions=/translations=)` | exts/isaacsim.core.experimental.objects/.../shapes/sphere.py |
| 引用 USD | `stage_utils.add_reference_to_stage(usd_path, path, *, prim_type="Xform", variants=None) -> Usd.Prim` | isaacsim.core.experimental.utils/.../impl/stage.py:442 |
| 打开 / 保存 / 新建 stage | `open_stage(usd_path) -> (bool, Usd.Stage)`；`save_stage(usd_path) -> bool`；`create_new_stage(*, template=None)`；`define_prim(path, type_name="Xform")` | 同上 :316 / :390 / :216 / :747 |
| 停止与退出 | `app_utils.stop()`；`simulation_app.close(exit_code=0)`；关闭前多 `update()` 几次 | getting_started.py:86；isaacsim.simulation_app/config/python_api.md:13 |
| 获取 articulation | `Articulation(path)`；`.dof_names`、`.link_names`、`.link_paths`、`get_dof_positions()`、`set_dof_position_targets(...)`、`set_default_state(dof_positions=[...])`、`reset_to_default_state()`、`get_world_poses()` | tutorials/state_machine/franka_stacking.py:398-516 |
| 碰撞检测 | `RigidPrim(paths, contact_filter_paths=[...])` + `get_contact_force_matrix(dt=)`（两两接触，形状 N×过滤数×3）；或 `set_enabled_contact_tracking(True)` + `get_net_contact_forces(dt=)`（总接触力）。均为 tensor 后端，需在 play 之后读取 | isaacsim.core.experimental.prims/.../rigid_prim.py:116 / 1357 / 1447 / 1489 |

## 骨架用的是已弃用 API

`sim/isaac/build_scene.py` 的 `from isaacsim.core.api import World` / `FixedCuboid` 等：6.2 中所有 `isaacsim.core.api` 示例都在 `standalone_examples/deprecated/`，当前示例（141 个）用 `isaacsim.core.experimental`。建议改用上表写法。

## UR 机型与运动规划配置（M2.2 选型依据）

- 资产库（6.2，探测 `/Isaac/Robots/UniversalRobots/<m>/<m>.usd`）存在：UR3、UR3e、UR5、**UR5e**、UR10、UR10e、UR16e、UR20、UR30；不存在：UR7e、UR12e、UR15
- 新栈 cuMotion（`isaacsim.robot_motion.cumotion`，`load_cumotion_supported_robot`）现成配置：**仅 franka、ur10**
- 已弃用的 Lula 栈（`extsDeprecated/isaacsim.robot_motion.motion_generation`，自 6.0 弃用）`policy_map.json` 含 UR3/UR3e/UR5/**UR5e**/UR10/UR10e/UR16e 等；UR5e 有完整的 URDF、robot_description、RMPflow 配置，末端坐标系 `tool0`
- 资产库中 6.2 的示例机器人还有一套 `/Isaac/Robots_Multiphysics/...` 路径（`isaacsim.robot_motion.examples/.../robots.py` 的 franka、ur10 用的就是它）

## IK 与运动规划

**Lula（已弃用栈，含 UR5e）** — `extsDeprecated/isaacsim.robot_motion.motion_generation`
- 配置：`interface_config_loader.load_supported_lula_kinematics_solver_config("UR5e")` 返回 `LulaKinematicsSolver` 的关键字参数（`robot_description_path`、`urdf_path`）
- 求解器：`LulaKinematicsSolver(robot_description_path, urdf_path)`（lula/kinematics.py:49）
  - `set_robot_base_pose(robot_position, robot_orientation)`（:363）
  - `compute_inverse_kinematics(frame_name, target_position, target_orientation=None, warm_start=None, position_tolerance=None, orientation_tolerance=None) -> (joints, success)`（:405）
  - `compute_forward_kinematics(frame_name, joint_positions, position_only=False)`（:388）
  - `get_joint_names()`（:372）
- UR5e 末端坐标系 `tool0`，cspace 为 shoulder_pan / shoulder_lift / elbow / wrist_1 / wrist_2 / wrist_3
- 自 6.0 弃用（extension.toml 的 deprecation 警告），6.2 仍随包提供
- 启用：`omni.kit.app.get_app().get_extension_manager().set_extension_enabled_immediate("isaacsim.robot_motion.motion_generation", True)` 后再 import；`target_orientation` 是四元数 wxyz（kinematics.py:449）
- M2.2 实测：arm_program.json 的 39 个 movel 位姿全部解出，正解回代误差 ≤ 0.0012 mm；资产路径小写 `/Isaac/Robots/UniversalRobots/ur5e/ur5e.usd`（`UR5e/UR5e.usd` 不存在）

**cuMotion（新栈，仅 franka / ur10）** — `isaacsim.robot_motion.cumotion`
- `load_cumotion_supported_robot("ur10")`；或 `load_cumotion_robot(directory, urdf_filename, xrdf_filename)` 加载自备的 URDF + XRDF
- 控制器：`RmpFlowController(cumotion_robot=, cumotion_world_interface=, robot_joint_space=, robot_site_space=, tool_frame=)`，配合 `mg.WorldBinding` 与 `CumotionWorldInterface`（franka_stacking.py:403-439）

## 已知坑

- Lula IK 默认容差：位置 1 mm、姿态 0.01 rad（`_default_position_tolerance` / `_default_orientation_tolerance`），乘上工具长度后末端误差可达 1 mm；要微米级需设 `bfgs_gradient_norm_termination=1e-12`、`bfgs_max_iterations=500`、`ccd_descent_termination_delta=1e-12`，并先默认容差粗解、再以粗解为 warm start 传 `orientation_tolerance`/`position_tolerance` 细化（M3.2 实测）
- Lula 从 warm start 收敛失败时会随机换初值（`max_num_descents` 默认 100），可能返回另一构型分支或绕工具轴翻转的等效解；轨迹中必须检查与上一解的连续性（M3.2 实测）。`max_num_descents=1` 反而在初始姿态附近直接失败
- Lula `compute_forward_kinematics` 返回的旋转矩阵列为坐标轴（`R[:, 2]` = 工具 z），与 `compute_inverse_kinematics` 的 wxyz 四元数约定一致（非对称姿态往返核实）
- 接触矩阵：`RigidPrim(links, contact_filter_paths=[...])` 之后还要 `set_enabled_contact_tracking([True])`（play 前），否则报 “did not match any rigid contact for filters”；过滤对象可以是静态碰撞体
- 机械臂连杆是 articulation 的一部分，也能用 `RigidPrim(...).get_world_poses()` 读物理位姿（tensor 后端）

- `define_prim("/World")` 要在任何 `/World/...` 子路径（如材质）之前调用，否则 `/World` 已作为无类型 prim 存在会报错（M3.1 实测）
- UR5e 资产（`ur5e.usd`）里没有 `tool0` prim；`wrist_3_link` 的坐标系与 Lula `tool0` 重合（零位正解对比，M3.1 实测），另有 `wrist_3_link/flange`（绕 z 转 90°）
- 关节初始姿态可写 USD 属性 `state:angular:physics:position` 与 `drive:angular:physics:targetPosition`（单位 deg），play 后生效

- 视口截图返回后文件可能尚未落盘，需轮询等待（M0.2 实测）
- 资产默认走海外 S3 会卡住主线程；本机已设国内镜像（DEC-ENV-001）
- 机械臂末端姿态若把绕竖轴朝向固定在世界系，手腕会随底座转动累积到限位附近、RmpFlow 卡住（另一个项目 factory_twin 实测）；轴对称工具（电批）宜让朝向随底座方位角转动

- **XformPrim.set_world_poses 的姿态精度**（M3.3 实测）：传入的四元数先转成 float32 再写进 quatd；写入后 USD `ComputeLocalToWorldTransform` 的偏航与目标差约 2e-5 rad（70 mm 处约 1.5 µm），原因未查明。需要 µm 级真值时从场景读回（如基准球世界坐标），不要用写入目标。用 acos 比较 float32 姿态会有约 0.02° 的假误差。

- **Cylinder 图元渲染只有十边形**（M3.4 截图）：RTX 下 `isaacsim.core.experimental.objects.Cylinder` 近景能看出棱角。近景物体（产品各层、螺钉、孔位环）改用 `UsdGeom.Mesh` 自建 96 段网格，静态碰撞用 `GeomPrim(..., apply_collision_apis=True).set_collision_approximations("none")`（三角网格）。挂在关节连杆上的形状（执行器）不能用三角网格碰撞（动态刚体），继续用 Cylinder。
- **运行中显隐与换材质**（M3.4 实测可用）：`UsdGeom.Imageable(prim).MakeVisible()/MakeInvisible()`、`XformPrim(path).apply_visual_materials(mat)`、`XformPrim.set_world_poses / set_local_poses` 在 play 期间改无碰撞的可视 prim，下一帧渲染即生效。隐藏不影响碰撞：要「看不见但参与碰撞」（如 bit_collider）就隐藏一个带碰撞的 prim。
- **静态碰撞体的移动与开关**（M3.4 / M3.5b 实测）：play 前后用 `XformPrim.set_local_poses / set_world_poses` 移动静态碰撞体、用 `GeomPrim.set_enabled_collisions([...])` 开关碰撞，下一步物理即生效（对照实验：一直关闭时方块穿过板子，空中打开后方块落在板上）。孪生按此实现放件：零件隐藏 + 碰撞关闭，落位后打开。注意自己写的时间判断别用浮点相等（第一次实验因此误判为无效）。
- **网格 prim 没有 xformOp**：`UsdGeom.Mesh.Define` 建的 prim 没有 translate 属性，`XformPrim(path).set_local_poses` 会断言失败；用 `XformPrim(path, reset_xform_op_properties=True)`。
- **无界面截图**：`capture_viewport_to_file` 在 headless 下也能出图，前提是截图前调用几次 `RenderingManager.render()` + `app.update()`（run 循环里 headless 不渲染）。

## 夹爪与多 articulation（M4 ①，2026-10-25 本机实测）

- **物理设备要显式设成 CPU**：默认状态 `SimulationManager.get_device()` 报 `cpu`，`is_gpu_dynamics_enabled()` 却是 `True`。场景里同时有 UR5e 与带 mimic 关节的 Robotiq 2F-85 时，`app.update()` 在几步内随机卡死（手指不动也卡）；`SimulationManager.set_device("cpu")` 后完整场景开合 600 步无卡顿，每步 0.013 s。twin.start 与 build_scene 截图前都已设置。
- **不要在 play 中每步 `Articulation.set_world_poses` 移动另一个 articulation 的根**：工站场景里会随机卡死（空场景里不卡）。
- **play 中改 `physics:jointEnabled`、删除 / 新建 FixedJoint 都不生效**：物理引擎不理会运行中的关节改动。快换不能靠切换关节。
- 结论：夹爪做成运动学的。`sim/isaac/record_gripper.py` 在空场景里用物理驱动手指，录 0–45° 每 1° 各连杆相对夹爪根的位姿（`build/assets/robotiq_2f85_fingers.json`），并生成去掉物理的显示版 `robotiq_2f85_visual.usd`；工站里夹爪按表插值显示，接触按几何判定（外夹：两指内侧间距 = 外径；内撑：外侧跨度 = 内孔）。
- **instance 原型**：资产网格放在 instanceable prim 的原型里，`Usd.PrimRange` 默认进不去。量网格要用 `Usd.TraverseInstanceProxies()`；要删掉原型里的物理属性，先 `SetInstanceable(False)` 再 `TraverseAll()`，否则残留静态碰撞体。
- 2F-85 实测几何：手指沿夹爪根 +z 伸出、开合沿 y；指尖在根部前方 148–162 mm（越合越远）；0° 内侧间距 87.1 mm、外侧 99.9 mm，45° 内侧 3.9 mm、外侧 16.7 mm；单指厚 6.4 mm、宽 22 mm。曾误把指根当指尖（取了 z 最小端），得到"外侧 66 mm"的错误结论，已更正。
- 资产本地化：`sim/isaac/fetch_assets.py` 把 2F-85 与 UR5e 合成单文件到 `build/assets/`，场景优先引用本地文件。

## 运行中换贴图（流水线看板大屏，2026-10-26 本机实测）

- play 中改 `UsdUVTexture` 的 `inputs:file`，视口里的贴图会刷新；同名文件覆盖重写后再 Set 一次也会刷新。
- 但每 0.25 s 就重写一次时，大屏一直显示占位色、看不到内容：改为只在看板状态变化时重写，并在两个文件名之间交替写。
- 文字贴图：Kit 自带 Python 有 PIL 12.3；中文用 `C:/Windows/Fonts/msyh.ttc` / `msyhbd.ttc`（▶ 等符号该字体没有，会显示成方框，用 ●）。

## 关节跟踪：位置目标要配速度前馈（放件臂抖动，2026-10-26 本机实测）

- UR5e 资产自带增益（`Articulation.get_dof_gains()`）：kp ≈ [5.4e5, 5.7e5, 5.9e5, 2.3e5, 2.3e5, 5.7e4]，kd ≈ [21.7, 23.6, 234.5, 90.4, 3.5, 0.2]。wrist_1 的阻尼相对刚度大。
- 只调 `set_dof_position_targets` 时速度目标为 0，阻尼项在运动全程把关节往回拽、顶到力矩上限：wrist_1 最多落后指令 34°，轨迹结束后以 170°/s 追上，画面上就是动作结束时「一抖」（放件下降段每次 7–10°）。
- 修正：每步同时 `set_dof_velocity_targets(指令差分 / DT)`，最后一步归零（`twin.py` 的 `_command`）。实测 wrist_1 滞后 ≤ 1.7°，静止时法兰每步位移 8.8 mm → 0.06 mm，节拍不变。
- 诊断：环境变量 `JSDEMO_TRACE=<文件>` 让孪生逐步记录两臂关节实测 / 指令 / 速度与法兰位置（jsonl）。

## app.update() 在 play 状态下也推进物理（2026-10-26 本机实测）

- 实测：play 后把某关节 kp 设 0、kd 设大、给速度目标 1 rad/s，`SimulationManager.step()` 与 `simulation_app.update()` 各让关节走 0.01667 rad（= 一个 DT）；`RenderingManager.render()` 不走（它内部临时把 `/app/player/playSimulations` 设为 False 再 update）。
- 原来孪生每个控制步是 `SimulationManager.step()` + `app.update()`，物理实际走两步；截图等文件落盘时的一串 `app.update()` 还会多走几十步。前者让关节跟踪看起来比实际好，后者在加了速度前馈后让放件臂在截图期间冲过目标、撞快换架托轨（只在录屏时出现）。
- 现在：控制步 = `SimulationManager.step()` + `RenderingManager.render()`；截图等待也只用 `RenderingManager.render()`。
- `app.run_coroutine(capture_viewport_to_file(...).wait_for_result())` 等待时内部也会 update 并推进物理：截图期间把 `/app/player/playSimulations` 设为 False，截完恢复。之后录屏与不录屏的两臂关节轨迹逐步完全相同（差值 0）。
- 一步一物理后暴露关节限速：UR5e 资产各关节最大速度 180°/s（`get_dof_max_velocities()`，rad/s）。直线运动转工具时 wrist_3 被要到 300°/s，落后 30°。movel 规划改为关节速度不超过上限的 90%，超了整段降速（twin.py JOINT_V_FRAC）。修正后两臂运动中与静止时关节误差都 ≤ 0.014°，单台节拍 108.65 → 109.43 s。
