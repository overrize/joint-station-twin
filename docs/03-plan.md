# 实施计划

每次只做第一个没勾选的任务。完成标准 = 该任务的验收条件 + `python -m pytest` 全部通过。

## M0 环境准备

- [x] **M0.1 确认基础环境**
  - Windows 11（Isaac Sim 6.2 不支持 Windows 10）
  - NVIDIA 驱动 ≥ 595.97
  - 安装 Visual Studio 2022 及 MSVC v143 构建工具（否则 Isaac 会报 WinError 126）
  - 系统 Python 3.11 以上：`pip install -r requirements.txt`
  - 验收：`python -m pytest` 显示 28 passed；`python -m jsdemo run --yes --ecn ecn/ECN-001.yaml` 状态为 accepted
  - 2026-10-23：Windows 11 专业版 26200、驱动 610.88、VS Community 2022 17.14（装在 F:\vs22，MSVC 14.38/14.44）均满足；系统 Python 用 C:\Python314（3.14.0，PATH 首位），Isaac 脚本用 F:\isaacsim\python.bat（内置 3.12.14）；28 passed，流水线 accepted。无异常

- [x] **M0.2 Isaac Sim 6.2 安装与自检**
  - 本机已装好 6.2.0 Windows 独立包，位于短路径 `F:\isaacsim`（路径过长会导致启动报错）；只做检测，不重装
  - 不使用 7.0 Alpha，不使用 Docker（Isaac 容器只支持 Linux，WSL2 不支持）
  - `F:\isaacsim\python.bat -m pip install pyyaml pytest`
  - 运行一个官方 standalone example，确认窗口能打开、渲染正常
  - 验收：在 `kb/decisions/` 写一条决策记录，固定能跑通的「Isaac 版本 + 驱动版本」组合，演示前不再升级
  - 2026-10-23：F:\isaacsim 已有安装只检测不重装；Isaac Python 补装 pytest 9.1.1；官方 getting_started 示例带窗口运行返回码 0，用 Isaac 视口截图确认渲染正常；写入 kb/decisions/DEC-ENV-001.yaml 并经用户确认。遇到：视口截图返回后文件可能尚未落盘，需要等待；资产需走国内镜像（环境变量已设）

## M1 流水线（已完成，随包提供）

- [x] **M1.1 数据与流水线**：产品数据、模块库、工艺库、九个阶段、四个关口
- [x] **M1.2 验收测试**：方案合法性、可达性、节拍、状态机、职责边界、一致性、擦除、设计变更、曲线判定、自标定

## M2 Isaac API 确认

- [x] **M2.1 写 `docs/isaac-api-notes.md`**
  - 对照本机 standalone_examples，确认并记录：SimulationApp 入口；World；创建立方体、圆柱、球；资产库根路径；引用 USD；保存 stage；获取 articulation；IK 或运动规划（Lula / RMPflow）的用法
  - 列出本机资产库里可用的 UR 系列型号，以及哪些型号有现成的运动规划配置
  - 验收：新建 `sim/isaac/smoke.py`，创建场景、放一个方块、仿真 60 帧、正常退出，返回码 0
  - 2026-10-23：笔记逐项注明本机出处；smoke.py 有界面/无界面均返回码 0（方块从 0.50 m 落到 0.05 m）；28 passed。遇到：骨架用的 isaacsim.core.api（World、FixedCuboid）在 6.2 已弃用，当前写法是 isaacsim.core.experimental；UR5e 资产存在，但现成运动规划配置只在已弃用的 Lula 栈里，新栈 cuMotion 只有 franka/ur10，留给 M2.2 选定

- [x] **M2.2 选定机械臂型号**（需要我签字）
  - 若 UR5e 没有现成运动规划配置，列出可用替代，等我选定；选定后在 `kb/modules/` 新增或修改模块 YAML，让 composer 重新选择，不在 Isaac 脚本里写死型号
  - 验收：`python -m pytest` 仍全部通过
  - 2026-10-24：选 A（UR5e + Lula），决策记录 kb/decisions/DEC-ARM-001.yaml（已确认）；arm-ur5e.yaml 填入核实的资产路径与 isaac_ik 字段，型号不变；Lula 逆解 39 个 movel 位姿全部解出（tool0，未计电批 TCP 偏移，M3.2 复核）；28 passed，流水线 accepted

## M3 数字孪生

- [x] **M3.1 生成工站 USD**（`sim/isaac/build_scene.py`，原计划中的 M5）
  - 先运行 `python -m jsdemo run --yes` 生成 build/station
  - 尺寸和位置全部来自 `sim/isaac/_common.py` 的 SCENE 和模块库，不另写数字
  - 验收：生成 `build/twin/JS-01.usd`；重新打开后 prim 名与 station.yaml 的模块 id 对应；保存一张截图 `build/twin/scene.png`
  - 2026-10-24：build_scene.py 改用 isaacsim.core.experimental；有界面/无界面均返回码 0；重新打开后 10 个模块全部对上（prim 名 = id 中 '-' 换 '_'，原 id 存 customData jsdemo:module_id），结果写 build/twin/scene_check.json；截图 scene.png。SCENE 补了占位尺寸（桌面中心、垫高、基板、浮动批头座、三个电气盒），只用于外形。遇到：/World 要先于材质创建；UR5e 资产没有 tool0 prim，wrist_3_link 与 Lula tool0 重合，执行器挂在它下面；供钉器按给定中心高度悬空约 60 mm（M3.1b 已处理）；USD 引用国内镜像上的 UR5e，重新打开需联网

- [x] **M3.1b 供钉器建模**（用户 2026-10-24 提出：独立小型机器、每种螺钉一路、可接入 MES）
  - 用户选定：吹气送钉（机械臂程序与节拍不变）；MES 数据在演示中要动起来，放到 M3.2
  - 2026-10-24：feeder-fd3l.yaml 增加 feed_method、lane_assignment（第 i 路 = screw_kinds 第 i 种）、mes 数据点（经 GW-1，协议缺失）；lane_capacity_pcs 缺失，演示余量递减前需提供；孪生里供钉器放到桌面、移出输送段，3 路料斗/出钉口/状态灯，lane prim 的 customData 记螺钉型号；10 个模块仍全部对上；28 passed。送钉软管随臂运动未建模（留 M4）

- [x] **M3.2 在孪生里跑完一整件**（`sim/isaac/run_flow.py`，原计划中的 M6）
  - 按 arm_program.json 运动；hold 段停留 fasten_s_base 秒；回放 curves 并用 `jsdemo.procmodel.judge` 判定
  - 验收：`build/twin/twin_report.json` 中 ok 为 true；13 个点全部到达；无碰撞
  - 实测节拍与估算偏差超过 20% 时，**记录并告诉我**，不要自动修改模块库里的节拍参数
  - 新增 `tests/test_twin_report.py`：报告不存在时跳过；存在时检查字段完整、ok 为 true
  - 2026-10-24：run_flow.py 有界面/无界面均返回码 0，twin_report.json ok=true；13/13 到位（实测 TCP 误差 ≤ 0.0125 mm，判据 0.1 mm），无碰撞（碰撞检测自检 --collision-selftest 能报出），13/13 判定合格；直线运动路径偏离 ≤ 0.114 mm（下压段 ≤ 0.008 mm）；网关数据流 gateway.jsonl（供钉器 MES 数据点 + 紧固结果），供钉器余量 47/46/44（容量 50 为演示数）；新增 tests/test_twin_report.py，33 passed。**节拍偏差 +40.8%（实测 57.9 s vs 估算 41.1 s，已记录、已告知，未改模块库；用户 2026-10-24 决定：保持现状，演示时说明为保守的仿真节拍）**：每点下压 30 mm 按 slow 0.03 m/s 需 1.08 s，单点动作合计约 2.1 s，估算 move_s_avg 1.2 s；另有首段 1.8 s、回 home 3.2 s。codegen 的运动参数提为常量与孪生共用（URScript 输出不变）。遇到：Lula 默认容差（1 mm / 0.01 rad）× TCP 150 mm 致末端偏 0.6~0.9 mm；只收紧容差会解不出，需收紧 BFGS 终止条件并「默认容差粗解→紧容差细化」；细化失败时 Lula 随机换初值会跳到翻转解（肘翻转撞治具、wrist_3 跳 179°），需检查与粗解/上一节点的连续性；连杆要 set_enabled_contact_tracking 才能取接触矩阵

- [x] **M3.3 现场自标定演示**（`sim/isaac/calibrate_demo.py`，原计划中的 M7）
  - 默认偏差：平移 [0.8, -0.5, 0.2] mm，旋转 [0.1, -0.05, 0.3]°；探测噪声 0.02 mm
  - 验收：`build/twin/calibration_report.json` 中标定后平移误差 < 0.1 mm；用标定后位姿重跑整件，ok 为 true
  - 2026-10-24：Twin 类从 run_flow.py 抽到 sim/isaac/twin.py（重构后 M3.2 重跑结果逐项一致）；calibrate_demo.py 有界面/无界面均返回码 0：机械臂按名义位姿下探 DT1~DT3，标定前平移误差 0.964 mm / 0.319°（不标定时孔位最大偏 1.158 mm），标定后 0.013 mm / 0.022°；同一次仿真内用标定位姿重跑整件 ok=true，13/13 到位、无碰撞、孔位误差 ≤ 0.033 mm；run_flow.py --fixture-pose 已实现（单独重跑同样 ok）；新增 tests/test_calibration_report.py，38 passed。探测模型：实测 = 场景基准球实际中心 + 0.02 mm 噪声，未模拟触碰传感。遇到：XformPrim.set_world_poses 写入后 USD 世界矩阵偏航与目标差约 2e-5 rad（基准球偏约 1.5 µm），真值改取场景读回；arm_program 只带治具偏航，标定出的横滚/俯仰只体现在孔位坐标上。**已知问题（用户 2026-10-24 指出）**：产品是实心圆柱占位且无碰撞，执行器是 r20 实心圆柱，下压时穿进产品，看不出拧螺丝；M3.2/M3.3 的「无碰撞」不含产品

- [x] **M3.4 装配工件与拧螺钉可视化、异常演示**（用户 2026-10-24 提出：看不出在拧螺丝、执行器与产品穿模；要能看到螺钉打进去，有 1% 异常概率，体现纠错与问题记录）
  - 用户选定：端盖在拧 E 组前出现；机螺钉 C/E 组浮高时执行器自动复拧一次（DEC-NG-001，D 组不复拧）；执行器用占位尺寸（本体 r20×70、浮动头 r24×20、批头 r2.5×60）；DFM 未关闭的 D3/C12 如实建模
  - 2026-10-24：产品改为分层外形（壳体、编码器座、PA66 支架、环形驱动板、端盖，层高由紧固点 z 反推，径向为占位）并加静态碰撞（端盖无碰撞）；障碍物 C12/J3 按产品数据放置，碰撞半径 0.5 mm = DFM-01 下限 3.0 − 批头半径 2.5，使孪生判据与 DFM 一致；执行器分本体/浮动头/批头，可见批头随拧入上下移动并转动，碰撞批头固定在起始位置；每点一颗螺钉（头 2d×0.7d、杆长取型号），吹到批头后随 TCP 运动，hold 期间按曲线角度拧入，深度 = 曲线角度/理论角度（浮高停在 60%）；孔位状态环 灰/黄/绿/红，工站状态灯 绿/黄/红；近景截图在机械臂退回后拍
  - 异常：每颗螺钉每次拧按 --fault-rate 抽签（种子 + 序列号，可复现），只注入 synth_curve 支持的滑牙/浮高，--inject 指定点位；判定仍用 procmodel.judge；复拧在执行器侧，机械臂程序不变（106 条，URScript 不变）；问题记录 issues*.jsonl（模式、每次拧的曲线、处置、CTQ、状态「待复判」），网关加 exec_retry / issue_opened / cap_placed 事件
  - 结果（均返回码符合预期）：基线 D3 批头撞 C12、未到位（被顶偏 1.55 mm），与 DFM-01 一致，记干涉问题、整件 NG；--dfm-fix（C12 沿孔位方向外移 1.2 mm，只改场景）13/13 到位、无碰撞、整件 OK；注入 D2 滑牙 → 不复拧、整件停在 D2；E3 浮高→复拧成功 → 整件 OK（节拍 +2.4 s）；C2 两次浮高 → 标 NG 继续、整件 NG，问题关联 CTQ-2；1% 抽签（种子 0，TWIN-0002）抽中 C1 滑牙 → 标 NG 继续；自标定重跑改为带 --dfm-fix，仍 ok；碰撞自检仍能报出。tests 改写 test_twin_report.py（基线：孪生干涉 = DFM 未关闭问题且已记问题；dfmfix：原 M3.2 验收），新增 test_twin_faults.py 与状态机复拧测试，45 passed。截图在 build/twin/media/
  - **发现（待用户决定）**：① 整件停止时 URScript 在 branch_on_fault 处直接 halt，on_ng 里的 retract 没有被机械臂程序执行，批头停在螺钉上；② 流程缺「放端盖」一步；③ 机械臂程序没有干涉检测，D3 撞上后仍继续；④ 复拧的反转参数缺失（执行器侧），复拧时长为演示估计

- [x] **M3.5a SOP 交接标准**（用户 2026-10-24 提出：每次传入的装配内容与结构不同，上游 agent 输出工站 SOP，孪生按 SOP 安装；要给 agent 开发提需求、立输出格式标准）
  - 用户选定：零件外形用简化体（圆柱/环/长方体）+ 可选网格附件，本期只用简化体；标准放项目内并发布可分享页面
  - 交付：`docs/06-sop-contract.md`（给 agent 开发方的需求与格式标准）、`kb/contracts/sop-v1.schema.json`、`jsdemo/sop.py`（校验 + 转换到 build/sop/）、`kb/sop/joint-v1.sop.yaml`（示例）、`tests/test_sop.py`
  - 验收：示例 SOP 校验无错误；转换结果与 kb/product/joint-v1 一致（点位、组、障碍物、装配顺序、CTQ、不合格与复拧策略）；故意改坏的 SOP（端盖先放、引用不存在的零件/螺钉、点位不在零件表面、必填项缺失）都能被查出并给出原因；不改孪生
  - 2026-10-24：交付标准文档、JSON Schema、校验转换模块（纯 Python，不新增依赖；四级结果 error/blocker/review/warning，转换输出 build/sop/<product>/）、joint-v1 示例 SOP、tests/test_sop.py（15 项，含 12 类改坏用例），60 passed；标准发布为私有页面 https://claude.ai/artifact/GoXFpCPgYxmeg6RLoFGnrw。示例校验：0 error、12 blocker（放件执行者/时长缺失、open_items）、1 review（D3/C12，与 DFM 一致）、18 warning。发现：占位外形下螺钉长度不够（W-STACK，每个点）；现有数据 D 组 part 字段指承接件，与 C/E 组语义不一致
- [x] **M3.5b 孪生按 SOP 安装与运行**：场景按 SOP 零件生成，运行按 SOP 工步执行放件与拧紧；joint-v1 结果与 M3.4 一致，另换一个结构不同的示例产品能装能跑
  - 2026-10-24：流水线加 `--sop`（校验有 error 即打回，状态 stopped:sop_invalid；策略与治具取自 SOP；借用治具记 DEC-004 草稿；SOP 里整件停止的组记 DEC-003-<组> 草稿、理由写缺失）；joint-v1 走 SOP 与走 kb 的机械臂程序、URScript、业务流、工站、文档、曲线逐字一致。孪生场景改为按 assembly.yaml 生成零件（简化体 → 网格/Cube，元件即障碍物，状态环挂在被紧固件下），零件初始隐藏、碰撞关闭，运行中按 SOP 放件工步从上方落位后打开碰撞（upstream 开始即在位；时长缺失时演示 1 s 并标 assumed）；去掉 M3.4 写死的产品分层与端盖逻辑。joint-v1 结果与 M3.4 一致（基线 D3 撞 C12、dfmfix/注入/1%/自标定/碰撞自检均同前），节拍 +5 s 为 5 个缺失时长的放件工步。示例二 kb/sop/sensor-hub-v1.sop.yaml（长方体、4 根支柱、PC 自攻、绕 z 转 90° 装夹、6 个零件 3 组 10 点）：流水线 accepted，孪生 10/10 到位、无碰撞、整件 OK；注入 K1 滑牙 → 整件停在 K1。JSDEMO_STATION 环境变量切换工站目录，孪生产出到 build/twin_<目录名>。新增 tests/test_sop_station.py，66 passed。遇到：先前判断「play 中切换碰撞开关无效」是实验脚本的时间比较 bug，修正后对照实验确认有效；网格 prim 没有 xformOp，移动前要 reset_xform_op_properties

- SOP 标准 2026-10-25 定为 demo 版（用户：demo 阶段按我的方案，不确定处标 demo），demo 标注见 docs/06-sop-contract.md 第 9 节
- 挂起（用户 2026-10-24：先做 SOP）：① 整件停止时 URScript 直接 halt、未执行 retract；② 流程缺「放端盖」（M3.5 中变为 SOP 放件工步）；③ 机械臂程序无干涉检测；④ 复拧反转参数缺失

- 挂起项处理（用户 2026-10-25：①③④ 按建议做；② 改为输送线送件 + 第二台机械臂放件装配）
  - [x] ① 整件停止先退回再停：codegen 的 branch_on_fault 带 `on_stop`（退回接近位 + 回原位），URScript 在 `halt` 前执行；孪生同样执行并加处置检查 retract_home_before_stop。D2 滑牙复跑：handling ok，批头不再留在螺钉上，节拍 28.8 → 32.5 s。2026-10-25，75 passed
  - [x] ④ 复拧反转参数：flow.EXECUTOR_REVERSE_DEMO（反转退到起点、300 rpm、停顿 0.3 s、同参数重拧，source=demo_placeholder）写入业务流 executor_retry.reverse 和 C/E 组参数包 retry 段，作业指导书注明「反转参数为 demo 占位」；孪生反转时长 = 已拧入角度 / 反转转速 + 停顿。E3 浮高复跑：复拧等待 3.2 s，合格。2026-10-25
  - [x] ③ 机械臂程序离线干涉检查：新增 jsdemo/clearance.py，执行器包络（kb/modules/exec-ts1.yaml envelope，demo 占位，测试核对与孪生场景一致）+ 拿着的螺钉，沿程序每段直线 1 mm 步长扫过当时已放零件和元件的简化体。流水线验收加 clearance 项（只在 --sop 有外形时检查，否则如实记跳过），结果写 clearance.yaml；与 DFM 已记录问题对得上的记 known，新干涉判不通过。joint-v1：只有 D3 碰 C12（批头深 0.9 mm、螺钉头 0.4 mm），与孪生基线一致；sensor-hub 无干涉；人为在 E1 旁加障碍能查出。演示页测试验收加「干涉检查」。新增 tests/test_clearance.py，80 passed。2026-10-25
  - [x] ② 输送线送件 + 放件机械臂（用户 2026-10-25 确认方案）：
    - 流水线：新模块 GR-VG1 真空吸盘、TR-KT6 六穴配套料盘（kb/modules，demo 占位）；composer 在 SOP 有归放件臂的放件工步（actor robot，或缺失时演示交给放件臂并标 actor_assumed）时加 handler_arm（同型号 UR5e）+ gripper + tray，记 DEC-005 草稿。新增 jsdemo/handling.py：料盘穴位、吸取点（覆盖原点的顶面中心 / 环面上避开元件的点）、搬运高度、握手点；生成 handling.yaml、handler_program.json/.urscript（DI3 料盘到位、DI2 区域空闲、工具 DO0 真空、整数寄存器 0 放件计数）。拧紧臂程序在 C1/D1/E1 前退到停靠位（肩部转 90°）、给区域空闲、等放件计数 2/4/5，自身无放件动作。验收的 boundary/reach 覆盖放件臂程序；离线干涉检查加放件路径（吸盘 + 吸着零件按自身圆柱 / 环简化体），驱动板支架套编码器最小间隙 0.5 mm。
    - 孪生：Twin 拆出 Rig（每台臂的关节、接触、Lula 逆解；放件臂用 set_robot_base_pose 在世界系解算，本机核实），两台臂都查接触，放件臂的接触过滤含拧紧臂连杆；料盘随输送线进场（2 s），零件吸住后随 TCP 走，松开时记放置偏差再落到装配位姿，吸盘抬起后再开碰撞。两臂按握手串行。
    - 结果（dfmfix）：5 件全部放到位，放置偏差 ≤ 0.012 mm，零碰撞，13/13 合格；节拍 138.2 s = 放件臂 62.5 s + 拧紧臂 69.7 s + 上下料 6 s（串行，未做两臂并行）。新增 tests/test_handling.py，88 passed。2026-10-25

- 细化（用户 2026-10-25）：① 取件改用夹爪：Robotiq 2F-85 两指（环形件内撑、实心件夹外圆），端盖 Ø90 无孔夹不了 → 放件臂加快换、端盖用吸盘；手指物理合拢、接触后锁定；② 螺钉每颗去供钉器出钉口取（DEC-FEED-001）。顺序：先取钉，再夹爪
  - [x] ② 取钉：新增 jsdemo/layout.py（供钉器、出钉位、放件臂、料盘布局，流水线与孪生共用；demo 占位）；拧紧臂每颗螺钉先到对应料道出钉位（接近 25 mm → 快降到 3 mm → 慢降 → 等 DI4 批头真空确认 → 抬起）再去孔位，程序里统一叫 feed，不出现 golden 禁词。供钉器模块 feed_method 改 nest_pick。孪生：出钉位上先立起螺钉，批头落下吸住再带走；取钉位到位判据 0.3 mm（出钉位有导向、批头浮动；第 2、3 路在臂展 660–710 mm 处仿真关节静差约 0.125 mm，孔位仍按 0.1 mm）。离线干涉检查加取钉往返段。dfmfix：13 颗都取到、零碰撞、13/13 合格，取钉每颗约 3.2 s，节拍 182.9 s（放件臂 62.5 + 拧紧臂 114.4 + 上下料 6）。新增 tests/test_feed.py，test_reach 计数加上取钉位姿，91 passed。2026-10-25
  - [x] ① 2F-85 夹爪 + 吸盘快换（用户 2026-10-25：按外形自动选末端；运动学夹爪）：新模块 GR-2F85（本机实测开合表、手指与本体尺寸）、TC-2 快换架；handling.plan 对每个零件试 2F-85 外夹 / 内撑（指尖从底面往上找第一个放到位不碰已装零件、接触 ≥ 5 mm 的高度，开合方向每 15° 试），不行再试吸盘，都不行报出来，理由写进 why_tool。joint-v1：壳体内撑 Ø68、编码器外夹 Ø38、支架内撑 Ø39（指尖抬到编码器顶面上方，接触 6.4 mm），驱动板（1.6 mm 太薄）与端盖（Ø90 超开口）用吸盘，快换 1 次。放件臂程序：开机先去快换架接工具，每条运动带 tcp_mm，下降前预置手指开口。离线干涉检查按所选末端（手指 3 圆 ×2 + 本体圆柱）。孪生：物理合拢 + 快换在工站场景里做不到（见 docs/isaac-api-notes.md 夹爪一节：默认 GPU 动力学开着会卡死、运行中移动 articulation 根会卡死、运行中切换关节不生效），改为运动学夹爪：空场景里录的连杆姿态表插值显示，接触按几何判定；同时物理设备显式改 CPU。dfmfix：5 件放到位（偏差 ≤ 0.019 mm）、零碰撞、13/13 合格，节拍 193.1 s（放件臂 72.8 + 拧紧臂 114.4 + 上下料 6）。92 passed。2026-10-25

- 用户 2026-10-25：① 核实 Tesollo DG-3F-M 三指手资产、量手指；② 快换台架穿模与干涉；③ 做流水线：配件送来装配后由传送带送走，再装下一台，共两台
  - [x] ① DG-3F-M：vendor/tesollo_model（BSD-3，只读模型文件）。dg3f_m.usd 在 6.2 能打开，articulation 根 root_joint，12 个转动关节（每指 j1 偏转 / j2 侧摆 / j3、j4 屈曲），17 个刚体，无 mimic；资产驱动刚度很弱（0.2–2），物理驱动会下垂。实测：安装座 Ø62×12，手掌立柱约 71×71、高约 140 mm，指根在手掌上方 71 mm、半径约 27 mm 处，手指截面约 24×20 mm，伸直时指尖离安装面约 250 mm；手指 2/3 偏转 ±60° 可三指均布。对 joint-v1：只有壳体（外抓 Ø90 / 内撑 Ø68）能抓，端盖勉强（指尖贴壳体外圆），编码器、支架抓不了（手指太粗）。未接入，等用户决定是否作为第三种末端
  - [x] ② 快换台架：整块托板横穿工具本体 → 改为叉形托轨（两侧托法兰盘下沿，内侧离轴线 50 mm）+ 后方横梁与立柱，带碰撞并纳入接触监测；监测随即查出放件臂大臂下探时扫到相邻工位托轨。试过把工位抬到 300 mm（立柱变高，撞得更多），最终保持 150 mm、两工位拉开到 x=640 / 900。dfmfix 零碰撞，13/13 合格。2026-10-25
  - [x] ③ 两台连续装配的流水线（用户 2026-10-25：放件臂吸起整件放到下游；串行）：layout 加 OUT_STOP（治具下游输送线托盘）与 TOOL_VIA（去快换架前的中间点，直线会过放件臂基座正上方、逆解失败）。拧紧臂程序末尾退到停靠位、给 unit_done（DO2）；放件臂程序加下线段（等 unit_done → 吸端盖顶面中心提起整件 → 放到下游托盘 → 工具放回快换架，每个循环从空手开始）。场景备两套零件（{产品}_u2、screws_u2）与下线托盘，输送线下游延长到 y=420。孪生 run_line：每台 放件 → 拧紧 → 下线 → 输送线送走（整件隐藏、托盘回位）→ 空料盘退回、第 k 套零件随料盘送进来；DFM 外移对每台生效，每台报告独立。run_flow 加 --units、--record-every（每 4 步截一帧 = 15 fps，录制时间减半，播放仍为正常速度）。结果：两台都 13/13 合格、零碰撞，周期 217.5 / 220.2 s，合计 437.6 s；视频 build/twin/media/dfmfix_line/TWIN-L_line.mp4（7 分 24 秒）。新增 tests/test_line.py。2026-10-26

- 用户 2026-10-26：① 节拍加速；② 流水线（线体看板、前后工站、连续流动感）；③ 模块化：上游 LLM 输出什么才能直接说明工站布局与控制（固件形式 PLC ST + 参数文件可以）
  - [x] ① 节拍加速：运动参数 快移 0.25 → 0.8 m/s、2 m/s²，关节 2 rad/s、3 rad/s²，下压只留最后 5 mm 慢速（200 → 118.5 s）；两臂并行：twin 把推进物理的函数改成生成器（原名保留为一口气跑完的包装），调度器每个物理步让两臂各推进一步，靠 zone_free / placed_count / unit_done 协调，run_flow 默认并行（--serial 切回）。并行暴露两处真干涉并修掉：放件臂取料盘最靠治具的穴时与去供钉器的拧紧臂小臂相撞 → 料盘停止位 y -330 → -430；放件臂原位实测（Lula 正解）定为肩部 +90°（手臂伸向 -y 桌边）。放完一批在料盘上方待命点就发计数、不回原位；料盘由输送线任务独立进场，放件臂先接工具。试过等放件时先取钉 → 拧紧臂伸向供钉器时小臂横在治具上方，与放件臂相撞，撤回。结果 109.1 s（单台，零碰撞，13/13），拧紧臂仍等待 14.9 / 19.0 / 3.1 s：D 组前要放支架、换手、放驱动板，必须等拧紧臂让出治具。2026-10-26
  - [x] ② 流水线（用户 2026-10-26：看板两种都做、做三台）：桌面与输送线上下游延长（y -1000…760）；上游「配套拣选工站」、下游「检测 / 下线工站」外形 + 标牌（PIL 文字贴图）；料盘 A / B 轮换：下一盘在排队位等，当前盘一取空就下沉到回流线、排队的补进来、回流的装上后一台配件再出来排队；每台一个下线托盘，成品沿输送线进下游缓存位并留在那里，送出与下一台装配同时进行（调度器的后台任务）。线体看板：网关事件推出状态（开工、每点结果、下线、送达），录屏时叠加在画面右上角，场景里拧紧臂后方立一块大屏（运行中改 UsdUVTexture file 刷新，本机实测；只在状态变化时重写、两个文件名交替，否则一直显示占位色），演示页加「线体看板 · 孪生回放」（同一套规则，可播放 / 拖动）。三台：都 13/13 合格、零碰撞，开工间隔 115.7 s，合计 347 s。tests/test_line.py 加连续流动的开工间隔检查。2026-10-26
  - [x] ③ 模块化（用户 2026-10-26：链路里只有上游方案 agent，布局坐标、治具建议与工站设计都由它给）：新标准 jsdemo.station/1（docs/07-station-contract.md、kb/contracts/station-v1.schema.json、示例 kb/station/joint-v1.station.yaml 由现有工站反写）；校验器 jsdemo/station.py（型号在库、位姿在工站空间内、两臂基座间距、治具 use / suggestion（≥3 不共线基准）与 SOP 一致、料道覆盖螺钉、工步分配与资源类型、料盘穴位、可达（紧固点 / 出钉位 / 料盘 / 治具 / 出件位 / 快换工位）、并行必须有互斥分区、缺失与来源、引用的 SOP 不能签字则随之不能签字），to_layout 与现有布局常量一致。模块库补接口描述（io 段：信号名、方向、类型、含义；臂与控制器标容量）。固件生成 jsdemo/firmware.py：io_map.yaml（模块 out→%I、in→%Q 自动分配；臂间握手由 PLC 转接，编号与 URScript 一致）、station.st（IEC 61131-3 ST：总状态机、治具区互斥、换盘、送出、灯塔）、config/（供钉器料道、快换工位、料盘穴位、抓取表）。新增 tests/test_station_spec.py、tests/test_firmware.py。踩坑：YAML 键名 on 被解析成 True，改用 parent。PLC 程序未在孪生里执行
  - [x] ③ 收尾：流水线直接读工站设计（用户 2026-10-26）。jsdemo/layout.py 不再写布局数值，按工站设计算出（--station 指定，缺省取示例）；供钉器外形移到模块库 geometry；治具位姿取 fixture.pose；原位 / 停靠位取各臂 home / park_joints_deg。流水线 --station：先校验，有 error 打回（stopped:station_invalid），SOP 取 meta.sop；产出 station.yaml 记 layout_source，孪生 _common 按同一份工站设计覆盖场景位置（桌面 = 工站空间、输送线、停止位、供钉器、控制器等）。示例工站设计下生成的全部程序与改造前逐字节一致。测试：改布局（出件位、供钉器、停靠位）后程序跟着变、其余不变；工站设计有 error 时不生成程序。2026-10-26
  - [x] 放件臂抖动（用户 2026-10-26：动作结束、特别是放置时抖）：逐步记录两臂关节实测 / 指令后定位为只发位置目标、速度目标为 0，资产阻尼把 wrist_1 拖后最多 34°，停下后猛追。改为位置目标 + 速度前馈（twin.py _command）：滞后 ≤ 1.7°，静止时法兰每步位移 8.8 → 0.06 mm；单台 13/13、零碰撞，节拍 108.7 s。诊断开关 JSDEMO_TRACE。2026-10-26。同日录屏时又发现：play 状态下 app.update() 也推进物理，孪生每个控制步物理实际走了两步、截图等待期间还会多走，录屏版放件臂撞快换架托轨。改为 SimulationManager.step() + RenderingManager.render()（只渲染），并让直线运动遵守资产关节限速 180°/s（超了整段降速）：两臂关节误差 ≤ 0.014°，单台节拍 109.4 s，录屏与不录屏物理一致（见 isaac-api-notes）
  - [x] 换手时拧紧臂不干等（用户 2026-10-26，选「等待期间先取钉」）：放件规划标出中途换手的握手点（tool_change_window_before_point，joint-v1 为 D1）；放件臂放完支架、到快换中间点（区域外）给 zone_left，回来先等 zone_free 再放驱动板；拧紧臂收到 zone_left 后收回区域、去供钉器取 D1 的螺钉、回停靠位交出区域，再等放件计数，之后直接去 D1。URScript：拧紧臂 DI5、放件臂 DO0，PLC 转接。单台 13/13、零碰撞，节拍 109.43 → 108.02 s：拧紧臂不能停在供钉器上方等（小臂会横在治具上方），要回停靠位，多一段往返；剩下 12 s 是放件臂换手 + 放驱动板的关键路径。2026-10-26

## M4 演示打磨

- [x] **M4.1 录屏素材**：孪生整件运行、自标定两段视频或截图序列，放在 `build/twin/media/`
  - 2026-10-25：`run_flow.py` / `calibrate_demo.py` 加 `--record`。Isaac 视口截图（不用屏幕截图），每 2 个物理步一帧 = 30 fps，视频时长 = 仿真时长；omni.videoencoding 编 mp4（本机核实），帧序列编完即删，留 4 张预览帧 + 1 张孔位近景帧。机位：全景；每组第一颗、异常点切孔位近景，拧完回全景；片尾整件近景停 4 s。字幕按事件写 .srt（放件、每点判定与扭矩、复拧、记问题、整件结论），不烧进画面。四段：TWIN-e3retryok（E3 浮高→复拧合格，63 s）、TWIN-d2slip（D2 滑牙→整件停止，27 s）、calibration（治具偏 0.96 mm→标定后 0.013 mm→重跑合格，74 s）、HUB-0001（第二个产品整件合格，61 s）。全景机位改为 eye [1.75,-1.45,1.05] / target [0.35,0,0.28]，原机位裁掉上臂。新增 tests/test_media.py（视频不在时跳过），70 passed
- [x] **M4.2 演示页接入真实数据**：`presenter/joint-station-demo.html` 中的数值改为读取 `build/station/report.json` 和 `build/twin/twin_report.json` 的结果（构建时注入即可，不要求页面联网）
  - 2026-10-25：新增 `python -m jsdemo.presenter`，读 build/station（report.json、run_log.txt、flow.yaml、ecn_impact.yaml、D1 曲线、产品数据）和 build/twin/twin_report_dfmfix.json，把整段 `<script id="demo-data">` 替换后写到 build/presenter/joint-station-demo.html；源页面里留一次运行结果的快照（source=snapshot），页脚写明数据来源和时间。九步的数值、表格、运行日志、首件曲线、ECN 影响全部从数据渲染，关口行改成「等待签字」由现场点。按用户决定：节拍卡并列估算 41.1 s 与孪生实测 62.9 s（慢 53%），孪生没跑时显示「未运行」不补示例数；视频不进页面。与旧页面示例值不同之处：几何规则 12→4 条、模块 8→10 个、D 组首件终止扭矩 0.178–0.191→0.172 N·m、审查「52 项核对」改为实际发现 2 处。run_flow.py 加 `--keep-open`（现场带界面跑完不退出）。新增 tests/test_presenter.py，74 passed

## M5 Relay 接入（之后再做）

- [ ] **M5.1** 见 `docs/05-relay.md`
