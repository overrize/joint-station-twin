# 工站设计标准 v1（给上游方案 agent 开发方）

> 版本 `jsdemo.station/1` · 2026-10-26 · 状态：**demo 版**（用户 2026-10-26：链路里只有上游方案 agent，布局、治具建议与工站设计都由它给出）
> 格式定义：`kb/contracts/station-v1.schema.json` · 示例：`kb/station/joint-v1.station.yaml` · 校验器：`python -m jsdemo.station <文件>`
> 生成：`python -m jsdemo run --yes --station <文件>`（先校验，有 error 打回；SOP 取 `meta.sop`）· 固件：`python -m jsdemo.firmware <文件>`
> 配套：产品装配用 `jsdemo.sop/1`（docs/06-sop-contract.md），工站设计引用它，不重复零件与紧固点

## 1. 这份标准管什么

上游方案 agent 一次给出两份文件：

- **SOP**（`jsdemo.sop/1`）：产品怎么装——零件、外形、工步、紧固点；
- **工站设计**（本标准）：工站怎么搭、怎么控——选哪些模块、每个放在哪（坐标）、治具用哪个或怎么设计、物料怎么进出、工步由谁做、哪些区域互斥、异常怎么处置。

链路里没有人工改方案的环节。工具链只做两件事：**校验**（不合格的打回给 agent，带位置和原因）与 **生成**（控制程序、各模块配置、数字孪生），生成结果在孪生里自动验证，验证报告同样回给 agent。

```
产品资料 ─► 上游方案 agent ─► SOP + 工站设计 ─► 校验 ─┬─ error   ─► 打回 agent
                                                       ├─ blocker ─► 可在孪生里排练，不能签字
                                                       └─ 通过    ─► 生成 I/O 表、PLC 程序、机械臂程序、模块配置、孪生 ─► 孪生验证报告
```

## 2. 对上游 agent 的要求

| # | 要求 | 为什么 |
|---|---|---|
| S1 | **只选模块库里的型号**（`kb/modules`）。库里没有的，写进 `open_items`，不要发明型号 | 生成器只认识库里的接口与参数 |
| S2 | **布局给坐标**：每个落地模块给工站坐标系下的 `pose {xyz, yaw}`；装在别的模块上的给 `mount {parent, at}` | 工具链不替 agent 摆位置，只检查摆得对不对 |
| S3 | **治具二选一**：库里现成的写 `use`；没有就给 `suggestion`（类型、定位、夹紧、≥ 3 个不共线基准点） | 基准点用于现场自标定；新治具要结构工程师确认 |
| S4 | **SOP 的每个工步都要分配**给一个资源（机械臂 id，或 `operator` / `upstream`） | 没人做的工步会让工站卡住 |
| S5 | **两臂并行就必须给互斥分区**，至少治具上方一个 | 孪生实测：没有分区时两臂会撞 |
| S6 | **不编造**：不知道写 `缺失`；估算、占位标 `source: estimate / placeholder` | 与 SOP 规则一致 |
| S7 | **不输出控制代码与 I/O 地址** | 由生成器按模块接口自动分配，保证接线一致 |
| S8 | YAML 键名**不要用** `on` / `off` / `yes` / `no` | YAML 1.1 会把它们解析成布尔值（本示例踩过） |

## 3. 文件结构

| 字段 | 必填 | 说明 |
|---|---|---|
| `schema` | 是 | `jsdemo.station/1` |
| `meta` | 是 | `station`、`name`、`product`、`product_revision`、`sop`（SOP 文件路径）、`generated_by`、`status`、`signed_by` |
| `units` | 是 | 固定 `{length: mm, angle: deg, time: s}` |
| `frame` | 是 | `description`（原点与轴向的文字说明）、`floor_z`（安装面高度）、`envelope {x, y, z_max}`（可用空间） |
| `targets` | 否 | `takt_s {value, source}`、`units_demo` |
| `line` | 否 | `flow`（输送方向）、`upstream {name, delivers, signal}`、`downstream {name, receives, signal}` |
| `modules` | 是 | 模块实例，见第 4 节 |
| `fixture` | 是 | 见第 5 节 |
| `handling` | 否 | `tray_slots [{slot, part}]`、`grasp`（`auto` 或逐件 `{part, tool, mode}`）、`unload {resource, tool, to}` |
| `process` | 是 | `allocation [{sop_step, resource, tool}]`、`parallel`、`screw_feed`（`nest_pick` 或 `blow_feed`） |
| `zones` | 是 | `[{id, description, box {center, size}, exclusive [资源 id]}]` |
| `control` | 是 | `interlock`（v1 只有 `handshake`）、`error_policy`（`from_sop`）、`mes_points`、`andon` |
| `open_items` | 是 | `[{id, text}]`，没有写 `[]` |

## 4. 模块实例

每个实例：`id`（本工站内唯一，如 `R1`）、`module`（库里型号）、`role`，再按角色补字段：

| role | 额外字段 |
|---|---|
| `fastening_arm` / `handling_arm` | `pose`、`riser_mm`、`home_joints_deg`（必填）、（拧紧臂）`park_joints_deg` 让出治具时的停靠位（两臂并行时必填） |
| `executor`、`float_head` | `mount {parent: 拧紧臂 id, at: flange}` |
| `gripper` | `tool_of`（机械臂 id）、`dock {changer, slot}` |
| `tool_changer` | `slots [{slot, xy}]`、`via`（去快换架前经过的中间点，可选） |
| `feeder` | `pose`（v1 只支持 yaw 0）、`lanes [{lane, screw}]`（螺钉型号必须在工艺库）。外形（尺寸、路距、出钉位）取模块库 `geometry` |
| `conveyor` | `path {x, y: [起, 止], width, top_z}`、`stops {tray_station, tray_prestop, tray_stop, out_stop, buffer: [...]}` |
| `tray` | `count`（轮换用几个料盘） |
| 其余（基板、控制器、接口盒、网关、视觉） | `pose` |

## 5. 治具

```yaml
fixture:
  use: FX-JOINT-V1                 # 或者不写 use，改写 suggestion
  pose: {xyz: [450, 0, 0], yaw: 0} # 治具坐标系原点（治具顶面中心）在工站坐标系里
  product_mounting: from_sop       # 产品在治具上的装夹取 SOP 的 frame.mounting_in_fixture
  suggestion:                      # 新治具建议（库里没有合适的治具时）
    id: FX-NEW-01                  # SOP 的 mounting_in_fixture.fixture 要引用它
    type: nest                     # nest / pins_and_face / vee_block / vacuum
    locating: 两销一面，销 Ø6 h7，间距 120
    clamping: 气动压爪 ×2，压在端面
    datums: [{id: D1, xyz: [-50, -50, 0]}, {id: D2, xyz: [50, -50, 0]}, {id: D3, xyz: [-50, 50, 0]}]
    material: AL6061 + POM 接触面
```

## 6. 校验结果

校验器写 `build/station_spec/<station>/validation.json`，四级与 SOP 一致：

| 级别 | 代码 |
|---|---|
| error（打回） | `E-SCHEMA` 结构 · `E-UNIT` 单位 · `E-REF` 引用不存在（型号、SOP、工步、工具、快换工位、治具不一致） · `E-DUP` 重复 · `E-MISSING` 缺必需模块 · `E-OUTSIDE` 出工站空间 · `E-OVERLAP` 两臂基座过近 · `E-REACH` 够不到（紧固点、出钉位、料盘、治具、出件位、快换工位） · `E-FEED` 料道或料盘穴位没覆盖 · `E-CAPACITY` 超出模块容量 · `E-ALLOC` 工步没分配或分错资源 · `E-ZONE` 并行却没有互斥分区 · `E-GEOM` 治具基准共线 · `E-UNSUPPORTED` v1 不支持的布局（如供钉器转角度） · `E-SIGN` 签字条件不满足 |
| blocker | `B-MISSING` 缺失 · `B-OPEN` 未决项 · `B-SOP` 引用的 SOP 还不能签字 |
| review | `R-FIXTURE` 借用或新设计治具 · `R-ALLOC` 工站设计改了 SOP 写的执行者 |
| warning | `W-SOFT` 估算 / 占位 · `W-TRAY` 料盘里有零件但没人取 |

## 7. 工具链产出（生成器，确定性）

| 产出 | 依据 |
|---|---|
| 布局（程序生成、场景搭建、孪生） | `modules.*.pose`、`fixture.pose`、`conveyor.path / stops`、`frame.envelope / floor_z`、各臂 `home / park_joints_deg`。流水线读入后记在产出 `station.yaml` 的 `layout_source`，孪生按同一份搭场景 |
| 拧紧臂、放件臂程序（URScript） | SOP 紧固点 + 治具位姿 + 分工 + 互斥分区握手 |
| I/O 分配表、工站总控 PLC（IEC 61131-3 ST） | 模块接口 + `zones` + `control` |
| 各模块配置：执行器参数包、供钉器料道、夹爪抓取表、快换工位表 | 工艺库 + `lanes` + `handling` + 零件外形 |
| 离线干涉检查、孪生验证报告 | 以上全部 |

## 8. v1 的限制（demo 标注）

- 只支持竖直向下拧紧、零件与装夹只绕 z 转（同 SOP v1）。
- 互斥分区只支持轴对齐的盒子；握手协议固定为 zone_free / placed_count / unit_done，放件臂中途换手时加 zone_left（离开治具区，拧紧臂趁机取钉）。
- 可达性按「到基座的距离在臂展内」判断，不做逆解；真正的姿态可达与干涉由孪生验证。
- 供钉器只支持 yaw 0（出钉口朝 -y）；出钉位外形取模块库 `kb/modules/feeder-fd3l.yaml` 的 `geometry`。
- 不给 `--station` 时，流水线的布局取示例 `kb/station/joint-v1.station.yaml`（demo），不做工站设计校验。
- 场景里的大屏、上下游工站外形、桌面厚度等装饰仍是孪生自己的占位，不在工站设计里。

## 9. 版本管理

与 SOP 相同：新增可选字段不升版本；改名、删字段、改语义升版本，写决策记录并请用户确认。
