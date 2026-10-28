# 工站 SOP 交接标准 v1（给上游 agent 开发方）

> 版本 `jsdemo.sop/1` · 2026-10-25 · 状态：**demo 版**（用户 2026-10-25：演示阶段按此执行，不确定处标 demo，数据和需求清晰后修订，见第 9 节）
> 格式定义：`kb/contracts/sop-v1.schema.json`（JSON Schema 2020-12）
> 示例：`kb/sop/joint-v1.sop.yaml`
> 校验器：`python -m jsdemo.sop <文件>`

## 1. 这份标准管什么

每次进来的产品，零件、结构和装配顺序都可能不同。上游 agent 读产品资料，输出一份**工站 SOP**。我们这边只按这份 SOP 工作：

1. 校验 SOP，不合格的打回给上游 agent；
2. 把 SOP 转成流水线的产品数据，接着做可制造性检查、工艺规划、工站方案、机械臂程序；
3. 在数字孪生里按 SOP 搭场景，按工步放零件、拧螺钉。

SOP 是两边唯一的交接物。SOP 里没写的，我们不猜；写错了，校验器会指出位置和原因。

```
产品资料 ─► 上游 agent ─► SOP（YAML）─► jsdemo.sop 校验 ─┬─ 有 error   ─► 打回上游 agent
                                                        ├─ 有 blocker ─► 可在孪生里排练，不能签字，缺失项交人补
                                                        └─ 无 blocker ─► 人签字 ─► 流水线 ─► 孪生
```

### 怎么用

```powershell
python -m jsdemo.sop kb/sop/joint-v1.sop.yaml                       # 只校验并转换，结果在 build/sop/<product>/
python -m jsdemo run --yes --sop kb/sop/joint-v1.sop.yaml          # 按 SOP 跑流水线（有 error 时停在 intake，状态 stopped:sop_invalid）
F:\isaacsim\python.bat sim\isaac\build_scene.py                    # 孪生按 SOP 的零件搭场景
F:\isaacsim\python.bat sim\isaac\run_flow.py                       # 按 SOP 工步放件、拧紧
# 另一个产品放到别的工站目录，互不覆盖：
python -m jsdemo run --yes --sop kb/sop/sensor-hub-v1.sop.yaml --out build/station_hub
$env:JSDEMO_STATION = "build/station_hub"; F:\isaacsim\python.bat sim\isaac\build_scene.py   # 孪生产出在 build/twin_station_hub
```

## 2. 对上游 agent 的要求

| # | 要求 | 原因 |
|---|---|---|
| A1 | **不编造。**资料里没有的数值写字符串 `缺失`，不要填一个看起来合理的数 | 编出来的数会一路传到机械臂程序和工艺参数里，现场才暴露 |
| A2 | **每个数值都标来源**：`drawing` 图纸、`cad` 模型、`measured` 实测、`experiment` 试验、`customer` 客户、`supplier` 供应商、`estimate` 估算、`placeholder` 占位 | 估算和占位会被标成警告，签字前要有人确认 |
| A3 | **不输出工艺参数。**扭矩窗口、转速、角度都不由 SOP 给，统一从我们的工艺库取。库里没有时，流水线会停下来问人 | 工艺数据只有一个来源 |
| A4 | **自己知道没解决的问题写进 `open_items`**，不能省略。没有就写 `[]` | 每一条都算未决项，挡住签字 |
| A5 | **工步顺序就是现场的执行顺序**，包括放零件，不只有拧螺钉 | 孪生按这个顺序装；后放的零件会不会挡住还没拧的孔，也靠顺序来判断 |
| A6 | **一个产品版本只输出一个文件**，`meta.product` 加 `meta.revision` 唯一；同样的输入要得到同样的输出（id 稳定，不随机） | 便于比对差异、追溯 |
| A7 | **id 规则**：零件、元件、工步、点位、拧紧组、CTQ 各自不重复；点位 id 全文件唯一；只用字母、数字、`-`、`_` | 点位 id 会进入机械臂程序、曲线文件名和追溯记录 |
| A8 | **输出前先自检**：用 JSON Schema 做一遍结构校验，有条件的再跑 `python -m jsdemo.sop` | 结构问题不该留到交接时才发现 |
| A9 | **收到校验报告后**：error 必须改完再交；blocker 能补就补，补不了的保留 `缺失` 并在 `open_items` 说明；review 不用改，由人判断 | 见第 8 节 |
| A10 | **不把示例当真数据。**`source: example` 只允许出现在 `example_data: true` 的示例文件里 | 防止示例数据混进真实项目 |

## 3. 文件结构

YAML，UTF-8。顶层字段：

| 字段 | 必填 | 说明 |
|---|---|---|
| `schema` | 是 | 固定为 `jsdemo.sop/1` |
| `meta` | 是 | `product`、`name`、`revision`、`station`（可选）、`generated_by {agent, version, at}`、`status`（`draft` 或 `signed`）、`signed_by` |
| `units` | 是 | 固定为 `{length: mm, angle: deg, time: s}` |
| `frame` | 是 | `description`（文字说明产品坐标系），`mounting_in_fixture {fixture, xyz, rpy, source}` |
| `takt_target_s` | 是 | `{value, source}` |
| `parts` | 是 | 零件，见第 4 节 |
| `screws` | 是 | 螺钉型号，`id` 必须在工艺库里（如 `M3x8-machine`）；可选 `bom_id`、`name`、`head {d, h, drive, source}` |
| `ctq` | 否 | `{id, name}` 列表；工步或点位里用 `ctq: [id]` 引用 |
| `steps` | 是 | 有序工步，见第 5、6 节 |
| `open_items` | 是 | `{id, about, text}` 列表 |

## 4. 坐标系与外形

- **产品坐标系**：由 agent 定义，在 `frame.description` 里写清楚。必须是 **z 轴向上、拧紧方向为 −z**。
- **装夹位姿** `frame.mounting_in_fixture`：产品坐标系在治具坐标系里的位姿。我们把所有点位换算到治具坐标系，现场只标定治具，不逐点示教。治具还没定时写 `缺失`，校验会报 blocker。
- **零件** `parts[]`：
  - 字段：`id`、`name`、`material`、`pose {xyz, rpy}`（装配好后在产品坐标系里的位姿）、`geometry {source, shapes, mesh}`。
  - `shapes`：零件坐标系下的**简化体**，零件外形取它们的并集。孪生建模和干涉检查都只用简化体。
    - `cylinder {r, xy, z: [下, 上]}`
    - `ring {r_in, r_out, xy, z: [下, 上]}`
    - `box {center, size, yaw}`
  - `mesh`（可选）：`{file, format: usd|step|stl|obj, units: mm}`，只用来让画面更好看，v1 暂不使用。
  - `components`（可选）：板上元件这一类，随零件一起放入。它们既是外形，也是可制造性检查里的障碍物。每个写 `{id, name, shape, source}`。
- 简化体的要求是**不小于真实外形**，尤其是拧紧点附近。外形宁大勿小：画小了，干涉就查不出来。

## 5. 工步

每个工步：`id`、`type`、`actor`（`operator` 人工 / `robot` 机械臂 / `executor` 执行器 / `upstream` 上游工站已完成 / `缺失`）。

- `type: place`：放零件。写 `part`、`duration_s`。每个零件只能放一次。
- `type: fasten`：拧紧一组螺钉，见第 6 节。本工站由机械臂和执行器完成，`actor` 写 `robot`。
- `type: inspect`：检查（v1 只记录，不执行）。

顺序规则（校验器会检查）：

1. 拧紧时，`fastens`（被紧固件）和 `into`（承接件）都必须已经放好；
2. 每个点位正上方 30 mm 内，不能有已放的零件或元件挡住。挡住的报 `E-BLOCKED`。例：端盖放在编码器螺钉之前；
3. 点位周围的批头间隙小于 DFM-01 下限（3.0 mm）的报 `DFM-01`，交给人判断。

## 6. 拧紧工步

| 字段 | 说明 |
|---|---|
| `group` | 拧紧组 id，同组同螺钉、同孔、同工艺 |
| `name` | 组名（如「端盖」） |
| `fastens` | 被紧固的零件：螺钉头压在它的顶面上 |
| `into` | 承接零件：螺纹或底孔在它里面 |
| `screw` | `screws[]` 里的型号 |
| `hole` | `material`（孔所在材料）、`spec`（如 `M3-thread`）或 `d_mm`（自攻底孔径）、`depth_mm`（拧入深度）、`source` |
| `approach` | 接近方向，v1 只支持 `[0, 0, -1]` |
| `on_ng` | `action`：`continue`（退回、标 NG、继续）或 `stop_unit`（退回、标 NG、整件停止）；`retry`（可选）：`{max, on_modes, decision}`，`decision` 必须是 `kb/decisions` 里已确认的决策 id |
| `ctq` | 本组关联的 CTQ |
| `points` | `{id, xyz, ctq?}`；`xyz` 在产品坐标系，**取螺钉头落座的那个表面上的点** |

校验器对每个点检查：

- 点在 `fastens` 零件的顶面上，容差 0.05 mm；
- 正下方有 `into` 零件；
- 螺钉长度 ≥ 被紧固件厚度 + 拧入深度。不满足报警告，提示核对沉孔或尺寸。

## 7. 「缺失」与来源

- 不知道的值写字符串 `缺失`，任何字段都可以这样写。
- 外形尺寸、点位坐标写 `缺失` 算 **error**：没有它们，工站根本搭不出来。
- 执行者、时长、孔深、治具、目标节拍写 `缺失` 算 **blocker**：可以先排练，不能签字。
- 来源为 `estimate` 或 `placeholder` 算 **warning**。

## 8. 校验结果

校验器写 `build/sop/<product>/validation.json`：

```json
{"buildable": true, "signable": false,
 "counts": {"error": 0, "blocker": 12, "review": 1, "warning": 18},
 "findings": [{"level": "review", "code": "DFM-01", "path": "steps[5].points[2]",
               "message": "D3 批头间隙不足：距 C12 外缘 1.6 mm，下限 3.0 mm"}]}
```

`path` 指向 SOP 里出问题的位置。`fix`（有时有）是建议的改法。

| 级别 | 含义 | 上游 agent 怎么做 |
|---|---|---|
| error | 结构错误，或几何上做不到 | 必须改完再交 |
| blocker | 信息缺失、工艺库没有样本、决策未确认 | 能补就补；补不了的保留 `缺失`，并在 `open_items` 说明 |
| review | 可制造性问题 | 不用改，由人在 DFM 关口判断 |
| warning | 估算、占位数据，或几何上可疑 | 尽量用真实数据替换 |

主要代码：

- **error**：`E-SCHEMA` 结构，`E-UNIT` 单位，`E-REF` 引用不存在，`E-DUP` id 重复，`E-ORDER` 顺序，`E-SURFACE` 点不在零件顶面，`E-INTO` 下方没有承接件，`E-BLOCKED` 被挡住，`E-MISSING` 外形缺失，`E-UNSUPPORTED` v1 不支持，`E-SOURCE` 示例来源误用，`E-SIGN` 签字条件不满足。
- **blocker**：`B-MISSING` 缺失，`B-PROCESS` 工艺库无样本，`B-DECISION` 决策未确认，`B-FIXTURE` 治具不在模块库，`B-OPEN` 声明的未决项。
- **review**：`DFM-01` 批头间隙不足。
- **warning**：`W-SOFT` 估算或占位，`W-STACK` 螺钉长度不够，`W-UNPLACED` 零件没有放件工步，`W-MESH-IGNORED` 网格附件未使用，`W-SCREW-LEN` 型号读不出长度。

## 9. v1 的限制

- 只支持竖直向下拧紧；零件和装夹只允许绕 z 旋转。
- 只用简化体，网格附件只做记录。
- `inspect` 工步只记录，不执行。
- 一个 SOP 对应一个工站。

超出这些范围时校验器报 `E-UNSUPPORTED`，不会静默忽略。

### demo 标注：目前按演示需要定的、以后要用真实数据修订的

| 项 | 现在的做法（demo） | 以后要定的 |
|---|---|---|
| 零件外形 | 简化体（长方体 / 圆柱 / 环）够用来看干涉和放件 | 是否必须带网格、网格精度 |
| 放件时长缺失 | 孪生按 1 s 演示，结果标 `assumed` | 由上游给实测或标准工时 |
| 执行者缺失 | 字幕显示「执行者未定」，照常演示 | 人工 / 机械臂 / 上游的分工 |
| 自动复拧 | 只对浮高复拧 1 次（DEC-NG-001） | 哪些异常可复拧、几次、反转参数 |
| 校验四级 | error / blocker / review / warning 的划分按演示需要定 | 哪些缺失必须挡住签字 |
| 只支持竖直拧紧、绕 z 旋转 | 覆盖两个示例产品 | 斜向孔、翻面工站 |

这些项在孪生报告、字幕和演示页里出现时都如实标注（`assumed`、「示例值」、「未定」），不当成真实数据。

## 10. 版本管理

- `schema` 字段带版本号。
- 新增可选字段时版本号不变；改名、删字段、改语义时升版本。
- 每次升版本都写决策记录，并请用户确认。
- 校验器只接受它认识的版本。

## 11. 用示例 SOP 跑出来的发现（joint-v1）

示例 SOP 由现有产品数据转写而来，转换结果与 `kb/product/joint-v1` 一致，有测试保证。校验时暴露了现有数据本身的几个问题：

- **W-STACK，每个点都报**：按 M3.4 的占位外形，螺钉长度不够穿过被紧固件再拧到规定深度。例：端盖厚 6 + 拧入 6 = 12 mm，M3x8 只有 8 mm。真实零件应该有沉孔或更薄的压紧面，占位外形需要按图纸修正。
- **D 组的 `part` 字段语义不统一**：现有数据里 C、E 组的 `part` 指被紧固件（编码器、端盖），D 组却指承接件（支架 BRK-01）。SOP 用 `fastens`/`into` 两个字段消除了这个歧义；转换后 `part` 取 `fastens`，D 组会变成 PCB-01。
- **放件工步的执行者和时长缺失**：对应 M3.4 的发现「流程缺放端盖」。
- **DFM-01**：D3 距 C12 外缘 1.6 mm。流水线 DFM 阶段按中心距算是 2.1 mm，两边结论一致。
