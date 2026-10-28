# 关节紧固工站 demo · 给 Claude Code 的工作说明

用中文和我沟通。

## 这个仓库是什么

一个演示：agent 从关节模组的产品资料出发，完成可制造性检查、工艺规划、工站方案、业务流、机械臂程序和整套文件，人在四个关口签字；工站在 Isaac Sim 6.2 的数字孪生里运行。

- `jsdemo/`：流水线本体，纯 Python，不依赖 Isaac。已经能跑通，测试全部通过
- `sim/kinematic.py`：不依赖 Isaac 的简化运动学和自标定计算
- `sim/isaac/`：Isaac Sim 脚本，**这是你主要要完成的部分**，目前是带 TODO 的骨架
- `kb/`：知识库，唯一真源。产品数据、模块库、规则、术语表、决策记录
- `process/`：工艺库与工艺模型
- `tests/`：验收测试。测试就是需求
- `docs/`：场景、架构、计划、Isaac 环境说明。**从 docs/03-plan.md 开始**
- `presenter/`：给人演示用的网页

## 怎么工作

1. 每次只做 `docs/03-plan.md` 里第一个没勾选的任务
2. 动手前读该任务列出的文档和文件
3. 完成标准是该任务写明的验收条件，加上 `python -m pytest` 全部通过
4. 完成后勾选任务，在任务下面写一行：日期、做了什么、遇到什么
5. 需要人签字的事项，停下来问我，不要替我决定

## 不可违反的约束

这些对应 `kb/rules/golden.yaml`，测试会检查：

- **不要修改 `kb/rules/golden.yaml` 和 `tests/golden.sha256`。**需要改时停下来问我
- **不要为了让测试通过而改弱测试。**觉得测试本身有问题，停下来说明理由
- **不要编工艺数据。**工艺窗口只能来自 `process/library/` 或 `jsdemo.procmodel`。模型返回 unknown 时停下来问我，不要补一个看起来合理的数
- **机械臂程序里不能出现紧固逻辑。**扭矩、紧固判定都是执行器的事，机械臂只负责到位、保持、退出
- **所有点位都在治具坐标系下定义。**现场只标定治具坐标系，不逐点示教
- **`jsdemo/` 里不能 import Isaac。**Isaac 相关代码只放在 `sim/isaac/`
- **生成的东西只放 `build/`。**不要把生成物写进 `kb/`、`process/`
- **示例数据保持 `example_data: true` 标记。**

## Isaac Sim 的写法

- **不要凭记忆写 Isaac API。**版本之间模块路径变化很大。先在本机 `F:\isaacsim` 的 standalone_examples 和文档里找到对应写法，确认后记到 `docs/isaac-api-notes.md`，再写代码。骨架里标 `VERIFY-API` 的地方都要这样处理
- 不仿真螺纹接触的物理过程。紧固结果回放执行器曲线（`build/station/curves/`），判定调用 `jsdemo.procmodel.judge`
- 找不到某个资产或运动规划配置时，先告诉我有哪些可用的替代，不要自己换型号了事

## 常用命令

```powershell
python -m pytest                                   # 全部验收测试
python -m jsdemo run --yes --ecn ecn/ECN-001.yaml  # 自动签字跑完整流程，输出到 build/station
python -m jsdemo run                               # 交互式排练，关口处要输入 y
python -m jsdemo.procmodel recommend --material PA66 --screw M2x6-tapping --hole 1.6
F:\isaacsim\python.bat sim\isaac\build_scene.py    # Isaac 脚本都用 Isaac 自带的 python 运行（本机 Isaac 只装在 F:\isaacsim）
```

## 记录决策

做出不可逆、跨边界或偏离默认做法的决定时，按 `kb/decisions/TEMPLATE.yaml` 写一条记录（决定、否掉的方案及原因、约束和假设、失效条件、确认人），然后请我确认。缺哪个字段就写「缺失」，不要补一个说得通的理由。
