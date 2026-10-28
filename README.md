# 关节紧固工站 demo · v0.1

agent 从关节模组的产品资料出发，完成可制造性检查、工艺规划、工站方案、业务流、机械臂程序和整套文件，人在四个关口签字；工站在 Isaac Sim 6.2 的数字孪生里运行。

**所有产品、扭矩、节拍数据都是示例，不构成工艺推荐。**

## 包里已经有什么

- 可运行的流水线 `jsdemo/`：九个阶段、四个签字关口、决策记录草稿
- 工艺模型 `jsdemo/procmodel.py`：库内引用、库内插值推荐、超出范围返回未知
- 机械臂程序生成：设备无关的中间指令 + URScript
- 自标定计算：三个基准点求治具坐标系
- 28 项验收测试，全部通过
- 给人演示的网页 `presenter/joint-station-demo.html`

## 还没有的

Isaac Sim 数字孪生部分。`sim/isaac/` 里是骨架，由 Claude Code 在你的机器上对照本机 Isaac Sim 6.2 完成，计划见 `docs/03-plan.md`。

## 开始

```powershell
cd F:\isaac_digifact\demo-joint-station   # 本机项目位置（短路径）；Isaac Sim 在 F:\isaacsim
pip install -r requirements.txt
python -m pytest                        # 应显示 28 passed
python -m jsdemo run --yes --ecn ecn/ECN-001.yaml
```

输出在 `build/station/`：`report.json`、`run_log.txt`、`flow.yaml`、`arm_program.urscript`、`params/`、`docs/`、`decisions/`、`curves/`、`ecn_impact.yaml`。

交互式排练（关口处输入 y 签字）：

```powershell
python -m jsdemo run --ecn ecn/ECN-001.yaml
```

## 交给 Claude Code

在这个目录下启动 Claude Code。它会读取 `CLAUDE.md`。然后：

- `/next`：做计划里下一个任务，先复述目标等你确认，做完跑测试再勾选
- `/verify`：只检查不修改，汇报测试、流水线和孪生结果
- `/rehearse`：带你排练演示，在每个关口提示讲解要点

建议顺序：先让它做 M0（环境）和 M2（确认本机 Isaac API），再做 M3（孪生）。M2 那一步不要跳，Isaac 各版本的 API 路径差别很大，骨架里标了 `VERIFY-API` 的地方都要对照本机确认。

## 目录

```
CLAUDE.md                 给 Claude Code 的工作说明和约束
docs/                     场景、架构、计划、Isaac 环境、Relay 接入
kb/                       知识库：产品、模块库、规则、术语表、决策记录
process/library/          工艺库
ecn/                      设计变更示例
jsdemo/                   流水线（纯 Python）
sim/kinematic.py          简化运动学与自标定计算
sim/isaac/                Isaac Sim 脚本骨架
tests/                    验收测试
presenter/                演示网页
```
