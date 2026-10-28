# 之后接入 Relay

流水线的九个阶段对应 Relay 的角色：

| 阶段 | Relay 角色 | 验收方式 |
|---|---|---|
| intake、dfm | 调研 Worker | dfm 结果文件存在且问题已列出 |
| process、compose、generate | 实现 Worker | test_suite：test_feasibility、test_erasure、test_takt |
| review | 审查 Worker，只看产出不看生成过程 | test_suite：test_consistency |
| verify、孪生运行 | 验证 Worker | test_suite：全部测试 + twin_report |
| 关口 | PM 停下来问人 | 人签字 |
| 决策记录 | 秘书 | 写入 kb/decisions，人确认 |

做法：每个阶段派工时把对应的 pytest 文件声明为 `acceptance_checks` 的 `test_suite`，由 Relay 控制器在 Worker 停止后独立运行。Worker 自报完成不算。

审查 Worker 必须在私有副本里只读取 build/ 下的产出和 kb/rules，不读取实现 Worker 的会话，保证核对有效。
