依次运行并汇报结果，不做任何修改：

1. `python -m pytest`
2. `python -m jsdemo run --yes --out build/verify --ecn ecn/ECN-001.yaml`，读 build/verify/report.json，报告 status 和 verify.checks
3. 如果 build/twin/twin_report.json 存在，报告 ok、takt_measured_s、collisions 数量，并与 build/station/report.json 里的节拍估算对比

有失败就说明失败在哪，不要自行修复。
