# Isaac Sim 在这台机器上的环境说明

信息截至 2026-10。以官方文档为准。

## 版本

- 使用 **Isaac Sim 6.2.0**（2026-10-14 正式版）
- 不使用 7.0.0 Alpha（2026-09 预览版）

## 系统与驱动

- 只支持 **Windows 11**，Windows 10 不支持
- Windows 驱动要求 **595.97**（Linux 为 595.58.03）
- 显卡最低 RTX 4080 16GB，推荐 RTX 5080 16GB
- 本机 RTX 5090D v2：Blackwell GB202，24GB GDDR7，满足要求。官方文档未单独列出 D 系列型号，以实测为准

## Docker / WSL2：不可用

- 官方：Isaac Sim 容器只支持 Linux
- NVIDIA 论坛答复：容器目前不支持 WSL2。实际表现为 CUDA 能识别 GPU，但 Vulkan 初始化失败（ERROR_INCOMPATIBLE_DRIVER）
- 需要容器化时：双系统 Ubuntu 24.04，或 Linux 服务器跑无界面版、Windows 上用 6.2 配套的 2.0.0 桌面客户端查看

## 6.2.0 在 Windows 上的已知问题

- 缺少构建工具会报 WinError 126：安装 Visual Studio 2022 和 MSVC v143
- 某个文件完整路径超过 260 字符会启动报错：本机安装在 `F:\isaacsim` 这类短路径
- 独立应用关闭时线程可能清理不干净：关闭前多调用几次 update
- Blackwell 上 Franka 开抽屉示例默认失败，把物理频率提到 600 可解决（与本项目无关，记录备查）

## 参考

- 要求：https://docs.isaacsim.omniverse.nvidia.com/latest/installation/requirements.html
- 已知问题：https://docs.isaacsim.omniverse.nvidia.com/6.2.0/overview/known_issues.html
- 下载：https://docs.isaacsim.omniverse.nvidia.com/6.2.0/installation/download.html
- 发布记录：https://github.com/isaac-sim/IsaacSim/releases
