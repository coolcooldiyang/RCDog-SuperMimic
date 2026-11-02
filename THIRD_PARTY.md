# 来源与许可

本仓库是自研 12 自由度四足机器人 RCDog 的 SuperMimic 动作跟踪实验，基于 [Motphys/Uni-SuperMimic](https://github.com/Motphys/Uni-SuperMimic)。它不是原始 SuperMimic 官方实现，也不代表上游作者对 RCDog 的支持。

| 内容 | 来源 | 许可/说明 |
| --- | --- | --- |
| 可微世界模型、策略网络、训练循环、G1 任务与仿真后端 | [Uni-SuperMimic](https://github.com/Motphys/Uni-SuperMimic/tree/faf7818e8abac3ec6abb8f00e6e904d3577f800b)，commit `faf7818e8abac3ec6abb8f00e6e904d3577f800b` | Apache-2.0；原历史、NOTICE 与 `LICENSES/Apache-2.0-Uni-SuperMimic.txt` 保留 |
| 原始方法 | [SuperMimic 项目](https://supermimic.github.io/)、[论文](https://arxiv.org/abs/2610.28480) | Qiayuan Liao 等，*SuperMimic: Scaling Humanoid Whole-Body Control with One Differentiable World Model*；方法出处，不声明为论文作者 |
| 上游借鉴代码 | `supermimic_vibed`（见上游 NOTICE） | 原 MIT 许可保留于 `LICENSES/MIT-supermimic.txt` |
| 仿真与训练依赖 | [UniLab](https://github.com/unilabsim/UniLab)、UniSim、MuJoCo、MuJoCo Warp、PyTorch、Hydra | 依赖各自许可；本仓库不重新许可这些包 |
| RCDog 机器人模型 | 项目作者的 CAD 导出（`source/robot.urdf`、`source/meshes/`），由 `tools/rcdog/prepare_rcdog_assets.py` 转为 MJCF | 模型资产由项目作者提供并随本仓库公开用于复现；不随软件 GPL 重新许可 |
| 参考动作 | `tools/rcdog/generate_omni.py` 按给定速度、步态周期和抬脚高度生成 | 本仓库自行生成，不使用第三方动作捕捉数据 |
| 训练权重 | 本项目在 RTX 4090 上训练 | 随仓库发布，便于复现 `results/SIM2SIM_SUMMARY.json` 中的指标 |

## 本仓库包含什么

- 源代码、RCDog 模型与 MJCF、生成的全向参考动作、训练配置、训练与评估脚本。
- `results/` 中三档策略权重（`omni050`、`omni100`、`omni150`）、每档的 `run_config.json`、逐动作评估 JSON 和汇总 `SIM2SIM_SUMMARY.json`。
- `initial_policy/strafe025.pt`：训练链的起点权重，来自同一机器人的横移阶段。
- `real_deploy/bundle_omni150/`：1.5 m/s 档的 ONNX/TorchScript actor、参考表、固定输入和导出校验记录。
- `docs/assets/rcdog/omni150_mujoco.mp4`：本机 MuJoCo 渲染的策略轨迹与参考对比视频。

不包含：`runs/` 中完整的 `last.pt`（含优化器状态）、逐帧轨迹 NPZ、训练日志原始输出和本机完整压缩包。这些文件可由仓库内脚本重新生成。

## 软件许可范围

本发布版的 RCDog 扩展与上游框架组合后按根目录 `LICENSE`（GPL-3.0-only）提供；原有 Apache-2.0 与 MIT 文件的版权、许可与 NOTICE 保留。机器人模型、参考动作、训练权重与依赖不被一概声明为 GPL。上游 README 中的 Apache-2.0 声明描述上游版本，不能替代本文件对 RCDog 发布版的范围说明。
