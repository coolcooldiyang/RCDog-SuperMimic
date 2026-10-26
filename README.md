# RPO-SuperMimic

**基于 [Motphys/Uni-SuperMimic](https://github.com/Motphys/Uni-SuperMimic) 的 RPO 23 自由度人形机器人动作模仿实验：跑步与后空翻。**

本项目增加 RPO 机器人适配、公开来源动作的 G1→RPO 重定向、并联踝电机约束、训练配置与仿真验收。Uni-SuperMimic 是在 UniLab 上对 [SuperMimic](https://supermimic.github.io/) 的独立复现；本仓库继续基于它扩展，保留上游历史和署名。**当前仅完成仿真验证，未接入 RDK 或验证实机动作。**

## 效果演示

以下为实际自由基座 MuJoCo 物理 rollout。实体机器人是策略输出，蓝色半透明机器人是参考动作；物理运行没有固定根部、外力支撑或逐帧重置姿态。GIF 为预览，点击视频链接可查看完整演示。

| 跑步 | 后空翻 |
| --- | --- |
| ![RPO running](docs/assets/rpo/running.gif) | ![RPO backflip](docs/assets/rpo/backflip.gif) |
| [完整视频：rpo_running.mp4](docs/assets/rpo/rpo_running.mp4) | [完整视频：rpo_backflip.mp4](docs/assets/rpo/rpo_backflip.mp4) |

| 动作 | 结果 | 小扰动验收 |
| --- | --- | --- |
| 跑步 | 约 8 秒沿参考方向前进 7.49 m | 8/8 完成 |
| 后空翻 | 完整翻转并落地，末尾蹲姿收稳 | 8/8 完成 |

验收只覆盖轻微初始扰动，详细范围、判据和原始汇总见 [复现说明](docs/REPRODUCE.md)及 [acceptance_summary.json](results/acceptance_summary.json)。这是两份固定参考动作策略，尚不支持摇杆控制任意方向/速度跑步。后空翻未包含起身恢复。

## 做了什么

- 适配 RPO 23 个关节、14 个跟踪刚体、模型碰撞、默认姿态与关节范围。
- 跑步使用 LAFAN1→G1 的公开来源参考，后空翻使用 CMU 90_19 / AMASS→GMR→G1 参考，再转换到 RPO。
- 按 RoboParty 的并联踝几何实现 Torch Jacobian，每 4 ms 在电机空间施加 ±27 Nm 扭矩饱和，加入工作空间和转速约束项。
- 用 MJWarp 训练、CPU MuJoCo 验收，记录完整配置、策略契约、训练日志和八次扰动结果。

单张 **RTX 4090 24 GB** 上两任务并行，基线训练加耦合微调至停止约 **80 分钟**；环境搭建、RPO 适配、数据重定向和评估另计。这不是在本机器人上验证的“40 分钟通用训练”承诺。

## 使用与复现

见 **[从数据准备到训练、评估、复播](docs/REPRODUCE.md)**。代码支持此次验证过的 MJWarp 训练和 MuJoCo CPU 评估。

```text
src/unisupermimic/tasks/motion_tracking/rpo/  RPO 任务、运动学与并联踝控制
src/unisupermimic/assets/robots/rpo/          RPO MJCF、网格与源 URDF
src/unisupermimic/conf/supermimic/task/       任务参数
tools/rpo/                                 重定向、训练与验收入口
results/                                    配置、日志、验收与来源记录
docs/assets/rpo/                           本次 RPO 实验演示
```

由于源数据许可限制，**公开仓库不附参考 CSV、训练权重和密集轨迹**；完整实验文件保存在本机。使用者需先从原始提供方取得数据使用/转换授权，再执行重定向和训练。详见 [来源与许可](THIRD_PARTY.md)。

## 当前限制

仿真中仍有踝电机瞬时速度超过 URDF 标称 8 rad/s；电机转矩-转速曲线、间隙、柔性和惯量尚未完整辨识。因此演示不代表已满足实机电机约束，不能作为实机可用性结论。

## 框架出处与致谢

- **原始方法**：[SuperMimic 项目主页](https://supermimic.github.io/)，Qiayuan Liao 等，[*SuperMimic: Scaling Humanoid Whole-Body Control with One Differentiable World Model*](https://arxiv.org/abs/2610.28480)。
- **直接代码基础**：[Motphys/Uni-SuperMimic](https://github.com/Motphys/Uni-SuperMimic)，基线提交 `faf7818e8abac3ec6abb8f00e6e904d3577f800b`。原版介绍保留于 [英文](README.upstream.md) / [中文](README.upstream_zh.md)。
- **机器人和并联踝**：[RoboParty / roboto_origin](https://github.com/Roboparty/roboto_origin)、[RPO description](https://github.com/Roboparty/rpo_description)。
- **动作来源**：[LAFAN1](https://github.com/ubisoft/ubisoft-laforge-animation-dataset)、[Lafan1_retarget](https://github.com/wenconggan/Lafan1_retarget)、[CMU MoCap](http://mocap.cs.cmu.edu/)、[AMASS](https://amass.is.tue.mpg.de/)、[GMR](https://github.com/YanjieZe/GMR)、[chunwan-flip](https://github.com/lez666/chunwan-flip)。

上游 Apache-2.0 / MIT 通知完整保留；含 GPL 并联踝适配的组合软件采用 GPL-3.0-only，RPO 模型资产保留 CERN-OHL-W-2.0 等原许可。具体范围见 [THIRD_PARTY.md](THIRD_PARTY.md)、[NOTICE](NOTICE) 和 [适配记录](CHANGES_RPO.md)。本仓库与上述作者或机构无官方隶属关系。
