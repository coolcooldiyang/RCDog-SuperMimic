# 来源与许可

本项目是 RPO 适配实验，既不是原始 SuperMimic 官方实现，也不是 RoboParty 官方支持的实机策略。

| 内容 | 来源 | 许可/说明 |
| --- | --- | --- |
| 基础算法、G1 任务、训练框架 | [Motphys/Uni-SuperMimic](https://github.com/Motphys/Uni-SuperMimic/tree/faf7818e8abac3ec6abb8f00e6e904d3577f800b) | Apache-2.0；保留原历史、NOTICE 和 `LICENSES/Apache-2.0-Uni-SuperMimic.txt` |
| 原始方法 | [SuperMimic 项目](https://supermimic.github.io/)、[论文](https://arxiv.org/abs/2610.28480) | Qiayuan Liao 等，SuperMimic: Scaling Humanoid Whole-Body Control with One Differentiable World Model；方法出处，不声明为论文作者 |
| 上游借鉴代码 | supermimic_vibed（见上游 NOTICE） | 原 MIT 许可保留于 `LICENSES/MIT-supermimic.txt` |
| 仿真/环境依赖 | [UniLab](https://github.com/unilabsim/UniLab)、UniSim、MuJoCo、MuJoCo Warp、PyTorch | 依赖各自许可；本仓库不重新许可这些包 |
| RPO 并联踝几何 | [RoboParty](https://github.com/Roboparty/roboto_origin)，`decouple_rpo.cpp`；Copyright (C) 2025–2026 Luo1imasi | GPL-3.0；Torch 移植位于 `parallel.py`；本项目组合软件采用 GPL-3.0-only |
| RPO 机器人几何/MJCF/网格 | [rpo_description](https://github.com/Roboparty/rpo_description)、[roboto_origin](https://github.com/Roboparty/roboto_origin) | CERN-OHL-W-2.0；模型目录提供源文件、修改说明与源位置；不由软件 GPL 重新许可 |
| RPO 训练 URDF | [RoboParty RoboLab](https://github.com/Roboparty/roboparty_train) | 保留 RoboLab / Isaac Lab 的 BSD-3-Clause 通知，另附模型几何许可 |
| 跑步参考 | [Lafan1_retarget](https://github.com/wenconggan/Lafan1_retarget/tree/57e5a1921417bf826697f87ba33adb248d4d177f)，`run1_subject2.csv`，原始帧 4110:4350（右端不含） | 原始 [Ubisoft LAFAN1](https://github.com/ubisoft/ubisoft-laforge-animation-dataset) 采用 [CC BY-NC-ND 4.0](https://github.com/ubisoft/ubisoft-laforge-animation-dataset/blob/master/license.txt)，不把可访问误写成可自由重发 |
| 后空翻参考 | [chunwan-flip](https://github.com/lez666/chunwan-flip/tree/c493a897f2ca81153e980a65ee6f0faf7cb07487)，`data/90_19_stageii_unitree_g1.csv` | 源仓库 README 声明 MIT；该动作进一步来源于 CMU 90_19、AMASS 和 GMR，源数据条款仍需遵守 |
| 人体数据/重定向链 | [CMU MoCap](http://mocap.cs.cmu.edu/)、[AMASS](https://amass.is.tue.mpg.de/)、[GMR](https://github.com/YanjieZe/GMR) | AMASS: Mahmood et al., ICCV 2019；[AMASS 数据许可](https://amass.is.tue.mpg.de/license.html)含非商业及禁止重发条款 |
| 时间对照参考 | [allmimic](https://github.com/Shimingyi/allmimic) | 仅作训练耗时对照，没有使用其训练代码或 BONES-SEED 数据 |

## 发布范围

公开仓库包含源代码、RPO 模型及相应许可、配置、研究演示、训练日志和汇总评估。**不分发人体原始数据、G1/RPO 参考 CSV、密集轨迹 NPZ、训练权重或原本机完整压缩包**。完整实验文件继续保存在本机。数据许可不由本项目的软件许可替代；视频是非商业实验结果展示，不构成动作数据集或商业使用授权。

如需复现，先从原始来源取得相应使用/转换许可，再把授权文件放到本地。仅下载到公开仓库中的文件不等于已经取得再分发许可。尤其不得据源仓库的 MIT 简写覆盖 AMASS 或 LAFAN1 数据本身的条款。

## 软件许可范围

本发布版的 RPO 扩展及其与 GPL 并联踝实现构成的组合软件按根目录 `LICENSE`（GPL-3.0-only）提供。原有 Apache-2.0 与 MIT 文件的版权、许可及 NOTICE 保留，模型资产适用各自许可。模型、数据和依赖不被一概声明为 GPL 或 Apache。原上游 README 中的 Apache 声明描述上游版本，不能替代本文件对 RPO 发布版的范围说明。
