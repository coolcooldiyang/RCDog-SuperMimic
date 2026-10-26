# 复现步骤

## 已测试环境

Linux、Python 3.12、RTX 4090 24 GB；训练使用 MJWarp，CPU 评估使用 MuJoCo。
核心版本：UniLab 1.5.1、UniSim Core 2.1.1、MuJoCo / MuJoCo Warp 3.11.0、Warp 1.16.0、PyTorch 2.14.1（CUDA 13.0 构建）。精确包快照见 [requirements-tested-cloud.txt](../requirements-tested-cloud.txt)。原上游 `uv.lock` 保留作上游记录，不等于本次云端环境的锁文件。

```bash
python3.12 -m venv .venv
source .venv/bin/activate
# PyTorch/CUDA 构建按本机驱动选取；快照记录了本次训练版本。
python -m pip install -r requirements-tested-cloud.txt
python -m pip install -e .
python -m pip install scipy osqp
```

该项目直接运行仿真，不需要连接 RDK。RPO 的 MJWarp 训练和 CPU MuJoCo 评估经过测试；未验证 RPO 耦合模型的 Motrix 后端。

## 数据准备

源数据受各自条款限制，仓库没有附带参考 CSV 或权重。先阅读 [THIRD_PARTY.md](../THIRD_PARTY.md)，从原始提供方获得使用/转换授权后准备：

- 跑步：Lafan1_retarget 的 G1 `run1_subject2.csv`（30 Hz）。
- 后空翻：chunwan-flip 的 G1 `90_19_stageii_unitree_g1.csv`（30 Hz；CMU 90_19 / AMASS）。

这两条路径由使用者指定；脚本不会代为接受数据许可。G1→RPO 使用命名关节 IK、足部净空保持与约束时间平滑。后空翻的“人体→G1”由源数据提供方使用 GMR 完成，本项目执行“G1→RPO”，不声称直接使用 GMR 完成人体到 RPO 的训练。

```bash
python tools/rpo/retarget_public_motions.py   --run-source /path/to/authorized/run1_subject2.csv   --backflip-source /path/to/authorized/90_19_stageii_unitree_g1.csv
```

输出位于 `src/unisupermimic/assets/motions/rpo/`：50 Hz、每帧 30 列，依次为 root XYZ、root quaternion XYZW、23 个命名关节角。关节顺序见 `results/reference_manifest.json`，与电机零点/硬件总线顺序不能直接混用。并联电机参考转速平滑上限 7 rad/s 是参考生成约束，不保证实际动力学转速始终满足。

## 训练

此次实验先进行独立执行器基线，再从基线热启动耦合模型微调。以下命令重现阶段与参数，训练数值不保证逐位相同；两个任务分别使用独立权重。

```bash
bash tools/rpo/train_rpo.sh run 6000 independent
bash tools/rpo/train_rpo.sh backflip 5000 independent
mkdir -p initializers
cp runs/rpo_run_v1/last.pt initializers/rpo_run_v1_last.pt
cp runs/rpo_backflip_v1/last.pt initializers/rpo_backflip_v1_last.pt
bash tools/rpo/train_rpo.sh run 2500 coupled
bash tools/rpo/train_rpo.sh backflip 2500 coupled
```

每任务 1024 环境；世界模型 `[1024,1024,1024]`，策略 `[1024,512,256,128]`；历史 10 帧、未来参考 10 帧；物理步长 4 ms、控制步长 20 ms。耦合阶段每物理子步计算 PD，并在并联电机空间作 ±27 Nm 饱和后映射回关节。工作空间无效会终止，转速使用惩罚项，尚未构建实测转矩-转速包络。

本次两任务在一张 4090 上并行训练，基线加微调至停止约 79.8 分钟，选中两份微调第 2500 次 checkpoint；不包含环境搭建、模型适配、重定向和验收时间。

## 本地权重复播与评估

自行训练后，将所选 checkpoint 和同目录 `run_config.json`、`policy_contract.json` 复制到 `results/running/` 或 `results/backflip/`；文件名使用 `ckpt_002500.pt`。保留本地参考 CSV，确保与 `motion_provenance.json`/SHA256 对应。

```bash
bash tools/rpo/replay_result.sh run eval
bash tools/rpo/replay_result.sh backflip video
python tools/rpo/evaluate_trials.py   --ckpt results/backflip/ckpt_002500.pt --trials 8 --scale 0.15   --output outputs/backflip_acceptance.json
```

评估固定从参考首帧开始，不依赖随机挑选有利帧。8 次试验使用 seed 101–108：关节 ±0.015 rad，根部 roll/pitch ±0.015 rad、yaw ±0.03 rad，XY ±7.5 mm，以及轻微根部初速度。跑步要求完成全片且沿参考方向进度 >80%；后空翻要求完成全片、累计身体 pitch 旋转 <−5 rad、末 0.5 秒平均根部速度 <0.25 m/s。这些只覆盖轻微初始扰动。

## 当前边界

这是固定动作参考跟踪，不是摇杆速度指令跑步控制器；后空翻以蹲姿结束，没有起身恢复。尚未导出/接入 RDK，没有实机后空翻测试。实际仿真中踝电机瞬时速度会超过 URDF 标称 8 rad/s；电机转矩-转速曲线、间隙、柔性和惯量细节仍需辨识。请把这里的结果用于仿真研究。
