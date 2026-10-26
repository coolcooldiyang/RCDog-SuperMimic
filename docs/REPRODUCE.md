# 续训、参数与常见问题

安装、数据准备、训练、评估和视频导出的完整步骤见 [README](../README.md)。以下补充续训命令和排查时常用的配置位置。

## 续训

`last.pt` 保存优化器等状态，用于接着同一个训练阶段继续训练。`ckpt_*.pt` 只保存世界模型和策略权重，用于评估或初始化另一阶段。

例如，将后空翻的并联踝微调接着训练到 5000 iterations：

```bash
CUDA_VISIBLE_DEVICES=0 unisupermimic-train \
  task=rpo_super_mimic/mjwarp \
  algo.resume=true \
  algo.load_run=runs/rpo_backflip_v2 \
  algo.checkpoint=last.pt \
  algo.iterations=5000 \
  algo.num_envs=1024 \
  'algo.wm_hidden=[1024,1024,1024]' \
  'algo.policy_hidden=[1024,512,256,128]' \
  algo.wm_batch=1024 algo.pi_batch=1024 algo.wm_batches=10 \
  algo.wm_only_iters=500 algo.ckpt_every=500 algo.log_every=25 \
  env.scene.model_file=src/unisupermimic/assets/robots/rpo/scene_supermimic_coupled.xml \
  env.commands.motion.mjcf_path=src/unisupermimic/assets/robots/rpo/supermimic_coupled.xml \
  'env.commands.motion.motion_files=[src/unisupermimic/assets/motions/rpo/backflip_reference.csv]'
```

`algo.iterations=5000` 是这一阶段的总迭代数。续训沿用原来的模型、动作、网络与采样参数；修改过参数的 run，以它自己的 `run_config.json` 为准。续训时不传 `training.init_checkpoint`。

基线到耦合模型的微调使用 `training.init_checkpoint`：只加载网络权重，优化器和采样缓冲区重新建立。`tools/rpo/train_rpo.sh` 会在 `initializers/` 找到对应权重后添加这个参数；没有找到时会从头训练耦合模型。

## 参数位置

| 修改内容 | 文件 |
| --- | --- |
| 关节顺序、默认角度、PD 增益、动作缩放 | [robot.py](../src/unisupermimic/tasks/motion_tracking/rpo/supermimic/robot.py) |
| 世界模型/策略网络、batch、环境数、checkpoint 间隔 | [train_rpo.sh](../tools/rpo/train_rpo.sh) |
| 动作路径、采样频率、随机化、终止条件 | [base.yaml](../src/unisupermimic/conf/supermimic/task/rpo_super_mimic/base.yaml) |
| 参考数据字段与插值 | [motion.py](../src/unisupermimic/tasks/motion_tracking/rpo/supermimic/motion.py) |
| 可微奖励与关节/转速惩罚 | [mdp.py](../src/unisupermimic/tasks/motion_tracking/rpo/supermimic/mdp.py) |
| 并联踝几何与 ±27 Nm 电机扭矩限幅 | [parallel.py](../src/unisupermimic/tasks/motion_tracking/rpo/supermimic/parallel.py) |
| 每个 4 ms 子步的 PD 控制 | [coupled_backend.py](../src/unisupermimic/tasks/motion_tracking/rpo/supermimic/coupled_backend.py) |
| 质量、惯量、碰撞、关节范围 | [RPO 模型目录](../src/unisupermimic/assets/robots/rpo/) |

23 个策略关节不等于硬件电机编号；并联踝由两个电机共同驱动。参考 CSV 使用模型关节角，不能直接作为电机零点角度下发。

## 扰动测试

`evaluate_trials.py --trials 8 --scale 0.15` 使用 seed 101–108，从参考首帧开始测试。初始扰动范围：

| 项目 | 范围 |
| --- | --- |
| 关节角 | ±0.015 rad |
| 根部 roll / pitch | ±0.015 rad |
| 根部 yaw | ±0.03 rad |
| 根部 X / Y 位置 | ±0.0075 m |
| 根部 XYZ 线速度 | ±[0.075, 0.075, 0.03] m/s |
| 根部 XYZ 角速度 | ±[0.078, 0.078, 0.117] rad/s |

两个动作都要求执行完整段、没有提前终止，另有动作判据：

- 跑步：沿参考位移方向的进度超过参考的 80%。
- 后空翻：累计身体 pitch 旋转小于 −5 rad，末 0.5 秒平均根部速度小于 0.25 m/s。

这组测试检查轻微初始扰动下的动作完成情况，不覆盖外力推搡和实机误差。原始结果见 [acceptance_summary.json](../results/acceptance_summary.json)。

## 常见问题

### 找不到参考 CSV

先运行重定向脚本，确认 `src/unisupermimic/assets/motions/rpo/` 下有 `run_reference.csv`、`backflip_reference.csv`。`results/` 中的 JSON 是实验记录，不包含参考数据。

### 启动后加载了 G1

`unisupermimic-train` 默认任务仍是上游 G1。RPO 训练使用 `tools/rpo/train_rpo.sh`，或在完整训练命令中指定 `task=rpo_super_mimic/mjwarp`。上游的 `unisupermimic-eval` 和 `unisupermimic-video` 也是 G1 入口；RPO 使用 README 中的 Python 模块入口。

### 微调没有加载基线

检查文件名是否为 `initializers/rpo_run_v1_last.pt` 或 `initializers/rpo_backflip_v1_last.pt`。成功加载时终端会显示 `Warm start from iteration ...`。

### 评估提示模型、动作或网络配置不一致

保留 checkpoint 同目录的 `run_config.json`、`policy_contract.json`，并使用对应的参考 CSV。不要把 `results/` 中另一份实验配置覆盖到新权重目录。修改过模型、参考动作或关节顺序后，需要重新核对策略输入输出。

### 显存不足

在 `tools/rpo/train_rpo.sh` 中调低 `algo.num_envs`、`algo.wm_batch`、`algo.pi_batch`。改动后属于新的训练配置，保存对应日志即可。

### 视频导出提示 EGL 初始化失败

先检查 `nvidia-smi` 与 NVIDIA EGL/OpenGL 驱动是否可用。无显示器的 GPU 服务器使用 `MUJOCO_GL=egl`；有桌面显示环境时可尝试 `MUJOCO_GL=glfw`。

### 需要把自己训练的权重放到 results 下复播

README 中的评估和导出命令可以直接使用 `runs/` 内的 checkpoint。已有的 `replay_result.sh` 是本次实验的快捷入口，固定读取 `results/running/ckpt_002500.pt` 和 `results/backflip/ckpt_002500.pt`；使用它时，也要复制同一次训练的配置和契约。
