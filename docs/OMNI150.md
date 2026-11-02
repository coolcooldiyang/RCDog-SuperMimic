# RCDog 全向 1.5 m/s

目标：八个平移方向的合速度达到 1.5 m/s，原地左右旋转达到 1.5 rad/s。一份策略覆盖十段参考。框架来自 [Motphys/Uni-SuperMimic](https://github.com/Motphys/Uni-SuperMimic)。

## 结果

最终权重 `results/omni150/ckpt_002500.pt`。2026-10-27 在本机 MuJoCo 3.16 / Torch 2.6 CPU 的 sim2sim 中，10 次基准和 80 次初态扰动全部通过。

| 平移方向 | 实测合速度 |
| --- | --- |
| 前进 | 1.525 m/s |
| 左前 | 1.537 m/s |
| 左移 | 1.523 m/s |
| 左后 | 1.506 m/s |
| 后退 | 1.507 m/s |
| 右后 | 1.502 m/s |
| 右移 | 1.499 m/s |
| 右前 | 1.513 m/s |

左转 +1.491 rad/s，右转 −1.502 rad/s。旋转动作约转过 8.64 / −8.73 rad，结束后站定。

| 关节类别 | 90 次测试中的峰值速度 | 峰值扭矩 |
| --- | --- | --- |
| 髋 | 17.685 rad/s | 23.70 N·m |
| 大腿 | 17.365 rad/s | 23.70 N·m |
| 小腿 | 14.548 rad/s | 28.19 N·m |

小腿速度达到设定 15.05 rad/s 上限的 96.7%；髋、大腿出现峰值扭矩限幅。结果基于本文电机仿真参数，尚未进行实机验证。

三档分别选用第 1000、500、2500 次权重，权重内记录训练时间为 421.20、163.32、940.62 秒，使用已有权重及编译缓存。各档通过后停止训练；GPU 和框架 MuJoCo 后端复核均无失败动作。最终档 GPU / 框架 CPU 归一化回报为 0.9451 / 0.9340。

汇总与权重校验值见 `results/SIM2SIM_SUMMARY.json`。视频在独立 CPU MuJoCo 中渲染，实体为策略实际轨迹、蓝色半透明为参考，保留各自全局位置。

## 训练安排

| 阶段 | 平移合速度 | 双向旋转 | 平移步态周期 / 占空比 / 抬脚 |
| --- | --- | --- | --- |
| omni050 | 0.5 m/s | 0.75 rad/s | 0.40 s / 0.60 / 4 cm |
| omni100 | 1.0 m/s | 1.00 rad/s | 0.32 s / 0.58 / 3.5 cm |
| omni150 | 1.5 m/s | 1.50 rad/s | 0.28 s / 0.56 / 3 cm |

每档用一份策略覆盖前、后、左、右、四个斜向及左右纯旋转，共 10 段参考。斜向分量为合速度除以 √2。旋转参考周期 0.36 s，占空比 0.60、抬脚 3.5 cm。

每段 9 秒，1 秒后平滑加速 1.2 秒，6.8 秒开始减速，8 秒后站定。验收速度统计取 2.5–6.8 秒。保持 FL + RR / FR + RL 对角交替。

第一档从上一版左右横移策略初始化；后续档使用上一档验收通过的权重。世界模型和策略继续学习，经验与优化器重新建立。1024 环境，前 250 次只训练世界模型，每 500 次保存 checkpoint。架构沿用世界模型 `[512,512,512]`、策略 `[512,256,128]`。

## 模型和限制

沿用原模型质量、惯量、碰撞和关节变换。髋 / 大腿 Go1 电机输出轴直驱，小腿额外 2:1；小腿额外传动效率假定 90%。扭矩限幅为髋 / 大腿 ±23.7、小腿 ±42.66 N·m，速度上限分别为 30.1、15.05 rad/s。Kp=60、Kd=2，电机控制 4 ms，策略 20 ms。

三档 30 段参考均检查有界 IK、关节速度、自碰撞。最终档横移要求最高，参考峰值约为设定速度上限的 87.5%。验收还检查每个物理子步的实际关节速度；参考可行不等于策略已达到目标。

## 复现

在仓库根目录：

```bash
uv run python tools/rcdog/generate_omni.py
uv run python tools/rcdog/validate_runtime.py --backend mjwarp --num-envs 32
uv run python tools/rcdog/train_pipeline.py
```

生成动作需要 SciPy。训练依赖版本见 `requirements-tested-cloud.txt`，本机独立 sim2sim 依赖见 `tools/rcdog/requirements-local-sim2sim.txt`。

`tools/rcdog/plan.json` 为分档配置，`tools/rcdog/plan_150.json` 为单档配置；`tools/rcdog/pipeline_status.json` 记录当前阶段、进程和已选权重。已有 run 时程序会拒绝覆盖。只有当前档全部动作通过，才进入下一档。

## 验收

独立 CPU MuJoCo 中自由基座运行，初始重置后不设置根部轨迹、不施加辅助力。速度、位移相对目标误差不超过 ±10%，同时检查对角触地相位、四足抬脚、漂移、非足部触地、关节角与子步速度。纯旋转还检查机身 XY 漂移。

前两档每段做 3 次初态扰动，最终档每段做 8 次。扰动范围：关节角 ±0.01 rad、根部 XY ±0.005 m、线速度 ±0.03 m/s、角速度 ±0.05 rad/s。

通过后以框架 MuJoCo 后端和本机 MuJoCo 3.16 进行 sim2sim。权重、模型、关节顺序、参考动作和运行配置需配套使用。这里训练的是带未来动作参考的仿真策略。

已保存权重可直接复核：

```bash
uv run python tools/rcdog/evaluate_locomotion.py \
  --stage omni150 --ckpt results/omni150/ckpt_002500.pt \
  --output results/omni150/recheck.json

uv run python tools/rcdog/evaluate_locomotion.py \
  --stage omni150 --ckpt results/omni150/ckpt_002500.pt \
  --output results/omni150/recheck_trials.json --trials 8 --noise 0.01
```
