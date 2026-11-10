# command150训练与结果

## 版本与参数

当前发布checkpoint：`results/command150/ckpt_000500.pt`。

- checkpoint SHA-256：`f9a9736547d8325949b03b0e3c635e69bf1fcbdb14431f4d60abd525c8772205`
- ONNX SHA-256：`f5ac62876e067585864849170ea8a45ada97fe1ea9adc1aaeb8e5cfebda10be4`
- 源URDF SHA-256：`a8265c6998848e1a66784e450e4fa7f374c95a4f67d88e0f92142798723423c7`
- 质量11.1134 kg，包含电池、控制器和线束；源URDF与转换模型位于仓库的 `source/` 与 `src/unisupermimic/assets/robots/rcdog/`。
- 策略周期20 ms，训练物理步长5 ms，CPU独立评估步长1 ms；PD20/0.5、5帧历史、动作比例0.25。
- 策略输入 `(1,5,48)` 观测历史、`(1,390)` 未来参考；输出 `(1,12)`。训练归一化已嵌入网络，不重复归一化。

## 训练衔接

父策略 `parent_lateral150.pt` 来自同一机器人、同一SuperMimic框架的横移/旋转阶段。发布目录提供父策略，下面复现的是从该权重开始的前后速度扩展与全向微调；早期10帧策略不是这条训练链的直接输入。

| 阶段 | 输入 | 总迭代 | 输出目录 |
| --- | --- | --- | --- |
| mid | 横移父策略 | 750 | `runs/command150_mid/` |
| high | mid750 | 2000 | `runs/command150_high/` |
| final | high2000 | 750 | `runs/command150_final/` |

最终训练到750次，选中500次的checkpoint。750次候选在连续混合测试中后退横向漂移超限，没有替换已通过的500次候选。训练记录在 `results/command150/run_config.json`、`training_records/` 和 `train_log.csv`。

每阶段通过 `+training.init_checkpoint` 加载网络权重，重建经验缓冲和优化器。要恢复同一个中断run，使用上游 `algo.resume`、`algo.load_run` 和 `algo.checkpoint`，同时沿用该run的配置；不要把“加载父策略微调”与“恢复优化器续训”混用。

## 参考生成

`experiments/command150/build_references.py` 使用足端轨迹、MuJoCo模型与逆运动学生成速度网格，`command_reference.py` 插值当前速度和步态相位，生成未来10帧、每帧39维参考。平移网格节点保留原始分量，只对请求的平移合速度做1.5 m/s限幅，避免对角方向插值时速度被额外压低。

参考周期0.32 s、占空比0.5、最大抬脚0.025 m。训练包含72段速度、对角、静止和启停CSV。它是平地参考，不包含楼梯、视觉或地形高度扫描。

训练额外惩罚机身倾斜、超过15°的倾斜、横滚/俯仰角速度，以及零速时的关节速度、动作变化和机身速度。训练终止还包括25°倾角和调试关节速度上限；完整公式在 `src/unisupermimic/tasks/motion_tracking/rcdog/supermimic/mdp.py`。

## 两种停步处理与结果

| 测试 | 使用的参考处理 | 结果 | 记录 |
| --- | --- | --- | --- |
| 多方向、多速度、8 ms延迟与噪声 | 训练配套平滑速度参考 | 60/60通过 | `results/command150/sim2sim_final500.json` |
| 电机SDK增益量化、传动与采样模型 | 原部署参考处理 | 14/14通过 | `results/command150/native_final500.json` |
| 无重置连续启停与换向 | 平滑参考 | 98.6秒通过 | `results/command150/mixed_final500.json` |
| 回中立即清零、0.4秒后再次推杆 | 当前实机的 `instant_center_reference.py` | 10/12通过 | `results/command150/neutral_continuity_20261110.json` |

最后一组中，0/8 ms延迟的1.5 m/s横移均在停步后约0.32秒触发“调试关节速度越限”。0.5 m/s前进/横移、1.5 m/s前进以及0.5/1.5 rad/s旋转的测试完成；所有案例的回中参考都按当前帧清零。这个问题保留在报告中，未通过提高速度保护阈值处理。

`experiments/command150/command_sim.py` 保留训练配套平滑参考，供复现60案例基准。`real_deploy/controller/tools/verify_neutral_continuity.py` 使用当前实机的完整actor、即时回中处理和电机侧换算模型，供复现后一组专项测试。运行后者不打开电机串口：

```bash
cd real_deploy/controller
source .venv/bin/activate
python -m pip install mujoco==3.11.0
PYTHONPATH=.:tools python tools/verify_neutral_continuity.py
```

该脚本会在 `reports/` 写入新报告；存在未通过案例时返回非零退出码。它与源码副本的一致性可由发布清单核对。

## 实机视频

`docs/assets/rcdog/rcdog_real_20261110.mp4` 来自用户提供的当前策略实机录像，转为H.264/AAC并移除容器元数据。原始与发布视频SHA见同目录JSON。画面为室内地面测试，有人持扶手跟随；没有同步遥测，不能从视频读出实际最高速度或持续稳定运行时间。

导出包中的 `hardware_ready: false` 是导出时记录，保持原义。后续实机录像单独列出，不把它回写成对任意机器的标定确认。
