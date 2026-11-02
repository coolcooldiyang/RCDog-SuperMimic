# RCDog 1.5 m/s 策略部署包

本目录提供 `omni150` 策略的离线推理文件、参考动作和标定说明。适用硬件：12 自由度四足，四条腿各占一个通信端口，每腿髋 ID=1、大腿 ID=2、小腿 ID=3；髋和大腿直驱，小腿额外 2:1 减速。

**本仓库不包含实机电机通信控制程序，也没有在实机上使能过电机。** 下面的步骤需要读者根据实际电机 SDK 实现适配层。

## 1. 包含的文件

| 路径 | 用途 |
| --- | --- |
| `bundle_omni150/actor_fp32.onnx` | actor 推理图，输入 `obs_hist (1,10,48)`、`goal (1,390)`，输出 `action (1,12)` |
| `bundle_omni150/actor_fp32.ts` | 同一权重的 TorchScript 版本，便于对照 |
| `bundle_omni150/references/*.npz` | 十段参考的 `goal[f]`、`R_ref[f]`、`q_ref`、`qd_ref`、`root_p_ref`、`q0`、`dt` |
| `bundle_omni150/golden_inputs.npz` | 固定输入与原 PyTorch 输出，用于核对推理实现 |
| `bundle_omni150/export_report.json` | 权重哈希、输入输出形状、TorchScript / ONNX 误差 |
| `bundle_omni150/policy_contract.json` | 关节顺序、零位、增益、限幅、观测字段与坐标定义 |
| `policy_contract.json` | 同上的顶层副本 |
| `CALIBRATION_AND_INTERFACE.md` | 标定、坐标变换、SDK 单位和状态机的详细说明 |
| `verify_bundle.py` | 离线核对 ONNX 输出并测本机推理耗时，不打开任何设备 |
| `check_hardware_config.py` | 离线检查硬件配置模板字段，不打开端口 |
| `hardware.template.yaml` | 四端口、12 电机、传动、零位和 IMU 配置模板 |

## 2. 先做离线核对

```bash
python real_deploy/verify_bundle.py --bundle real_deploy/bundle_omni150
```

脚本会用 `golden_inputs.npz` 比较 ONNX 输出，要求最大绝对误差小于 `1e-4`，并报告 CPU 推理耗时。本仓库发布时的结果为 `5.96e-7`（100 个样本）。

策略输入输出约定：

- 策略周期 **20 ms**（50 Hz），电机控制周期 **4 ms**（250 Hz）。
- 观测历史 `obs_hist` 形状 `(1,10,48)`，按时间从旧到新；重置时首帧重复十次，上一动作为零。
- 单帧 48 维：`q-q0`（12）、`dq`（12）、机身系陀螺仪（3）、机身系重力（3）、上一原始动作（12）、相对参考旋转 6D（6）。
- 未来参考 `goal` 形状 `(1,390)`：当前帧之后的 10 帧，每帧 39 维。
- 输出 `action` 形状 `(1,12)`，经 `q0 + 0.35 * action` 和 ±1.55 rad 限幅后作为关节目标。
- 网络内已包含训练得到的归一化参数，**不要再做一次归一化**。
- 原 PyTorch 权重见 `results/omni150/ckpt_002500.pt`，SHA-256 `6bcbcb23c3ce2717c7b48588f5c0959b7b7ed45ee091fe4cdad01d4e63d44fe6`。

字段级定义和坐标变换见 [CALIBRATION_AND_INTERFACE.md](CALIBRATION_AND_INTERFACE.md)。

## 3. 部署顺序

1. 确认四个串口与 IMU 的稳定设备路径，不要把 `/dev/ttyUSB0～3` 写死。
2. 标定每个电机的方向、零位和减速比；小腿 2:1 可能不是简单对角映射，需要实测传动矩阵。
3. 对齐 IMU 安装矩阵与初始航向，核对四元数方向、陀螺仪单位和时间戳。
4. 用固定输入核对 actor，再核对站立测量输入。
5. 单关节低速测试，确认方向、限位和力矩符号。
6. 支撑站立，确认四脚承重后进入参考播放。
7. 先低速档，再逐档提速。本仓库只包含 1.5 m/s 档；实机初次落地不建议直接使用该档。

策略是**参考动作条件化**的：必须同时提供 `obs_hist` 和当前参考帧的未来十帧 `goal`。只给三个速度指令不能运行；连续摇杆控制和任意方向切换需要另外生成参考并验证切换过程。

## 4. 安全与限制

- 未在实机验证；电机型号、减速比、编码器方向、IMU 安装和传动刚度都需要实测。
- 小腿仿真扭矩上限 `42.66 = 23.7 × 2 × 0.9 N·m`，其中 0.9 是训练假定的传动效率，未实测。
- 仿真中髋和大腿在高速档会达到 23.7 N·m 限幅；实机连续运行需要考虑发热和电流限制。
- 修改滤波器、动作混合、目标斜率限制等控制逻辑会改变闭环，应在 MuJoCo 中重新验证，不能靠混合动作掩盖零位或坐标错误。
