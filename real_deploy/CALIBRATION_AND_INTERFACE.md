# RCDog27 标定与策略接口

配合 [部署流程](README.md) 使用。本文件用于实现硬件适配器；示例公式不是已经连接电机的控制程序。

## 1. 关节和电机编码器

策略顺序固定为 `RR, RL, FR, FL`，每腿 `hip, thigh, calf`；端口内电机 ID 分别为 `1,2,3`。所有策略角度均为 URDF 弧度。

设 `theta_o` 为电机减速器**输出轴**角度，`q` 为 URDF 关节角，同一已知姿态下记录 `theta_cal`、`q_cal`：

```text
q       = q_cal + A (theta_o - theta_cal)
dq      = A dtheta_o
theta_des = theta_cal + inv(A) (q_des - q_cal)
tau_o   = A.T tau_joint                 # 理想虚功映射
```

每条腿有独立的 A 和标定向量。确认轴独立后才使用 `A=diag(s1,s2,s3/2)`。若小腿电机外壳固定在近端支架而不随大腿转动，编码器可能与大腿角耦合；需从多组已知姿态识别 A，不能仅按皮带轮直径填写。

辨识方法：固定膝关节夹角移动大腿，再固定大腿改变膝关节，覆盖数个位置、往返重复，检查线性关系和回差。标定后用未参与拟合的姿态验证预测角度。若传动关系非线性或有明显回差，应建对应模型，而不是强行通过配置检查。

耦合传动下，电机输出侧扭矩约束形成耦合的关节力矩可行域。当前仿真按独立轴建模；若实物存在耦合，应把标定 A、转动惯量和电机限制加入仿真，重新做 sim2sim，必要时重训。

理想输出轴 PD 增益矩阵为 `K_o=A.T K_joint A`。独立 2:1 小腿对应 `Kp_o=60/4=15`、`Kd_o=2/4=0.5`；这仍不是 SDK 的最终数值。矩阵含非对角项时，独立的电机固件位置环不能直接实现完整关节 PD，优先使用主机计算力矩。

小腿仿真扭矩上限 `42.66=23.7×2×0.9 N·m`。0.9 是训练假定效率，尚未实测；不能据此把所有标定力矩或增益统一除以 0.9。

## 2. SDK 内部减速与单位

额外皮带减速和电机内部减速是两层不同的关系。如果实际 SDK 命令／反馈均为转子侧，内部减速比为 G：

```text
theta_o = theta_rotor / G
dtheta_o = dtheta_rotor / G
tau_rotor_cmd = tau_output_cmd / G
Kp_rotor = Kp_output / G²
Kd_rotor = Kd_output / G²
```

查实际 `MotorType` 并用 SDK 的 `queryGearRatio()` 或该型号资料取得 G，不从“Go1”推定数值。若现有封装已经转换成输出轴反馈，就不能再次除以 G。位置零位和多圈角也要遵循该封装的约定。

[Unitree actuator SDK](https://github.com/unitreerobotics/unitree_actuator_sdk) 对 A1/B1 的增益还有协议换算；[A1 官方示例](https://github.com/unitreerobotics/unitree_actuator_sdk/blob/master/example/example_a1_motor_output.cpp) 使用：

```text
sdk_kp = (Kp_output / G²) / 26.07
sdk_kd = (Kd_output / G²) * 100
```

这些因子不能无条件套给 GO-M8010-6。官方示例会主动摆动电机，阅读和借鉴单位转换即可，不要直接作为连机检查程序执行。确认总线、模式、故障码和数据校验也必须使用实际型号对应接口。

## 3. HI12 数据与坐标

HI12 的常见数据包为 `5A A5` 帧头、0x91 数据包，76 字节负载／82 字节总帧，默认 115200、8N1。保留 CRC 校验，不要在控制器中使用跳过 CRC 的等价行为。

| 字段 | 程序中的含义 |
| --- | --- |
| 负载偏移 24 的 3 个 float32 | 原始角速度，°/s |
| 负载偏移 60 的 4 个 float32 | 四元数 WXYZ |
| `--gyro-unit rad` | 仅转换输出显示；`sample.gyr` 与 CSV 仍为 °/s |
| 欧拉角 | Z-X-Y / 312；Roll 对应 Y，Pitch 对应 X，不能直接作为普通 ZYX 使用 |
| 数据时间戳 | 设备启动后的 ms；与主机单调时钟不是同一个时基 |

使用四元数生成旋转矩阵，不经过欧拉角组装。检查有限数和四元数范数；正常的小数值误差可归一化，异常范数、CRC 错误、设备重启和时间戳不连续要拒收或触发重初始化。

定义：I 为 IMU 传感器坐标，C 为 URDF 机身坐标，N 为 IMU 导航坐标，W 为策略世界坐标。`R_AB` 表示把 B 系向量转成 A 系向量。

```text
C: 前 -Y_C，左 +X_C，上 +Z_C
B: 前 +X_B，左 +Y_B，上 +Z_B
R_CB = [[0,1,0],[-1,0,0],[0,0,1]]
```

实测安装矩阵为 `R_CI`，必须正交且 det=+1。若核对后四元数表示 `R_NI`，则：

```text
R_WC    = R_WN @ R_NI @ R_CI.T
gyro_C  = R_CI @ (gyr_raw_dps * pi/180)
gravity_C = -R_WC[2,:]
```

若四元数实际表示反方向，只在适配器中统一转置一次。导航系是 Z-up 还是 Z-down、轴旋转正负均需实测；现有解析器仅解包，不能替代安装验证。

训练初始 `R_WC=Rz(+90°)`。在确认导航系与机身朝向后，水平站立时可做一次航向对齐：

```text
R_NC0 = R_NI0 @ R_CI.T
R_WN = Rz(pi/2 - yaw(R_NC0))
```

这里只补偿初始航向，不抹去实际横滚、俯仰。重新选择参考片段时按它的起始航向重建对齐关系。六轴 IMU 无磁航向约束；长时间航向漂移需实测。

直接填写下面的观测时使用 `gyro_C`。若复用训练源码 `MDP.obs`，它的状态接口接收世界系角速度，再在内部转到机体系，因此应传 `w_world=R_WC @ gyro_C`，避免重复旋转。

## 4. actor 输入输出

来源：本仓库 `src/unisupermimic/tasks/motion_tracking/rcdog/supermimic/mdp.py` 及 `rl/algos/supermimic/networks.py`。输入输出都是 float32，batch 实机取 1。

`obs_hist` 形状 `[1,10,48]`。时间顺序为旧到新；重置时首帧重复十次，上一原始动作为零。

| 单帧切片，左闭右开 | 内容 |
| --- | --- |
| `[0:12]` | 实测 `q_joint-q0`，rad；软件标定后不再另加训练关节偏置 |
| `[12:24]` | 实测 `dq_joint`，rad/s，**不乘 0.1** |
| `[24:27]` | `gyro_C`，rad/s |
| `[27:30]` | `gravity_C` |
| `[30:42]` | 上一次网络原始输出 action，不是目标位置或限幅后的力矩 |
| `[42:48]` | `mat_to_6d(R_WC.T @ R_ref[f])` |

`mat_to_6d(R)=R[:,:2].reshape(6)`，按行排列，得到 `[r00,r01,r10,r11,r20,r21]`；单位矩阵为 `[1,0,0,1,0,0]`。不能按整列连接得到另一种 6D 表示。

没有手工观测缩放；源码中虽然定义了 `OBS_SCALE`，实际 actor 观测路径没有使用它。导出网络已经包含训练得到的 RunningNorm，再归一化一次会改变策略。

`goal` 形状 `[1,390]`：从当前参考帧 f 取 **f+1 到 f+10**，每帧 39 维，在片段末尾夹到最后一帧。每个未来帧内容：

| 每帧切片 | 内容 |
| --- | --- |
| `[0:3]` | 相对当前参考根位置的位置差，转到当前参考航向系 |
| `[3:9]` | 相对当前参考航向系的未来根旋转，同样使用上述 6D 排列 |
| `[9:12]` | 未来参考根部局部线速度 |
| `[12:15]` | 未来参考根部局部角速度 |
| `[15:27]` | 未来参考 q，12 轴 |
| `[27:39]` | 未来参考 dq **乘 0.1** |

推荐直接读取每包 `references/*.npz` 的 `goal[f]`、`R_ref[f]`、`q0` 和 `dt`；不要重新实现差分和裁边。表中还有 `q_ref`、`qd_ref`、`root_p_ref`，用于记录和核对。原始 CSV 的根四元数是 **XYZW**，与 HI12 的 WXYZ 不同。

输出 action 为 12 维无量纲关节偏移参数，经 `q0+0.35*action`、目标限幅、关节 PD 和力矩限幅后才到电机。输出不直接代表 rad、rad/s 或 N·m。

## 5. 控制循环实现规格

以下是接口伪代码，省略了状态机、时间同步和故障处理，不能直接作为实机控制器运行：

```python
# 20 ms，POLICY 状态
q, dq = map_feedback_to_urdf(latest_12_motors, calibration)
R_WC, gyro_C = map_hi12(latest_imu, mounting, heading_alignment)
obs = concatenate(q-q0, dq, gyro_C, -R_WC[2, :],
                  previous_raw_action, sixd(R_WC.T @ R_ref[f]))
history = append_oldest_first(history, obs)
action = actor(history[None].astype(float32), goal[f][None].astype(float32))
q_des = clip(q0 + 0.35*action, -1.55, 1.55)
previous_raw_action = action
# 按单调时钟推进参考帧；超时进入故障，不用补跑一串旧帧追赶

# 4 ms，各腿独立线程，使用已验证的最新目标
q, dq = map_this_leg_feedback()
tau_joint = clip(60*(q_des-q)-2*dq, -joint_tau_limit, joint_tau_limit)
tau_output = A.T @ tau_joint
check_motor_envelope_and_faults(tau_output)
send_model_specific_sdk_torque(tau_output)
```

四脚承重站稳、姿态与参考初始状态接近后才能进入 POLICY。若在运行中附加较强滤波、动作混合或额外目标斜率限制，应把这些逻辑也放进 MuJoCo 再验证；它们会改变闭环。不能靠混合动作隐藏零位、方向或坐标错误。

验收时先在离线固定输入上核对 actor，再核对站立测量输入：`q-q0≈0`、`dq≈0`、`gyro≈0`、重力约 `[0,0,-1]`，初始姿态与参考相同则相对旋转 6D 约 `[1,0,0,1,0,0]`。站姿网络输出不一定严格等于零，应与同一输入的原 PyTorch 输出比较。
