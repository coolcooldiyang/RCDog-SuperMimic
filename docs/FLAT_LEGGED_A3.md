# V3：legged_control参考的高抬脚平地实验

## 对应文件

- actor：`real_deploy/policies/newurdf_legged_a3_1000/actor_fp32.onnx`。
- 原始网络checkpoint：`results/flat_legged_a3/a3/ckpt_001000.pt`，SHA256 `758b3ebed59cdbd8a2c7d830d0a64c9cc22cf496fa964888d084054d462045ff`。
- 参考：`experiments/flat_legged_a3/reference_candidates/legged_control/` 和68段 `src/unisupermimic/assets/motions/rcdog_flat_legged/`。
- 训练阶段：a0从V2 final500加载；a2回到a0使用温和滑移奖励；a3从a2用低学习率和失败动作重置采样细化。a1/a4不在选中链中。
- 实机视频：[0.5 m/s指令片段](assets/rcdog/legged_a3_real050_20261111.mp4)。作者提供，约19.6秒，有扶手跟随；没有同步速度遥测。视频元数据与处理记录见 [REAL_VIDEO.json](../results/flat_legged_a3/REAL_VIDEO.json)。

## 控制与动作

模型质量11.1134kg。Kp20/Kd0.5按关节侧定义；内部减速6.33，小腿总传动12.66。策略50Hz，原生PD指令200Hz；训练与CPU评估物理1ms。髋/大腿位置目标变化率30.1rad/s，小腿15.05rad/s，反馈速度不截断。actor输入没有相机或地形扫描。

Go1的trot/flying_trot时序与SplineCpg摆腿样条被重定向到RCDog IK；支撑足路径按SE(2)运动抵消机身速度，水平摆腿匹配触地速度。参考高度上限8cm，极低足速按活动比例缩小，周期按足端速度从0.6秒过渡到0.4秒，占空比从0.5过渡到0.375。CSV侧的 `.gait.npz` 保存独立于足高的相位/支撑/高度标签。

本次0.5m/s平移CPU完整摆腿峰值约7.4～7.8cm，0.5rad/s纯旋转约4.7～4.9cm；其参考纯旋转高度约5.2cm。滑移指标来自真实接触帧，不能用理想参考的零滑移代替；门槛为平均触地足速度<0.12m/s、95分位<0.3m/s，以及摆腿高度、对角接触与稳定性检查。

## 评估与限制

原始报告在 [results/flat_legged_a3/reports/](../results/flat_legged_a3/reports/)。a3全速稳定性58/60，实际步态57/60；0.5档部署链22/22。未放宽测试门槛以接纳失败候选。

仍失败的测试：

- 无延迟 `diag_bl`：触地滑移95分位约0.3265m/s，超过0.3。
- 8ms延迟噪声 `right_1.5`：真实关节速度越限。
- 8ms延迟噪声 `ccw_1.5`：停步目标摆幅约7.118°，超过6°。

实机还出现过USB公共链路断联、右后UART4超过20ms，以及回中后IMU倾角增长到30.4°。这些记录说明通信与停步两类问题均需继续处理；本次视频不代表这些问题已全部修复。公开配置默认0.5m/s与0.5rad/s，保持12ms新目标关节反馈、20ms IMU有效帧和倾角等保护。

## 部署计算优化

新参考不能使用旧版固定周期捷径。实机适配器批量计算动态周期和独立收腿参考，和包内原始生成器逐项核对；同一实测状态回放中目标/动作差为0，计算95分位约从5.9ms降到0.69ms，见 [runtime_timing.json](../results/flat_legged_a3/reports/runtime_timing.json)。这是指定机器上的离线耗时，不是实时调度保证。

控制器遇到短暂旧反馈时在8ms预算内等待新帧，接受新目标时仍要求关节≤12ms、IMU≤20ms；关节20ms硬时限、IMU30ms读取器过期与串口异常仍失败。位姿使用实际检查通过的IMU帧，不因后续新帧到达而接受之前的旧姿态。

## 复现与来源

训练、评估、导出和手柄命令见 [首页](../README.md)。所有命令从仓库根目录执行；不需要复制本机目录或云端连接信息。`results/flat_legged_a3/source_snapshot_a3/`保留选中策略在观测老化修正之前的源码快照；根框架默认老化0ms，支持后续可选实验。

参考方法来源：[qiayuanl/legged_control](https://github.com/qiayuanl/legged_control/tree/a7f381c0367e98e31c01336e678eef47e304d40d)、[OCS2](https://github.com/leggedrobotics/ocs2)。配置与源码通知保存在reference_candidates的upstream目录。训练方法与权重来自本项目Uni-SuperMimic适配，没有使用MPC输出或第三方动作捕捉数据。
