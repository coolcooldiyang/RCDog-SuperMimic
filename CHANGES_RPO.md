# RPO 适配记录（2026-10-26）

基线：[Motphys/Uni-SuperMimic](https://github.com/Motphys/Uni-SuperMimic)，提交 `faf7818e8abac3ec6abb8f00e6e904d3577f800b`。保留上游提交历史；此前的 G1 功能、算法主体和测试来自上游。

本分支增加：

- RPO 23 自由度机器人模型、命名关节映射、Torch 前向运动学、动作跟踪任务和配置。
- RPO 并联踝 Jacobian、每 4 ms 的电机空间 ±27 Nm 扭矩饱和、工作空间检查与转速惩罚。几何公式移植自 RoboParty GPL 代码。
- 训练入口支持 `training.init_checkpoint` 热启动；恢复模型与策略权重，重新建立优化器和缓冲区。
- 仿真契约保存/恢复实际 `env.scene.model_file`，避免耦合电机模型在评估时回退到独立位置执行器。
- 公开动作来源的 G1→RPO IK 脚本、训练/复播入口、8 次小扰动评估脚本、实际物理轨迹演示和实验记录。

发布版工具路径已改为相对仓库根目录。重定向工具的并联 Jacobian 使用已包含的 Torch 实现，不再依赖本机 C++ 动态库。
