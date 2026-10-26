# 实验记录

此目录保存本次训练配置、日志、契约和评估汇总。记录中的 checkpoint、reference CSV 和轨迹 NPZ 文件名用于关联本机完整实验，**这些文件不在公开仓库中**。权重哈希见 `checkpoint_manifest.json`；演示文件位于 `../docs/assets/rpo/`。

发布时只移除了云端主机的绝对路径前缀，数值指标保持原样。实际使用自己的 checkpoint 时，应同时使用其训练生成的契约和配置，不能只把公开配置套到其他权重上。
