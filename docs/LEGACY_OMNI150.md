# 早期全向策略归档

早期版本使用10帧历史、动作比例0.35，按omni050 → omni100 → omni150训练。权重、报告与演示保留在 `results/omni*/`、`real_deploy/bundle_omni150/` 和 `docs/assets/rcdog/omni150_mujoco.mp4`。

原代码与完整复现说明固定在提交 [b2251f5](https://github.com/coolcooldiyang/RCDog-SuperMimic/tree/b2251f5)，原首页见 [该版本README](https://github.com/coolcooldiyang/RCDog-SuperMimic/blob/b2251f5/README.md)。需要复现旧结果时，使用独立checkout：

```bash
git worktree add ../RCDog-SuperMimic-legacy b2251f5
```

当前分支使用11.1134 kg新模型、5帧历史和0.25动作比例。旧权重、旧模型与旧参考必须一起使用，不能只把旧ONNX放入当前实机控制器。
