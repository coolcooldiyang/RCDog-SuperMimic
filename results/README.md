当前新增V3高抬脚实验版，权重与完整成功/失败记录见 [flat_legged_a3/](flat_legged_a3/)。V2与早期结果保留在下面原有目录；版本对应关系见 [版本索引](../docs/VERSIONS.md)。

# 实验记录

本目录保存三档训练的配置、验收结果和汇总记录。

## 内容

| 路径 | 说明 |
| --- | --- |
| `omni050/ckpt_001000.pt` | 0.5 m/s、0.75 rad/s 档策略权重，第 1000 次迭代 |
| `omni100/ckpt_000500.pt` | 1.0 m/s、1.0 rad/s 档策略权重，第 500 次迭代 |
| `omni150/ckpt_002500.pt` | 1.5 m/s、1.5 rad/s 档策略权重，第 2500 次迭代 |
| `omni*/run_config.json` | 训练时的完整 Hydra 配置，含动作文件、网络结构和采样参数 |
| `omni150/ckpt_002500.json` | 十段参考的基准评估汇总 |
| `omni150/ckpt_002500_omni150_*_seed101.json` | 每段参考的单项指标 |
| `omni150/ckpt_002500_trials.json` | 80 次初态扰动的逐次记录 |
| `omni*/framework_eval.json` | 框架自带 MuJoCo 后端的复核结果 |
| `omni150/gpu_eval.json` | GPU 后端复核结果 |
| `SIM2SIM_SUMMARY.json` | 三档的汇总、权重哈希、模型与动作哈希 |
| `pipeline_status.json` | 训练流水线的阶段状态与验收明细 |

## 权重哈希

| 权重 | SHA-256 |
| --- | --- |
| `omni050/ckpt_001000.pt` | `a3edd6fa4038acd40fe8f1cddbc2ec5ce47e95287406fd66c7170b70f4a760f9` |
| `omni100/ckpt_000500.pt` | `2340c90bd4199dab946bf7cb130b22a7cd60dc6f33fd496316cf1a286129ca46` |
| `omni150/ckpt_002500.pt` | `6bcbcb23c3ce2717c7b48588f5c0959b7b7ed45ee091fe4cdad01d4e63d44fe6` |
| `initial_policy/strafe025.pt` | `fe0c70622f445939b68cae7981e2c401787e5d134aff3e0e51b0da66d68af109` |

权重中的 `checkpoint` 字段已改为仓库相对路径。发布时移除了逐帧轨迹 NPZ、`runs/` 中的完整 `last.pt`（含优化器状态）和本机压缩包；需要时用 `tools/rcdog/evaluate_locomotion.py` 重新生成轨迹，用 `tools/rcdog/train_pipeline.py` 重新训练。
