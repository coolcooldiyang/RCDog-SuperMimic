# RCDog版本索引

版本以actor、观测接口、参考生成器、控制后处理和训练源码共同确定。相同输入维度不代表参考可混用。

| 版本 | 固定源码 | 参考/实验入口 | 权重与部署包 |
| --- | --- | --- | --- |
| V1 omni050/100/150 | 历史提交 `b2251f5`，[旧版说明](LEGACY_OMNI150.md) | `tools/rcdog/`，10帧历史 | `results/omni*`，`real_deploy/bundle_omni150/` |
| V2 command150/final500 | tag `v2-command150-final500`，提交 `3ab8fab` | `experiments/command150/`；固定0.32秒、最高2.5cm参考，5帧历史 | `results/command150/`，`real_deploy/policies/newurdf_command150_final500/` |
| V3 flat_legged_a3/1000 | tag `v3-flat-legged-a3`；本次主分支训练框架 | `experiments/flat_legged_a3/`；动态0.4～0.6秒、8cm参考，5帧历史 | `results/flat_legged_a3/`，`real_deploy/policies/newurdf_legged_a3_1000/` |

V2与V3均使用48维单帧观测、390维参考目标、12维动作；V3增加位置目标变化率限制，历史previous_action记录限制后的实际动作。V3还需要独立的摆腿相位/支撑标签、动态周期与回中收腿处理。V1为10帧历史，不能用V2/V3控制器加载。

V3按参考和步态评价选择a3，不是全部验收通过的正式稳定版。对比报告包含被拒绝的a1/a4，不把候选末次成绩替换成中间检查点成绩。实机视频对应作者提供的0.5m/s指令表现，不能替代1.5m/s全速验证。

精确复现V2源码时，在独立克隆中使用：

```bash
git switch --detach v2-command150-final500
```

当前分支保留V2资产与入口以便对照；根目录训练框架已加入V3分支和可选观测老化支持。V3选中a3未使用观测老化训练（默认0ms），a4的0～15ms插值训练是未采用的独立实验。训练入口会清除继承的RCDOG地形变量并显式设置本版本开关，避免把平地训练切换到暂停的地形实验。

公开控制器的 `configure_ports.py --policy legged-a3` 选择新示例，`--policy command150`选择旧示例。已有 `config/` 受忽略规则保护，不随git更新覆盖；切换时应先结束实机动作和后台，再备份并重建本机配置。发布操作本身不启动实机服务。
