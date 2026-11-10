# 常用命令与日志

以下命令在仓库的 `real_deploy/controller/` 下执行。安装与本机配置见 [部署说明](README.md)。

## 每次启动

终端一启动后台，已有后台时不重复启动：

```bash
./dognew serve --gamepad
```

终端二查看状态。要求12轴在线、错误码为0，狗处于原标定趴姿，手柄按键松开、摇杆回中：

```bash
./dognew status
./dognew init-lie &&
./dognew stand &&
sleep 8 &&
./dognew servo-check &&
./dognew arm-walk
```

中间命令失败时，`&&` 后续命令不会执行。`arm-walk` 解锁后持续跟踪运行故障；`Ctrl+C` 退出这个跟踪窗口不会停止后台或机器人。

## 动作与状态

| 指令 | 行为 |
| --- | --- |
| `./dognew status` | 查看在线轴、故障、IMU、手柄、当前策略与解锁状态 |
| `./dognew init-lie` | 趴姿建立软件零位，不使能；站姿不执行 |
| `./dognew stand` | 起立到默认姿态并保持 |
| `./dognew servo-check` | 检查策略PD下的静态保持，结束恢复站姿 |
| `./dognew arm-walk` | 解锁全向手柄控制并跟踪错误 |
| `./dognew arm-walk --no-watch` | 解锁后立即退出命令行，不持续显示错误 |
| `./dognew watch` | 跟踪当前运行状态和故障 |
| `./dognew policy-stop` | 请求停止策略，返回站姿保持；不是撤去出力 |
| `./dognew lie` | 策略运行中先减速、恢复站姿，再趴下 |
| `./dognew shadow` | 约8秒影子推理，电机保持站姿 |
| `./dognew play` | 已解锁后直接进入策略 |
| `./dognew clear-fault` | 检查12轴后清除软件锁存故障，撤去出力，不自动恢复动作 |
| `./dognew reconnect` | 未使能时重新连接配置的设备，随后重新读零 |
| `./dognew release` | 立即全部松弛，后台继续运行 |
| `./dognew stop` | 立即全部松弛并退出后台 |

`release`、`stop`、`clear-fault` 会撤去出力，机身需被托住。`policy-stop` 后仍需观察后台模式回到 `IDLE` 才进行其他姿态操作。正常结束测试可先 `lie`，完全趴下后再 `stop`。

## 手柄

| 操作 | 功能 |
| --- | --- |
| 左摇杆 | 前后、左右与斜向平移，合速度上限1.5 m/s |
| 右摇杆左右 | 旋转，角速度上限1.5 rad/s |
| 摇杆回中 | 当前帧给零速度参考，保持策略 |
| B | 停止策略后趴下 |
| A | 停止策略、返回站姿 |
| LB+A | 未使能时起立，已使能时请求趴下 |
| LB+RB+A | 软件急停，立即松弛并锁存故障 |

## 故障恢复

先查看原因，恢复供电/通信等实际故障；狗摆回原标定趴姿后：

```bash
./dognew clear-fault &&
./dognew init-lie &&
./dognew stand &&
sleep 8 &&
./dognew servo-check &&
./dognew arm-walk
```

USB重连、断电或后台重启后，不沿用旧会话零位。`clear-fault` 不能修复未回复电机，也不能清除电机固件仍在上报的错误码。

## 日志

让后台输出保存为当前日志，同时在窗口显示：

```bash
mkdir -p reports
./dognew serve --gamepad 2>&1 | tee -a reports/newurdf_service.log
```

不要在已有后台时再运行这条。查看日志：

```bash
./dognew status
ls -lt reports/command_errors/ | head
ls -lt reports/faults/ | head
tail -f reports/newurdf_service.log
python -m json.tool reports/command_errors/具体文件名.json | less
```

命令错误JSON中的 `response.error` 是这条命令被拒绝的原因；`status.fault` 是锁存故障；`fault_detail.report_path` 指向故障现场。`reports/faults/*.json` 保存触发前状态，`.post.jsonl` 保存撤力后约3秒，`.timing.json` 保存调度与IMU证据。`reports/newurdf_policy/*.jsonl` 保存策略帧、速度指令、观测与目标角。

| 现象 | 先看什么 |
| --- | --- |
| `No such file or directory`，后台状态不可用 | 后台进程和本项目IPC是否存在，先启动serve |
| 某整条腿三个电机均无回传 | 对应UART、首个电机连接与这条腿共用供电 |
| 12轴反馈过期 | USB是否重连、FTDI延迟、线程调度与后台占用；不是仅凭灯亮判断通信 |
| IMU过期 | 时间戳、CRC、接收间隔、端口错误和CPU占用 |
| 检查前提示手柄未就绪 | `deployment.gamepad_readiness` 中的轴值、按下按键与armed |
| 倾角保护 | 触发时的姿态和现场碰撞/支撑情况 |

累计失败数包含历史断电或拔插，判断当前通信应比较一段时间内的新增失败；日志末尾的旧错误也不一定是本次失败原因。
