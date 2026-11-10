# RCDog实机部署

当前版本是5帧历史的 `command150 / final500`，对应首页实机视频。早期10帧的 `bundle_omni150/` 仍保留作归档，不用于下面的控制器。

## 硬件与连接

| 部件 | 本项目配置 |
| --- | --- |
| 电机 | 12个GO-M8010-6，内部减速比6.33:1 |
| 外部传动 | 髋、大腿直驱；小腿皮带2:1 |
| 通信 | 宇树USB转四路RS-485，每腿独占一路，4 Mbps |
| UART1 / UART2 | 左前FL / 右前FR |
| UART3 / UART4 | 左后RL / 右后RR |
| 每腿电机ID | 髋1、大腿2、小腿3 |
| IMU | HI12M0-MI1-000，115200波特率，约100 Hz；传感器X向前、Y向左 |
| 手柄 | ShanWan接收器，使用Linux joystick接口 |

策略关节顺序是 **RR、RL、FR、FL**，每腿髋/大腿/小腿；状态机顺序是 **FL、FR、RR、RL**。控制程序按腿名映射，不按列表下标直接拼接。

## 1. 安装运行环境和SDK

以下命令在本目录下的 `controller/` 执行。独立控制器不需要ROS或GPU：

```bash
cd real_deploy/controller
python3 -m venv .venv
source .venv/bin/activate
python -m pip install -r requirements.txt
```

编译依赖为CMake、C++编译器和当前Python版本的开发头文件。Ubuntu可安装：

```bash
sudo apt install build-essential cmake python3-dev python3-venv
bash tools/install_unitree_sdk.sh
```

安装脚本从宇树官方仓库获取固定提交 `5b79a42d81cd69adac1367db79efdb972a898fd1`，对 `sendRecv` 应用释放Python GIL的绑定补丁，再编译对应Python扩展。脚本只构建和导入SDK，不打开电机串口。SDK核心库、编译产物和本机配置不纳入本仓库。

四路485、IMU和手柄采集使用独立线程；未释放GIL的同步串口调用会阻塞其他采集线程。运行 `status` 时应看到 `sdk_send_recv_releases_gil: true`。

## 2. 配置端口

接入USB后，确认当前用户有串口和手柄访问权限。自动识别唯一的四路FTDI与CP2102N设备，并生成本机配置：

```bash
python tools/configure_ports.py
```

多个适配器时手工指定稳定设备路径，`--motor-prefix` 不包含末尾的接口编号：

```bash
python tools/configure_ports.py \
  --motor-prefix /dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_YOUR_SERIAL \
  --imu /dev/serial/by-id/usb-Silicon_Labs_CP2102N_USB_to_UART_Bridge_Controller_YOUR_SERIAL-if00-port0
```

脚本把 `examples/` 复制到忽略提交的 `config/`，不打开端口。已有配置时拒绝覆盖；确需重建时加 `--force`。USB重插后控制器按by-id身份重连，不自动读零、使能或恢复行走。

检查并设置FTDI延迟：

```bash
python tools/set_ftdi_latency.py --check
sudo python3 tools/set_ftdi_latency.py --ms 1
```

四路均应为1 ms；拔插或重启后需重新检查。脚本只写配置中的四路驱动延迟，不发送电机命令。

## 3. 站姿、传动与坐标

`config/stand_poses.yaml` 使用**相对原标定趴姿、带方向符号的电机输出轴角度，单位度**。本机默认站姿为：

| 腿 | 髋 | 大腿 | 小腿电机输出轴 |
| --- | --- | --- | --- |
| FL | −0.5° | 105.0° | 111.9° |
| FR | −0.1° | 105.0° | 111.2° |
| RR | −0.1° | 106.3° | 112.8° |
| RL | −0.1° | 106.9° | 112.4° |

这些角度不是URDF关节角，也不是电机转子角。`deploy_newurdf.yaml` 中的实测标定角用于把默认站姿绑定到策略 `q0`，不要用表格四舍五入值覆盖实测值。

传动在每条腿中写成 `q = q_cal + A · (theta_out - theta_cal)`，其中 `theta_out = theta_rotor / 6.33`；小腿2:1由矩阵中的0.5体现。转子侧PD增益按总减速比平方换算：髋、大腿除以 `6.33²`，小腿除以 `12.66²`。训练中的小腿速度能力是15.05 rad/s，扭矩上限42.66 N·m，传动效率假定0.9。

这些方向、站姿和标定值属于本项目这台RCDog。另一台机器需核对电机零点、转向与IMU安装后再使用；`init-lie` 只建立当前会话的软件参考，不会执行电机固件的零点标定。不要在站立时执行 `init-lie`。

## 4. 起立并运行策略

先让机器人处于原标定趴姿、机身有防摔支撑，手柄开机且按键松开、摇杆回中。

终端一：

```bash
./dognew serve --gamepad
```

启动只进行回传查询，不使能。出现 `online: 12` 后，终端二进入相同的 `real_deploy/controller/`：

```bash
./dognew init-lie &&
./dognew stand &&
sleep 8 &&
./dognew servo-check &&
./dognew arm-walk
```

`stand` 使用原起立状态机，约6秒展开。`servo-check` 在策略PD下静态保持后返回站姿，通过后才允许 `arm-walk` 解锁。实走时四脚承重、支撑仅防摔。

左摇杆平移，右摇杆左右旋转。回中在当前策略帧把速度参考清零，不重置相位、历史或航向，也不退出策略；再次推杆无需等待固定冷却时间。B请求停止策略后趴下。

`arm-walk` 默认持续跟踪日志；另开终端可执行 `./dognew lie`。在日志跟踪终端按Ctrl+C只退出监视；在 `serve` 终端按Ctrl+C会退出后台并撤去出力。

## 5. 控制参数与保护

| 项目 | 当前值 |
| --- | --- |
| 策略周期 / 电机指令周期 | 20 ms / 5 ms |
| 策略关节侧PD | Kp20 / Kd0.5 |
| 观测历史 / 动作比例 | 5帧 / 0.25 |
| 平移合速度 / 旋转指令上限 | 1.5 m/s / 1.5 rad/s |
| 新目标要求的关节反馈年龄 | 12 ms以内；20 ms硬时限 |
| IMU有效数据年龄 | 20 ms以内；站姿检查最多等8 ms新帧，读取器30 ms过期仍拒绝 |
| 起立保持倾角 / 策略倾角 | 15° / 30° |
| 调试关节速度阈值 | 髋8、大腿10、小腿10 rad/s |
| 温度阈值 | 70°C |

策略训练的物理步长为5 ms，独立CPU评估为1 ms。物理能力上限30.1/30.1/15.05 rad/s与上述实机调试阈值是不同字段。当前发布的平地策略没有后续地形实验的位置目标限速后处理。

故障会锁存并撤去出力。`clear-fault` 只有在12轴反馈有效、错误码为0时才成功；之后恢复原趴姿、重新读零与起立。详细操作见 [COMMANDS.md](COMMANDS.md)。

如果Clash的WebKit界面占用大量CPU，可在用户同意限制该界面的电脑上执行 `python tools/limit_clash_ui_cpu.py`。公开版启动器默认不修改其他进程；设置 `RCDOG_LIMIT_CLASH_UI=1` 后每次 `serve` 才会应用。此措施不能保证实时调度。

## 6. 离线检查与通信监视

固定输入与ONNX输出核对，在仓库根目录运行：

```bash
python real_deploy/verify_bundle.py \
  --bundle real_deploy/policies/newurdf_command150_final500
```

控制器的离线测试不打开电机端口，在 `real_deploy/controller/` 下执行：

```bash
PYTHONPATH=.:tools python -m unittest discover -s tools -p 'test_*.py'
```

监视上位机只读取已运行后台的数据，不打开第二个电机串口。安装可选依赖后启动：

```bash
python -m pip install PyQt5
python tools/feedback_monitor.py
```

窗口显示12轴的反馈年龄、轮询间隔、收发耗时、窗口失败率、有效回传频率及最近失败原因，可保存CSV。这里的SDK失败率不等于已经确认的线缆丢包率。

## 文件对应关系

`policies/newurdf_command150_final500/` 包含当前actor与配套的参考生成器、表格、固定输入和导出记录。`controller/` 是从实机程序整理的可独立部署副本，目录和端口配置做了通用化；本次仓库整理只做离线测试，没有操作实机电机。

包内导出记录中的 `hardware_ready: false` 保留了导出时的状态，不代表没有后续实机视频，也不会作为自动完成新机器标定的凭据。实机视频、仿真报告和配置示例分别保存，视频没有同步速度遥测。
