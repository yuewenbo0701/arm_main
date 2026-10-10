# 树莓派 5 控制机械臂 + Pixhawk — 接线与部署说明

## 一、硬件连接（机械臂：USB 转 TTL 接 0/1 脚）

```
树莓派 5 (Ubuntu 24.04, USB-A 口)
        │
   USB 转 TTL 模块 (CH340/CP2102/FTDI)
        │ TXD ──────────────> Arduino 引脚 0 (RX)
        │ RXD <────────────── Arduino 引脚 1 (TX)
        │ GND ──────────────> Arduino GND
```

- Arduino 的 0/1 脚就是固件里的协议口 `Serial`，**固件无需任何改动**。
- 只接 3 根线（TXD/RXD/GND），**不要接模块的 5V 到 Arduino**——Arduino 沿用
  原来的独立供电（舵机/电调也必须独立供电，绝不能从该模块或树莓派取电）。
- **电平**：模块必须是 **5V 电平**（有跳线帽的选 5V 档）。3.3V 电平的模块接
  Mega 的 5V TX 有烧坏风险。
- 注意 TX/RX 是**交叉**连接：模块 TXD → 0(RX)，模块 RXD ← 1(TX)。
- **烧录固件时**：先拔掉 0/1 脚上的接线（或用 Arduino 自带 USB 口烧录后再接线），
  因为 0/1 脚与板载 USB 口共用 UART0，同时接会抢线导致上传失败。
- 此接法 DTR 未连接，打开串口**不会**复位 Arduino，反而更省心。
- 设备名为 `/dev/ttyUSB0`（CH340/CP210x）或 `/dev/ttyACM0`（部分 FTDI）。

### 备选：GPIO 串口直连（不占 USB 口时再用）

```
树莓派 GPIO14 (TXD) ───────────────> Arduino RX1 (引脚19)
树莓派 GPIO15 (RXD) ──[1kΩ]──┬─────> Arduino TX1 (引脚18)
                             [2kΩ]
                              │
树莓派 GND ───────────────────┴───> Arduino GND
```

- Arduino TX1 输出 5V，树莓派 RX 只耐 3.3V，**必须加 1kΩ+2kΩ 分压**；
  树莓派 TX 3.3V 可直接接 Arduino RX1。
- 此方式下固件无需改串口（协议口改回 `Serial1` 即可），树莓派侧需
  `sudo raspi-config` → Interface Options → Serial Port → 控制台选 No、硬件串口选 Yes，
  串口设备为 `/dev/serial0`。Python 客户端写法不变。

## 二、固件改动点（已完成）

| 改动 | 说明 |
|---|---|
| 协议口 Serial1 → Serial | 树莓派经 USB 口收发二进制帧，协议格式不变 |
| 删除正/逆运动学 | `getPosition()`、`solve()`、连杆参数、`Point3D/JointAngles`、`CMD_MOVE_POSE` 全部移除 |
| 删除串口交互菜单 | 原 `t/s/w/f/i` 菜单与协议同口冲突，由树莓派完全接管 |
| `MOVE_JOINT` 改 4 角直控 | 4 个关节角全由树莓派下发；兼容旧 3 角格式（自动 θ4=-(θ2+θ3)） |
| 新增 `CMD_STATUS_REQ` (0x07) | 树莓派可随时主动请求一次状态（平时每 200ms 自动上报） |
| 新增角度超限拒绝 | `MOVE_JOINT` 超限时回 `RSP_ERROR / ERR_ANGLE_OUT_RANGE`，不再静默截断 |
| 调试输出移到 Serial1 | 引脚 18/19 接 USB-TTL 模块可看日志；`DEBUG_ENABLED 0` 可关闭 |
| 启用 VL53L1X | `setup()` 中初始化已启用；I2C：Mega2560=20(SDA)/21(SCL)，Uno=A4/A5 |
| 状态帧扩展 18→22 字节 | 末尾追加 `fsr1_raw`/`fsr2_raw`（uint16 ADC 原始值 0~1023），旧字段位置不变，后向兼容 |

保持不变：平滑移动、风扇渐变控制、FSR 中值滤波、自动工作序列、
`arm_config.h` 标定数据、`arm_protocol.cpp` 编解码逻辑。

烧录：用 Arduino IDE 打开 `arm_main.ino`（`arm_config.h / arm_protocol.h /
arm_protocol.cpp` 保持在同目录），板型选 **Arduino Mega 2560**，上传即可。
（若实际为 Uno R3：无 Serial1，须把 `DEBUG_ENABLED` 改为 `0`；I2C 接 A4/A5。）

## 三、树莓派端部署（Ubuntu 24.04）

```bash
# 1. 安装依赖 (Ubuntu 24.04 系统 Python 有 PEP 668 限制, 建议 venv)
sudo apt update
sudo apt install -y python3-venv
python3 -m venv ~/venv8fly
source ~/venv8fly/bin/activate
pip install pyserial pymavlink
# 只用机械臂、不碰飞控时也可: sudo apt install -y python3-serial

# 2. 把本目录的 rpi/ 拷到树莓派
scp -r rpi/ pi@<树莓派IP>:~/

# 3. 插上两个 USB-TTL 模块 (机械臂 + 飞控), 分清设备号
ls /dev/serial/by-id/          # 按适配器型号/序列号区分, 最可靠
dmesg | tail                   # 看插拔记录确认芯片型号

# 4. 当前用户加入串口组（免 sudo，重新登录生效）
sudo usermod -aG dialout $USER

# 5. 跑演示（归零→动臂→风扇→归零）
cd ~/rpi
python3 arm_demo.py                       # 自动找口
python3 arm_demo.py /dev/ttyUSB0          # 或手动指定
```

> **Ubuntu 24.04 特有坑**：系统自带的 `brltty`（盲文显示器服务）会自动抢占
> CH340/CP210x 串口设备。如果 `/dev/ttyUSB0` 插上后一闪而过或打不开，执行：
> `sudo apt remove -y brltty`（确定不需要盲文设备的话）。

## 四、Python API 速查（arm_client）

```python
from arm_client import ArmClient, find_port

with ArmClient(find_port()) as arm:
    arm.home()                       # 归零
    arm.wait_done(30)                # 等移动完成 (True=完成, False=超时)

    arm.move_joint(1, [0, 20, 20, -40])   # 臂1 四关节角(度)
    arm.move_joint(2, [0, 20, 20])        # 3角格式: 固件自动 θ4=-(θ2+θ3)

    arm.fan_on(1)                    # 臂1 风扇开 (默认 1200us)
    arm.set_fan(2, 1500)             # 臂2 风扇油门
    arm.fan_off(1)                   # 关

    arm.auto_start()                 # 启动 Arduino 内置自动序列
    arm.stop()                       # 急停

    s = arm.status                   # 最近一帧状态 (每200ms自动更新)
    # s.moving, s.fan1, s.arm1_angles, s.laser_mm, s.fsr1, s.fsr1_raw ...
    # s.fsr1_raw/s.fsr2_raw: FSR ADC 原始值 0~1023 (旧 18 字节固件为 None)
    arm.status_time                  # 最近状态帧接收时刻 (判断数据是否过期)
    arm.request_status()             # 立即再要一帧状态
```

## 五、协议速查（与原版一致）

帧格式：`[0xAA][0x55][CMD][LEN][PAYLOAD...][XOR]`，XOR = CMD^LEN^PAYLOAD 逐字节异或。
float 均为 **float32 小端**（Arduino AVR 与 Python `struct '<f'` 一致）。

| 方向 | CMD | 含义 | 负载 |
|---|---|---|---|
| 下行 | 0x01 | 归零 | 无（完成后回 0x82 DONE） |
| 下行 | 0x02 | 关节控制 | `[arm:1B][angles: 3~4×float]` |
| 下行 | 0x04 | 风扇 | `[arm:1B][throttle:uint16]` |
| 下行 | 0x05 | 自动序列 | 无（回 0x80 ACK） |
| 下行 | 0x06 | 急停 | 无（回 0x80 ACK） |
| 下行 | 0x07 | 请求状态 | 无（回 0x81 STATUS） |
| 上行 | 0x80 | ACK | 无 |
| 上行 | 0x81 | 状态（每200ms） | 22B（旧固件 18B），布局见下 |
| 上行 | 0x82 | 移动完成 | 无 |
| 上行 | 0x83 | 错误 | `[err:1B]`，0x01=角度超限 |

状态帧（0x81）负载布局，解析见 `ArmStatus.from_payload`：

```
[0]      moving:  u8
[1]      auto:    u8
[2:4]    fan1:    u16 油门 us
[4:6]    fan2:    u16
[6]      fsr1:    u8 触发标志
[7]      fsr2:    u8
[8:10]   laser:   u16 mm (VL53L1X 未就绪时恒 0)
[10:14]  arm1角度: 4×i8
[14:18]  arm2角度: 4×i8
[18:20]  fsr1_raw: u16 ADC 原始值 0~1023   ← 新固件追加
[20:22]  fsr2_raw: u16                     ← 新固件追加
```

## 六、Pixhawk 飞控连接（fc_client.py）

**已确认：PX4 固件 + TELEM2 + 921600。**

```
树莓派 5 (USB-A 口)
        │
   USB 转 TTL 模块 (必须 3.3V 电平!)
        │ TXD ──────────────> Pixhawk TELEM2 RX
        │ RXD <────────────── Pixhawk TELEM2 TX
        │ GND ──────────────> Pixhawk GND
```

- Pixhawk TELEM 口是 **3.3V 电平**，模块必须选 3.3V 档（与机械臂的 5V 模块不同，
  两个模块不要混用/接错）。
- PX4 参数确认：`MAV_1_CONFIG = TELEM2`，该链路波特率 921600；
  `COM_OBL_ACT`（OFFBOARD 丢失保护动作）按飞手预案设置；
  遥控器模式切换开关可用（中止 = 飞手随时切走 OFFBOARD）。
- 两个 USB-TTL 同时在线时，用 `ls /dev/serial/by-id/` 分清哪个是机械臂、
  哪个是飞控，再用 `--arm` / `--fc` 指定。

```python
from fc_client import FCClient

with FCClient('/dev/ttyUSB1', 921600) as fc:
    fc.takeover_hold()               # 切 OFFBOARD, 定点保持当前位置 (True=成功)
    fc.set_velocity_body(0.2)        # 机体系速度: vx 前向为正 (m/s, 限幅 ±2.0)
    fc.hold_current_position()       # 回到定点
    fc.release()                     # 停设定点 (PX4 将触发 OFFBOARD 丢失保护)
    fc.land(); fc.disarm()           # 备用
```

## 七、贴墙吸附任务（mission.py）

```bash
cd ~/rpi
python3 mission.py --arm /dev/ttyUSB0 --fc /dev/ttyUSB1
```

流程：飞手手动起飞到墙面前 → 回车 → OFFBOARD 定点 → 激光 P 控制接近至
300mm → 臂 1 伸臂（预备位→下压位）→ 开涵道风扇 → FSR 饱和判定 → 吸附保持。

中止（任意时刻）：飞手切模式 / 按 `q`+回车 / Ctrl+C
→ 自动停飞控设定点 + 机械臂急停。

**所有可调参数集中在 `mission.py` 文件头**：目标距离 300mm、P 增益
`APPROACH_KP`、速度上限 `APPROACH_VX_MAX`、伸臂角度 `POSE_PREPARE/POSE_PRESS`、
FSR 饱和判定（`FSR_SAT_FRAMES`/`FSR_SAT_MIN_RAW`/`FSR_ADC_THRESHOLD`）、各阶段超时。
首次试飞建议把 `APPROACH_VX_MAX` 调小（如 0.15）。

单元测试（无需硬件，开发机上跑）：

```bash
cd rpi && python3 -m unittest discover -v
```

## 八、真机前 checklist

1. **固件**：已烧录最新 `arm_main.ino`（VL53L1X 启用、22 字节状态帧）；
   烧录时已拔掉 D0/D1 接线；板型确认 Mega 2560（Uno 需 `DEBUG_ENABLED 0`）。
2. **接线**：机械臂模块 5V 电平、飞控模块 3.3V 电平，TX/RX 均交叉；
   Arduino/舵机/电调独立供电，与树莓派仅共地。
3. **串口**：`ls /dev/serial/by-id/` 分清两个 ttyUSB；`brltty` 已移除；
   `arm_demo.py` 通信演示通过。
4. **传感器**：地面静止验证 `arm.status.laser_mm` 随遮挡变化、`fsr1_raw`
   随按压变化（`python3 arm_demo.py` 或自行打印 `arm.status`）。
5. **飞控**：PX4 参数（MAV_1_CONFIG/波特率/COM_OBL_ACT）已确认；遥控器
   模式开关正常；`fc_client` 能读到心跳与 `LOCAL_POSITION_NED`
   （`fc.mode_name`、`fc.get_position()` 非 None）。
6. **任务参数**：伸臂角度 `POSE_PREPARE/POSE_PRESS` 已地面静态验证；
   `FSR_SAT_MIN_RAW`/`FSR_ADC_THRESHOLD` 按实测标定；`APPROACH_VX_MAX` 保守起步。
7. **预案**：飞手明确「切模式=中止接管」；首次自动接近时手不离杆，
   确认吸附后再考虑收尾降落流程。

## 九、常见问题

- **打开串口后没反应**：先确认设备号（`ls /dev/ttyUSB*`，`dmesg | tail` 看插拔
  记录）；再检查 TX/RX 是否交叉接对、模块电平档位是否正确；最后排查 `brltty`
  抢占（见第三节）。USB-TTL 接法不会复位 Arduino，无需等待。
- **权限拒绝 (Permission denied)**：执行第 4 步加入 `dialout` 组并重新登录。
- **烧录固件失败/打不开端口**：烧录时先拔掉 0/1 脚上的接线，与板载 USB 口会抢线。
- **想关调试口**：把 `arm_main.ino` 顶部 `DEBUG_ENABLED` 改为 `0`。
- **θ4 耦合**：旧固件强制 θ4=-(θ2+θ3)（保持末端垂直）。新固件 4 角直控，
  需要该约束时在树莓派侧算好再下发，或直接发 3 个角（固件自动补 θ4）。
- **飞控等不到心跳**：确认接的是 TELEM2 而非 TELEM1、波特率 921600、
  模块为 3.3V 电平、PX4 的 `MAV_1_CONFIG` 已指向 TELEM2。
- **切 OFFBOARD 被拒**：PX4 要求切换前已有 >2Hz 设定点流（`takeover_hold`
  已自动处理）；仍失败则检查 `LOCAL_POSITION_NED` 是否有效、RC 是否已校准。
- **激光读数恒 0**：VL53L1X 未就绪（I2C 接线/板型引脚），状态帧 laser 字段
  保持 0，`mission.py` 会按「激光无效」立即中止——这是预期保护行为。
