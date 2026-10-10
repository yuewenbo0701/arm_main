#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
8fly · Pixhawk 飞控客户端 (PX4 固件, pymavlink 封装)
=====================================================
树莓派经 MAVLink 串口接管 PX4:
  - 切 OFFBOARD 并持续发位置设定点, 实现定点保持
  - 机体坐标系速度设定点 (vx 前向为正), 用于闭环接近墙面
  - 定高解除 (release_altitude_hold): 只放开 Z 轴为固定推力,
    X/Y 仍定点 — 树莓派外环 PD (位置误差->期望倾角) + PX4 姿态内环,
    供爬墙步态中双臂同步动作期间使用
  - 解锁/上锁 (arm/disarm), 支持全自动起飞-降落流程
  - 监视心跳模式, 飞手切走 OFFBOARD 时回调通知 mission 中止

定高解除原理:
  PX4 的 alt_valid 是 EKF2 内部状态, MAVLink 无法外部写入, 失效会触发
  failsafe, 不能当作"定高开关"。正确做法是 OFFBOARD 下改发
  SET_ATTITUDE_TARGET (期望倾角 + 固定推力 MPC_THR_HOVER), PX4 位置/速度
  控制器被旁路 = Z 轴定高关; 同时树莓派以 10~20Hz 外环把 X/Y 位置误差
  转成期望倾角叠加进姿态设定点, 保住 X/Y 定点。结束后换回
  SET_POSITION_TARGET_LOCAL_NED = 定高开。全程设定点流不断、不切飞行
  模式, 无 failsafe。

接线: Pixhawk TELEM2 --USB-TTL(3.3V 电平)--> 树莓派 USB 口
      TELEM2 TX -> 模块 RXD, TELEM2 RX -> 模块 TXD, GND 共地
      (已与用户确认: PX4 固件 + TELEM2 + 921600)

TODO(上机前请确认):
  - DEFAULT_PORT 实际设备号 (机械臂与飞控各一个 USB-TTL, 注意区分, 见 README)
  - PX4 参数: MAV_1_CONFIG=TELEM2, MAV_1_RATE 匹配 921600,
    COM_OBL_ACT (OFFBOARD 丢失保护动作) 按飞手预案设置
  - 若飞控实为 ArduPilot: 无 OFFBOARD, 须改 _set_offboard_mode() 为
    GUIDED(custom_mode=4) 并改用 MAV_FRAME_BODY_OFFSET_NED 之外的确认流程

依赖: pip install pymavlink   (Ubuntu 24.04 建议 venv 安装, 见 README)

示例:
    from fc_client import FCClient

    with FCClient('/dev/ttyUSB1') as fc:
        if fc.takeover_hold():              # 切 OFFBOARD, 定点保持当前位置
            fc.arm()                        # 解锁 (需设定点流已建立)
            fc.set_position_local(x, y, z - 0.2)   # 起飞到离地 20cm
            fc.release_altitude_hold()      # 定高关: Z 固定推力, X/Y 保持
            fc.resume_altitude_hold()       # 定高开: 恢复全轴定点
            fc.disarm()                     # 上锁 (仅可在落地后!)
        fc.release()                        # 停止设定点 (飞手接管)
"""

import math
import threading
import time

from pymavlink import mavutil

# ------------------------------------------------------------
# 连接参数 (TODO: 按实际接线确认设备号)
# ------------------------------------------------------------
DEFAULT_PORT = '/dev/ttyUSB0'   # Pixhawk 经 USB-TTL(3.3V) 接树莓派 (TELEM1 或 TELEM2)
DEFAULT_BAUD = 921600           # 已确认: PX4 + TELEM2 + 921600

# ------------------------------------------------------------
# 控制参数
# ------------------------------------------------------------
SETPOINT_RATE_HZ = 10           # 设定点发送频率 (PX4 OFFBOARD 要求 >2Hz)
MAX_SETPOINT_VEL = 2.0          # 速度设定点硬限幅 (m/s), mission 内部另有更严限制
DEFAULT_HOVER_THRUST = 0.5      # 悬停推力兜底值; 正常会被 MPC_THR_HOVER 覆盖
HOVER_THRUST_PARAM = 'MPC_THR_HOVER'  # PX4 悬停推力参数名 (在线学习更新)

# 定高解除时 X/Y 保持的外环 PD 参数 (树莓派侧)
XY_HOLD_KP = 0.8                # 位置 P 增益 (m/s^2 per m)
XY_HOLD_KD = 0.6                # 速度 D 增益 (m/s^2 per m/s)
XY_HOLD_MAX_ACC = 1.0           # 最大纠正加速度 (m/s^2), 约 5.8° 倾角
GRAVITY = 9.80665

# ------------------------------------------------------------
# PX4 自定义模式编码 (custom_mode: bit16-23=主模式, bit24-31=子模式)
# ------------------------------------------------------------
PX4_MAIN_MODE_OFFBOARD = 6
PX4_MAIN_MODES = {
    1: 'MANUAL', 2: 'ALTCTL', 3: 'POSCTL', 4: 'AUTO', 5: 'ACRO',
    6: 'OFFBOARD', 7: 'STABILIZED', 8: 'RATTITUDE', 9: 'SIMPLE',
}
PX4_AUTO_SUBMODES = {
    1: 'AUTO.READY', 2: 'AUTO.TAKEOFF', 3: 'AUTO.LOITER', 4: 'AUTO.MISSION',
    5: 'AUTO.RTL', 6: 'AUTO.LAND', 7: 'AUTO.RTGS', 8: 'AUTO.FOLLOW_TARGET',
    9: 'AUTO.PRECLAND',
}

# 位置/速度设定点的 type_mask (忽略不用的字段)
_m = mavutil.mavlink
TYPE_MASK_POSITION = (           # 仅用 x/y/z 位置
    _m.POSITION_TARGET_TYPEMASK_VX_IGNORE | _m.POSITION_TARGET_TYPEMASK_VY_IGNORE |
    _m.POSITION_TARGET_TYPEMASK_VZ_IGNORE |
    _m.POSITION_TARGET_TYPEMASK_AX_IGNORE | _m.POSITION_TARGET_TYPEMASK_AY_IGNORE |
    _m.POSITION_TARGET_TYPEMASK_AZ_IGNORE |
    _m.POSITION_TARGET_TYPEMASK_FORCE_SET |
    _m.POSITION_TARGET_TYPEMASK_YAW_IGNORE |
    _m.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE)
TYPE_MASK_VELOCITY = (           # 仅用 vx/vy/vz 速度
    _m.POSITION_TARGET_TYPEMASK_X_IGNORE | _m.POSITION_TARGET_TYPEMASK_Y_IGNORE |
    _m.POSITION_TARGET_TYPEMASK_Z_IGNORE |
    _m.POSITION_TARGET_TYPEMASK_AX_IGNORE | _m.POSITION_TARGET_TYPEMASK_AY_IGNORE |
    _m.POSITION_TARGET_TYPEMASK_AZ_IGNORE |
    _m.POSITION_TARGET_TYPEMASK_FORCE_SET |
    _m.POSITION_TARGET_TYPEMASK_YAW_IGNORE |
    _m.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE)

# SET_ATTITUDE_TARGET 的 type_mask:
# bit0/1/2 = 忽略 body roll/pitch/yaw rate -> 使用姿态四元数 + 推力
TYPE_MASK_ATTITUDE = 0b00000111


def px4_mode_name(custom_mode):
    """PX4 custom_mode -> 可读模式名 (如 'MANUAL' / 'AUTO.LOITER' / 'OFFBOARD')"""
    main = (custom_mode >> 16) & 0xFF
    sub = (custom_mode >> 24) & 0xFF
    if main == 4:                            # AUTO 带子模式
        return PX4_AUTO_SUBMODES.get(sub, 'AUTO.%d' % sub)
    return PX4_MAIN_MODES.get(main, 'MAIN_%d' % main)


def _yaw_from_quat(q):
    """四元数 (w,x,y,z) -> NED 航向角 yaw (rad)"""
    w, x, y, z = q
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def _quat_from_acc_yaw(ax, ay, yaw):
    """
    由期望水平加速度 (NED, m/s^2) 和航向角构造姿态四元数 (w,x,y,z)。
    期望比力 f = (ax, ay, -g), 机体 z 轴(向下)对齐 -f 方向, 航向锁定 yaw。
    即"想让飞机往哪边加速就把机顶往哪边倾"的标准构造。
    """
    fx, fy, fz = ax, ay, -GRAVITY
    n = math.sqrt(fx * fx + fy * fy + fz * fz)
    zx, zy, zz = -fx / n, -fy / n, -fz / n     # 机体 z 轴 (世界系表达)
    cx, cy = math.cos(yaw), math.sin(yaw)      # 期望机头方向
    # y_b = normalize(z_b x c)
    yx, yy, yz = -zz * cy, zz * cx, zx * cy - zy * cx
    m = math.sqrt(yx * yx + yy * yy + yz * yz)
    yx, yy, yz = yx / m, yy / m, yz / m
    # x_b = y_b x z_b
    xx = yy * zz - yz * zy
    xy = yz * zx - yx * zz
    xz = yx * zy - yy * zx
    # 旋转矩阵 (列为 x_b, y_b, z_b) -> 四元数 (标准 trace 法)
    r00, r01, r02 = xx, yx, zx
    r10, r11, r12 = xy, yy, zy
    r20, r21, r22 = xz, yz, zz
    tr = r00 + r11 + r22
    if tr > 0:
        s = math.sqrt(tr + 1.0) * 2.0
        w, x, y, z = 0.25 * s, (r21 - r12) / s, (r02 - r20) / s, (r10 - r01) / s
    elif r00 > r11 and r00 > r22:
        s = math.sqrt(1.0 + r00 - r11 - r22) * 2.0
        w, x, y, z = (r21 - r12) / s, 0.25 * s, (r01 + r10) / s, (r02 + r20) / s
    elif r11 > r22:
        s = math.sqrt(1.0 + r11 - r00 - r22) * 2.0
        w, x, y, z = (r02 - r20) / s, (r01 + r10) / s, 0.25 * s, (r12 + r21) / s
    else:
        s = math.sqrt(1.0 + r22 - r00 - r11) * 2.0
        w, x, y, z = (r10 - r01) / s, (r02 + r20) / s, (r12 + r21) / s, 0.25 * s
    return (w, x, y, z)


class FCClient:
    """Pixhawk(PX4) MAVLink 客户端 (后台接收线程 + 设定点发送线程)"""

    def __init__(self, port=DEFAULT_PORT, baud=DEFAULT_BAUD,
                 on_mode_lost=None, on_mode_change=None, heartbeat_timeout=10,
                 setpoint_rate=SETPOINT_RATE_HZ):
        """
        port: 串口设备 (TELEM2 经 USB-TTL, 通常 /dev/ttyUSBx)
        on_mode_lost: 回调(old, new) — 接管后飞手把模式切离 OFFBOARD 时触发
        on_mode_change: 回调(old, new) — 任意模式变化 (仅提示用)
        setpoint_rate: 设定点发送频率 Hz (默认 10; X/Y 保持外环建议 20)
        """
        self.master = mavutil.mavlink_connection(port, baud=baud)

        self.position = None              # 最新 LOCAL_POSITION_NED (x,y,z) 米, NED
        self.velocity = None              # 最新 LOCAL_POSITION_NED (vx,vy,vz) m/s
        self.position_time = 0.0          # 位置接收时刻
        self.attitude_quat = None         # 最新 ATTITUDE_QUATERNION (w,x,y,z)
        self.attitude_time = 0.0          # 姿态接收时刻
        self.last_rx_time = 0.0           # 任意 MAVLink 消息接收时刻 (接收线程活性)
        self.last_heartbeat_time = 0.0    # 飞控心跳接收时刻 (链路活性判据)
        self.hover_thrust = DEFAULT_HOVER_THRUST  # 悬停推力 0~1 (定高解除时用)
        self.mode_name = None             # 最新模式名
        self.armed = False                # 最新解锁状态 (心跳 base_mode)

        self.on_mode_lost = on_mode_lost
        self.on_mode_change = on_mode_change

        self._pos_lock = threading.Lock()
        self._att_lock = threading.Lock()
        self._params_lock = threading.Lock()
        self._params = {}                 # PARAM_VALUE 缓存 {param_id: value}
        self._mode_lock = threading.Lock()
        self._write_lock = threading.Lock()   # 串口写互斥 (设定点线程/T265桥线程/命令共用)
        self._sp_lock = threading.Lock()
        self._sp_mode = 'hold'            # 'hold' 位置 | 'velocity' 速度 | 'xy_hold' 定高解除
        self._sp_data = (0.0, 0.0, 0.0)
        self._sp_rate = setpoint_rate
        self._sp_running = False
        self._sp_thread = None
        self._watch_mode = False          # 接管流程启动后置 True
        self._in_offboard = False         # 心跳确认已在 OFFBOARD

        # 等心跳 (新旧 pymavlink: 超时抛异常或返回 None, 都按失败处理)
        try:
            msg = self.master.wait_heartbeat(timeout=heartbeat_timeout)
        except Exception:
            msg = None
        if msg is None:
            self.master.close()
            raise RuntimeError('等待飞控心跳超时, 检查接线/波特率/MAV_1 参数')
        if msg.autopilot != mavutil.mavlink.MAV_AUTOPILOT_PX4:
            print('! 警告: 对端不是 PX4 固件 (autopilot=%d), 模式逻辑可能不适用'
                  % msg.autopilot)

        # 用首帧心跳填充 mode_name / armed (否则启动瞬间为 None)
        self._on_heartbeat(msg)

        self._running = True
        self._thread = threading.Thread(target=self._reader_loop, daemon=True)
        self._thread.start()

        # 请求飞控以 10Hz 上报 LOCAL_POSITION_NED (定点保持要用)
        self.master.mav.command_long_send(
            self.master.target_system, self.master.target_component,
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
            mavutil.mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED,
            100000, 0, 0, 0, 0, 0)        # 100000us = 10Hz

        # 请求飞控以 10Hz 上报 ATTITUDE_QUATERNION (定高解除时锁航向要用)
        self.master.mav.command_long_send(
            self.master.target_system, self.master.target_component,
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
            mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE_QUATERNION,
            100000, 0, 0, 0, 0, 0)

        # 读一次悬停推力参数 (非致命, 失败保留默认值)
        if self.fetch_hover_thrust(timeout=2.0) is None:
            print('! 未能读取 %s, 悬停推力暂用默认值 %.2f'
                  % (HOVER_THRUST_PARAM, self.hover_thrust))
            # 顺带自检命令通道: 收得到心跳却读不到任何参数,
            # 大概率是 树莓派->飞控 方向 (USB-TTL TXD -> TELEM2 RX) 断线
            if not self.tx_link_check(timeout=1.5):
                print('! 自检: 命令通道疑似不通 —— '
                      '重点检查 USB-TTL 的 TXD -> TELEM2 RX 连线!')

    # ---------------- 接收 ----------------
    def _reader_loop(self):
        rx_errors = 0
        while self._running:
            try:
                msg = self.master.recv_match(blocking=True, timeout=1.0)
            except Exception as e:
                # 不再静默退出线程! 老版本一次瞬时读错 (USB-TTL 在电机
                # EMI/震动下常见) 就永久停止解析: 心跳/位置/姿态全部变成
                # 无人察觉的陈旧值, 表现为"数据看似正常, 指令却全丢"。
                rx_errors += 1
                if self._running and (rx_errors <= 3 or rx_errors % 20 == 0):
                    print('\n! [fc_client] 串口接收异常 #%d: %r (继续监听)'
                          % (rx_errors, e))
                time.sleep(0.2)
                continue
            if msg is None:
                continue
            self.last_rx_time = time.time()
            mtype = msg.get_type()
            if mtype == 'HEARTBEAT':
                if msg.get_srcSystem() == self.master.target_system:
                    self.last_heartbeat_time = self.last_rx_time
                self._on_heartbeat(msg)
            elif mtype == 'LOCAL_POSITION_NED':
                with self._pos_lock:
                    self.position = (msg.x, msg.y, msg.z)
                    self.velocity = (msg.vx, msg.vy, msg.vz)
                    self.position_time = time.time()
            elif mtype == 'ATTITUDE_QUATERNION':
                with self._att_lock:
                    self.attitude_quat = (msg.q1, msg.q2, msg.q3, msg.q4)
                    self.attitude_time = time.time()
            elif mtype == 'PARAM_VALUE':
                pid = msg.param_id
                if isinstance(pid, bytes):
                    pid = pid.decode('ascii', 'ignore')
                with self._params_lock:
                    self._params[pid] = msg.param_value
            elif mtype == 'TIMESYNC':
                # PX4 时钟同步请求, 立即应答 (视觉定位数据靠它对齐时间戳,
                # 必须在唯一接收线程里处理, 不能被其他线程抢读)
                try:
                    with self._write_lock:
                        self.master.mav.timesync_send(int(time.time_ns()), msg.ts1)
                except Exception:
                    pass

    def _on_heartbeat(self, msg):
        if msg.get_srcSystem() != self.master.target_system:
            return                          # 忽略链路上其他系统 (如地面站)
        if msg.autopilot != mavutil.mavlink.MAV_AUTOPILOT_PX4:
            return                          # 非 PX4 的模式编码不解析
        self.armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
        name = px4_mode_name(msg.custom_mode)
        main = (msg.custom_mode >> 16) & 0xFF

        lost_cb = change_cb = None
        with self._mode_lock:
            old = self.mode_name
            if name == old:
                return
            self.mode_name = name
            if self._watch_mode and main == PX4_MAIN_MODE_OFFBOARD:
                self._in_offboard = True
            if self._in_offboard and main != PX4_MAIN_MODE_OFFBOARD:
                # 接管期间被飞手切走 -> 通知 mission 中止
                self._in_offboard = False
                lost_cb = self.on_mode_lost
            else:
                change_cb = self.on_mode_change
        if lost_cb:
            lost_cb(old, name)
        elif change_cb:
            change_cb(old, name)

    # ---------------- 状态 ----------------
    def wait_position(self, timeout=3.0):
        """等待收到 LOCAL_POSITION_NED。返回 True=已有有效位置"""
        deadline = time.time() + timeout
        while time.time() < deadline:
            with self._pos_lock:
                if self.position is not None:
                    return True
            time.sleep(0.05)
        return False

    def get_position(self):
        """最新位置 (x,y,z) NED 米; 未收到返回 None"""
        with self._pos_lock:
            return self.position

    def get_velocity(self):
        """最新速度 (vx,vy,vz) NED m/s; 未收到返回 None"""
        with self._pos_lock:
            return self.velocity

    def wait_attitude(self, timeout=3.0):
        """等待收到 ATTITUDE_QUATERNION。返回 True=已有有效姿态"""
        deadline = time.time() + timeout
        while time.time() < deadline:
            with self._att_lock:
                if self.attitude_quat is not None:
                    return True
            time.sleep(0.05)
        return False

    def get_attitude_quat(self):
        """最新姿态四元数 (w,x,y,z); 未收到返回 None"""
        with self._att_lock:
            return self.attitude_quat

    # ---------------- 链路活性 / 数据新鲜度 ----------------
    def link_alive(self, max_age=3.0):
        """飞控心跳仍在刷新 (心跳 1Hz, 阈值 3s)。
        链路真正活性的判据: 心跳停更 = 接收线程异常或链路中断,
        此时 position/attitude 缓存都是不可信的旧值。"""
        return (self.last_heartbeat_time > 0 and
                time.time() - self.last_heartbeat_time <= max_age)

    def position_fresh(self, max_age=2.0):
        """位置不仅收到过, 且仍在刷新 (防陈旧缓存误判)"""
        return (self.position_time > 0 and
                time.time() - self.position_time <= max_age)

    def attitude_fresh(self, max_age=2.0):
        """姿态不仅收到过, 且仍在刷新"""
        return (self.attitude_time > 0 and
                time.time() - self.attitude_time <= max_age)

    def fetch_hover_thrust(self, timeout=3.0):
        """
        读取 PX4 参数 MPC_THR_HOVER (悬停推力 0~1)。
        成功则更新 self.hover_thrust 并返回该值; 超时返回 None。
        """
        pid = HOVER_THRUST_PARAM
        with self._params_lock:
            self._params.pop(pid, None)
        try:
            with self._write_lock:
                self.master.mav.param_request_read_send(
                    self.master.target_system, self.master.target_component,
                    pid.encode('ascii'), -1)    # -1 = 按名称读取
        except Exception:
            return None
        deadline = time.time() + timeout
        while time.time() < deadline:
            with self._params_lock:
                val = self._params.get(pid)
            if val is not None:
                self.hover_thrust = val
                return val
            time.sleep(0.05)
        return None

    def tx_link_check(self, timeout=2.0, retries=2):
        """
        自检 树莓派->飞控 方向的命令通道:
        请求必然存在的参数 SYS_AUTOSTART, 收到 PARAM_VALUE 即说明通道通。
        典型故障画像: 心跳/姿态收得到, 但参数读不到、模式切不动
        -> 本检查失败, 基本是 USB-TTL TXD -> TELEM2 RX 这根线没接好。
        返回 True=命令通道正常。
        """
        if self.last_heartbeat_time > 0 and not self.link_alive():
            print('! 自检: 飞控心跳已停止刷新 —— 下行链路中断, 参数请求不会有回应')
            print('  → 典型原因: USB-TTL 接触不良/电机 EMI 致串口瞬时掉线')
            return False
        pid = 'SYS_AUTOSTART'
        for _ in range(retries):
            with self._params_lock:
                self._params.pop(pid, None)
            try:
                with self._write_lock:
                    self.master.mav.param_request_read_send(
                        self.master.target_system, self.master.target_component,
                        pid.encode('ascii'), -1)
            except Exception:
                return False
            deadline = time.time() + timeout
            while time.time() < deadline:
                with self._params_lock:
                    if pid in self._params:
                        return True
                time.sleep(0.05)
        return False

    # ---------------- 视觉定位转发 (T265 桥用) ----------------
    def send_vision_position_estimate(self, x, y, z, roll, pitch, yaw, timestamp_us=None):
        """
        发送 VISION_POSITION_ESTIMATE (供 T265 桥线程调用, 与设定点线程互斥写串口)。
        协方差置 NaN -> PX4 使用 EKF2_EVP_NOISE / EKF2_EVA_NOISE 默认噪声。
        时间戳靠 TIMESYNC 对齐 (_reader_loop 已自动应答)。
        """
        if timestamp_us is None:
            timestamp_us = int(time.time() * 1e6)
        cov = [float('nan')] * 21
        with self._write_lock:
            self.master.mav.vision_position_estimate_send(
                int(timestamp_us),
                float(x), float(y), float(z),
                float(roll), float(pitch), float(yaw), cov)
    def _set_offboard_mode(self):
        # PX4: base_mode=CUSTOM_MODE_ENABLED, custom_mode = 主模式<<16 (子模式 0)
        # ArduPilot 须改为 GUIDED: custom_mode=4 直接进 param2, 且无设定点流前置要求
        self.master.mav.command_long_send(
            self.master.target_system, self.master.target_component,
            mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            PX4_MAIN_MODE_OFFBOARD << 16, 0, 0, 0, 0, 0)

    def _start_setpoint_thread(self):
        if self._sp_running:
            return
        self._sp_running = True
        self._sp_thread = threading.Thread(target=self._setpoint_loop, daemon=True)
        self._sp_thread.start()

    def _setpoint_loop(self):
        period = 1.0 / self._sp_rate
        last_hb = 0.0
        last_xy_stale_warn = 0.0
        while self._sp_running:
            with self._sp_lock:
                mode, data = self._sp_mode, self._sp_data
            try:
                with self._write_lock:
                    if mode == 'velocity':
                        vx, vy, vz = data
                        self.master.mav.set_position_target_local_ned_send(
                            0, self.master.target_system, self.master.target_component,
                            mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
                            TYPE_MASK_VELOCITY,
                            0, 0, 0, vx, vy, vz, 0, 0, 0, 0, 0)
                    elif mode == 'xy_hold':
                        # 定高解除 (XY 保持): Z 轴固定推力放开,
                        # X/Y 由外环 PD 把位置误差转成期望倾角, PX4 姿态环执行
                        xt, yt, yaw, thrust = data
                        pos = self.get_position()
                        vel = self.get_velocity()
                        if pos is not None and vel is not None and self.position_fresh():
                            ax = XY_HOLD_KP * (xt - pos[0]) - XY_HOLD_KD * vel[0]
                            ay = XY_HOLD_KP * (yt - pos[1]) - XY_HOLD_KD * vel[1]
                            ax = max(-XY_HOLD_MAX_ACC, min(XY_HOLD_MAX_ACC, ax))
                            ay = max(-XY_HOLD_MAX_ACC, min(XY_HOLD_MAX_ACC, ay))
                        else:
                            # 位置停滞/丢失: 禁止拿陈旧数据算纠正 (会朝错误
                            # 方向倾), 降级为 水平姿态+固定推力 等链路恢复
                            ax = ay = 0.0
                            if time.time() - last_xy_stale_warn >= 2.0:
                                print('\n! [fc_client] 定高关期间位置失效, '
                                      'X/Y 保持降级为水平姿态 —— 检查链路!')
                                last_xy_stale_warn = time.time()
                        q = _quat_from_acc_yaw(ax, ay, yaw)
                        self.master.mav.set_attitude_target_send(
                            0, self.master.target_system, self.master.target_component,
                            TYPE_MASK_ATTITUDE,
                            [q[0], q[1], q[2], q[3]],
                            0.0, 0.0, 0.0, thrust)
                    else:
                        x, y, z = data
                        self.master.mav.set_position_target_local_ned_send(
                            0, self.master.target_system, self.master.target_component,
                            mavutil.mavlink.MAV_FRAME_LOCAL_NED,
                            TYPE_MASK_POSITION,
                            x, y, z, 0, 0, 0, 0, 0, 0, 0, 0)
                    if time.time() - last_hb >= 1.0:
                        # 伴随机心跳 (PX4 OFFBOARD 需要看到外部设备存活)
                        self.master.mav.heartbeat_send(
                            mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
                            mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                            0, 0, mavutil.mavlink.MAV_STATE_ACTIVE)
                        last_hb = time.time()
            except Exception:
                pass                        # 串口瞬断, 下一周期重试
            time.sleep(period)

    def takeover_hold(self, timeout=6.0):
        """
        接管: 切 OFFBOARD 并定点保持在当前位置 (地面或空中均可)。
        PX4 要求切换前已有 >2Hz 设定点流, 否则拒绝切换。
        返回 True=接管成功 (已在 OFFBOARD 定点)。
        """
        if not self.wait_position(3.0):
            print('! 未收到 LOCAL_POSITION_NED, 无法定点')
            return False
        if not self.position_fresh():
            print('! LOCAL_POSITION_NED 已停止刷新 (链路中断/接收线程异常), 拒绝接管')
            return False
        self._watch_mode = True
        self.hold_current_position()
        self._start_setpoint_thread()
        time.sleep(0.6)                     # 先建立设定点流

        deadline = time.time() + timeout
        while time.time() < deadline:
            self._set_offboard_mode()
            t0 = time.time()
            while time.time() - t0 < 1.2:
                if self.mode_name == 'OFFBOARD':
                    return True             # 心跳已确认
                time.sleep(0.05)
        print('! 切 OFFBOARD 失败 (检查设定点流/RC 标定/COM_RC_IN_MODE)')
        self.release()
        return False

    def set_velocity_body(self, vx, vy=0.0, vz=0.0):
        """机体坐标系速度设定点 (m/s): vx 前向为正, vy 右向, vz 下向 (NED)"""
        lim = MAX_SETPOINT_VEL
        vx = max(-lim, min(lim, vx))
        vy = max(-lim, min(lim, vy))
        vz = max(-lim, min(lim, vz))
        with self._sp_lock:
            self._sp_mode = 'velocity'
            self._sp_data = (vx, vy, vz)

    def hold_current_position(self):
        """由速度/定高解除切回定点, 保持当前位置。返回 False=尚无位置"""
        pos = self.get_position()
        if pos is None:
            return False
        with self._sp_lock:
            self._sp_mode = 'hold'
            self._sp_data = pos
        return True

    def set_position_local(self, x, y, z):
        """定点到指定 NED 位置 (x,y,z 米)。用于起飞/降落/原地升降"""
        with self._sp_lock:
            self._sp_mode = 'hold'
            self._sp_data = (x, y, z)

    def release_altitude_hold(self, thrust=None):
        """
        定高解除 (X/Y 保持): OFFBOARD 设定点从位置切换为 姿态+固定推力,
        PX4 位置/速度环被旁路 = Z 轴定高关; 同时树莓派外环 PD 以
        setpoint_rate Hz 把 X/Y 位置误差转成期望倾角, 保住 X/Y 定点。
        X/Y 目标和航向锁定为调用瞬间的状态。
        返回 False=尚无位置/姿态数据 (无法安全解除)。
        前置: 需已 takeover_hold() 且设定点线程在跑。
        """
        pos = self.get_position()
        att = self.get_attitude_quat()
        if pos is None or att is None:
            return False
        if thrust is None:
            thrust = self.hover_thrust
        yaw = _yaw_from_quat(att)
        with self._sp_lock:
            self._sp_mode = 'xy_hold'
            self._sp_data = (pos[0], pos[1], yaw, thrust)
        return True

    def resume_altitude_hold(self):
        """
        定高恢复: 设定点切回当前位置定点保持, 位置环重新接管 (全轴)。
        返回 False=尚无位置 (此时 PX4 仍停留在姿态+推力控制!)。
        """
        return self.hold_current_position()

    def release(self):
        """
        停止设定点线程 (飞手接管)。
        注意: PX4 失去设定点流会触发 OFFBOARD 丢失保护 (COM_OBL_ACT),
        飞手应立即用遥控器切回手动模式!
        """
        self._sp_running = False
        if self._sp_thread is not None:
            self._sp_thread.join(timeout=2.0)
            self._sp_thread = None
        with self._mode_lock:
            self._watch_mode = False
            self._in_offboard = False

    @property
    def in_offboard(self):
        with self._mode_lock:
            return self._in_offboard

    # ---------------- 解锁 / 上锁 ----------------
    def arm(self):
        """
        解锁 (MAV_CMD_COMPONENT_ARM_DISARM param1=1)。
        OFFBOARD 下需设定点流已建立 (>2Hz); 重复调用无害。
        是否解锁成功看 self.armed (心跳 base_mode 更新有延迟)。
        """
        self.master.mav.command_long_send(
            self.master.target_system, self.master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
            1, 0, 0, 0, 0, 0, 0)

    def disarm(self):
        """
        上锁 (MAV_CMD_COMPONENT_ARM_DISARM param1=0)。
        谨慎使用 — 空中上锁会坠机! 仅限已落地/已触地场景。
        """
        self.master.mav.command_long_send(
            self.master.target_system, self.master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
            0, 0, 0, 0, 0, 0, 0)

    # ---------------- 备用 ----------------
    def land(self):
        """备用: 原地降落 (MAV_CMD_NAV_LAND)"""
        self.master.mav.command_long_send(
            self.master.target_system, self.master.target_component,
            mavutil.mavlink.MAV_CMD_NAV_LAND, 0,
            0, 0, 0, 0, 0, 0, 0)

    # ---------------- 收尾 ----------------
    def close(self):
        self.release()
        self._running = False
        try:
            self.master.close()
        except Exception:
            pass

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()
