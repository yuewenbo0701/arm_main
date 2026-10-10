#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
8fly 机械臂 · 树莓派上位机客户端
=====================================
通过 USB 串口与 Arduino 通信, 二进制帧协议 (与 arm_protocol.h 一致):

    [0xAA][0x55][CMD][LEN][PAYLOAD...][XOR_CHECKSUM]
    XOR_CHECKSUM = CMD ^ LEN ^ PAYLOAD 各字节异或

依赖: pip install pyserial   (树莓派 OS 也可: sudo apt install python3-serial)

示例:
    from arm_client import ArmClient, find_port

    with ArmClient(find_port()) as arm:
        arm.home();  arm.wait_done()          # 归零并等待完成
        arm.move_joint(1, [0, 20, 20, -40])   # 臂1 四关节角 (度)
        arm.wait_done()
        arm.fan_on(1)                         # 臂1 风扇开
        arm.stop()                            # 急停
"""

import struct
import threading
import time
from dataclasses import dataclass

import serial

# ------------------------------------------------------------
# 协议定义 (与 arm_protocol.h 保持一致)
# ------------------------------------------------------------
HEADER1, HEADER2 = 0xAA, 0x55
MAX_PAYLOAD = 32

# 下行命令 (树莓派 -> Arduino)
CMD_HOME       = 0x01   # 归零所有舵机
CMD_MOVE_JOINT = 0x02   # 关节控制 [arm:1B][angles: 4xfloat32 小端]
CMD_FAN_CTRL   = 0x04   # 风扇控制 [arm:1B][throttle:uint16 小端]
CMD_AUTO_START = 0x05   # 启动自动工作序列
CMD_STOP       = 0x06   # 急停
CMD_STATUS_REQ = 0x07   # 请求立即上报一次状态

# 上行响应 (Arduino -> 树莓派)
RSP_ACK    = 0x80
RSP_STATUS = 0x81
RSP_DONE   = 0x82
RSP_ERROR  = 0x83

ERR_TEXT = {
    0x01: "角度超限",
    0x03: "超时",
    0x04: "忙",
}


@dataclass
class ArmStatus:
    """Arduino 周期性上报的状态 (18 字节旧负载; 双臂新固件 22 字节, 末尾追加 FSR 原始值;
    四臂固件 30 字节, 再追加 臂3/臂4 当前角度)"""
    moving: bool        # 是否正在移动
    auto: bool          # 自动序列是否运行中
    fan1: int           # 臂1 风扇油门 (us)
    fan2: int           # 臂2 风扇油门 (us)
    fsr1: bool          # 臂1 压力传感器触发
    fsr2: bool          # 臂2 压力传感器触发
    laser_mm: int       # 激光测距 (mm, 未启用时恒 0)
    arm1_angles: tuple  # 臂1 当前 4 关节角 (度, 整数近似)
    arm2_angles: tuple  # 臂2 当前 4 关节角 (度, 整数近似)
    fsr1_raw: int = None   # 臂1 FSR ADC 原始值 (0~1023; 18 字节旧帧为 None)
    fsr2_raw: int = None   # 臂2 FSR ADC 原始值 (同上)
    arm3_angles: tuple = None  # 臂3 当前 4 关节角 (仅 30 字节四臂帧, 否则 None)
    arm4_angles: tuple = None  # 臂4 当前 4 关节角 (仅 30 字节四臂帧, 否则 None)

    @classmethod
    def from_payload(cls, p: bytes):
        fan1, fan2 = struct.unpack_from('<HH', p, 2)
        laser = struct.unpack_from('<H', p, 8)[0]
        fsr1_raw = fsr2_raw = None
        if len(p) >= 22:    # 新固件: 末尾追加 2xuint16 FSR 原始值
            fsr1_raw, fsr2_raw = struct.unpack_from('<HH', p, 18)
        arm3_angles = arm4_angles = None
        if len(p) >= 30:    # 四臂固件: 末尾再追加 臂3/臂4 角度
            arm3_angles = struct.unpack_from('<4b', p, 22)
            arm4_angles = struct.unpack_from('<4b', p, 26)
        return cls(
            moving=bool(p[0]), auto=bool(p[1]),
            fan1=fan1, fan2=fan2,
            fsr1=bool(p[6]), fsr2=bool(p[7]),
            laser_mm=laser,
            arm1_angles=struct.unpack_from('<4b', p, 10),
            arm2_angles=struct.unpack_from('<4b', p, 14),
            fsr1_raw=fsr1_raw, fsr2_raw=fsr2_raw,
            arm3_angles=arm3_angles, arm4_angles=arm4_angles,
        )


def build_frame(cmd: int, payload: bytes = b'') -> bytes:
    """构建一帧: [0xAA][0x55][CMD][LEN][PAYLOAD...][XOR]"""
    chk = cmd ^ len(payload)
    for b in payload:
        chk ^= b
    return bytes([HEADER1, HEADER2, cmd, len(payload)]) + payload + bytes([chk])


class FrameParser:
    """逐字节解析器, 状态机与 Arduino 端 arm_parse_byte 一致"""

    def __init__(self, on_frame):
        self.on_frame = on_frame   # 回调: on_frame(cmd, payload_bytes)
        self.state = 0
        self.cmd = 0
        self.length = 0
        self.buf = bytearray()
        self.chk = 0

    def feed(self, b: int):
        if self.state == 0:
            if b == HEADER1:
                self.state = 1
        elif self.state == 1:
            if b == HEADER2:
                self.state = 2
                self.chk = 0
            elif b != HEADER1:
                self.state = 0
        elif self.state == 2:
            self.cmd = b
            self.chk ^= b
            self.state = 3
        elif self.state == 3:
            self.length = b
            self.chk ^= b
            if self.length > MAX_PAYLOAD:
                self.state = 0            # 长度非法, 丢弃
            elif self.length == 0:
                self.buf = bytearray()    # 清空, 避免残留上一帧负载
                self.state = 4            # 无负载, 直接等校验
            else:
                self.buf = bytearray()
                self.state = 5
        elif self.state == 4:             # 等校验字节
            if self.chk == b:
                self.on_frame(self.cmd, bytes(self.buf[:self.length]))
            self.state = 0
        elif self.state == 5:             # 读负载
            self.buf.append(b)
            self.chk ^= b
            if len(self.buf) >= self.length:
                self.state = 4


class ArmClient:
    """机械臂串口客户端 (后台线程持续接收状态/应答)"""

    def __init__(self, port: str, baud: int = 115200, status_cb=None):
        """
        port: 串口设备, 树莓派上通常为 /dev/ttyACM0 (或 /dev/ttyUSB0)
        status_cb: 可选回调, 每次收到状态帧时调用 status_cb(ArmStatus)
        """
        self.ser = serial.Serial(port, baud, timeout=0.1)
        time.sleep(2.0)                 # 打开串口会复位 Arduino, 等待其重启
        self.ser.reset_input_buffer()

        self.status = None              # 最近一帧 ArmStatus
        self.status_time = 0.0          # 最近一帧状态的接收时刻 (time.time), 0=尚未收到
        self.last_error = None          # 最近一次错误码 (ERR_TEXT 可查含义)
        self.ack_count = 0

        self._status_cb = status_cb
        self._done = threading.Event()
        self._parser = FrameParser(self._on_frame)
        self._lock = threading.Lock()
        self._running = True
        self._thread = threading.Thread(target=self._reader_loop, daemon=True)
        self._thread.start()

    # ---------------- 接收 ----------------
    def _reader_loop(self):
        while self._running:
            try:
                data = self.ser.read(64)
            except serial.SerialException:
                break
            for b in data:
                self._parser.feed(b)

    def _on_frame(self, cmd, payload):
        if cmd == RSP_STATUS and len(payload) >= 18:
            self.status = ArmStatus.from_payload(payload)
            self.status_time = time.time()
            if self._status_cb:
                self._status_cb(self.status)
        elif cmd == RSP_DONE:
            self._done.set()
        elif cmd == RSP_ACK:
            self.ack_count += 1
        elif cmd == RSP_ERROR and payload:
            self.last_error = payload[0]

    # ---------------- 发送 ----------------
    def _send(self, cmd, payload=b''):
        with self._lock:
            self.ser.write(build_frame(cmd, payload))

    # ---------------- 指令 API ----------------
    def home(self, wait=False, timeout=30):
        """所有舵机归零"""
        self._done.clear()
        self._send(CMD_HOME)
        if wait:
            return self.wait_done(timeout)

    def move_joint(self, arm, angles, wait=False, timeout=30):
        """
        下发关节角 (度)
        arm:    1=左上 2=右上 3=左下 4=右下 (旧双臂固件仅接受 1/2)
        angles: [a1, a2, a3, a4] 四个关节角;
                兼容 [a1, a2, a3] —— 固件自动取 a4 = -(a2 + a3)
        wait:   True 时阻塞等待移动完成
        """
        angles = list(angles)
        if len(angles) not in (3, 4):
            raise ValueError("angles 需要 3 或 4 个关节角")
        payload = bytes([arm]) + struct.pack('<%df' % len(angles), *angles)
        self._done.clear()
        self._send(CMD_MOVE_JOINT, payload)
        if wait:
            return self.wait_done(timeout)

    def set_fan(self, arm, throttle):
        """风扇油门 1000~2000 (us), 1000 = 关闭"""
        throttle = max(1000, min(2000, int(throttle)))
        self._send(CMD_FAN_CTRL, bytes([arm]) + struct.pack('<H', throttle))

    def fan_on(self, arm, throttle=1200):
        self.set_fan(arm, throttle)

    def fan_off(self, arm):
        self.set_fan(arm, 1000)

    def auto_start(self):
        """启动 Arduino 端内置的自动工作序列"""
        self._send(CMD_AUTO_START)

    def stop(self):
        """急停: 停止移动 / 自动序列 / 风扇"""
        self._send(CMD_STOP)

    def request_status(self):
        """请求立即上报一次状态 (平时每 200ms 自动上报)"""
        self._send(CMD_STATUS_REQ)

    # ---------------- 同步等待 ----------------
    def wait_done(self, timeout=30):
        """等待移动完成 (RSP_DONE)。返回 True=完成, False=超时"""
        ok = self._done.wait(timeout)
        if not ok:
            return False
        if self.last_error is not None:
            # 移动前若收到过错误帧, 由调用方决定如何处理
            pass
        return True

    # ---------------- 收尾 ----------------
    def close(self):
        self._running = False
        try:
            self.ser.close()
        except Exception:
            pass

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()


def find_port():
    """自动查找 Arduino 串口。
    Linux/树莓派: /dev/ttyACM* 或 /dev/ttyUSB*; Windows: 描述含 Arduino/CH340 等。
    找不到返回 None。"""
    from serial.tools import list_ports
    for p in list_ports.comports():
        dev = (p.device or '').lower()
        desc = (p.description or '').lower()
        if ('ttyacm' in dev or 'ttyusb' in dev or 'arduino' in desc
                or 'ch340' in desc or 'usb serial' in desc):
            return p.device
    return None
