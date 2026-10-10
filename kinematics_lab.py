# -*- coding: utf-8 -*-
"""
kinematics_lab.py — 8fly 机械臂正/逆运动学实验脚本
=====================================================
完整移植自 backup_original/arm_main/arm_main.ino + arm_config.h：

  - 连杆参数 L1~L5、坐标系定义(朝机头: x前/y上/z侧向, 单位 mm)
  - 正运动学 getPosition()  → fk()
  - 逆运动学 solve()        → ik()   (含 theta4 = -(theta2+theta3) 约束)
  - 两臂关节角度限位 (度)
  - 目标坐标限幅 (固件交互模式的 ±300mm) + 工作空间几何检查
  - 角度 ↔ PWM 三点标定换算 (便于估算实际舵机指令)

用法:
  python kinematics_lab.py                  # 交互模式: 循环提示输入 fk/ik/check 命令
  python kinematics_lab.py demo             # 跑内置演示/自测
  python kinematics_lab.py fk 0 20 20       # 正解: 3个关节角(度) → 末端坐标
  python kinematics_lab.py ik 150 200 0     # 逆解: 坐标(mm) → 关节角
  python kinematics_lab.py ik 150 200 0 --arm 2
  python kinematics_lab.py check 150 200 0  # 只检查坐标可达性(不动)

串口控制 (需 pip install pyserial):
  交互模式里 connect 后, ik/fk 会同时下发 MOVE_POSE/MOVE_JOINT 让机械臂动。
  注意: 固件在 Serial1 收指令 (Mega: RX1=19/TX1=18), 不是 USB 调试口;
  用 USB 转串口模块接 TX→19, RX→18, GND→GND, 波特率 115200。
"""

import math
import sys

PI = math.pi

# ============================================================
# 连杆参数 (mm) —— 与 arm_main.ino 第 72-76 行一致
# ============================================================
L1_x, L1_y, L1_z = 84.0, 48.0, 0.0
L2_x, L2_y, L2_z = 20.0, 18.0, 36.0
L3_x = 80.0
L4_x = 20.0
L5_x, L5_y, L5_z = 30.0, 136.0, -28.0

# ============================================================
# 角度限位 (运动学角度, 度) —— 与 arm_config.h 第 27-30 行一致
# 关节顺序: [t1, t2, t3, t4]
# ============================================================
ANGLE_MIN = {1: [-60.0, -30.0, -30.0, -75.0],
             2: [-60.0, -30.0, -30.0, -75.0]}
ANGLE_MAX = {1: [60.0, 75.0, 75.0, 75.0],
             2: [60.0, 75.0, 75.0, 75.0]}

# ============================================================
# 坐标限幅 (mm)
#   固件 solveInputMode 里对每个轴限制 [-300, 300]
# ============================================================
COORD_LIMIT = 300.0

# ============================================================
# 舵机三点标定 (PWM) —— 与 arm_config.h 第 17-24 行一致
# ============================================================
CALIB_ZERO   = {1: [1500, 1500, 1500, 1500], 2: [1450, 1500, 1500, 1500]}
CALIB_P90    = {1: [2450,  550, 2400,  650], 2: [ 550, 2300,  650, 2350]}
CALIB_M90    = {1: [ 550, 2450,  600, 2350], 2: [2300,  700, 2300,  650]}


def rad2deg(r):
    return r * 180.0 / PI


def deg2rad(d):
    return d * PI / 180.0


# ============================================================
# 角度限幅检查 / 钳制
# ============================================================
# 浮点容差: 逆解回代的边界角(如 -30°)可能算出 -30.000000000000004,
# 加一个极小容差避免把边界解误过滤 (对实际使用无影响)
ANGLE_EPS = 1e-6


def angle_in_range(angle, idx, arm):
    """对应固件 checkAngleRange(); idx 为 0..3"""
    return ANGLE_MIN[arm][idx] - ANGLE_EPS <= angle <= ANGLE_MAX[arm][idx] + ANGLE_EPS


def clamp_angle(angle, idx, arm):
    """对应固件 loadTarget() 里的 clamp 行为"""
    return max(ANGLE_MIN[arm][idx], min(ANGLE_MAX[arm][idx], angle))


def check_angles(t1, t2, t3, t4, arm=1):
    """返回 (ok, [逐关节检查结果])"""
    angles = [t1, t2, t3, t4]
    detail = []
    ok = True
    for i, a in enumerate(angles):
        r = angle_in_range(a, i, arm)
        detail.append({
            "joint": i + 1, "angle": a, "ok": r,
            "min": ANGLE_MIN[arm][i], "max": ANGLE_MAX[arm][i],
        })
        ok = ok and r
    return ok, detail


# ============================================================
# 坐标限幅检查 / 钳制
# ============================================================
def clamp_coordinate(x, y, z):
    """把目标点钳制到固件允许的 ±300mm 立方体内"""
    c = lambda v: max(-COORD_LIMIT, min(COORD_LIMIT, v))
    return c(x), c(y), c(z)


def workspace_check(x, y, z, arm=1):
    """
    坐标可达性检查(不动角度), 返回 (ok, 原因列表)
    1) 固件坐标限幅 ±300mm
    2) 逆解中的 sqrt() 必须非负  →  基座回转平面内可达
    3) 两连杆余弦定理: |L3-L4| <= D <= L3+L4
    """
    reasons = []
    ok = True

    for name, v in (("x", x), ("y", y), ("z", z)):
        if abs(v) > COORD_LIMIT:
            reasons.append("%s=%.1f 超出固件限幅 ±%.0f mm" % (name, v, COORD_LIMIT))
            ok = False

    # 对应 solve() 里 offset_x 的根号项
    dx = x - L1_x
    radicand = z * z + dx * dx - (L2_z + L5_z) ** 2
    if radicand < 0:
        reasons.append(
            "回转半径不足: z^2+(x-L1_x)^2=%.1f < (L2_z+L5_z)^2=%.1f, sqrt 无解"
            % (z * z + dx * dx, (L2_z + L5_z) ** 2))
        return False, reasons

    offset_x = math.sqrt(radicand)
    inter_x = offset_x - L2_x - L5_x
    inter_y = y - L1_y - L2_y - L5_y
    D = math.hypot(inter_x, inter_y)

    if D > (L3_x + L4_x):
        reasons.append("目标太远: D=%.1f > L3+L4=%.1f mm" % (D, L3_x + L4_x))
        ok = False
    if D < abs(L3_x - L4_x):
        reasons.append("目标太近: D=%.1f < |L3-L4|=%.1f mm" % (D, abs(L3_x - L4_x)))
        ok = False
    if not reasons:
        reasons.append("几何可达 (D=%.1f mm); 最终能否动到还取决于角度限位" % D)
    return ok, reasons


# ============================================================
# 正运动学 —— 对应固件 getPosition()
# 输入 t1,t2,t3 (度), t4 由约束决定不参与位置
# ============================================================
def fk(t1, t2, t3, arm=1):
    r1, r2, r3 = deg2rad(t1), deg2rad(t2), deg2rad(t3)
    offset_x = L2_x + L3_x * math.cos(r2) + L4_x * math.cos(r2 + r3) + L5_x
    x = L1_x + offset_x * math.cos(r1) - (L2_z + L5_z) * math.sin(r1)
    y = L1_y + L2_y + L3_x * math.sin(r2) + L4_x * math.sin(r2 + r3) + L5_y
    z = offset_x * math.sin(r1) + (L2_z + L5_z) * math.cos(r1)
    return (x, y, z)


# ============================================================
# 逆运动学 —— 对应固件 solve()
# 返回解列表 [{t1,t2,t3,t4}, ...], 已按角度限位过滤; 无解返回 []
# ============================================================
def ik(x, y, z, arm=1):
    dx = x - L1_x
    radicand = z * z + dx * dx - (L2_z + L5_z) ** 2
    if radicand < 0:
        return []

    offset_x = math.sqrt(radicand)
    rad1 = math.atan2(z, dx) - math.atan2((L2_z + L5_z), offset_x)
    t1 = rad2deg(rad1)

    inter_x = offset_x - L2_x - L5_x
    inter_y = y - L1_y - L2_y - L5_y
    D = math.hypot(inter_x, inter_y)
    if D > (L3_x + L4_x) + ANGLE_EPS or D < abs(L3_x - L4_x) - ANGLE_EPS:
        return []

    cos_r3 = (D * D - L3_x * L3_x - L4_x * L4_x) / (2 * L3_x * L4_x)
    cos_r3 = max(-1.0, min(1.0, cos_r3))

    solutions = []
    for r3 in (math.acos(cos_r3), -math.acos(cos_r3)):
        k1 = L3_x + L4_x * math.cos(r3)
        k2 = L4_x * math.sin(r3)
        r2 = math.atan2(inter_y, inter_x) - math.atan2(k2, k1)
        t2 = rad2deg(r2)
        t3 = rad2deg(r3)
        t4 = -t2 - t3  # 固定约束: 末端垂直于墙面

        if all(angle_in_range(a, i, arm)
               for i, a in enumerate((t1, t2, t3, t4))):
            solutions.append({"t1": t1, "t2": t2, "t3": t3, "t4": t4})
    return solutions


# ============================================================
# 角度 ↔ PWM (三点标定, 用 0°/+90° 两点)
# ============================================================
def angle_to_pwm(angle, idx, arm):
    return angle * (CALIB_P90[arm][idx] - CALIB_ZERO[arm][idx]) / 90.0 \
        + CALIB_ZERO[arm][idx]


def pwm_to_angle(pwm, idx, arm):
    return (pwm - CALIB_ZERO[arm][idx]) * 90.0 \
        / (CALIB_P90[arm][idx] - CALIB_ZERO[arm][idx])


# ============================================================
# 展示辅助
# ============================================================
def show_fk(t1, t2, t3, arm=1):
    t4 = -t2 - t3
    ok, detail = check_angles(t1, t2, t3, t4, arm)
    print("[FK] arm%d 关节角 t1=%.2f t2=%.2f t3=%.2f (t4=%.2f 由约束得到)"
          % (arm, t1, t2, t3, t4))
    for d in detail:
        flag = "OK " if d["ok"] else "超限"
        print("    关节%d: %8.2f°  [%6.1f°, %6.1f°]  %s"
              % (d["joint"], d["angle"], d["min"], d["max"], flag))
    if not ok:
        print("    ⚠ 有角度超限, 固件 loadTarget() 会 clamp 到边界, 实际位置会偏离!")
        t1c = clamp_angle(t1, 0, arm)
        t2c = clamp_angle(t2, 1, arm)
        t3c = clamp_angle(t3, 2, arm)
        print("    clamp 后: t1=%.2f t2=%.2f t3=%.2f t4=%.2f"
              % (t1c, t2c, t3c, -t2c - t3c))
        x, y, z = fk(t1c, t2c, t3c, arm)
    else:
        x, y, z = fk(t1, t2, t3, arm)
    print("    → 末端坐标: x=%.2f  y=%.2f  z=%.2f  (mm)" % (x, y, z))
    pwms = [round(angle_to_pwm(a, i, arm)) for i, a in enumerate((t1, t2, t3, t4))]
    print("    → 对应 PWM: %s (us)" % pwms)
    return (x, y, z)


def show_ik(x, y, z, arm=1):
    xc, yc, zc = clamp_coordinate(x, y, z)
    print("[IK] arm%d 目标点 x=%.2f y=%.2f z=%.2f (mm)" % (arm, x, y, z))
    if (xc, yc, zc) != (x, y, z):
        print("    ⚠ 超出固件坐标限幅 ±%.0fmm, clamp 到 x=%.1f y=%.1f z=%.1f"
              % (COORD_LIMIT, xc, yc, zc))

    ok, reasons = workspace_check(x, y, z, arm)
    for r in reasons:
        print("    工作空间检查: %s" % r)

    sols = ik(x, y, z, arm)
    print("    → 找到 %d 组解 (已按角度限位过滤)" % len(sols))
    for i, s in enumerate(sols):
        print("    解%d: t1=%8.2f° t2=%8.2f° t3=%8.2f° t4=%8.2f°"
              % (i + 1, s["t1"], s["t2"], s["t3"], s["t4"]))
        # 回代验证
        xr, yr, zr = fk(s["t1"], s["t2"], s["t3"], arm)
        err = math.dist((x, y, z), (xr, yr, zr))
        print("        回代FK: x=%.2f y=%.2f z=%.2f  误差=%.3f mm"
              % (xr, yr, zr, err))
        pwms = [round(angle_to_pwm(s[k], j, arm))
                for j, k in enumerate(("t1", "t2", "t3", "t4"))]
        print("        PWM: %s (us)" % pwms)
    if not sols:
        print("    ✗ 固件会回 ERR_IK_NO_SOLUTION (0x02)")
    return sols


# ============================================================
# 内置演示 / 自测
# ============================================================
def demo():
    print("=" * 60)
    print("1) 正运动学: 归零位 → 末端在哪?")
    print("=" * 60)
    show_fk(0, 0, 0, arm=1)

    print()
    print("=" * 60)
    print("2) 正运动学: 工作预备位 (0, 20, 20) 与 下压位 (-40, 20, 20)")
    print("=" * 60)
    show_fk(0, 20, 20, arm=1)
    print()
    show_fk(-40, 20, 20, arm=1)

    print()
    print("=" * 60)
    print("3) 逆运动学: 拿正解结果反推, 验证闭环")
    print("=" * 60)
    x, y, z = fk(10, 30, 20, 1)
    show_ik(x, y, z, arm=1)

    print()
    print("=" * 60)
    print("4) 逆运动学: 不可达点演示")
    print("=" * 60)
    print("-- 太远 --")
    show_ik(100, 400, 0, arm=1)
    print()
    print("-- 超坐标限幅 --")
    show_ik(500, 200, 0, arm=1)
    print()
    print("-- 几何可达但角度限位可能过滤 --")
    show_ik(120, 60, 0, arm=1)

    print()
    print("=" * 60)
    print("5) 角度↔PWM 换算抽查 (arm1 关节1: 0°/+90°/-90°)")
    print("=" * 60)
    for a in (0, 90, -90):
        p = angle_to_pwm(a, 0, 1)
        print("    %4d° → PWM %5d us → 反算 %.1f°" % (a, p, pwm_to_angle(p, 0, 1)))

    print()
    print("=" * 60)
    print("6) 批量闭环自测: 网格采样 FK→IK→FK")
    print("=" * 60)
    total = closed = 0
    max_err = 0.0
    for t1 in range(-60, 61, 15):
        for t2 in range(-30, 76, 15):
            for t3 in range(-30, 76, 15):
                t4 = -t2 - t3
                if not (-75 <= t4 <= 75):
                    continue
                total += 1
                x, y, z = fk(t1, t2, t3, 1)
                sols = ik(x, y, z, 1)
                for s in sols:
                    xr, yr, zr = fk(s["t1"], s["t2"], s["t3"], 1)
                    e = math.dist((x, y, z), (xr, yr, zr))
                    max_err = max(max_err, e)
                if sols:
                    closed += 1
    print("    采样 %d 组合法角度, %d 组能逆解回 (%.1f%%), 最大回代误差 %.4f mm"
          % (total, closed, 100.0 * closed / total, max_err))


# ============================================================
# 串口控制 (可选) —— 把解算结果发给机械臂
#   协议与 backup arm_protocol.h 一致 (XOR = CMD ^ LEN ^ PAYLOAD)
#   需要: pip install pyserial
#   接线: 固件在 Serial1 上收指令 (Mega: RX1=19, TX1=18),
#         USB 转串口模块 TX→RX1(19), RX→TX1(18), GND→GND, 115200
# ============================================================
import struct
import threading
import time

HEADER1, HEADER2 = 0xAA, 0x55
CMD_HOME, CMD_MOVE_JOINT, CMD_MOVE_POSE = 0x01, 0x02, 0x03
CMD_FAN_CTRL, CMD_AUTO_START, CMD_STOP = 0x04, 0x05, 0x06
RSP_ACK, RSP_STATUS, RSP_DONE, RSP_ERROR = 0x80, 0x81, 0x82, 0x83
ERR_TEXT = {0x01: "角度超限", 0x02: "逆解无解(IK no solution)",
            0x03: "超时", 0x04: "忙"}


def build_frame(cmd, payload=b""):
    """[0xAA][0x55][CMD][LEN][PAYLOAD...][XOR]"""
    chk = cmd ^ len(payload)
    for b in payload:
        chk ^= b
    return bytes([HEADER1, HEADER2, cmd, len(payload)]) + payload + bytes([chk])


class ArmLink:
    """与 backup 固件 Serial1 通信的轻量客户端"""

    def __init__(self, port, baud=115200):
        import serial  # 延迟导入, 没装 pyserial 时不影响纯计算功能
        self.ser = serial.Serial(port, baud, timeout=0.1)
        time.sleep(1.0)               # 若是 USB 直连复位了板子, 稍等重启
        self.ser.reset_input_buffer()

        self.status = None            # 最近一帧状态 (dict)
        self.last_error = None
        self._done = threading.Event()
        self._state = 0
        self._cmd = 0
        self._len = 0
        self._buf = bytearray()
        self._chk = 0
        self._running = True
        self._thread = threading.Thread(target=self._reader, daemon=True)
        self._thread.start()

    # ---------- 接收 ----------
    def _reader(self):
        while self._running:
            try:
                data = self.ser.read(64)
            except Exception:
                break
            for b in data:
                self._feed(b)

    def _feed(self, b):
        if self._state == 0:
            if b == HEADER1:
                self._state = 1
        elif self._state == 1:
            if b == HEADER2:
                self._state, self._chk = 2, 0
            elif b != HEADER1:
                self._state = 0
        elif self._state == 2:
            self._cmd, self._chk = b, self._chk ^ b
            self._state = 3
        elif self._state == 3:
            self._len, self._chk = b, self._chk ^ b
            if self._len > 32:
                self._state = 0
            else:
                self._buf = bytearray()
                self._state = 4 if self._len == 0 else 5
        elif self._state == 4:      # 校验字节
            if self._chk == b:
                self._on_frame(self._cmd, bytes(self._buf))
            self._state = 0
        elif self._state == 5:      # 负载
            self._buf.append(b)
            self._chk ^= b
            if len(self._buf) >= self._len:
                self._state = 4

    def _on_frame(self, cmd, payload):
        if cmd == RSP_STATUS and len(payload) >= 18:
            fan1, fan2 = struct.unpack_from("<HH", payload, 2)
            laser = struct.unpack_from("<H", payload, 8)[0]
            self.status = {
                "moving": bool(payload[0]), "auto": bool(payload[1]),
                "fan1": fan1, "fan2": fan2,
                "fsr1": bool(payload[6]), "fsr2": bool(payload[7]),
                "laser_mm": laser,
                "arm1": struct.unpack_from("<4b", payload, 10),
                "arm2": struct.unpack_from("<4b", payload, 14),
            }
        elif cmd == RSP_DONE:
            self._done.set()
        elif cmd == RSP_ERROR and payload:
            self.last_error = payload[0]

    # ---------- 发送 ----------
    def _send(self, cmd, payload=b""):
        self.ser.write(build_frame(cmd, payload))

    def move_pose(self, arm, x, y, z, wait=True, timeout=30):
        """逆解位置控制: 固件收到后自己 solve() 并移动"""
        self.last_error = None
        self._done.clear()
        self._send(CMD_MOVE_POSE,
                   bytes([arm]) + struct.pack("<fff", x, y, z))
        if wait:
            return self.wait_done(timeout)

    def move_joint(self, arm, t1, t2, t3, wait=True, timeout=30):
        """关节角控制 (3 个角, 固件自动算 t4)"""
        self.last_error = None
        self._done.clear()
        self._send(CMD_MOVE_JOINT,
                   bytes([arm]) + struct.pack("<fff", t1, t2, t3))
        if wait:
            return self.wait_done(timeout)

    def home(self, wait=True, timeout=30):
        self.last_error = None
        self._done.clear()
        self._send(CMD_HOME)
        if wait:
            return self.wait_done(timeout)

    def stop(self):
        self._send(CMD_STOP)

    def fan(self, arm, throttle):
        throttle = max(1000, min(2000, int(throttle)))
        self._send(CMD_FAN_CTRL, bytes([arm]) + struct.pack("<H", throttle))

    def wait_done(self, timeout=30):
        """返回: 'done' / 'error:...' / 'timeout'"""
        if not self._done.wait(timeout):
            return "timeout"
        if self.last_error is not None:
            return "error: " + ERR_TEXT.get(self.last_error,
                                            "0x%02X" % self.last_error)
        return "done"

    def close(self):
        self._running = False
        try:
            self.ser.close()
        except Exception:
            pass


def find_arm_port():
    """自动查找串口: Linux/树莓派 ttyACM*/ttyUSB*; Windows 看描述"""
    try:
        from serial.tools import list_ports
    except ImportError:
        return None
    for p in list_ports.comports():
        dev = (p.device or "").lower()
        desc = (p.description or "").lower()
        if ("ttyacm" in dev or "ttyusb" in dev or "arduino" in desc
                or "ch340" in desc or "usb serial" in desc):
            return p.device
    return None


# ============================================================
# 交互模式 —— 不带参数运行时进入, 循环提示输入
# ============================================================
INTERACTIVE_HELP = """\
命令格式 (逐行输入, 回车执行):
  fk t1 t2 t3      正解: 3个关节角(度) → 末端坐标 + PWM
                   (已连接时: 同时下发 MOVE_JOINT 让机械臂动)
  ik x y z         逆解: 目标坐标(mm) → 关节角 + PWM
                   (已连接时: 同时下发 MOVE_POSE 让机械臂动)
  check x y z      只检查坐标可达性
  arm 1 / arm 2    切换操作的臂 (当前: arm{arm})
  ---- 机械臂连接 ----
  connect [串口]   连接机械臂 (省略串口则自动查找, 如 /dev/ttyUSB0)
  disconnect       断开连接
  home             全部归零
  stop             急停
  fan on / fan off 当前臂风扇开/关
  status           显示最新状态上报
  demo             跑一遍内置演示
  h                显示本帮助
  q                退出\
"""


def _try_connect(link_holder, port=None):
    """尝试连接机械臂, 成功返回 True"""
    if port is None:
        port = find_arm_port()
        if port is None:
            print("未找到串口, 请手动指定: connect /dev/ttyUSB0")
            return False
    try:
        link_holder[0] = ArmLink(port)
        print("已连接 %s (115200)" % port)
        return True
    except ImportError:
        print("缺少 pyserial, 请先: pip install pyserial")
        return False
    except Exception as e:
        print("连接失败: %s" % e)
        return False


def interactive_shell():
    arm = 1
    link_holder = [None]          # 当前连接 (None = 未连接, 只算不动)

    def link():
        return link_holder[0]

    print("=" * 60)
    print("8fly 机械臂运动学实验台 —— 交互模式")
    print("=" * 60)
    print(INTERACTIVE_HELP.format(arm=arm))
    while True:
        conn = "●" if link() else "○"
        try:
            line = input("\n%s arm%d > " % (conn, arm)).strip()
        except (EOFError, KeyboardInterrupt):
            print("\n退出.")
            if link():
                link().close()
            return
        if not line:
            continue
        parts = line.split()
        cmd = parts[0].lower()

        if cmd in ("q", "quit", "exit"):
            print("退出.")
            if link():
                link().close()
            return
        elif cmd in ("h", "help"):
            print(INTERACTIVE_HELP.format(arm=arm))
        elif cmd == "demo":
            demo()
        elif cmd == "arm":
            if len(parts) == 2 and parts[1] in ("1", "2"):
                arm = int(parts[1])
                print("已切换到 arm%d" % arm)
            else:
                print("用法: arm 1 或 arm 2")

        # ---------- 连接管理 ----------
        elif cmd == "connect":
            if link():
                print("已连接, 先 disconnect 再重连")
            else:
                _try_connect(link_holder, parts[1] if len(parts) > 1 else None)
        elif cmd == "disconnect":
            if link():
                link().close()
                link_holder[0] = None
                print("已断开")
            else:
                print("当前未连接")

        # ---------- 机械臂直接指令 ----------
        elif cmd == "home":
            if not link():
                print("未连接, 先 connect")
            else:
                print("归零中...", "结果:", link().home())
        elif cmd == "stop":
            if not link():
                print("未连接, 先 connect")
            else:
                link().stop()
                print("已发送急停")
        elif cmd == "fan":
            if not link():
                print("未连接, 先 connect")
            elif len(parts) == 2 and parts[1] in ("on", "off"):
                link().fan(arm, 1200 if parts[1] == "on" else 1000)
                print("arm%d 风扇 %s" % (arm, parts[1]))
            else:
                print("用法: fan on / fan off")
        elif cmd == "status":
            if not link():
                print("未连接, 先 connect")
            elif link().status is None:
                print("尚未收到状态帧 (固件每 200ms 上报一次, 稍等再试)")
            else:
                s = link().status
                print("moving=%s auto=%s fan=[%d,%d] fsr=[%s,%s] laser=%dmm"
                      % (s["moving"], s["auto"], s["fan1"], s["fan2"],
                         s["fsr1"], s["fsr2"], s["laser_mm"]))
                print("arm1角度=%s arm2角度=%s" % (s["arm1"], s["arm2"]))

        # ---------- 运动学 ----------
        elif cmd == "fk":
            if len(parts) != 4:
                print("用法: fk t1 t2 t3   (3个关节角, 单位: 度)")
                continue
            try:
                t1, t2, t3 = map(float, parts[1:4])
            except ValueError:
                print("角度必须是数字")
                continue
            show_fk(t1, t2, t3, arm)
            if link():
                ok, _ = check_angles(t1, t2, t3, -t2 - t3, arm)
                if not ok:
                    print("⚠ 有角度超限, 不下发 (固件也会 clamp, 位置会偏)")
                else:
                    print("下发 MOVE_JOINT...", "结果:",
                          link().move_joint(arm, t1, t2, t3))
        elif cmd == "ik":
            if len(parts) != 4:
                print("用法: ik x y z   (目标坐标, 单位: mm)")
                continue
            try:
                x, y, z = map(float, parts[1:4])
            except ValueError:
                print("坐标必须是数字")
                continue
            sols = show_ik(x, y, z, arm)
            if link():
                if not sols:
                    print("本地无解, 不下发")
                else:
                    print("下发 MOVE_POSE...", "结果:",
                          link().move_pose(arm, x, y, z))
        elif cmd == "check":
            if len(parts) != 4:
                print("用法: check x y z")
                continue
            try:
                x, y, z = map(float, parts[1:4])
            except ValueError:
                print("坐标必须是数字")
                continue
            ok, reasons = workspace_check(x, y, z, arm)
            for r in reasons:
                print(r)
            print("结论: %s" % ("可达(几何上)" if ok else "不可达"))
        else:
            print("未知命令 %r, 输入 h 查看帮助" % cmd)


# ============================================================
# CLI
# ============================================================
def _parse_arm(argv):
    arm = 1
    if "--arm" in argv:
        i = argv.index("--arm")
        arm = int(argv[i + 1])
        del argv[i:i + 2]
    if arm not in (1, 2):
        sys.exit("--arm 只能是 1 或 2")
    return arm, argv


def main():
    argv = sys.argv[1:]
    # 不带参数 → 进入交互模式, 循环提示输入
    if not argv or argv[0] in ("shell", "i", "play"):
        interactive_shell()
        return
    if argv[0] == "demo":
        demo()
        return

    cmd = argv[0]
    if cmd == "fk":
        arm, rest = _parse_arm(argv[1:])
        if len(rest) != 3:
            sys.exit("用法: fk t1 t2 t3 [--arm 1|2]   (角度单位: 度)")
        t1, t2, t3 = map(float, rest)
        show_fk(t1, t2, t3, arm)
    elif cmd == "ik":
        arm, rest = _parse_arm(argv[1:])
        if len(rest) != 3:
            sys.exit("用法: ik x y z [--arm 1|2]   (坐标单位: mm)")
        x, y, z = map(float, rest)
        show_ik(x, y, z, arm)
    elif cmd == "check":
        arm, rest = _parse_arm(argv[1:])
        if len(rest) != 3:
            sys.exit("用法: check x y z [--arm 1|2]")
        x, y, z = map(float, rest)
        ok, reasons = workspace_check(x, y, z, arm)
        for r in reasons:
            print(r)
        print("结论: %s" % ("可达(几何上)" if ok else "不可达"))
    else:
        sys.exit("未知命令 %r; 可用: demo / fk / ik / check" % cmd)


if __name__ == "__main__":
    main()
