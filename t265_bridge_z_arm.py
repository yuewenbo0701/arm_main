# -*- coding: utf-8 -*-
"""
T265 -> PX4 桥接 + Z 轴控制参数开关 + 四机械臂爬墙 + 前臂涵道风扇 (四合一版)
====================================================================

由 t265_bridge_z_toggle.py 与 climb4_wall_test.py (四臂爬墙) 合并而来。

功能 1 (视觉桥接, 与 t265_bridge_z_toggle.py 相同):
    读取 T265 位姿, 转换到 NED 系 (X前/Y右/Z下), 降频到 30Hz
    通过 VISION_POSITION_ESTIMATE 发给 Pixhawk, 供 EKF2 融合定点。
    相机安装朝向: 水平面内偏转 90°, 重映射关系与原版一致。

功能 2 (Z 轴三参数开关, 与 t265_bridge_z_toggle.py 相同):
    按 a : 三个参数同时进入"测试态":
             MPC_Z_P = 0          位置环不回拉: 被推后不再回原高度
             MPC_Z_VEL_I_ACC = 0  速度环无积分: 稳态误差无人补偿,
                                  悬停油门模型偏差/电池压降/机械臂载重
                                  变化都会变成持续缓慢爬升或下沉
             MPC_Z_VEL_P_ACC = 2  速度环比例减半(默认4): 刹车变软,
                                  被推后滑得更远, 抗风变差
           !! 出现持续快速/加速漂移立即按 b !!
    按 b : 三个参数都恢复到启动时各自读到的原值 (读不到用兜底值)。
    按 t : 参数自检 (读x3 -> 设测试值(回显验证) -> 恢复(回显验证), 9步)

功能 3 (四机械臂爬墙, 来自 climb4_wall_test.py, 含臂端涵道风扇吸附):
    臂号: 1=左上 2=右上 3=左下 4=右下 (观察者面对机头方向),
          四臂共用同一组关节角步态
    步态四组关节角 (度, 根部->末端, 4 个角全部下发):
          G1 {-30, 10, 10, -20} 低位贴墙 (初始化/循环终点"向下")
          G2 {-30,-10,-10,  20} 低位离墙
          G3 { 30,-10,-10,  20} 高位离墙 (抬升)
          G4 { 30, 10, 10, -20} 高位贴墙
    臂端涵道风扇 (吸附): 臂1左上=引脚8(风扇1) 臂2右上=引脚9(风扇2)
          臂3左下=引脚26(风扇5) 臂4右下=引脚27(风扇6);
          贴墙到位后直跳 1200us 启动再补到 1300us 吸附, 离墙前渐弱至 1000us(关闭)
          (升油门不能慢爬: 电调低速蠕动会触发失速保护锁死)
    按 l : 初始化 —— 四臂同时移至 G1 (低位贴墙), 到位后四臂风扇渐大吸附
    按 k : 爬墙循环 Step1~4 (左上->右上->左下->右下 依次
           风扇渐弱关 -> 离墙 -> 抬升 -> 贴墙 -> 风扇启动吸附)
           -> Step5 (四臂同时向下回 G1, 风扇保持吸附); 循环中再按 k = 停止并重新开始
    按 s : 机械臂急停 —— 中断当前动作/循环, 关全部风扇(含臂端/前扇), 发 STOP;
           只停机械臂, 不退出脚本 (桥接/参数开关继续运行)
    机械臂动作在独立线程执行, 阻塞等待不占用主循环,
    30Hz 视觉流 / TIMESYNC 应答 / 参数操作均不中断。

功能 4 (前臂涵道风扇: 左前臂=引脚33 / 右前臂=引脚32):
    按 f : 左/右前臂涵道风扇同时转到 1900us; 再按 f 关闭 (1000us)。
    按 g : 双前扇从 1100us 渐进到 1800us (每 0.5s +50us, 全程约 7s);
           渐进中再按 g = 取消并关闭; 完成则保持 1800us。
           协议上作为风扇 3(左)/4(右) 下发, 需配套固件支持
           (CMD_FAN_CTRL 接受风扇号 1~6)。s 急停 / q 退出会自动关闭。

定高说明:
    本脚本的"关 Z 定高"走功能 2 的 MPC_Z 参数路线 (a/b)。
    climb_test 原来的 OFFBOARD 姿态设定点旁路方案未并入: 它需要独占
    飞控链路发 10Hz 设定点, 与桥接共用串口冲突, 且原脚本中已停用。

键盘总表 (单键即触发, 无需回车):
    a = Z 三参数进测试态    b = Z 三参数恢复原值
    t = 参数自检            q = 退出 (臂急停+关全部风扇, 测试值参数自动恢复)
    l = 四臂初始化          k = 四臂爬墙循环 (循环中再按=重启)
    s = 机械臂急停(含前扇)  f = 前臂双涵道风扇转到1900 (再按=关闭)
    g = 前扇渐进 1100->1800 (渐进中再按=取消并关闭)

串口:
    飞控   /dev/ttyUSB0 @ 921600  (MAVLINK_ADDR 可改)
    机械臂 Arduino 端走 Serial1 (引脚18=TX1/19=RX1) @ 115200;
           树莓派侧依次探测 /dev/ttyUSB1, /dev/serial0, /dev/ttyAMA0
           (--arm-port 可指定, 都找不到再自动探测 USB 转串口)
    T265 启动失败 -> 降级为纯参数开关+机械臂模式;
    机械臂连接失败 -> 降级为桥接+参数开关模式 (l/k/s 提示未连接)。

依赖: pip install pyrealsense2 pymavlink numpy pyserial
运行: python t265_bridge_z_arm.py              (桥接 + Z开关 + 机械臂)
      python t265_bridge_z_arm.py --selftest  (纯参数自检, 不开相机/机械臂)
      python t265_bridge_z_arm.py --no-arm    (禁用机械臂)
      python t265_bridge_z_arm.py --cycles 5  (爬墙循环次数, 0=无限)
"""

import argparse
import os
import sys
import time
import math
import threading
from collections import deque

import numpy as np
from pymavlink import mavutil

# arm_client.py 在 rpi/ 子目录
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), 'rpi'))

# ========== 参数区 ==========
MAVLINK_ADDR   = '/dev/ttyUSB0'   # Pixhawk USB 直连
MAVLINK_BAUD   = 921600
SEND_RATE_HZ   = 30.0             # 视觉定位发送频率 (T265 输出 200Hz, 降频发给飞控)
MIN_CONFIDENCE = 2                # tracker_confidence: 0失败 1低 2中 3高

# 机械臂 (四臂版, 固件链路走 Arduino Serial1 = 引脚18/19)
# 树莓派侧候选串口: USB-TTL 接 18/19 时是 ttyUSBx; 树莓派 GPIO 串口直连时是 serial0/ttyAMA0
ARM_PORT_CANDIDATES = ['/dev/ttyUSB1', '/dev/serial0', '/dev/ttyAMA0']
ARM_BAUD   = 115200

# 步态参数 (与 climb4_wall_test.py 一致: 四组关节角, 根部->末端, 4 个角全部下发)
G1_DOWN_ATTACH = [-30.0,  10.0,  10.0, -20.0]   # 低位贴墙 (初始化 / 循环终点"向下")
G2_DOWN_DETACH = [-30.0, -10.0, -10.0,  20.0]   # 低位离墙
G3_UP_DETACH   = [ 30.0, -10.0, -10.0,  20.0]   # 高位离墙 (抬升)
G4_UP_ATTACH   = [ 30.0,  10.0,  10.0, -20.0]   # 高位贴墙

# 四臂定义: (协议臂号, 中文名) —— 循环动作顺序; 观察者面对机头方向
CLIMB_ARMS = [(1, '左上'), (2, '右上'), (3, '左下'), (4, '右下')]

# 爬墙时序参数
MOVE_SETTLE_S  = 0.5    # 每段移动到位后的稳定等待
STEP_SETTLE_S  = 1.0    # 每个臂完成"离墙->抬升->贴墙"后的等待
CYCLE_SETTLE_S = 1.0    # 每轮循环结束后的等待
MOVE_TIMEOUT_S = 30     # 等待 RSP_DONE 超时

# 前臂涵道风扇: 左前臂接 Arduino 引脚33 (协议风扇号 3), 右前臂接引脚32 (风扇号 4)
FRONT_FAN_L    = 3
FRONT_FAN_R    = 4
FRONT_FAN_HIGH = 1900

# 臂端涵道风扇 (爬墙吸附): 面对机头方向
#   臂1左上=引脚8(风扇号1) 臂2右上=引脚9(风扇号2)
#   臂3左下=引脚26(风扇号5) 臂4右下=引脚27(风扇号6)
# 贴墙时启动到 TIP_FAN_ATTACH, 离墙时渐弱到 1000(关闭)
# !!! 升油门不能慢爬: 电调低速蠕动会触发失速保护锁死 (渐变不转/直跳能转),
#     必须先直跳到 TIP_FAN_START 让电机转起来, 再补到目标 !!!
TIP_FAN_NO          = {1: 1, 2: 2, 3: 5, 4: 6}   # 臂号 -> 协议风扇号
TIP_FAN_ATTACH      = 1300    # 贴墙吸附目标油门 (us)
TIP_FAN_START       = 1200    # 电调启动油门 (直跳, 低于此值可能启动失败锁死)
TIP_FAN_START_HOLD_S = 0.3    # 启动油门保持时间 (s), 让电机稳定后再补到目标
TIP_FAN_STEP        = 50      # 降油门渐变每步 -50us
TIP_FAN_INTERVAL    = 0.1     # 降油门每步间隔 0.1s

# 前臂涵道风扇渐进测试 (按 g): 从 RAMP_START 逐步加到 RAMP_END
FAN_RAMP_START    = 1100
FAN_RAMP_END      = 1800
FAN_RAMP_STEP     = 50      # 每步 +50us
FAN_RAMP_INTERVAL = 0.5     # 每步间隔 0.5s (全程约 7s)

# 开关管理的参数清单:
#   name     = PX4 参数名
#   off      = 按 a 时的"测试态"目标值 (注意 Z_VP 是改小不是置0)
#   fallback = 启动读不到原值时的恢复兜底 (PX4 默认值)
#   short    = 状态行缩写; note_off/note_on = 按键成功后的提示语
PARAMS = [
    {'name': 'MPC_Z_P',         'off': 0.0, 'fallback': 1.0, 'short': 'Z_P',
     'note_off': '—— Z位置环已关 (被推后不再回原高度)',
     'note_on':  '—— Z位置环已开 (恢复回原高度)'},
    {'name': 'MPC_Z_VEL_I_ACC', 'off': 0.0, 'fallback': 2.0, 'short': 'Z_I',
     'note_off': '—— Z速度环I项已关 (无稳态补偿, 悬停可能缓慢漂移!)',
     'note_on':  '—— Z速度环I项已恢复'},
    {'name': 'MPC_Z_VEL_P_ACC', 'off': 2.0, 'fallback': 4.0, 'short': 'Z_VP',
     'note_off': '—— Z速度环P减半 (刹车变软, 被推后滑得更远!)',
     'note_on':  '—— Z速度环P已恢复 (刹车复原)'},
]
TRACKED     = [p['name'] for p in PARAMS]
OFF_TARGET  = {p['name']: p['off'] for p in PARAMS}
FALLBACK_ON = {p['name']: p['fallback'] for p in PARAMS}
NOTES       = {(p['name'], k): p[k] for p in PARAMS for k in ('note_off', 'note_on')}

VERIFY_TIMEOUT  = 2.0             # SET 回显等待超时 (秒); SET 回显实测毫秒级
READ_TIMEOUT    = 4.0             # READ 响应等待超时 (秒); READ 响应实测可能数秒
MAX_RETRY       = 3               # 每个参数操作的重试次数
REREAD_INTERVAL = 3.0             # 空闲时周期性读回参数, 让显示反映飞控真值

# ========== 全局状态 ==========
running = True
lock    = threading.Lock()
pending = None                    # None / 'off' / 'on' / 'test' —— 键盘置位, 主循环消费

# 参数操作状态机 (仅主线程读写, 无需加锁); 值都按参数名分槽存放
P = {
    'cur':         {n: None for n in TRACKED},   # 最近一次 PARAM_VALUE 读到的真值
    'on_value':    dict(FALLBACK_ON),            # 恢复目标 (启动时学习)
    'on_fallback': {n: True for n in TRACKED},   # on_value 仍是兜底值, 读到真值后更新
    'current_op':  None,                         # 正在等待回显的操作
    'st_results':  [],                           # 自检各步骤 True/False
    'st_ctx':      {},                           # 自检上下文: {'start': {name: value}}
    'st_total':    0,
    'st_running':  False,
}
op_queue = deque()                # 待执行参数操作队列

# ========== 机械臂状态 ==========
arm             = None            # ArmClient 实例, None = 未连接/已禁用
arm_init_event  = threading.Event()
arm_loop_event  = threading.Event()
fan_ramp_event  = threading.Event()   # g 键: 前扇渐进
arm_stop        = threading.Event()   # 中断当前臂动作/循环/渐进 (s 键 / 退出时置位)
arm_loop_active = False
fan_ramp_active = False               # 前扇渐进进行中
ARM_CYCLES      = 0                   # --cycles, 0 = 无限循环
front_fan_cur   = 1000                # 前臂双涵道风扇当前油门 (us, 1000=关)
tip_fan_cur     = {1: 1000, 2: 1000, 3: 1000, 4: 1000}   # 各臂末端风扇当前油门 (us)


def mod_count():
    """当前确认处于"测试值"的受管参数个数 (读不到的不计入)"""
    return sum(1 for n in TRACKED
               if P['cur'][n] is not None
               and abs(P['cur'][n] - OFF_TARGET[n]) < 1e-4)

def fmt(v):
    return f"{v:.3f}" if v is not None else "?"


# ========== 四元数工具 ==========
def quat_mul(q1, q2):
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    return np.array([
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2,
        w1*w2 - x1*x2 - y1*y2 - z1*z2
    ])

def quat_inverse(q):
    x, y, z, w = q
    norm2 = x*x + y*y + z*z + w*w
    return np.array([-x, -y, -z, w]) / norm2

# ========== T265 坐标系 -> NED 重映射 ==========
def remap_pos(p_t265):
    x, y, z = p_t265
    # 基础映射旧结果为 (-z, x, -y)，叠加90°旋转后:
    return np.array([-x, -z, -y])

def remap_quat(q_t265):
    x, y, z, w = q_t265
    return np.array([-z, x, -y, w])

# ========== 四元数 -> 欧拉角（ZYX，输出单位: 弧度） ==========
def quat_to_euler_rad(q):
    x, y, z, w = q
    pitch = -math.asin(np.clip(2.0 * (x * z - w * y), -1, 1))
    roll  = math.atan2(2.0 * (w * x + y * z), w * w - x * x - y * y + z * z)
    yaw   = math.atan2(2.0 * (w * z + x * y), w * w + x * x - y * y - z * z)
    return -pitch, roll, yaw


# ========== 键盘监听线程 ==========
# a 测试态 / b 全恢复 / t 自检 / q 退出  (参数部分, 置 pending 由主循环消费)
# l 臂初始化 / k 爬墙循环 / s 臂急停    (机械臂部分, 直接操作臂事件)
def keyboard_listener():
    global running, pending, arm_loop_active, front_fan_cur

    restore_termios = None
    try:
        import msvcrt                                  # Windows
        def read_key():
            if msvcrt.kbhit():
                return msvcrt.getch().decode('utf-8', errors='ignore').lower()
            return None
    except ImportError:                                # Linux / macOS
        import tty, termios, select
        fd = sys.stdin.fileno()
        old = termios.tcgetattr(fd)
        tty.setcbreak(fd)
        def restore_termios():                         # 退出时恢复终端, 否则 shell 会乱
            termios.tcsetattr(fd, termios.TCSADRAIN, old)
        def read_key():
            if select.select([sys.stdin], [], [], 0)[0]:
                return sys.stdin.read(1).lower()
            return None

    print("\n[键盘] a=Z测试态  b=Z恢复  t=参数自检  q=退出 | "
          "l=臂初始化  k=爬墙循环(再按=重启)  s=臂急停 | "
          "f=前扇1900(再按=关)  g=前扇渐进1100->1800(再按=取消)")
    try:
        while running:
            try:
                key = read_key()
            except Exception:
                break
            if key in ('a', 'b', 't'):
                with lock:
                    pending = {'a': 'off', 'b': 'on', 't': 'test'}[key]
            elif key == 'l':
                if arm is None:
                    print("\n[臂] 未连接, 'l' 无效")
                else:
                    print("\n>>> 收到臂初始化指令 'l'")
                    arm_init_event.set()
            elif key == 'k':
                if arm is None:
                    print("\n[臂] 未连接, 'k' 无效")
                else:
                    print("\n>>> 收到爬墙循环指令 'k'")
                    if fan_ramp_active:
                        print("[前扇] 渐进进行中, 先停止渐进...")
                        arm_stop.set()
                        while fan_ramp_active and running:
                            time.sleep(0.05)
                        arm_stop.clear()
                    if arm_loop_active:
                        print("[臂] 循环进行中, 停止当前循环并重启...")
                        arm_stop.set()
                        while arm_loop_active and running:
                            time.sleep(0.05)
                        arm_stop.clear()
                    arm_loop_event.set()
            elif key == 's':
                print("\n>>> 收到机械臂急停指令 's' (仅停臂, 脚本继续运行)")
                arm_stop.set()
                front_fan_cur = 1000
                if arm is not None:
                    all_tip_fans_off()   # 臂1~4 末端涵道风扇全关 (含记录复位)
                    try:
                        arm.fan_off(FRONT_FAN_L)
                        arm.fan_off(FRONT_FAN_R)
                        arm.stop()
                    except Exception:
                        pass
            elif key == 'f':
                if arm is None:
                    print("\n[前扇] 机械臂串口未连接, 'f' 无效")
                elif fan_ramp_active:
                    print("\n[前扇] 渐进进行中, 请先按 g 取消 (或按 s 急停)")
                else:
                    try:
                        if front_fan_cur > 1000:
                            arm.fan_off(FRONT_FAN_L)
                            arm.fan_off(FRONT_FAN_R)
                            front_fan_cur = 1000
                            print("\n>>> [前扇] 左/右前臂涵道风扇已关闭 (1000us)")
                        else:
                            arm.set_fan(FRONT_FAN_L, FRONT_FAN_HIGH)
                            arm.set_fan(FRONT_FAN_R, FRONT_FAN_HIGH)
                            front_fan_cur = FRONT_FAN_HIGH
                            print(f"\n>>> [前扇] 左/右前臂涵道风扇 -> {FRONT_FAN_HIGH}us "
                                  f"(再按 f 关闭)")
                    except Exception as e:
                        front_fan_cur = 1000
                        print(f"\n[前扇] 指令发送失败: {e}")
            elif key == 'g':
                if arm is None:
                    print("\n[前扇] 机械臂串口未连接, 'g' 无效")
                elif fan_ramp_active:
                    print("\n>>> [前扇] 取消渐进, 关闭双前扇")
                    arm_stop.set()   # do_fan_ramp 收尾时会关风扇并复位状态
                elif arm_loop_active:
                    print("\n[前扇] 爬墙循环进行中, 忽略 'g'")
                else:
                    print(f"\n>>> 收到前扇渐进指令 'g' "
                          f"({FAN_RAMP_START}->{FAN_RAMP_END}us)")
                    fan_ramp_event.set()
            elif key == 'q':
                running = False
                break
            time.sleep(0.05)
    finally:
        if restore_termios:
            restore_termios()


# ========== 阻塞式参数读写 (仅用于启动读取 / --selftest / 退出恢复) ==========
def _match_param_value(master, want_name, deadline):
    """在 deadline 前等一条名为 want_name 的 PARAM_VALUE, 返回其值; 超时返回 None"""
    while True:
        remain = deadline - time.time()
        if remain <= 0:
            return None
        msg = master.recv_match(type='PARAM_VALUE', blocking=True,
                                timeout=max(0.05, remain))
        if msg is None:
            return None
        name = msg.param_id
        if isinstance(name, bytes):
            name = name.decode('ascii', errors='ignore')
        if name.strip('\x00') == want_name:
            return msg.param_value


def get_param_blocking(master, name):
    """阻塞读一次 (调用方自行重试); 超时 READ_TIMEOUT, 读响应实测可能数秒"""
    master.mav.param_request_read_send(
        master.target_system, master.target_component,
        name.encode('ascii'), -1
    )
    return _match_param_value(master, name, time.time() + READ_TIMEOUT)


def set_param_blocking(master, name, value):
    for _ in range(MAX_RETRY):
        master.mav.param_set_send(
            master.target_system, master.target_component,
            name.encode('ascii'), float(value),
            mavutil.mavlink.MAV_PARAM_TYPE_REAL32
        )
        echo = _match_param_value(master, name, time.time() + VERIFY_TIMEOUT)
        if echo is not None and abs(echo - value) < 1e-4:
            return True
    return False


def run_blocking_selftest(master):
    """--selftest 模式: 不开 T265/键盘/机械臂, 阻塞式自检并打印每一步。
    9 步: 读 x3 -> 都设为测试值(回显验证) -> 都恢复(回显验证)。
    验证走 SET 回显: 协议规定 SET 后飞控必须广播已提交值, 回显即读回;
    不依赖实测不可靠的 PARAM_REQUEST_READ 响应 (仅前三步用它确认链路)。"""
    total = 3 * len(PARAMS)
    print(f"[自检] 阻塞模式: 读x{len(PARAMS)} -> 设测试值(回显) -> 恢复(回显)")
    results = []
    start = {}

    # ---- 步骤 1..3: 逐个读当前值 (各带重试) ----
    for i, p_ in enumerate(PARAMS):
        name = p_['name']
        v = None
        for _ in range(MAX_RETRY):
            v = get_param_blocking(master, name)
            if v is not None:
                break
        ok = v is not None
        if ok:
            start[name] = v
        print("[自检] %d/%d 读 %s ... " % (i + 1, total, name) +
              (f"OK ({v:.3f})" if ok else "FAIL"))
        results.append(ok)

    if all(results):
        # ---- 步骤 4..6: 都设为测试值, 回显验证 ----
        for i, p_ in enumerate(PARAMS):
            name = p_['name']
            ok = set_param_blocking(master, name, p_['off'])
            print("[自检] %d/%d 设 %s=%.3f (回显验证) ... %s" %
                  (len(PARAMS) + i + 1, total, name, p_['off'],
                   'OK' if ok else 'FAIL'))
            results.append(ok)

        # ---- 步骤 7..9: 都恢复原值, 回显验证 ----
        for i, p_ in enumerate(PARAMS):
            name = p_['name']
            ok = set_param_blocking(master, name, start[name])
            print("[自检] %d/%d 恢复 %s=%.3f (回显验证) ... %s" %
                  (2 * len(PARAMS) + i + 1, total, name, start[name],
                   'OK' if ok else 'FAIL'))
            results.append(ok)
    else:
        results += [False] * (2 * len(PARAMS))

    n = sum(results)
    restored = " ".join(f"{p_['short']}={fmt(start.get(p_['name']))}" for p_ in PARAMS)
    if n == total:
        print(f"[自检] 结果: PASS ({n}/{total}) —— 链路正常, 参数已恢复 {restored}")
    else:
        print(f"[自检] 结果: FAIL ({n}/{total}) —— 检查串口/波特率/固件版本")
    return n == total


# ========== 异步参数操作 (正常运行时使用, 不阻塞视觉流/机械臂) ==========
def _send_op(master, op):
    if op['kind'] == 'set':
        master.mav.param_set_send(
            master.target_system, master.target_component,
            op['name'].encode('ascii'), float(op['value']),
            mavutil.mavlink.MAV_PARAM_TYPE_REAL32
        )
    else:  # read
        master.mav.param_request_read_send(
            master.target_system, master.target_component,
            op['name'].encode('ascii'), -1
        )


def _complete_op(ok, echo):
    """当前操作结束: 按 tag 打印结果; 自检步骤累计, 齐了就出总结"""
    op = P['current_op']
    P['current_op'] = None
    if op is None:
        return
    tag = op['tag']

    if tag == 'toggle':
        if ok:
            key = 'note_off' if op.get('direction') == 'off' else 'note_on'
            note = NOTES.get((op['name'], key), '')
            print(f"\n>>> {op['label']} 成功: {op['name']} = {echo:.3f} {note}")
        else:
            print(f"\n>>> {op['label']} 失败! 状态未变, 可再按一次")

    elif tag == 'selftest':
        P['st_results'].append(bool(ok))
        if ok and op.get('store_start'):
            P['st_ctx'].setdefault('start', {})[op['name']] = echo
        tail = f" ({echo:.3f})" if (ok and echo is not None) else \
               (f" (读到 {echo})" if echo is not None else "")
        print(f"\n[自检] {op['label']} ... {'OK' if ok else 'FAIL'}{tail}")
        if len(P['st_results']) == P['st_total']:
            n = sum(P['st_results'])
            if n == P['st_total']:
                start = P['st_ctx'].get('start', {})
                restored = " ".join(f"{p_['short']}={fmt(start.get(p_['name']))}"
                                    for p_ in PARAMS)
                print(f"[自检] 结果: PASS ({n}/{P['st_total']}) —— 链路正常, "
                      f"参数已恢复 {restored}")
            else:
                print(f"[自检] 结果: FAIL ({n}/{P['st_total']}) —— "
                      f"检查串口/波特率/固件版本")
            P['st_running'] = False

    # tag == 'refresh': 静默, 只更新 P['cur'] 供状态行显示


def on_param_value(name, v):
    """drain 到一条受管参数的 PARAM_VALUE: 更新真值; 若在等回显则判定当前操作"""
    P['cur'][name] = v
    # 用"非测试值"的读数学习恢复目标 (避免把测试值学成原值)
    if P['on_fallback'][name] and abs(v - OFF_TARGET[name]) > 1e-4:
        P['on_value'][name] = v
        P['on_fallback'][name] = False

    op = P['current_op']
    if op is None or name != op['name']:
        return
    if op['kind'] == 'read':
        if op['expect'] is None or abs(v - op['expect']) < 1e-4:
            _complete_op(True, v)
    else:  # set: 回显值必须等于目标值; 不等于则继续等到超时再重试
        if abs(v - op['expect']) < 1e-4:
            _complete_op(True, v)


def process_param_ops(master):
    """主循环每次迭代调用: 空闲则从队列取一个操作发出; 超时则重试/判失败"""
    now = time.time()
    op = P['current_op']

    if op is None:
        while op_queue:
            op = op_queue.popleft()
            # 自检"恢复原值"步在此解析目标值: 优先测试开始时的读数,
            # 兜底当前已知真值, 再兜底 on_value, 保证一定能恢复
            if op['kind'] == 'set' and op['value'] is None:
                start = P['st_ctx'].get('start', {}).get(op['name'])
                if start is None:
                    start = P['cur'][op['name']]
                if start is None:
                    start = P['on_value'][op['name']]
                op['value'] = start
                op['expect'] = start
            timeout = READ_TIMEOUT if op['kind'] == 'read' else VERIFY_TIMEOUT
            op['attempt'] = 1
            _send_op(master, op)
            op['deadline'] = now + timeout
            P['current_op'] = op
            return
        return

    if now > op['deadline']:
        timeout = READ_TIMEOUT if op['kind'] == 'read' else VERIFY_TIMEOUT
        if op['attempt'] < MAX_RETRY:
            op['attempt'] += 1
            _send_op(master, op)
            op['deadline'] = now + timeout
            print(f"\n[重试 {op['attempt']}/{MAX_RETRY}] {op['label']}")
        else:
            _complete_op(False, None)


def enqueue_toggle(turn_off):
    """按 a/b: 三个参数依次入队 (逐个回显验证); a 进测试值, b 回原值"""
    for p_ in PARAMS:
        name = p_['name']
        target = p_['off'] if turn_off else P['on_value'][name]
        op_queue.append({
            'kind': 'set', 'name': name, 'value': target, 'expect': target,
            'tag': 'toggle', 'direction': 'off' if turn_off else 'on',
            'label': ('[a] ' if turn_off else '[b] ') +
                     ('测试值 ' if turn_off else '恢复 ') + name,
        })


def enqueue_selftest():
    """按 t: 异步自检 —— 9 步依次入队, 期间视觉/TIMESYNC/机械臂不中断。
    验证走 SET 回显 (协议规定 SET 后飞控必须广播已提交值, 回显即读回);
    不再依赖实测会延迟/丢弃的 PARAM_REQUEST_READ 响应。"""
    if P['st_running']:
        print("\n[自检] 已在进行中, 忽略")
        return
    P['st_running'] = True
    P['st_results'] = []
    P['st_ctx'] = {'start': {n: P['cur'][n] for n in TRACKED}}

    total = 3 * len(PARAMS)
    ops = []
    step = 0
    for p_ in PARAMS:                       # 读 x3
        step += 1
        ops.append({'kind': 'read', 'name': p_['name'], 'expect': None,
                    'store_start': True, 'tag': 'selftest',
                    'label': '%d/%d 读 %s' % (step, total, p_['name'])})
    for p_ in PARAMS:                       # 设测试值 x3
        step += 1
        ops.append({'kind': 'set', 'name': p_['name'], 'value': p_['off'],
                    'expect': p_['off'], 'tag': 'selftest',
                    'label': '%d/%d 设 %s=%.3f (回显验证)' %
                             (step, total, p_['name'], p_['off'])})
    for p_ in PARAMS:                       # 恢复 x3 (值出队时解析)
        step += 1
        ops.append({'kind': 'set', 'name': p_['name'], 'value': None,
                    'expect': 'start', 'tag': 'selftest',
                    'label': '%d/%d 恢复 %s (回显验证)' % (step, total, p_['name'])})

    P['st_total'] = len(ops)
    op_queue.extend(ops)
    print(f"\n[自检] 开始 (异步, 视觉流不中断): 读x{len(PARAMS)} -> "
          f"设测试值(回显) -> 恢复(回显)")


# ========== 四机械臂爬墙 (来自 climb4_wall_test.py, 独立线程执行) ==========
def arm_sleep(seconds: float) -> bool:
    """可被 s/退出 打断的睡眠; 被打断返回 False"""
    t0 = time.time()
    while time.time() - t0 < seconds:
        if arm_stop.is_set() or not running:
            return False
        time.sleep(0.05)
    return True


def arm_wait_move(timeout: float = MOVE_TIMEOUT_S) -> bool:
    """等待串口返回 RSP_DONE (移动完成)。支持中断与错误检查。"""
    t0 = time.time()
    while not arm._done.wait(0.05):
        if arm_stop.is_set() or not running:
            return False
        if time.time() - t0 > timeout:
            print("\n  ! [臂][超时] 等待舵机到达指定角度超时")
            return False

    if arm.last_error is not None:
        err_msg = arm.last_error
        print(f"\n  ! [臂][固件报错] 错误码 0x{err_msg:02X}，终止当前序列")
        arm.last_error = None
        arm_stop.set()
        return False
    return True


def move_all_arms(angles) -> bool:
    """四臂同时下发同一组关节角 (固件端同步插补, 同时到达), 等待完成。
    固件对一整次移动只回一个 RSP_DONE, 因此只需等待一次。"""
    for arm_num, _name in CLIMB_ARMS:
        arm.move_joint(arm_num, angles)
    return arm_wait_move()


def tip_fan_ramp(arm_num: int, target: int) -> bool:
    """臂端涵道风扇到 target (us): 贴墙启动到 1300, 离墙渐弱到 1000(关闭)。
    注意: 升油门不能慢爬 —— 电调低速蠕动会触发失速保护 (启动失败后锁死,
    直到油门回低位才允许重试), 表现为"渐变不转/直跳能转"。
    故升油门先直跳到 TIP_FAN_START 让电机转起来, 再补到 target;
    降油门方向保持渐变 (电机已在转, 无失速问题, 缓慢降更平顺)。
    可被 s 急停/退出打断; 被打断返回 False (s 会统一关闭)。"""
    fan = TIP_FAN_NO[arm_num]
    cur = tip_fan_cur[arm_num]
    if cur == target:
        return True

    if target > cur:
        # ---- 升油门: 直跳启动油门 -> 补到目标 ----
        try:
            if cur < TIP_FAN_START:
                print(f"  [臂端风扇] 臂{arm_num} {cur} -> {TIP_FAN_START}us (直跳启动)")
                arm.set_fan(fan, TIP_FAN_START)
                tip_fan_cur[arm_num] = TIP_FAN_START
                if not arm_sleep(TIP_FAN_START_HOLD_S): return False
            if target != TIP_FAN_START:
                print(f"  [臂端风扇] 臂{arm_num} {TIP_FAN_START} -> {target}us")
                arm.set_fan(fan, target)
                tip_fan_cur[arm_num] = target
        except Exception as e:
            print(f"\n  ! [臂端风扇] 臂{arm_num} 下发失败: {e}")
            return False
        return True

    # ---- 降油门: 渐变至 target ----
    step = -TIP_FAN_STEP
    print(f"  [臂端风扇] 臂{arm_num} {cur} -> {target}us "
          f"(每 {TIP_FAN_INTERVAL}s {step}us)")
    t = cur
    while t != target:
        if arm_stop.is_set() or not running:
            return False
        t += step
        if t < target:
            t = target
        try:
            arm.set_fan(fan, t)
        except Exception as e:
            print(f"\n  ! [臂端风扇] 臂{arm_num} 下发失败: {e}")
            return False
        tip_fan_cur[arm_num] = t
        if t != target and not arm_sleep(TIP_FAN_INTERVAL):
            return False
    return True


def all_tip_fans_off():
    """关闭全部四个臂端涵道风扇并复位记录 (急停/退出时调用)"""
    if arm is None:
        return
    for arm_num, fan in TIP_FAN_NO.items():
        try:
            arm.fan_off(fan)
        except Exception:
            pass
        tip_fan_cur[arm_num] = 1000


def do_arm_init():
    """[l 指令] 初始化: 四臂同时移动至 G1 (低位贴墙), 到位后臂端风扇渐大至 1300 吸附"""
    print("\n=== 机械臂初始化 (l) ===")
    print(f"[臂] 同时移动 臂1(左上)/臂2(右上)/臂3(左下)/臂4(右下) 至: {G1_DOWN_ATTACH}")
    if move_all_arms(G1_DOWN_ATTACH):
        print("[臂] ✓ 初始化位置到达成功, 臂端风扇启动至 "
              f"{TIP_FAN_ATTACH}us 吸附 ...")
        for arm_num, _name in CLIMB_ARMS:
            if not tip_fan_ramp(arm_num, TIP_FAN_ATTACH):
                break
    else:
        print("[臂] ✕ 初始化移动中断或超时")


def step_arm_lift(arm_num: int, arm_name: str) -> bool:
    """单臂爬墙步: 关闭吸附(风扇渐弱) -> 离墙(G1->G2) -> 抬升(G2->G3)
    -> 贴墙(G3->G4) -> 风扇渐大至 1300 吸附。
    其余三个臂保持不动作为支撑。"""
    # 0. 离墙前解除吸附: 臂端风扇渐弱至 1000us (关闭)
    if not tip_fan_ramp(arm_num, 1000): return False

    # 1. 离墙: 低位贴墙(G1) -> 低位离墙(G2)
    print(f"  [离墙] 臂{arm_num}({arm_name}) 移动至 {G2_DOWN_DETACH}...")
    arm.move_joint(arm_num, G2_DOWN_DETACH)
    if not arm_wait_move(): return False
    if not arm_sleep(MOVE_SETTLE_S): return False

    # 2. 抬升: 低位离墙(G2) -> 高位离墙(G3)
    print(f"  [抬升] 臂{arm_num}({arm_name}) 移动至 {G3_UP_DETACH}...")
    arm.move_joint(arm_num, G3_UP_DETACH)
    if not arm_wait_move(): return False
    if not arm_sleep(MOVE_SETTLE_S): return False

    # 3. 贴墙: 高位离墙(G3) -> 高位贴墙(G4)
    print(f"  [贴墙] 臂{arm_num}({arm_name}) 移动至 {G4_UP_ATTACH}...")
    arm.move_joint(arm_num, G4_UP_ATTACH)
    if not arm_wait_move(): return False
    if not arm_sleep(MOVE_SETTLE_S): return False

    # 4. 贴墙完成: 臂端风扇渐大至 1300us 吸附
    if not tip_fan_ramp(arm_num, TIP_FAN_ATTACH): return False

    print(f"  臂{arm_num}({arm_name}) 完成，等待 {STEP_SETTLE_S}s ...")
    return arm_sleep(STEP_SETTLE_S)


def run_arm_loop(max_cycles: int = 0):
    """[k 指令] Step 1~4 依次抬四个臂 (左上->右上->左下->右下) -> Step 5 四臂同时向下。
    定高解除/恢复不在这里做 —— 需要时先按 a 进 Z 测试态, 结束后按 b 恢复。"""
    global arm_loop_active
    arm_loop_active = True
    cycle = 0

    try:
        while not arm_stop.is_set() and running:
            cycle += 1
            print(f"\n================ [爬墙循环第 {cycle} 次] ================")

            # ---- Step 1~4: 左上 -> 右上 -> 左下 -> 右下 依次 离墙/抬升/贴墙 ----
            for step, (arm_num, arm_name) in enumerate(CLIMB_ARMS, start=1):
                print(f"\n--> Step {step}: 臂{arm_num}({arm_name}) 离墙 -> 抬升 -> 贴墙")
                if not step_arm_lift(arm_num, arm_name):
                    return

            # ---- Step 5: 四臂同时向下 (G4 -> G1) ----
            print(f"\n--> Step 5: 四个臂同时向下移动至 {G1_DOWN_ATTACH}")
            if not move_all_arms(G1_DOWN_ATTACH):
                break

            print(f"  Step 5 完成，等待 {CYCLE_SETTLE_S}s ...")
            if not arm_sleep(CYCLE_SETTLE_S):
                break

            if max_cycles > 0 and cycle >= max_cycles:
                print(f"\n[臂] 到达指定循环次数 ({max_cycles})，停止循环")
                break
    finally:
        arm_loop_active = False


def do_fan_ramp():
    """[g 指令] 前臂双涵道风扇渐进: FAN_RAMP_START -> FAN_RAMP_END。
    可被 s 急停 / 再按 g / q 退出打断; 打断即关闭双前扇, 完成则保持终点油门。"""
    global fan_ramp_active, front_fan_cur
    fan_ramp_active = True
    interrupted = False
    print(f"\n=== [前扇] 渐进测试: {FAN_RAMP_START} -> {FAN_RAMP_END}us "
          f"(每 {FAN_RAMP_INTERVAL}s +{FAN_RAMP_STEP}us, 再按 g 取消) ===")
    try:
        throttle = FAN_RAMP_START
        while True:
            if arm_stop.is_set() or not running:
                interrupted = True
                break
            arm.set_fan(FRONT_FAN_L, throttle)
            arm.set_fan(FRONT_FAN_R, throttle)
            front_fan_cur = throttle
            print(f"[前扇] {throttle}us")
            if throttle >= FAN_RAMP_END:
                break
            if not arm_sleep(FAN_RAMP_INTERVAL):
                interrupted = True
                break
            throttle = min(throttle + FAN_RAMP_STEP, FAN_RAMP_END)
    finally:
        fan_ramp_active = False
        if interrupted:
            try:
                arm.fan_off(FRONT_FAN_L)
                arm.fan_off(FRONT_FAN_R)
            except Exception:
                pass
            front_fan_cur = 1000
            print("\n[前扇] 渐进被打断, 双前扇已关闭")
        else:
            print(f"\n[前扇] ✓ 渐进完成, 保持 {FAN_RAMP_END}us (按 f/s 可关)")


def arm_worker():
    """机械臂后台线程: 消费 l/k/g 事件, 阻塞动作不占用主循环"""
    while running:
        if arm is not None and arm_init_event.is_set():
            arm_init_event.clear()
            arm_stop.clear()
            try:
                do_arm_init()
            except Exception as e:
                print(f"\n[臂] 初始化异常: {e}")
        if arm is not None and arm_loop_event.is_set():
            arm_loop_event.clear()
            arm_stop.clear()
            try:
                run_arm_loop(ARM_CYCLES)
            except Exception as e:
                print(f"\n[臂] 爬墙循环异常: {e}")
        if arm is not None and fan_ramp_event.is_set():
            fan_ramp_event.clear()
            arm_stop.clear()
            try:
                do_fan_ramp()
            except Exception as e:
                print(f"\n[前扇] 渐进异常: {e}")
        time.sleep(0.05)


def find_arm_port(exclude):
    """自动查找机械臂 Arduino 串口, 排除飞控口; 找不到返回 None"""
    try:
        from serial.tools import list_ports
    except Exception:
        return None
    for p in list_ports.comports():
        if p.device == exclude:
            continue
        dev = (p.device or '').lower()
        desc = (p.description or '').lower()
        if ('ttyacm' in dev or 'ttyusb' in dev or 'arduino' in desc
                or 'ch340' in desc or 'usb serial' in desc):
            return p.device
    return None


# ========== MAVLink 接收 ==========
def drain_messages(master):
    """非阻塞排空接收缓冲: 应答 TIMESYNC; 捕获受管参数的 PARAM_VALUE"""
    while True:
        msg = master.recv_match(blocking=False)
        if msg is None:
            break
        mtype = msg.get_type()
        if mtype == 'TIMESYNC':
            master.mav.timesync_send(int(time.time_ns()), msg.ts1)
        elif mtype == 'PARAM_VALUE':
            name = msg.param_id
            if isinstance(name, bytes):
                name = name.decode('ascii', errors='ignore')
            name = name.strip('\x00')
            if name in TRACKED:
                on_param_value(name, msg.param_value)


def param_state_str():
    parts = []
    known = 0
    for p_ in PARAMS:
        v = P['cur'][p_['name']]
        parts.append(f"{p_['short']}={fmt(v)}")
        if v is not None:
            known += 1
    if known == 0:
        state = "?"
    elif mod_count() == len(PARAMS):
        state = "测试态(不回高+无I+软P)"
    elif mod_count() == 0:
        state = "原值态"
    else:
        state = "部分改(%d/%d)" % (mod_count(), len(PARAMS))
    arm_state = "臂:循环中" if arm_loop_active else ("臂:已连接" if arm else "臂:无")
    if fan_ramp_active:
        front_state = f"前扇:渐进{front_fan_cur}"
    elif front_fan_cur > 1000:
        front_state = f"前扇:{front_fan_cur}"
    else:
        front_state = "前扇:关"
    return " ".join(parts) + f" [{state}] {arm_state} {front_state}"


# ========== 主流程 ==========
def main():
    global running, pending, arm, ARM_CYCLES

    parser = argparse.ArgumentParser(description="T265桥接 + Z参数开关 + 四机械臂爬墙")
    parser.add_argument('--selftest', action='store_true',
                        help='纯参数自检, 不开相机/机械臂')
    parser.add_argument('--no-arm', action='store_true', help='禁用机械臂')
    parser.add_argument('--arm-port', default=None,
                        help='机械臂串口, 默认依次探测 ttyUSB1/serial0/ttyAMA0, '
                             '再自动探测 USB 转串口')
    parser.add_argument('--cycles', type=int, default=0,
                        help='爬墙循环次数, 0 表示无限循环')
    args = parser.parse_args()
    ARM_CYCLES = args.cycles

    # ---- 连接飞控 ----
    print(f"连接飞控 {MAVLINK_ADDR} @ {MAVLINK_BAUD} ...")
    master = mavutil.mavlink_connection(MAVLINK_ADDR, baud=MAVLINK_BAUD)
    master.wait_heartbeat()
    print(f"已连接: sysid={master.target_system} compid={master.target_component}")

    # ---- --selftest: 纯自检, 不开 T265/键盘/机械臂, 跑完就退出 ----
    if args.selftest:
        try:
            ok = run_blocking_selftest(master)
        finally:
            master.close()
        sys.exit(0 if ok else 1)

    # ---- 启动时逐个读原值: 恢复目标 = 读到的值, 读不到用兜底 ----
    for p_ in PARAMS:
        name = p_['name']
        cur = get_param_blocking(master, name)
        if cur is None:
            print(f"[警告] 读不到 {name}, 恢复值先用兜底 {FALLBACK_ON[name]}, "
                  f"稍后读到真值会自动更新")
        else:
            P['cur'][name] = cur
            print(f"当前 {name} = {cur:.3f}")
            if abs(cur - p_['off']) < 1e-4:
                print(f"!! 警告: {name} 已处于测试值 {p_['off']:.3f} —— 很可能"
                      f"上次异常退出没恢复, 按 b 可恢复正常 !!")
            else:
                P['on_value'][name] = cur
                P['on_fallback'][name] = False

    # ---- 启动 T265 (失败则降级为纯参数开关+机械臂模式, 桥接功能关闭) ----
    pipe = None
    try:
        import pyrealsense2 as rs
        pipe = rs.pipeline()
        cfg = rs.config()
        cfg.enable_stream(rs.stream.pose)
        pipe.start(cfg)
        print("T265 已启动，等待位姿数据...")
    except Exception as e:
        print(f"[警告] T265 启动失败 ({e}), 降级为纯参数开关+机械臂模式 (无视觉桥接)")
        pipe = None

    # ---- 连接机械臂 (失败则降级, l/k/s 提示未连接) ----
    if not args.no_arm:
        port = args.arm_port
        if port is None:
            # 依次探测候选口: USB-TTL 接 Arduino 18/19 时是 ttyUSBx,
            # 树莓派 GPIO 串口直连 18/19 时是 serial0/ttyAMA0
            for cand in ARM_PORT_CANDIDATES:
                if os.path.exists(cand):
                    port = cand
                    break
        if port is None:
            port = find_arm_port(exclude=MAVLINK_ADDR)
        if port is None:
            print("[警告] 未找到机械臂串口, 机械臂功能禁用 (可用 --arm-port 指定)")
        else:
            try:
                from arm_client import ArmClient
                print(f"连接机械臂 {port} @ {ARM_BAUD} ...")
                arm = ArmClient(port, baud=ARM_BAUD)
                print("机械臂已连接")
            except Exception as e:
                print(f"[警告] 机械臂连接失败 ({e}), 机械臂功能禁用")
                arm = None
    else:
        print("[模式] --no-arm: 机械臂功能禁用")

    # ---- 启动键盘监听 + 机械臂后台线程 ----
    t = threading.Thread(target=keyboard_listener, daemon=True)
    t.start()
    t_arm = threading.Thread(target=arm_worker, daemon=True)
    t_arm.start()

    print(f"[发送] 视觉定位 {SEND_RATE_HZ:.0f}Hz" if pipe else "[模式] 无视觉桥接")

    # ---- 定时器 ----
    last_vision_send = 0.0
    last_refresh     = time.time()
    last_status      = 0.0

    send_count = 0
    drop_count = 0

    initial_pos   = None
    inv_init_quat = None

    try:
        while running:
            # ---- 取帧 / 无相机走 sleep ----
            frames = None
            if pipe is not None:
                # 100ms 超时: 即使相机帧没来, 循环也能准时走,
                # 保证 TIMESYNC 应答和参数操作不停摆
                try:
                    frames = pipe.wait_for_frames(100)
                except RuntimeError:
                    frames = None
            else:
                time.sleep(0.05)

            # ---- 应答 TIMESYNC + 捕获 PARAM_VALUE ----
            drain_messages(master)

            # ---- 消费键盘请求 -> 入队参数操作 ----
            with lock:
                req = pending
                pending = None
            if req == 'off':
                enqueue_toggle(turn_off=True)
            elif req == 'on':
                enqueue_toggle(turn_off=False)
            elif req == 'test':
                enqueue_selftest()

            # ---- 推进参数操作状态机 (发/重试/判超时) ----
            process_param_ops(master)

            now = time.time()

            # ---- 空闲时周期读回三个参数, 让状态行反映飞控真值 ----
            if now - last_refresh > REREAD_INTERVAL:
                last_refresh = now
                if P['current_op'] is None and not op_queue:
                    for name in TRACKED:
                        op_queue.append({'kind': 'read', 'name': name,
                                         'expect': None,
                                         'tag': 'refresh', 'label': '刷新'})

            # ---- 无相机模式: 定期刷状态行; 视觉模式的状态行走下面 30Hz ----
            if pipe is None:
                if now - last_status > 2.0:
                    last_status = now
                    print(f"\r[状态] {param_state_str()}   ", end="")
                continue

            if frames is None:
                continue
            pose_frame = frames.get_pose_frame()
            if not pose_frame:
                continue
            pose = pose_frame.get_pose_data()

            # ---- 视觉定位降频 ----
            if now - last_vision_send < 1.0 / SEND_RATE_HZ:
                continue
            last_vision_send = now

            # ---- 跟踪质量检查：丢失跟踪时不发，避免毒化 EKF ----
            if pose.tracker_confidence < MIN_CONFIDENCE:
                drop_count += 1
                print(f"\r[警告] 跟踪置信度低({pose.tracker_confidence})，暂停发送视觉数据，已丢 {drop_count} 帧",
                      end="")
                continue

            # ---- 坐标转换 ----
            pos = remap_pos(np.array([
                pose.translation.x,
                pose.translation.y,
                pose.translation.z
            ]))
            quat = remap_quat(np.array([
                pose.rotation.x,
                pose.rotation.y,
                pose.rotation.z,
                pose.rotation.w
            ]))

            if initial_pos is None:
                initial_pos = pos.copy()
                inv_init_quat = quat_inverse(quat.copy())
                rel_euler = quat_to_euler_rad(quat)
                print("基准已设定")
                print(f"初始位置: X前={initial_pos[0]:.3f} "
                      f"Y右={initial_pos[1]:.3f} Z下={initial_pos[2]:.3f} m")
                print(f"初始姿态: Roll={rel_euler[0]:.1f} "
                      f"Pitch={rel_euler[1]:.1f} Yaw={rel_euler[2]:.1f}°")
                continue

            quat = quat_mul(inv_init_quat, quat)
            roll, pitch, yaw = quat_to_euler_rad(quat)

            # ---- 发送 VISION_POSITION_ESTIMATE ----
            # 协方差第一元素置 NaN -> PX4 使用 EKF2_EVP_NOISE / EKF2_EVA_NOISE 默认噪声
            cov = [float('nan')] * 21
            master.mav.vision_position_estimate_send(
                int(now * 1e6),                  # UNIX 时间戳(us)，靠 TIMESYNC 对齐
                float(pos[0]), float(pos[1]), float(pos[2]),
                float(roll), float(pitch), float(yaw),
                cov
            )
            send_count += 1

            print(
                f"\r已发送 {send_count} | "
                f"X前={pos[0]:+.3f} Y右={pos[1]:+.3f} Z下={pos[2]:+.3f} m | "
                f"R={math.degrees(roll):+.1f} P={math.degrees(pitch):+.1f} "
                f"Y={math.degrees(yaw):+.1f}° | conf={pose.tracker_confidence} | "
                f"{param_state_str()}",
                end=""
            )

    except KeyboardInterrupt:
        print("\n停止。")

    finally:
        running = False
        t.join(timeout=1.0)     # 让键盘线程恢复终端设置

        # ---- 机械臂安全清理: 中断动作 -> 关双风扇 -> 急停 ----
        arm_stop.set()
        t_arm.join(timeout=2.0)
        if arm is not None:
            try:
                print("\n机械臂清理: 关风扇(含臂端/前臂涵道) + 急停 ...")
                arm.fan_off(FRONT_FAN_L)
                arm.fan_off(FRONT_FAN_R)
                arm.stop()           # 固件 STOP 会 fanOff(1~6) 兜底
                arm.close()
            except Exception:
                pass

        # ---- 退出保险: 哪个参数处于测试值就自动恢复哪个 ----
        for p_ in PARAMS:
            name = p_['name']
            v = P['cur'][name]
            if v is not None and abs(v - p_['off']) < 1e-4 \
                    and abs(p_['off'] - P['on_value'][name]) > 1e-4:
                print(f"\n退出前自动恢复 {name} 到 {P['on_value'][name]:.3f} ...")
                if set_param_blocking(master, name, P['on_value'][name]):
                    print(f"{name} 已恢复。")
                else:
                    print(f"!! {name} 自动恢复失败! 请重跑本脚本按 b, 或在 QGC 把 "
                          f"{name} 改回 {P['on_value'][name]:.3f} !!")
        if pipe is not None:
            pipe.stop()
        master.close()


if __name__ == '__main__':
    main()
