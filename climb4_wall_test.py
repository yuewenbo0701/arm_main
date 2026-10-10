#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
四机械臂 · 爬墙测试脚本 (climb4_wall_test.py)
================================================
在 climb4_test.py 基础上改为完整的 4 关节角步态 (从根部到末端 4 个角度全部下发)。
本脚本只控制四根机械臂关节 + 臂端涵道风扇(吸附), 不含飞控/T265/前臂风扇。

机械臂编号与 Arduino 引脚 (从根部到末端 4 个关节, 观察者面对机头方向):
  臂1 = 左上: 引脚 4, 5, 6, 7    臂端涵道风扇 = 引脚 8  (协议风扇号 1)
  臂2 = 右上: 引脚 10, 11, 12, 13  臂端涵道风扇 = 引脚 9  (协议风扇号 2)
  臂3 = 左下: 引脚 22, 23, 24, 25  臂端涵道风扇 = 引脚 26 (协议风扇号 5)
  臂4 = 右下: 引脚 28, 29, 30, 31  臂端涵道风扇 = 引脚 27 (协议风扇号 6)
四臂使用完全相同的关节角步态。

步态四组关节角 (度, 根部 -> 末端):
  G1 = {-30,  10,  10, -20}   低位贴墙 (循环终点"向下"的位置, INIT)
  G2 = {-30, -10, -10,  20}   低位离墙
  G3 = { 30, -10, -10,  20}   高位离墙 (抬升)
  G4 = { 30,  10,  10, -20}   高位贴墙

臂端涵道风扇联动:
  贴墙到位后 -> 直跳 1200us 启动再补到 1300us 吸附 (慢爬会触发电调失速保护锁死)
  离墙之前   -> 渐弱至 1000us 关闭
  s/q 急停退出 -> 固件 STOP 统一关闭全部风扇

控制命令:
  l  初始化: 四个臂同时移动至 G1 (低位贴墙), 到位后四臂风扇渐大吸附
  k  开始爬墙循环 (Step 1 -> 2 -> 3 -> 4 -> 5)
  s  急停并退出脚本 (发送 CMD_STOP)
  q  退出脚本 (同 s)

循环逻辑 (每轮):
  Step 1: 左上(臂1) 风扇关 -> 离墙(G2) -> 抬升(G3) -> 贴墙(G4) -> 风扇吸附
  Step 2: 右上(臂2) 同上
  Step 3: 左下(臂3) 同上
  Step 4: 右下(臂4) 同上
  Step 5: 四个臂同时向下 (G4 -> G1), 风扇保持吸附

键盘单键即触发, 无需回车: l / k / s / q

串口 (与 t265_bridge_z_arm.py 一致):
  固件链路走 Arduino Serial1 (Mega2560 引脚18=TX1 / 19=RX1) @115200,
  不是 Arduino 的 USB 口! 树莓派侧依次探测:
    /dev/ttyUSB1 (USB-TTL 接 18/19) -> /dev/serial0 -> /dev/ttyAMA0 (GPIO 直连)
  都找不到再自动探测其它 USB 串口 (自动探测排除 ttyUSB0 = Pixhawk 飞控)。
  也可用 --port 显式指定。

用法:
  python3 climb4_wall_test.py
  python3 climb4_wall_test.py --port /dev/ttyUSB1
  python3 climb4_wall_test.py --cycles 5
"""

import argparse
import os
import sys
import threading
import time

# arm_client.py 在 rpi/ 子目录
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), 'rpi'))

from arm_client import ArmClient

# ----- 串口候选 (与 t265_bridge_z_arm.py 一致) -----
# 固件链路是 Arduino Serial1 (引脚18=TX1 / 19=RX1), 不是 Arduino 的 USB 口!
# 树莓派侧: USB-TTL 接 18/19 时是 ttyUSBx; 树莓派 GPIO 串口直连时是 serial0/ttyAMA0
# 注意: ttyUSB0 通常是 Pixhawk 飞控, 绝不能连!
ARM_PORT_CANDIDATES = ['/dev/ttyUSB1', '/dev/serial0', '/dev/ttyAMA0']
ARM_BAUD = 115200

# ----- 步态参数: 四组关节角 (度, 根部 -> 末端, 4 个角全部下发) -----
G1_DOWN_ATTACH    = [-30.0,  10.0,  10.0, -20.0]   # 低位贴墙 (INIT / "向下"终点)
G2_DOWN_DETACH    = [-30.0, -10.0, -10.0,  20.0]   # 低位离墙
G3_UP_DETACH      = [ 30.0, -10.0, -10.0,  20.0]   # 高位离墙 (抬升)
G4_UP_ATTACH      = [ 30.0,  10.0,  10.0, -20.0]   # 高位贴墙

# ----- 四臂定义: (协议臂号, 中文名) —— 循环动作顺序 -----
ARMS = [
    (1, '左上'),
    (2, '右上'),
    (3, '左下'),
    (4, '右下'),
]

# ----- 臂端涵道风扇 (贴墙吸附): 面对机头方向 -----
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

# ----- 时序参数 -----
MOVE_SETTLE_S  = 0.5    # 每段移动到位后的稳定等待
STEP_SETTLE_S  = 1.0    # 每个臂完成"离墙->抬升->贴墙"后的等待
CYCLE_SETTLE_S = 1.0    # 每轮循环结束后的等待
MOVE_TIMEOUT_S = 30     # 等待 RSP_DONE 超时

# 线程通信与控制标志
# stop_flag: 中断当前动作/循环 (s 急停 / k 重启循环时置位)
# quit_flag: 退出整个程序 (s / q 置位)
stop_flag        = threading.Event()
quit_flag        = threading.Event()
start_init_event = threading.Event()
start_loop_event = threading.Event()
loop_active      = False
tip_fan_cur      = {1: 1000, 2: 1000, 3: 1000, 4: 1000}   # 各臂末端风扇当前油门 (us)


def sleep_interruptible(seconds: float) -> bool:
    """可被打断的睡眠等待函数。若期间收到停止信号，立即返回 False。"""
    t0 = time.time()
    while time.time() - t0 < seconds:
        if stop_flag.is_set() or quit_flag.is_set():
            return False
        time.sleep(0.05)
    return True


def tip_fan_ramp(arm_client: ArmClient, arm_num: int, target: int) -> bool:
    """臂端涵道风扇到 target (us): 贴墙启动到 1300, 离墙渐弱到 1000(关闭)。
    注意: 升油门不能慢爬 —— 电调低速蠕动会触发失速保护 (启动失败后锁死,
    直到油门回低位才允许重试), 表现为"渐变不转/直跳能转"。
    故升油门先直跳到 TIP_FAN_START 让电机转起来, 再补到 target;
    降油门方向保持渐变 (电机已在转, 无失速问题, 缓慢降更平顺)。
    可被打断; 被打断返回 False (s/q 退出时固件 STOP 会统一关闭全部风扇)。"""
    fan = TIP_FAN_NO[arm_num]
    cur = tip_fan_cur[arm_num]
    if cur == target:
        return True

    if target > cur:
        # ---- 升油门: 直跳启动油门 -> 补到目标 ----
        if cur < TIP_FAN_START:
            print(f"  [臂端风扇] 臂{arm_num} {cur} -> {TIP_FAN_START}us (直跳启动)")
            arm_client.set_fan(fan, TIP_FAN_START)
            tip_fan_cur[arm_num] = TIP_FAN_START
            if not sleep_interruptible(TIP_FAN_START_HOLD_S): return False
        if target != TIP_FAN_START:
            print(f"  [臂端风扇] 臂{arm_num} {TIP_FAN_START} -> {target}us")
            arm_client.set_fan(fan, target)
            tip_fan_cur[arm_num] = target
        return True

    # ---- 降油门: 渐变至 target ----
    print(f"  [臂端风扇] 臂{arm_num} {cur} -> {target}us "
          f"(每 {TIP_FAN_INTERVAL}s -{TIP_FAN_STEP}us)")
    t = cur
    while t != target:
        if stop_flag.is_set() or quit_flag.is_set():
            return False
        t -= TIP_FAN_STEP
        if t < target:
            t = target
        arm_client.set_fan(fan, t)
        tip_fan_cur[arm_num] = t
        if t != target and not sleep_interruptible(TIP_FAN_INTERVAL):
            return False
    return True


def wait_for_move(arm_client: ArmClient, timeout: float = MOVE_TIMEOUT_S) -> bool:
    """等待串口返回 RSP_DONE (移动完成)。支持中断与错误检查。"""
    t0 = time.time()
    while not arm_client._done.wait(0.05):
        if stop_flag.is_set() or quit_flag.is_set():
            return False
        if time.time() - t0 > timeout:
            print("  ! [超时] 等待舵机到达指定角度超时")
            return False

    if arm_client.last_error is not None:
        err_msg = arm_client.last_error
        print(f"  ! [固件报错] 错误码 0x{err_msg:02X}，终止当前序列")
        arm_client.last_error = None  # 复位错误
        stop_flag.set()
        return False
    return True


def listen_command():
    """后台线程：单键解析键盘输入 (l / k / s / q)，无需回车 (与 t265_bridge_z_arm 一致)"""
    global loop_active

    restore_termios = None
    try:
        import msvcrt                                  # Windows
        def read_key():
            if msvcrt.kbhit():
                return msvcrt.getch().decode('utf-8', errors='ignore').lower()
            return None
    except ImportError:                                # Linux / 树莓派 / macOS
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

    try:
        while not quit_flag.is_set():
            try:
                key = read_key()
            except Exception:
                break
            if key in ('s', 'q'):
                print(f"\n>>> 收到急停/退出指令 '{key}'")
                stop_flag.set()
                quit_flag.set()
                return
            elif key == 'l':
                print("\n>>> 收到初始化指令 'l'")
                start_init_event.set()
            elif key == 'k':
                print("\n>>> 收到循环启动指令 'k'")
                if loop_active:
                    print("循环正在进行中，重启循环...")
                    stop_flag.set()
                    while loop_active and not quit_flag.is_set():
                        time.sleep(0.05)
                    stop_flag.clear()
                start_loop_event.set()
            time.sleep(0.05)
    finally:
        if restore_termios:
            restore_termios()


def find_arm_port(exclude='/dev/ttyUSB0'):
    """自动查找机械臂 Arduino 串口, 排除飞控口 (默认 ttyUSB0 = Pixhawk); 找不到返回 None"""
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


def move_all_arms(arm_client: ArmClient, angles) -> bool:
    """四个臂同时下发同一组关节角 (固件端同步插补, 同时到达), 等待完成。"""
    for arm_num, _name in ARMS:
        arm_client.move_joint(arm_num, angles)
    return wait_for_move(arm_client)


def do_init(arm_client: ArmClient):
    """
    [L 指令逻辑] 初始化：
    - 四个臂同时移动至 G1 (低位贴墙) {-30, 10, 10, -20}
    - 到位后四臂末端涵道风扇渐大至 1300us 吸附
    """
    print("\n=== 执行初始化 (L) ===")
    print(f"同时移动 臂1(左上)/臂2(右上)/臂3(左下)/臂4(右下) 至: {G1_DOWN_ATTACH}")

    if move_all_arms(arm_client, G1_DOWN_ATTACH):
        print(f"✓ 初始化位置到达成功, 臂端风扇启动至 {TIP_FAN_ATTACH}us 吸附 ...")
        for arm_num, _name in ARMS:
            if not tip_fan_ramp(arm_client, arm_num, TIP_FAN_ATTACH):
                break
    else:
        print("✕ 初始化移动中断或超时")


def step_arm_lift(arm_client: ArmClient, arm_num: int, arm_name: str) -> bool:
    """
    单臂爬墙步骤: 风扇渐弱关 -> 离墙(G2) -> 抬升(G3) -> 贴墙(G4) -> 风扇渐大吸附
    其余三个臂保持不动作为支撑。
    """
    # 0. 离墙前解除吸附: 臂端风扇渐弱至 1000us (关闭)
    if not tip_fan_ramp(arm_client, arm_num, 1000): return False

    # 1. 离墙: 低位贴墙(G1) -> 低位离墙(G2)
    print(f"  [离墙] 臂{arm_num}({arm_name}) 移动至 {G2_DOWN_DETACH}...")
    arm_client.move_joint(arm_num, G2_DOWN_DETACH)
    if not wait_for_move(arm_client): return False
    if not sleep_interruptible(MOVE_SETTLE_S): return False

    # 2. 抬升: 低位离墙(G2) -> 高位离墙(G3)
    print(f"  [抬升] 臂{arm_num}({arm_name}) 移动至 {G3_UP_DETACH}...")
    arm_client.move_joint(arm_num, G3_UP_DETACH)
    if not wait_for_move(arm_client): return False
    if not sleep_interruptible(MOVE_SETTLE_S): return False

    # 3. 贴墙: 高位离墙(G3) -> 高位贴墙(G4)
    print(f"  [贴墙] 臂{arm_num}({arm_name}) 移动至 {G4_UP_ATTACH}...")
    arm_client.move_joint(arm_num, G4_UP_ATTACH)
    if not wait_for_move(arm_client): return False
    if not sleep_interruptible(MOVE_SETTLE_S): return False

    # 4. 贴墙完成: 臂端风扇渐大至 1300us 吸附
    if not tip_fan_ramp(arm_client, arm_num, TIP_FAN_ATTACH): return False

    print(f"  臂{arm_num}({arm_name}) 完成，等待 {STEP_SETTLE_S}s ...")
    return sleep_interruptible(STEP_SETTLE_S)


def run_loop(arm_client: ArmClient, max_cycles: int = 0):
    """[K 指令逻辑] Step 1~4 依次抬四个臂 -> Step 5 四臂同时向下"""
    global loop_active
    loop_active = True
    cycle = 0

    try:
        while not stop_flag.is_set() and not quit_flag.is_set():
            cycle += 1
            print(f"\n================ [爬墙循环第 {cycle} 次] ================")

            # ---- Step 1~4: 左上 -> 右上 -> 左下 -> 右下 依次 离墙/抬升/贴墙 ----
            for step, (arm_num, arm_name) in enumerate(ARMS, start=1):
                print(f"\n--> Step {step}: 臂{arm_num}({arm_name}) 离墙 -> 抬升 -> 贴墙")
                if not step_arm_lift(arm_client, arm_num, arm_name):
                    return

            # ---- Step 5: 四个臂同时向下 (G4 -> G1) ----
            print(f"\n--> Step 5: 四个臂同时向下移动至 {G1_DOWN_ATTACH}")
            if not move_all_arms(arm_client, G1_DOWN_ATTACH):
                break

            print(f"  Step 5 完成，等待 {CYCLE_SETTLE_S}s ...")
            if not sleep_interruptible(CYCLE_SETTLE_S):
                break

            if max_cycles > 0 and cycle >= max_cycles:
                print(f"\n到达要求的指定循环次数 ({max_cycles})，停止循环")
                break

    finally:
        loop_active = False


def main():
    parser = argparse.ArgumentParser(description="四机械臂爬墙测试 (climb4_wall_test)")
    parser.add_argument("--port", default=None,
                        help="机械臂串口, 默认依次探测 ttyUSB1/serial0/ttyAMA0, "
                             "再自动探测其它 USB 串口 (排除 ttyUSB0 飞控)")
    parser.add_argument("--cycles", type=int, default=0, help="运行循环次数, 0 表示无限循环")
    args = parser.parse_args()

    # ---- 机械臂串口选择 (与 t265_bridge_z_arm.py 一致: 候选口优先, 避开飞控) ----
    port = args.port
    if port is None:
        for cand in ARM_PORT_CANDIDATES:
            if os.path.exists(cand):
                port = cand
                break
    if port is None:
        port = find_arm_port(exclude='/dev/ttyUSB0')   # ttyUSB0 = Pixhawk 飞控, 不能连
    if not port:
        print("✕ 未找到机械臂串口！注意固件链路是 Arduino Serial1 (引脚18/19)，")
        print("  不是 Arduino 的 USB 口。请检查 USB-TTL 接线或用 --port 指定。")
        sys.exit(1)

    print("==============================================")
    print("      8fly 四机械臂 - 爬墙测试控制客户端       ")
    print("==============================================")
    print("臂号: 1=左上 2=右上 3=左下 4=右下 (臂端风扇: 8/9/26/27, 贴墙直跳启动到1300 离墙渐弱关)")
    print("步态: G1低位贴墙 -> G2低位离墙 -> G3高位离墙 -> G4高位贴墙 -> 四臂同时回G1")
    print("控制指令说明:")
    print("  l : 执行初始化姿态 (四臂同时到 G1 低位贴墙, 风扇渐大吸附)")
    print("  k : 开始爬墙循环 (左上->右上->左下->右下->四臂同时向下)")
    print("  s : 停止循环 / 急停退出")
    print("  q : 退出脚本")
    print("  (单键即触发, 无需回车)")
    print("----------------------------------------------")
    print(f"正在连接设备: {port} ...")

    with ArmClient(port, baud=ARM_BAUD) as arm:
        # 启动键盘监听后台线程
        listener = threading.Thread(target=listen_command, daemon=True)
        listener.start()

        print("设备连接成功, 随时可以输入指令 (l/k/s/q)...")

        # 主线程事件循环
        while not quit_flag.is_set():
            time.sleep(0.05)

            # 触发初始化
            if start_init_event.is_set():
                start_init_event.clear()
                stop_flag.clear()   # 清除上次急停/中断残留, 允许新动作
                do_init(arm)

            # 触发爬墙循环
            if start_loop_event.is_set():
                start_loop_event.clear()
                stop_flag.clear()
                run_loop(arm, args.cycles)

        # 退出清理
        print("\n正在停止并清理系统安全状态...")
        arm.stop()
        print("已发送急停 (CMD_STOP)。程序结束。")


if __name__ == '__main__':
    main()
