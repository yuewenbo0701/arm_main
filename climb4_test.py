#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
四机械臂 · 爬升测试脚本 (climb4_test.py)
=========================================
在 climb_test.py (双臂版) 基础上扩展为四臂, 不含飞控/T265, 只测机械臂动作。
本脚本只控制机械臂关节, 不控制任何风扇。

机械臂编号与 Arduino 引脚 (从根部到末端 4 个关节, 观察者面对机头方向):
  臂1 = 左上: 引脚 4, 5, 6, 7
  臂2 = 右上: 引脚 10, 11, 12, 13
  臂3 = 左下: 引脚 22, 23, 24, 25
  臂4 = 右下: 引脚 28, 29, 30, 31
四臂使用完全相同的关节角步态 (臂3/臂4 复制臂1/臂2 的动作)。

控制命令:
  l  初始化: 四个臂同时移动至 INIT_ANGLES
  k  开始爬升循环 (Step 1 -> 2 -> 3 -> 4 -> 5)
  s  急停并退出脚本 (发送 CMD_STOP)
  q  退出脚本 (同 s)

循环逻辑 (每轮):
  Step 1: 左上(臂1) 离开墙(->MID) -> 抬升贴墙(->TARGET)
  Step 2: 右上(臂2) 离开墙 -> 抬升贴墙
  Step 3: 左下(臂3) 离开墙 -> 抬升贴墙
  Step 4: 右下(臂4) 离开墙 -> 抬升贴墙
  Step 5: 四个臂同时向下 (-> INIT_ANGLES)

用法:
  python3 climb4_test.py
  python3 climb4_test.py --port /dev/ttyACM0
  python3 climb4_test.py --cycles 5
"""

import argparse
import os
import sys
import threading
import time

# arm_client.py 在 rpi/ 子目录
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), 'rpi'))

from arm_client import ArmClient, find_port

# ----- 步态参数 (与双臂版一致; 只发 3 个角, 第 4 关节固件自动取 -(a2+a3)) -----
INIT_ANGLES   = [10.8, -9.1, 18.5]   # 贴墙下位 (循环终点"向下"的位置)
MID_ANGLES    = [25.1, -11.9, 0.0]   # 离开墙的过渡角
TARGET_ANGLES = [31.7,   0.5, 0.0]   # 抬升后贴墙的目标角

# ----- 四臂定义: (协议臂号, 中文名) —— 循环动作顺序 -----
ARMS = [
    (1, '左上'),
    (2, '右上'),
    (3, '左下'),
    (4, '右下'),
]

# ----- 时序参数 -----
MOVE_SETTLE_S  = 0.5    # 每段移动到位后的稳定等待
STEP_SETTLE_S  = 1.0    # 每个臂完成"离开墙->抬升贴墙"后的等待
CYCLE_SETTLE_S = 1.0    # 每轮循环结束后的等待
MOVE_TIMEOUT_S = 30     # 等待 RSP_DONE 超时

# 线程通信与控制标志
stop_flag        = threading.Event()
start_init_event = threading.Event()
start_loop_event = threading.Event()
loop_active      = False


def sleep_interruptible(seconds: float) -> bool:
    """可被打断的睡眠等待函数。若期间收到停止信号，立即返回 False。"""
    t0 = time.time()
    while time.time() - t0 < seconds:
        if stop_flag.is_set():
            return False
        time.sleep(0.05)
    return True


def wait_for_move(arm_client: ArmClient, timeout: float = MOVE_TIMEOUT_S) -> bool:
    """等待串口返回 RSP_DONE (移动完成)。支持中断与错误检查。"""
    t0 = time.time()
    while not arm_client._done.wait(0.05):
        if stop_flag.is_set():
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
    """后台线程：解析键盘输入 (l / k / s / q)"""
    global loop_active
    while not stop_flag.is_set():
        try:
            line = input()
        except (EOFError, KeyboardInterrupt):
            stop_flag.set()
            return

        cmd = line.strip().lower()
        if cmd in ('s', 'q'):
            print(f">>> 收到急停/退出指令 '{cmd}'")
            stop_flag.set()
            return
        elif cmd == 'l':
            print(">>> 收到初始化指令 'l'")
            start_init_event.set()
        elif cmd == 'k':
            print(">>> 收到循环启动指令 'k'")
            if loop_active:
                print("循环正在进行中，重启循环...")
                stop_flag.set()
                while loop_active:
                    time.sleep(0.05)
                stop_flag.clear()
            start_loop_event.set()


def move_all_arms(arm_client: ArmClient, angles) -> bool:
    """四个臂同时下发同一组关节角 (固件端同步插补, 同时到达), 等待完成。"""
    for arm_num, _name in ARMS:
        arm_client.move_joint(arm_num, angles)
    return wait_for_move(arm_client)


def do_init(arm_client: ArmClient):
    """
    [L 指令逻辑] 初始化：
    - 四个臂同时移动至 INIT_ANGLES [10.8, -9.1, 18.5]
    - 不控制风扇，阻塞等待移动完成
    """
    print("\n=== 执行初始化 (L) ===")
    print(f"同时移动 臂1(左上)/臂2(右上)/臂3(左下)/臂4(右下) 至: {INIT_ANGLES}")

    if move_all_arms(arm_client, INIT_ANGLES):
        print("✓ 初始化位置到达成功")
    else:
        print("✕ 初始化移动中断或超时")


def step_arm_lift(arm_client: ArmClient, arm_num: int, arm_name: str) -> bool:
    """
    单臂爬升步骤: 离开墙 (MID) -> 抬升贴墙 (TARGET)
    其余三个臂保持不动作为支撑。不控制风扇。
    """
    # 1. 离开墙: 移动至过渡角
    print(f"  [离开墙] 臂{arm_num}({arm_name}) 移动至 {MID_ANGLES}...")
    arm_client.move_joint(arm_num, MID_ANGLES)
    if not wait_for_move(arm_client): return False
    if not sleep_interruptible(MOVE_SETTLE_S): return False

    # 2. 抬升贴墙: 移动至目标角
    print(f"  [抬升贴墙] 臂{arm_num}({arm_name}) 移动至 {TARGET_ANGLES}...")
    arm_client.move_joint(arm_num, TARGET_ANGLES)
    if not wait_for_move(arm_client): return False
    if not sleep_interruptible(MOVE_SETTLE_S): return False

    print(f"  臂{arm_num}({arm_name}) 完成，等待 {STEP_SETTLE_S}s ...")
    return sleep_interruptible(STEP_SETTLE_S)


def run_loop(arm_client: ArmClient, max_cycles: int = 0):
    """[K 指令逻辑] Step 1~4 依次抬四个臂 -> Step 5 四臂同时向下"""
    global loop_active
    loop_active = True
    cycle = 0

    try:
        while not stop_flag.is_set():
            cycle += 1
            print(f"\n================ [爬升循环第 {cycle} 次] ================")

            # ---- Step 1~4: 左上 -> 右上 -> 左下 -> 右下 依次 离开墙/抬升/贴墙 ----
            for step, (arm_num, arm_name) in enumerate(ARMS, start=1):
                print(f"\n--> Step {step}: 臂{arm_num}({arm_name}) 离开墙 -> 抬升 -> 贴墙")
                if not step_arm_lift(arm_client, arm_num, arm_name):
                    return

            # ---- Step 5: 四个臂同时向下 (回 INIT) ----
            print(f"\n--> Step 5: 四个臂同时向下移动至 {INIT_ANGLES}")
            if not move_all_arms(arm_client, INIT_ANGLES):
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
    parser = argparse.ArgumentParser(description="四机械臂爬升测试 (climb4_test)")
    parser.add_argument("--port", default=None, help="串口设备地址, 默认自动匹配")
    parser.add_argument("--cycles", type=int, default=0, help="运行循环次数, 0 表示无限循环")
    args = parser.parse_args()

    port = args.port or find_port()
    if not port:
        print("✕ 未找到 Arduino 串口，请检查连线或通过 --port 指定！")
        sys.exit(1)

    print("==============================================")
    print("      8fly 四机械臂 - 爬升测试控制客户端       ")
    print("==============================================")
    print("臂号: 1=左上 2=右上 3=左下 4=右下 (本脚本不控制风扇)")
    print("控制指令说明:")
    print("  l : 执行初始化姿态 (四臂同时到 INIT, 不开风扇)")
    print("  k : 开始爬升循环 (左上->右上->左下->右下->四臂同时向下)")
    print("  s : 停止循环 / 急停退出")
    print("  q : 退出脚本")
    print("----------------------------------------------")
    print(f"正在连接设备: {port} ...")

    with ArmClient(port) as arm:
        # 启动键盘监听后台线程
        listener = threading.Thread(target=listen_command, daemon=True)
        listener.start()

        print("设备连接成功, 随时可以输入指令 (l/k/s/q)...")

        # 主线程事件循环
        while not stop_flag.is_set():
            time.sleep(0.05)

            # 触发初始化
            if start_init_event.is_set():
                start_init_event.clear()
                do_init(arm)

            # 触发爬升循环
            if start_loop_event.is_set():
                start_loop_event.clear()
                run_loop(arm, args.cycles)

        # 退出清理
        print("\n正在停止并清理系统安全状态...")
        arm.stop()
        print("已发送急停 (CMD_STOP)。程序结束。")


if __name__ == '__main__':
    main()
