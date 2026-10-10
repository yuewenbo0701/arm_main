#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
双机械臂 · 爬升测试脚本 (climb_test.py)
======================================
控制命令:
  l  初始化: 臂1和臂2电机同时移动至 [10.8, -9.1, 18.5]，不开启风扇
  k  开始爬升循环 (State 1 -> State 2 -> State 3)
  s  急停并退出脚本 (关闭风扇并归零)

状态说明:
  State 1 (抬臂2): 
    - 臂1风扇开(渐大至1700) -> 0.5s
    - 臂2风扇关(1000) -> 臂2移至[25.1, -11.9, 0.0] -> 0.5s
    - 臂2移至[31.7, 0.5, 0.0] -> 0.3s -> 臂2风扇开(渐大至1700)
    - 完成后等待 1.0s
  State 2 (抬臂1): 
    - 与 State 1 镜像对称 (臂1与臂2角色对调)
    - 完成后等待 1.0s
  State 3 (双臂复位/上抬): 
    - 臂1和臂2同时移至 [10.8, -9.1, 18.5]，等待完成 -> 1.0s

用法:
  python3 climb_test.py
  python3 climb_test.py --port /dev/ttyACM0
  python3 climb_test.py --cycles 5
"""

import argparse
import sys
import threading
import time

from arm_client import ArmClient, find_port

# ----- 姿态定义 -----
INIT_ANGLES = [10.8, -9.1, 18.5]
MID_ANGLES  = [25.1, -11.9, 0.0]
TARGET_ANGLES = [31.7, 0.5, 0.0]

FAN_SPEED_ON = 1700      # 风扇开启目标油门 (us)
FAN_SPEED_OFF = 1000     # 风扇关闭油门 (us)
FAN_RAMP_TIME = 0.5      # 油门渐大爬升总时间 (s)
FAN_RAMP_STEP = 50       # 油门每步增量 (us)

# 线程通信与控制标志
stop_flag = threading.Event()
start_init_event = threading.Event()
start_loop_event = threading.Event()
loop_active = False


def sleep_interruptible(seconds: float) -> bool:
    """可被打断的睡眠等待函数。若期间收到停止信号，立即返回 False。"""
    t0 = time.time()
    while time.time() - t0 < seconds:
        if stop_flag.is_set():
            return False
        time.sleep(0.05)
    return True


def fan_ramp_on(arm_client: ArmClient, arm: int, target: int = FAN_SPEED_ON) -> bool:
    """风扇油门从关闭值渐大爬升至 target，避免瞬时大油门冲击。可被急停打断。"""
    if target <= FAN_SPEED_OFF:
        arm_client.set_fan(arm, target)
        return True
    n_steps = max(1, (target - FAN_SPEED_OFF + FAN_RAMP_STEP - 1) // FAN_RAMP_STEP)
    step_dt = FAN_RAMP_TIME / n_steps
    v = FAN_SPEED_OFF
    while v < target:
        if stop_flag.is_set():
            return False
        v = min(target, v + FAN_RAMP_STEP)
        arm_client.set_fan(arm, v)
        time.sleep(step_dt)
    return True


def wait_for_move(arm_client: ArmClient, timeout: float = 30) -> bool:
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
    """后台线程：解析键盘输入 (l / k / s)"""
    global loop_active
    while not stop_flag.is_set():
        try:
            line = input()
        except (EOFError, KeyboardInterrupt):
            stop_flag.set()
            return

        cmd = line.strip().lower()
        if cmd == 's':
            print(">>> 收到急停指令 's'")
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


def do_init(arm_client: ArmClient):
    """
    [L 指令逻辑] 初始化：
    - 两臂同时移动至 INIT_ANGLES [10.8, -9.1, 18.5]
    - 不控制风扇，阻塞等待移动完成
    """
    print("\n=== 执行初始化 (L) ===")
    print(f"同时移动臂1和臂2至: {INIT_ANGLES}")
    
    # 连续下发给臂1和臂2，两者在硬件层会几乎同时开始运动
    arm_client.move_joint(1, INIT_ANGLES)
    arm_client.move_joint(2, INIT_ANGLES)

    if wait_for_move(arm_client):
        print("✓ 初始化位置到达成功")
    else:
        print("✕ 初始化移动中断或超时")


def step_single_arm_climb(arm_client: ArmClient, active_arm: int, idle_arm: int) -> bool:
    """
    通用爬升单步流程 (封装 State 1 和 State 2 的对称逻辑)
    - active_arm: 本次抬起/移动的臂
    - idle_arm: 本次作为吸附支撑的臂
    """
    # 1. 支撑臂开启风扇 (渐大) -> 等待 0.5s
    print(f"  [吸附] 臂{idle_arm} 风扇渐大开启 (-> {FAN_SPEED_ON}us)...")
    if not fan_ramp_on(arm_client, idle_arm): return False
    if not sleep_interruptible(0.5): return False

    # 2. 动作臂关闭风扇 -> 移动至过渡角 -> 等待 Done -> 等待 0.5s
    print(f"  [释放] 臂{active_arm} 风扇关闭")
    arm_client.fan_off(active_arm)
    
    print(f"  [抬臂1阶段] 臂{active_arm} 移动至 {MID_ANGLES}...")
    arm_client.move_joint(active_arm, MID_ANGLES)
    if not wait_for_move(arm_client): return False
    if not sleep_interruptible(0.5): return False

    # 3. 动作臂移动至目标角 -> 等待 Done -> 等待 0.3s -> 打开动作臂风扇
    print(f"  [抬臂2阶段] 臂{active_arm} 移动至 {TARGET_ANGLES}...")
    arm_client.move_joint(active_arm, TARGET_ANGLES)
    if not wait_for_move(arm_client): return False
    if not sleep_interruptible(0.3): return False

    print(f"  [吸附] 臂{active_arm} 风扇渐大开启 (-> {FAN_SPEED_ON}us)")
    if not fan_ramp_on(arm_client, active_arm): return False

    # 阶段完成后统一等待 1 秒
    print("  状态完成，等待 1.0s ...")
    return sleep_interruptible(1.0)


def run_loop(arm_client: ArmClient, max_cycles: int = 0):
    """[K 指令逻辑] State 1 -> State 2 -> State 3 状态机循环"""
    global loop_active
    loop_active = True
    cycle = 0

    try:
        while not stop_flag.is_set():
            cycle += 1
            print(f"\n================ [爬升循环第 {cycle} 次] ================")

            # ---- State 1: 支撑臂1，抬高臂2 ----
            print(f"\n--> State 1: 保持臂1，移动臂2")
            if not step_single_arm_climb(arm_client, active_arm=2, idle_arm=1):
                break

            # ---- State 2: 支撑臂2，抬高臂1 (与 State 1 对称) ----
            print(f"\n--> State 2: 保持臂2，移动臂1")
            if not step_single_arm_climb(arm_client, active_arm=1, idle_arm=2):
                break

            # ---- State 3: 双臂同步上抬/复位 ----
            print(f"\n--> State 3: 双臂同时移动至 {INIT_ANGLES}")
            arm_client.move_joint(1, INIT_ANGLES)
            arm_client.move_joint(2, INIT_ANGLES)

            if not wait_for_move(arm_client):
                break
            
            print("  State 3 完成，等待 1.0s ...")
            if not sleep_interruptible(1.0):
                break

            if max_cycles > 0 and cycle >= max_cycles:
                print(f"\n到达要求的指定循环次数 ({max_cycles})，停止循环")
                break

    finally:
        loop_active = False


def main():
    parser = argparse.ArgumentParser(description="双机械臂爬升测试 (climb_test)")
    parser.add_argument("--port", default=None, help="串口设备地址, 默认自动匹配")
    parser.add_argument("--cycles", type=int, default=0, help="运行循环次数, 0 表示无限循环")
    args = parser.parse_args()

    port = args.port or find_port()
    if not port:
        print("✕ 未找到 Arduino 串口，请检查连线或通过 --port 指定！")
        sys.exit(1)

    print("==============================================")
    print("      8fly 双机械臂 - 爬升测试控制客户端       ")
    print("==============================================")
    print("控制指令说明:")
    print("  l : 执行初始化姿态 (电机到达指定角度, 不开风扇)")
    print("  k : 开始爬升循环 (State 1 -> State 2 -> State 3)")
    print("  s : 停止循环 / 急停退出")
    print("----------------------------------------------")
    print(f"正在连接设备: {port} ...")

    with ArmClient(port) as arm:
        # 启动键盘监听后台线程
        listener = threading.Thread(target=listen_command, daemon=True)
        listener.start()

        print("设备连接成功, 随时可以输入指令 (l/k/s)...")

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
        arm.fan_off(1)
        arm.fan_off(2)
        arm.stop()
        print("风扇已关，命令已停止。程序结束。")


if __name__ == '__main__':
    main()
