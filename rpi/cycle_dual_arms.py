#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
8fly 双机械臂 · 三步循环工作脚本
=====================================
臂1 和 臂2 同步执行以下三步, 每步 4 秒, 循环往复, 按 s 回车停止:

    步骤 | 角1 | 角2 | 角3 | 角4=-(角2+角3) | 风扇油门        | 持续
      1  | 40  | 15  | 45  |     -60        | 1200 (到位后开)  | 4s
      2  |  0  | 15  | 45  |     -60        | 1000 (渐弱)     | 4s
      3  |  0  |  0  |  0  |       0        | 1000 (关闭)     | 4s

时序说明:
    每步开始即下发关节角 → 等待移动完成 (wait_done) → 再调风扇油门
    → 补足该步剩余时间, 使每步总时长 ≈ 4s。
    风扇油门本身就是缓变的 (固件电调斜坡), 1000 即"渐弱至关"。

停止 (输入 s 回车):
    立即停止风扇 → 等当前动作走完 → 归零 → 退出。

用法:
    python3 cycle_dual_arms.py                      # 自动查找串口
    python3 cycle_dual_arms.py --port /dev/ttyACM0
    python3 cycle_dual_arms.py --arm 1              # 只跑臂1
    python3 cycle_dual_arms.py --cycles 10          # 只循环 10 次 (默认无限)
"""

import argparse
import sys
import threading
import time

from arm_client import ArmClient, find_port

# ------------------------------------------------------------
# 循环步骤表: (角1, 角2, 角3, 风扇油门, 持续时间s) — 角4 = -(角2+角3)
# ------------------------------------------------------------
STEPS = [
    (40, 15, 45, 1200, 4.0),
    (0,  15, 45, 1000, 4.0),
    (0,   0,  0, 1000, 4.0),
]

# 关节限位 (度), 与 arm_config.h 一致, 用于启动前预检
ANGLE_MIN = [-60, -30, -30, -75]
ANGLE_MAX = [60, 75, 75, 75]

stop_flag = threading.Event()


def precheck():
    """启动前校验所有步骤的角度都在限位内, 避免运行中被固件拒绝"""
    for n, (a1, a2, a3, _, _) in enumerate(STEPS, 1):
        a4 = -(a2 + a3)
        for i, a in enumerate((a1, a2, a3, a4)):
            if not (ANGLE_MIN[i] <= a <= ANGLE_MAX[i]):
                print(f"! 步骤{n}: 关节{i + 1} = {a}° 超限 "
                      f"(允许 {ANGLE_MIN[i]}° ~ {ANGLE_MAX[i]}°), 中止")
                sys.exit(1)


def listen_stop():
    """后台线程: 监听键盘, 输入 s 回车置停止标志"""
    while not stop_flag.is_set():
        try:
            line = input()
        except (EOFError, KeyboardInterrupt):
            stop_flag.set()
            return
        if line.strip().lower() == 's':
            stop_flag.set()
            return


def run_step(arm_client, arms, step, idx):
    """执行一步: 移动 → 到位后调风扇 → 补足剩余时间。返回 False 表示被停止"""
    a1, a2, a3, throttle, duration = step
    a4 = -(a2 + a3)
    t0 = time.time()

    print(f"  步骤{idx}: 角=({a1}, {a2}, {a3}, {a4}) 风扇={throttle}")
    for arm_num in arms:
        arm_client.move_joint(arm_num, [a1, a2, a3, a4])

    # 等待移动完成 (期间可被 s 打断)
    while not arm_client._done.wait(0.1):
        if stop_flag.is_set():
            return False
        if time.time() - t0 > 30:
            print("  ! 等待移动完成超时")
            return False

    if arm_client.last_error is not None:
        print(f"  ! 固件报错: 角度超限 (码 0x{arm_client.last_error:02X}), 中止")
        stop_flag.set()
        return False

    # 到位后再调风扇 (步骤1 的 1200 即 wait done 后开启; 1000 由电调斜坡渐弱至关)
    if stop_flag.is_set():
        return False
    for arm_num in arms:
        arm_client.set_fan(arm_num, throttle)

    # 补足该步剩余时间
    while time.time() - t0 < duration:
        if stop_flag.is_set():
            return False
        time.sleep(0.05)
    return True


def main():
    ap = argparse.ArgumentParser(description="双机械臂三步循环工作 (角4 = -(角2+角3))")
    ap.add_argument("--port", default=None, help="串口设备, 默认自动查找")
    ap.add_argument("--arm", type=int, choices=[1, 2], default=None,
                    help="只控制某一臂; 默认两臂同步")
    ap.add_argument("--cycles", type=int, default=0,
                    help="循环次数, 0 = 无限循环直到按 s (默认)")
    args = ap.parse_args()

    precheck()

    port = args.port or find_port()
    if not port:
        print("未找到 Arduino 串口, 请检查 USB 连接;")
        print("也可手动指定: python3 cycle_dual_arms.py --port /dev/ttyACM0")
        sys.exit(1)

    arms = [args.arm] if args.arm else [1, 2]

    print("循环序列 (每步 4s, 角4 自动 = -(角2+角3)):")
    for n, (a1, a2, a3, thr, dur) in enumerate(STEPS, 1):
        print(f"  步骤{n}: 角1={a1} 角2={a2} 角3={a3} 角4={-(a2 + a3)} "
              f"风扇={thr} 持续={dur}s")
    print("输入 s 回车随时停止。\n")

    print(f"连接 {port} ... (控制臂: {arms})")
    with ArmClient(port) as arm:
        listener = threading.Thread(target=listen_stop, daemon=True)
        listener.start()

        cycle = 0
        while not stop_flag.is_set():
            cycle += 1
            print(f"[循环 {cycle}]")
            for idx, step in enumerate(STEPS, 1):
                if not run_step(arm, arms, step, idx):
                    break
            if args.cycles and cycle >= args.cycles:
                break

        print("\n停止中: 关闭风扇 ...")
        for arm_num in arms:
            arm_client_safe_fanoff(arm, arm_num)
        arm.wait_done(10)
        print("归零 ...")
        arm.home()
        arm.wait_done(30)
        arm.stop()

    print("结束。")


def arm_client_safe_fanoff(arm_client, arm_num):
    """停止时立即关风扇, 忽略异常 (串口可能已中断)"""
    try:
        arm_client.fan_off(arm_num)
    except Exception:
        pass


if __name__ == '__main__':
    main()
