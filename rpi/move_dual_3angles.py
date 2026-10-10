#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
8fly 双机械臂 · 三角度控制脚本
=====================================
输入 3 个关节角 (a1, a2, a3), 第 4 角自动计算:

    a4 = -(a2 + a3)          # 保持末端姿态水平

同一组角度同时下发到 臂1 和 臂2, 两臂同步运动。

用法:
    # 命令行直接给角度 (单位: 度)
    python3 move_dual_3angles.py 0 20 20
    python3 move_dual_3angles.py 0 20 20 --port /dev/ttyACM0
    python3 move_dual_3angles.py 0 20 20 --arm 1     # 只动臂1

    # 不带参数进入交互模式, 循环输入三个角, 输入 q 退出
    python3 move_dual_3angles.py

依赖: 同目录下的 arm_client.py (pip install pyserial)
"""

import argparse
import sys

from arm_client import ArmClient, find_port

# 关节限位 (度), 与 arm_config.h 保持一致
ANGLE_MIN = [-60, -30, -30, -75]
ANGLE_MAX = [60, 75, 75, 75]


def calc_a4(a2, a3):
    """第 4 角: a4 = -(a2 + a3)"""
    return -(a2 + a3)


def check_range(angles):
    """检查 4 个角是否都在限位内, 返回 (是否合法, 越限信息)"""
    for i, a in enumerate(angles):
        if not (ANGLE_MIN[i] <= a <= ANGLE_MAX[i]):
            return False, f"关节{i + 1} = {a}° 超限 (允许 {ANGLE_MIN[i]}° ~ {ANGLE_MAX[i]}°)"
    return True, ""


def move(arm_client, arms, a1, a2, a3):
    """计算 a4 并把 [a1, a2, a3, a4] 下发给指定的臂 (1 和/或 2)"""
    a4 = calc_a4(a2, a3)
    angles = [a1, a2, a3, a4]

    ok, msg = check_range(angles)
    if not ok:
        print(f"  ! 拒绝下发: {msg}")
        return False

    print(f"  -> 目标角度: a1={a1}° a2={a2}° a3={a3}° a4=-(a2+a3)={a4}°")
    for arm_num in arms:
        arm_client.move_joint(arm_num, angles)

    # 等待移动完成
    if not arm_client.wait_done(30):
        print("  ! 等待完成超时")
        return False
    if arm_client.last_error is not None:
        print(f"  ! 固件报错: 错误码 0x{arm_client.last_error:02X} (角度超限)")
        arm_client.last_error = None
        return False
    return True


def interactive(arm_client, arms):
    """交互模式: 循环输入 a1 a2 a3, q 退出"""
    print("交互模式 —— 每行输入 3 个角度 (度), 例如: 0 20 20")
    print("           a4 自动取 -(a2+a3); 输入 q 退出, h 归零")
    while True:
        try:
            line = input("角度 a1 a2 a3 > ").strip()
        except (EOFError, KeyboardInterrupt):
            print()
            break
        if not line:
            continue
        if line.lower() in ('q', 'quit', 'exit'):
            break
        if line.lower() in ('h', 'home'):
            print("  -> 归零所有舵机 ...")
            arm_client.home()
            arm_client.wait_done(30)
            continue
        parts = line.replace(',', ' ').split()
        if len(parts) != 3:
            print("  ! 需要 3 个数字, 例如: 0 20 20")
            continue
        try:
            a1, a2, a3 = (float(x) for x in parts)
        except ValueError:
            print("  ! 输入无法解析为数字")
            continue
        move(arm_client, arms, a1, a2, a3)


def main():
    ap = argparse.ArgumentParser(description="双机械臂三角度控制 (a4 = -(a2+a3))")
    ap.add_argument("angles", nargs="*", type=float,
                    help="3 个关节角 a1 a2 a3 (度); 不填则进入交互模式")
    ap.add_argument("--port", default=None,
                    help="串口设备, 如 /dev/ttyACM0; 默认自动查找")
    ap.add_argument("--arm", type=int, choices=[1, 2], default=None,
                    help="只控制某一臂; 默认两臂同时")
    ap.add_argument("--home", action="store_true",
                    help="先归零再执行动作")
    args = ap.parse_args()

    if args.angles and len(args.angles) != 3:
        ap.error("需要恰好 3 个角度, 例如: python3 move_dual_3angles.py 0 20 20")

    port = args.port or find_port()
    if not port:
        print("未找到 Arduino 串口, 请检查 USB 连接;")
        print("也可手动指定: python3 move_dual_3angles.py 0 20 20 --port /dev/ttyACM0")
        sys.exit(1)

    arms = [args.arm] if args.arm else [1, 2]

    print(f"连接 {port} ... (控制臂: {arms})")
    with ArmClient(port) as arm:
        if args.home:
            print("先归零所有舵机 ...")
            arm.home()
            if not arm.wait_done(30):
                print("! 归零超时, 中止")
                return

        if args.angles:
            a1, a2, a3 = args.angles
            if move(arm, arms, a1, a2, a3):
                s = arm.status
                if s:
                    print(f"完成。当前角度: 臂1={s.arm1_angles}  臂2={s.arm2_angles}")
                else:
                    print("完成。")
        else:
            interactive(arm, arms)

    print("结束。")


if __name__ == '__main__':
    main()
