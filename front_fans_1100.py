#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
前臂涵道风扇低速测试 (front_fans_1100.py)
==========================================
唯一功能: 让引脚 33(左前)/32(右前) 两个涵道风扇转到 1100us 低速。
除此之外没有任何其他功能。

    左前臂风扇 = 引脚33 = 协议风扇号 3
    右前臂风扇 = 引脚32 = 协议风扇号 4

运行后先给 1000us 保持 2 秒让电调完成解锁, 然后两扇转到 1100us 并保持;
按回车 (或 Ctrl+C) 关闭并退出。
需要配套四臂固件 (CMD_FAN_CTRL 支持风扇号 1~4, 链路走 Serial1 引脚18/19)。

用法:
    python3 front_fans_1100.py                    # 自动探测串口
    python3 front_fans_1100.py --port /dev/serial0
    python3 front_fans_1100.py --throttle 1200    # 1100 不转时可试更高油门
"""

import argparse
import os
import sys
import time

# arm_client.py 在 rpi/ 子目录
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), 'rpi'))

FRONT_FAN_L = 3       # 左前臂 -> 引脚33
FRONT_FAN_R = 4       # 右前臂 -> 引脚32
THROTTLE    = 1100    # 低速测试油门 (us)

# 树莓派侧候选串口 (与 t265_bridge_z_arm.py 一致)
PORT_CANDIDATES = ['/dev/ttyUSB1', '/dev/serial0', '/dev/ttyAMA0']


def find_arm_port():
    """依次探测候选口, 再自动探测 USB 转串口"""
    for cand in PORT_CANDIDATES:
        if os.path.exists(cand):
            return cand
    from arm_client import find_port
    return find_port()


def main():
    parser = argparse.ArgumentParser(description="前臂涵道风扇 1100us 低速测试")
    parser.add_argument('--port', default=None, help='串口设备, 默认自动探测')
    parser.add_argument('--throttle', type=int, default=THROTTLE,
                        help=f'油门 us, 默认 {THROTTLE} (范围 1000~2000)')
    args = parser.parse_args()

    port = args.port or find_arm_port()
    if not port:
        print("✕ 未找到 Arduino 串口, 请检查连线或用 --port 指定")
        sys.exit(1)

    from arm_client import ArmClient
    print(f"连接机械臂 {port} @ 115200 ...")
    arm = ArmClient(port, baud=115200)
    print("已连接")

    try:
        # ---- 电调解锁: 先给 1000us 保持 2 秒 ----
        # 部分电调上电后需要一段最低油门信号才完成解锁, 直接加油门不转
        arm.set_fan(FRONT_FAN_L, 1000)
        arm.set_fan(FRONT_FAN_R, 1000)
        print("电调解锁中 (1000us 保持 2s)...")
        time.sleep(2.0)

        # ---- 唯一动作: 两个前扇 -> 目标油门 ----
        ack_before = arm.ack_count
        arm.set_fan(FRONT_FAN_L, args.throttle)
        arm.set_fan(FRONT_FAN_R, args.throttle)
        print(f">>> 左前(引脚33)/右前(引脚32) 涵道风扇 -> {args.throttle}us")

        time.sleep(0.5)
        ack_got = arm.ack_count - ack_before
        if ack_got >= 2:
            print(f"[链路] 双向通信正常, 固件已确认收到风扇指令 (ACK x{ack_got})")
        elif arm.status_time > 0:
            print("[链路] !! 只能收状态帧, 指令无应答 —— 树莓派TX -> Arduino 19(RX1) 这一路不通")
            print("      -> 检查: TX/RX 是否交叉、是否共地、5V/3.3V 电平是否匹配")
        else:
            print("[链路] !! 未收到任何数据 —— 检查 18/19 接线与固件烧录")

        # ---- 固件版本识别: 四臂新固件状态帧为 30 字节 (含臂3/4角度) ----
        # 注意: 旧固件对风扇3/4也会回 ACK 但不执行! 只能靠状态帧长度区分新旧固件
        if arm.status is not None:
            if arm.status.arm3_angles is not None:
                print("[固件] 四臂新固件确认 (30 字节状态帧)")
                if ack_got >= 2:
                    print("      -> 指令确认已执行; 若不转, 问题在电调侧: "
                          "供电/共地/信号线位置/解锁/启动油门")
            else:
                print("[固件] !! 正在运行的是旧版固件 (状态帧不含臂3/臂4)")
                print("      -> 旧固件不认风扇号3/4, 收到指令也不执行! "
                      "请烧录新版 arm_main.ino")

        input("风扇运转中... 按回车关闭并退出\n")
    except KeyboardInterrupt:
        print()
    finally:
        arm.fan_off(FRONT_FAN_L)
        arm.fan_off(FRONT_FAN_R)
        arm.stop()
        arm.close()
        print("风扇已关闭 (1000us), 退出")


if __name__ == '__main__':
    main()
