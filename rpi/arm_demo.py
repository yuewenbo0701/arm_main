#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
8fly 机械臂 · 树莓派端控制演示
用法:
    python3 arm_demo.py                # 自动查找串口
    python3 arm_demo.py /dev/ttyACM0   # 手动指定串口
"""
import sys
import time

from arm_client import ArmClient, find_port


def main():
    port = sys.argv[1] if len(sys.argv) > 1 else find_port()
    if not port:
        print("未找到 Arduino 串口, 请检查 USB 连接;")
        print("也可手动指定: python3 arm_demo.py /dev/ttyACM0")
        sys.exit(1)

    print(f"[1/6] 连接 {port} ...")
    with ArmClient(port) as arm:
        print("[2/6] 归零所有舵机 ...")
        arm.home()
        if not arm.wait_done(30):
            print("  ! 等待完成超时, 中止")
            return

        print("[3/6] 臂1 移动到 (0°, 20°, 20°, -40°) ...")
        arm.move_joint(1, [0, 20, 20, -40])
        if not arm.wait_done(30):
            print("  ! 等待完成超时, 中止")
            return

        print("[4/6] 臂1 风扇启动 (油门 1200), 运行 2 秒 ...")
        arm.fan_on(1)
        time.sleep(2)

        print("[5/6] 风扇关闭, 归零 ...")
        arm.fan_off(1)
        arm.home()
        arm.wait_done(30)

        print("[6/6] 最近一帧状态:")
        s = arm.status
        if s:
            print(f"  moving={s.moving}  auto={s.auto}")
            print(f"  arm1角度={s.arm1_angles}  arm2角度={s.arm2_angles}")
            print(f"  fan1={s.fan1}  fan2={s.fan2}  激光={s.laser_mm}mm  FSR={s.fsr1},{s.fsr2}")
        else:
            print("  (尚未收到状态帧)")
        arm.stop()

    print("演示结束。")


if __name__ == '__main__':
    main()
