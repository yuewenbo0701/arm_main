#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
臂端涵道风扇 · 单路排查脚本 (tip_fan_test.py)
================================================
逐个测试四根机械臂末端涵道风扇 (面对机头方向):
  风扇号 1 = 左上  = Arduino 引脚 8
  风扇号 2 = 右上  = Arduino 引脚 9
  风扇号 5 = 左下  = Arduino 引脚 26
  风扇号 6 = 右下  = Arduino 引脚 27

每一路: 1000 -> 渐大至 1300 (每 0.2s +50us) -> 保持 2s -> 渐弱回 1000。
某一路不转 = 该路硬件问题 (信号线/共地/电调解锁), 与固件和爬墙脚本无关。

串口 (与 t265_bridge_z_arm.py 一致):
  固件链路走 Arduino Serial1 (引脚18/19), 不是 Arduino USB 口!
  依次探测 /dev/ttyUSB1 -> /dev/serial0 -> /dev/ttyAMA0, --port 可指定。

用法:
  python3 tip_fan_test.py              # 四路轮流各测一遍
  python3 tip_fan_test.py --fan 5      # 只测风扇5 (左下), 循环测, Ctrl+C 结束
  python3 tip_fan_test.py --fan 6 --jump   # 直跳模式 (不渐变), 对比渐变用
  python3 tip_fan_test.py --hold 5     # 每路 1300us 保持 5 秒
"""

import argparse
import os
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), 'rpi'))

from arm_client import ArmClient

ARM_PORT_CANDIDATES = ['/dev/ttyUSB1', '/dev/serial0', '/dev/ttyAMA0']
ARM_BAUD = 115200

# (协议风扇号, 中文名, Arduino 引脚)
FANS = [
    (1, '左上', 8),
    (2, '右上', 9),
    (5, '左下', 26),
    (6, '右下', 27),
]

THROTTLE_OFF    = 1000
THROTTLE_TEST   = 1300
RAMP_STEP       = 50      # 每步 ±50us
RAMP_INTERVAL   = 0.2     # 每步间隔 0.2s


def find_arm_port(exclude='/dev/ttyUSB0'):
    """自动查找机械臂串口, 排除飞控口 (ttyUSB0 = Pixhawk); 找不到返回 None"""
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


def ramp(arm: ArmClient, fan: int, target: int):
    """从当前记录油门渐进到 target 并打印每一步"""
    cur = ramp.cur.get(fan, THROTTLE_OFF)
    step = RAMP_STEP if target > cur else -RAMP_STEP
    t = cur
    while t != target:
        t += step
        if (step > 0 and t > target) or (step < 0 and t < target):
            t = target
        arm.set_fan(fan, t)
        print(f"    风扇{fan} -> {t}us")
        time.sleep(RAMP_INTERVAL)
    ramp.cur[fan] = t


ramp.cur = {}


def test_one(arm: ArmClient, fan: int, name: str, pin: int, hold: float,
             jump: bool = False):
    print(f"\n=== 测试 风扇{fan} ({name}, 引脚{pin}) ===")
    if jump:
        # 直跳模式: 一步到位 (与命令行直接 set_fan 一致, 用来对比渐变模式)
        arm.set_fan(fan, THROTTLE_TEST)
        ramp.cur[fan] = THROTTLE_TEST
        print(f"  直跳至 {THROTTLE_TEST}us (无渐变)")
    else:
        print(f"  渐大至 {THROTTLE_TEST}us ...")
        ramp(arm, fan, THROTTLE_TEST)
    print(f"  保持 {THROTTLE_TEST}us {hold}s —— 现在 {name} 风扇应该在转!")
    time.sleep(hold)
    print(f"  渐弱回 {THROTTLE_OFF}us (关闭) ...")
    ramp(arm, fan, THROTTLE_OFF)


def main():
    parser = argparse.ArgumentParser(description="臂端涵道风扇单路排查")
    parser.add_argument("--port", default=None, help="机械臂串口, 默认自动探测")
    parser.add_argument("--fan", type=int, default=0,
                        help="只测某一路 (1/2/5/6), 循环测试直到 Ctrl+C")
    parser.add_argument("--hold", type=float, default=5.0,
                        help="1300us 保持秒数, 默认 5")
    parser.add_argument("--jump", action='store_true',
                        help="直跳模式: 一步到位 1300us, 不做渐变 (对比用)")
    args = parser.parse_args()

    port = args.port
    if port is None:
        for cand in ARM_PORT_CANDIDATES:
            if os.path.exists(cand):
                port = cand
                break
    if port is None:
        port = find_arm_port()
    if not port:
        print("✕ 未找到机械臂串口！固件链路是 Arduino Serial1 (引脚18/19)，")
        print("  不是 Arduino USB 口。请检查接线或用 --port 指定。")
        sys.exit(1)

    fans = FANS
    if args.fan:
        fans = [f for f in FANS if f[0] == args.fan]
        if not fans:
            print(f"✕ 风扇号 {args.fan} 无效, 可选: 1(左上) 2(右上) 5(左下) 6(右下)")
            sys.exit(1)

    print("==============================================")
    print("   臂端涵道风扇排查: 1=左上 2=右上 5=左下 6=右下")
    print("==============================================")
    print(f"连接: {port}  (测试完成后会自动全部关闭)")
    print("----------------------------------------------")

    with ArmClient(port, baud=ARM_BAUD) as arm:
        try:
            while True:
                for fan, name, pin in fans:
                    test_one(arm, fan, name, pin, args.hold, jump=args.jump)
                if args.fan:
                    print(f"\n--- 风扇{args.fan} 本轮测完, 2s 后重复 (Ctrl+C 结束) ---")
                    time.sleep(2.0)
                else:
                    break
        except KeyboardInterrupt:
            print("\n中断。")
        finally:
            print("\n全部风扇关闭 ...")
            arm.stop()   # 固件 STOP 会 fanOff(1~6)
    print("结束。")


if __name__ == '__main__':
    main()
