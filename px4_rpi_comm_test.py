#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
PX4 <-> 树莓派 双向通信验证脚本 (基于 MAVLink / pymavlink)

验证内容:
  1. [树莓派 <- PX4] 等待并接收 PX4 的心跳包 (HEARTBEAT)
  2. [树莓派 -> PX4] 向 PX4 发送机载计算机心跳
  3. [来回往返 RTT] 发送 MAV_CMD_REQUEST_MESSAGE 请求 AUTOPILOT_VERSION,
     只有 PX4 收到了我们的命令并回包才算双向链路打通
  4. [数据流] 请求姿态数据流并读取数条 ATTITUDE 消息

运行环境: 树莓派 (Raspberry Pi OS), Python 3 + pymavlink
用法:
  python3 px4_rpi_comm_test.py                  # 默认 /dev/ttyAMA0 @ 921600
  python3 px4_rpi_comm_test.py -d /dev/ttyUSB0 -b 57600
"""

import argparse
import sys
import time

try:
    from pymavlink import mavutil
except ImportError:
    sys.exit("缺少 pymavlink，请先安装: pip install pymavlink")


def main():
    ap = argparse.ArgumentParser(description="PX4 <-> 树莓派 双向通信验证")
    ap.add_argument("-d", "--device", default="/dev/ttyAMA0",
                    help="串口设备 (GPIO串口 /dev/ttyAMA0 或 /dev/serial0; USB转串口 /dev/ttyUSB0)")
    ap.add_argument("-b", "--baud", type=int, default=921600,
                    help="波特率, 需与 PX4 参数 SER_TELx_BAUD 一致 (默认 921600)")
    ap.add_argument("-t", "--timeout", type=float, default=10.0,
                    help="每步等待超时秒数")
    args = ap.parse_args()

    results = {}

    print("=" * 56)
    print(f"连接 {args.device} @ {args.baud} ...")
    try:
        # source_system=255: 标准的地面站/机载计算机系统 ID
        mav = mavutil.mavlink_connection(
            args.device, baud=args.baud, source_system=255)
    except Exception as e:
        sys.exit(f"[FAIL] 无法打开串口: {e}\n"
                 f"  检查: 设备名是否正确 / 是否在 dialout 用户组 / 串口是否被占用")

    # ---------- 步骤 1: 接收 PX4 心跳 (下行) ----------
    print(f"\n[1/4] 等待 PX4 心跳包 (timeout {args.timeout}s) ...")
    hb = mav.wait_heartbeat(timeout=args.timeout)
    if hb is None:
        results["下行: 接收PX4心跳"] = False
        summary(results)
        sys.exit(1)
    results["下行: 接收PX4心跳"] = True
    print(f"  OK  收到心跳: system={mav.target_system} component={mav.target_component} "
          f"autopilot={hb.autopilot} type={hb.type}")

    # ---------- 步骤 2: 发送机载计算机心跳 (上行) ----------
    print("\n[2/4] 向 PX4 发送机载计算机心跳 ...")
    mav.mav.heartbeat_send(
        mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
        mavutil.mavlink.MAV_AUTOPILOT_INVALID,
        0, 0,
        mavutil.mavlink.MAV_STATE_ACTIVE)
    results["上行: 发送心跳"] = True
    print("  OK  已发送 (该方向是否真正到达, 由步骤 3 的往返测试判定)")

    # ---------- 步骤 3: 往返测试 - 请求 AUTOPILOT_VERSION ----------
    print(f"\n[3/4] 往返测试: 请求 AUTOPILOT_VERSION (timeout {args.timeout}s) ...")
    mav.mav.command_long_send(
        mav.target_system, mav.target_component,
        mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE, 0,
        mavutil.mavlink.MAVLINK_MSG_ID_AUTOPILOT_VERSION,
        0, 0, 0, 0, 0, 0)
    t0 = time.time()
    ver = None
    while time.time() - t0 < args.timeout:
        msg = mav.recv_match(type="AUTOPILOT_VERSION", blocking=True, timeout=1)
        if msg:
            ver = msg
            break
    results["往返: PX4应答命令"] = ver is not None
    if ver:
        rtt_ms = (time.time() - t0) * 1000
        fw = ".".join(str((ver.flight_sw_version >> s) & 0xFF)
                      for s in (24, 16, 8, 0))
        print(f"  OK  PX4 已应答 (往返约 {rtt_ms:.0f} ms), 固件版本: {fw}")
    else:
        print("  FAIL 未收到应答 — PX4 没有收到树莓派发出的消息, 上行链路有问题")

    # ---------- 步骤 4: 请求姿态数据流 ----------
    # 用 MAV_CMD_SET_MESSAGE_INTERVAL (511) 请求 ATTITUDE(消息ID 30) @ 4 Hz,
    # 这是 PX4 推荐方式, 且不依赖 pymavlink 版本中可能缺失的枚举常量
    print(f"\n[4/4] 请求姿态数据流并读取消息 (timeout {args.timeout}s) ...")
    ATTITUDE_MSG_ID = 30
    INTERVAL_US_4HZ = 250000
    mav.mav.command_long_send(
        mav.target_system, mav.target_component,
        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
        ATTITUDE_MSG_ID, INTERVAL_US_4HZ, 0, 0, 0, 0, 0)
    t0 = time.time()
    att = None
    while time.time() - t0 < args.timeout:
        msg = mav.recv_match(type="ATTITUDE", blocking=True, timeout=1)
        if msg:
            att = msg
            break
    results["数据流: ATTITUDE"] = att is not None
    if att:
        print(f"  OK  roll={att.roll:+.3f} rad pitch={att.pitch:+.3f} rad "
              f"yaw={att.yaw:+.3f} rad")
    else:
        print("  FAIL 未收到姿态数据 (PX4 参数 MAV_x_MODE / 数据流配置可能未开)")

    summary(results)
    sys.exit(0 if all(results.values()) else 1)


def summary(results):
    print("\n" + "=" * 56)
    print("测试结果汇总:")
    for k, ok in results.items():
        print(f"  [{'PASS' if ok else 'FAIL'}] {k}")
    if all(results.values()):
        print("\n双向通信正常 ✔  树莓派与 PX4 链路已打通")
    else:
        print("\n存在失败项, 排查建议:")
        print("  - 接线: PX4 TELEM 口 TX->树莓派RX, RX->树莓派TX, GND 共地")
        print("  - PX4 参数: MAV_1_CONFIG=TELEM 2, SER_TEL2_BAUD 与脚本波特率一致")
        print("  - 树莓派: raspi-config 关闭串口控制台、启用串口硬件")


if __name__ == "__main__":
    main()
