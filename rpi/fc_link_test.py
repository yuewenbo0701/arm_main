#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
8fly · 树莓派 <-> 飞控 链路双向测试 (fc_link_test.py)
=====================================================
纯地面诊断脚本, 不接管飞控/不发设定点/不解锁, 只验证串口链路两个方向:

  方向1  飞控 -> 树莓派:  能否收到心跳, 各消息流的实际速率
  方向2  树莓派 -> 飞控:  发 PARAM_REQUEST_READ / COMMAND_LONG, 等 PARAM_VALUE
                          / COMMAND_ACK 回包 —— 有回包才证明飞控"收到"了我们的消息
                          (单向收到心跳不代表命令通道通, 典型故障是
                           USB-TTL TXD -> TELEM2 RX 断线)

用法:
  python3 fc_link_test.py                        # 默认 /dev/ttyUSB0 @ 921600
  python3 fc_link_test.py --port /dev/ttyUSB1
  python3 fc_link_test.py --rounds 20            # 命令通道打 20 个来回统计成功率

退出码: 0=双向正常, 1=连接失败, 2=飞控->树莓派异常, 3=树莓派->飞控异常
"""

import argparse
import sys
import time
from collections import Counter

from pymavlink import mavutil


def test_rx(master, duration=5.0):
    """方向1: 统计 duration 秒内收到的各类消息及速率"""
    print(f"\n[方向1] 飞控 -> 树莓派: 统计 {duration:.0f}s 内收到的消息 ...")
    counts = Counter()
    t0 = time.time()
    while time.time() - t0 < duration:
        try:
            msg = master.recv_match(blocking=True, timeout=1.0)
        except Exception as e:
            print(f"  ! 接收异常: {e}")
            break
        if msg is None:
            continue
        counts[msg.get_type()] += 1
        if msg.get_type() == 'BAD_DATA':
            continue

    total = sum(counts.values())
    if total == 0:
        print("  ✕ 一条消息都没收到! 检查 USB-TTL RXD <- TELEM2 TX 连线")
        return False
    for mtype, n in counts.most_common():
        print(f"    {mtype:<24} {n:>4} 条  ({n / duration:5.1f} Hz)")
    if counts.get('BAD_DATA', 0) > total * 0.05:
        print("  ! BAD_DATA 占比偏高 —— 波特率不匹配或线受干扰")
        return False
    print("  ✓ 下行链路正常")
    return True


def test_tx_param(master, rounds=10, timeout=1.0):
    """
    方向2a: 反复请求 SYS_AUTOSTART 参数, 统计 PARAM_VALUE 回包成功率/延迟。
    回包 = 飞控确实收到并处理了我们的请求。
    """
    print(f"\n[方向2] 树莓派 -> 飞控: 参数请求打 {rounds} 个来回 ...")
    pid = b'SYS_AUTOSTART'
    lat = []
    lost = 0
    for i in range(rounds):
        master.mav.param_request_read_send(
            master.target_system, master.target_component, pid, -1)
        t0 = time.time()
        msg = None
        while time.time() - t0 < timeout:
            m = master.recv_match(type='PARAM_VALUE', blocking=True,
                                  timeout=timeout - (time.time() - t0))
            if m is None:
                break
            mid = m.param_id
            if isinstance(mid, bytes):
                mid = mid.decode('ascii', 'ignore')
            if mid.rstrip('\x00') == 'SYS_AUTOSTART':
                msg = m
                break
        if msg is None:
            lost += 1
            print(f"    第 {i + 1:>2} 次: ✕ 超时无回包")
        else:
            dt = (time.time() - t0) * 1000
            lat.append(dt)
            print(f"    第 {i + 1:>2} 次: ✓ 回包 {dt:5.1f} ms "
                  f"(值={msg.param_value:.0f})")
        time.sleep(0.1)

    ok = rounds - lost
    print(f"  成功率: {ok}/{rounds}", end="")
    if lat:
        print(f" | 回包延迟 平均 {sum(lat) / len(lat):.1f} ms, "
              f"最大 {max(lat):.1f} ms")
    else:
        print()
    if ok == 0:
        print("  ✕ 命令通道不通: 飞控收不到我们的消息!")
        print("    → 重点检查 USB-TTL 的 TXD -> TELEM2 RX 这根线, 以及是否共地")
        return False
    if ok < rounds:
        print("  ! 有丢包: 线序/接触不良或串口被其他进程占用")
        return False
    print("  ✓ 命令通道正常")
    return True


def test_tx_command(master, timeout=2.0):
    """
    方向2b: 发 COMMAND_LONG (MAV_CMD_REQUEST_MESSAGE 请求 AUTOPILOT_VERSION),
    等 COMMAND_ACK + AUTOPILOT_VERSION, 双向都验证了命令通路。
    """
    print("\n[方向2] 命令应答测试: 请求 AUTOPILOT_VERSION ...")
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE, 0,
        mavutil.mavlink.MAVLINK_MSG_ID_AUTOPILOT_VERSION, 0, 0, 0, 0, 0, 0)
    t0 = time.time()
    got_ack = got_ver = False
    while time.time() - t0 < timeout:
        msg = master.recv_match(type=['COMMAND_ACK', 'AUTOPILOT_VERSION'],
                                blocking=True,
                                timeout=timeout - (time.time() - t0))
        if msg is None:
            break
        if msg.get_type() == 'COMMAND_ACK':
            if msg.command == mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE:
                got_ack = True
                ok = (msg.result == mavutil.mavlink.MAV_RESULT_ACCEPTED)
                print(f"    COMMAND_ACK: {'ACCEPTED' if ok else f'result={msg.result}'}")
        elif msg.get_type() == 'AUTOPILOT_VERSION':
            got_ver = True
            print(f"    AUTOPILOT_VERSION 已收到 "
                  f"(flight_sw_version=0x{msg.flight_sw_version:08x})")
    if got_ack and got_ver:
        print("  ✓ 飞控收到并执行了我们的命令")
        return True
    print(f"  ✕ 应答不完整 (ACK={'有' if got_ack else '无'}, "
          f"VERSION={'有' if got_ver else '无'}) —— 命令通道异常")
    return False


def main():
    parser = argparse.ArgumentParser(description="树莓派 <-> 飞控 链路双向测试")
    parser.add_argument("--port", default="/dev/ttyUSB0",
                        help="飞控串口 (默认 /dev/ttyUSB0)")
    parser.add_argument("--baud", type=int, default=921600,
                        help="波特率 (默认 921600)")
    parser.add_argument("--rounds", type=int, default=10,
                        help="参数请求来回次数 (默认 10)")
    parser.add_argument("--rx-time", type=float, default=5.0,
                        help="下行消息统计时长 s (默认 5)")
    args = parser.parse_args()

    print("==============================================")
    print("  8fly - 树莓派 <-> 飞控 链路双向测试")
    print("==============================================")
    print(f"串口: {args.port} @ {args.baud}")
    print("说明: 本脚本不接管飞控/不发设定点/不解锁, 可在地面安全运行")
    print("----------------------------------------------")

    try:
        master = mavutil.mavlink_connection(args.port, baud=args.baud)
    except Exception as e:
        print(f"✕ 打不开串口: {e}")
        print("  检查设备号/权限/是否被其他进程占用")
        sys.exit(1)

    print("等待飞控心跳 (10s) ...")
    try:
        hb = master.wait_heartbeat(timeout=10)
    except Exception:
        hb = None
    if hb is None:
        print("✕ 10s 内解析不出心跳, 做原始字节嗅探定位原因 ...")
        raw = b''
        t0 = time.time()
        while time.time() - t0 < 3.0:
            try:
                n = master.port.in_waiting
                if n:
                    raw += master.port.read(min(n, 4096))
            except Exception:
                pass
            time.sleep(0.05)
        if len(raw) == 0:
            print("✕ 端口一片死寂: 3s 内连一个原始字节都没有")
            print("  → 设备号不对 (试 --port /dev/ttyUSB1, 两个USB-TTL枚举顺序会变)")
            print("  → 或飞控没往这个口发: 查 PX4 参数 MAV_1_CONFIG(=TELEM2)")
        else:
            sample = ' '.join(f'{b:02x}' for b in raw[:16])
            print(f"✕ 端口有数据 ({len(raw)} 字节/3s, 前16字节: {sample} ...)")
            print("  但解析不出合法心跳, 两种典型原因:")
            print("  → 另一个进程 (t265桥/alt_hold_test) 正在抢读同一串口,")
            print("    每个字节只被其中一个进程读到! 先 sudo lsof /dev/ttyUSB0 查占用")
            print("  → 波特率不匹配: 确认 MAV_1_RATE 与 --baud 一致 (921600)")
        sys.exit(2)
    ap = {mavutil.mavlink.MAV_AUTOPILOT_PX4: 'PX4',
          mavutil.mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA: 'ArduPilot'}.get(
        hb.autopilot, f'type={hb.autopilot}')
    print(f"✓ 心跳正常: sysid={master.target_system} "
          f"compid={master.target_component} 固件={ap}")

    rx_ok = test_rx(master, args.rx_time)
    tx_ok = test_tx_param(master, args.rounds) and test_tx_command(master)

    print("\n==============================================")
    if rx_ok and tx_ok:
        print("  结论: 双向链路正常 ✓  (树莓派能发, 飞控能收能回)")
        code = 0
    elif rx_ok:
        print("  结论: 下行通, 上行(树莓派->飞控)不通 ✕")
        print("        → 查 USB-TTL TXD -> TELEM2 RX 连线和共地")
        code = 3
    else:
        print("  结论: 下行(飞控->树莓派)异常 ✕")
        print("        → 查 USB-TTL RXD <- TELEM2 TX 连线/波特率")
        code = 2
    print("==============================================")
    master.close()
    sys.exit(code)


if __name__ == '__main__':
    main()
