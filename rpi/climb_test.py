#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
双机械臂 · 爬升测试脚本 (climb_test.py) — PX4 交互已全部关闭
==========================================================
注意: PX4 飞控交互已全部关闭 (不连接飞控 / 无 OFFBOARD 接管 / 无定高解除),
      本脚本现为纯机械臂控制, 等同原 --no-fc 模式。
      恢复方法: 把下方 PX4_ENABLED 改为 True 即恢复全部飞控功能。

控制命令:
  l  初始化: 臂1和臂2电机同时移动至 [10.8, -9.1, 18.5]，不开启风扇
  t  (已关闭) 接管飞控: PX4 交互已关闭, 此指令无效
  k  开始爬升循环 (State 1 -> State 2 -> State 3)
  s  急停并退出脚本 (关闭风扇并归零)

状态说明:
  State 1 (抬臂2):
    - 臂1风扇开(1150) -> 0.5s
    - 臂2风扇关(1000) -> 臂2移至[25.1, -11.9, 0.0] -> 0.5s
    - 臂2移至[31.7, 0.5, 0.0] -> 0.5s -> 臂2风扇开(1150)
    - 完成后等待 1.5s
  State 2 (抬臂1):
    - 与 State 1 镜像对称 (臂1与臂2角色对调)
    - 完成后等待 1.5s
  State 3 (双臂复位/上抬):
    - 臂1和臂2同时移至 [10.8, -9.1, 18.5]，等待完成
    - 完成后等待 1.0s
    - (原"解除/恢复定高"飞控动作已随 PX4 关闭而停用)

定高解除原理 (历史说明, 当前已停用):
  alt_valid 是 PX4 估计器 (EKF2) 的内部状态, MAVLink 无法从外部写入;
  它一旦失效会触发 failsafe (掉高/降落), 不是"软开关"。
  正确做法: OFFBOARD 下把设定点从 SET_POSITION_TARGET_LOCAL_NED 换成
  SET_ATTITUDE_TARGET (期望倾角 + 悬停推力), PX4 位置/速度控制器被旁路
  = Z 轴定高关; 同时树莓派外环把 X/Y 位置误差转成期望倾角, 保住 X/Y
  定点。结束后换回位置设定点 = 定高开。全程设定点流不断、不切飞行
  模式, PX4 无感切换, 无 failsafe。悬停推力读取 PX4 参数
  MPC_THR_HOVER, 也可用 --thrust 手动指定。

用法:
  python3 climb_test.py                        # 纯机械臂控制 (PX4 已关闭)
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

FAN_SPEED_ON = 1150
FAN_SPEED_OFF = 1000

# PX4 飞控交互总开关: False = 全部关闭 (不连飞控/无接管/无定高解除)
PX4_ENABLED = False

# 线程通信与控制标志
stop_flag = threading.Event()
start_init_event = threading.Event()
start_takeover_event = threading.Event()
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
    """后台线程：解析键盘输入 (l / t / k / s)"""
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
        elif cmd == 't':
            print(">>> 收到接管飞控指令 't'")
            start_takeover_event.set()
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


def do_takeover(fc, thrust_override=None):
    """
    [T 指令逻辑] 接管飞控 (当前已停用: PX4_ENABLED = False):
    - 确定悬停推力 (手动指定优先, 否则读取 PX4 参数 MPC_THR_HOVER)
    - 切 OFFBOARD 并定点保持当前位置
    """
    print("\n=== 接管飞控 (T) ===")
    if fc.in_offboard:
        print("已处于 OFFBOARD 定点保持, 无需重复接管")
        return

    if thrust_override is not None:
        fc.hover_thrust = thrust_override
        print(f"悬停推力: 手动指定 = {thrust_override:.3f}")
    else:
        h = fc.fetch_hover_thrust()
        if h is not None:
            print(f"悬停推力: MPC_THR_HOVER = {h:.3f}")
        else:
            print(f"! 读取 MPC_THR_HOVER 超时, 使用当前值 {fc.hover_thrust:.3f}")

    if not fc.wait_attitude(3.0):
        print("! 未收到 ATTITUDE_QUATERNION, State 3 将无法解除定高")

    # 命令通道自检: 不通就别尝试切 OFFBOARD (切了也是白等 6 秒)
    if not fc.tx_link_check():
        print("✕ 命令通道不通: 飞控收不到我们的指令, 无法接管")
        print("  → 心跳/姿态收得到但参数读不到, 典型是 USB-TTL TXD -> TELEM2 RX 断线")
        print("  → 检查连线/虚焊后重新输入 't'")
        return

    if fc.takeover_hold():
        print("✓ 已接管: OFFBOARD 定点保持当前位置")
        print("  现在可以输入 'k' 开始爬升循环")
    else:
        print("✕ 接管失败 (检查设定点流/RC 标定/COM_RC_IN_MODE)")


def step_single_arm_climb(arm_client: ArmClient, active_arm: int, idle_arm: int) -> bool:
    """
    通用爬升单步流程 (封装 State 1 和 State 2 的对称逻辑)
    - active_arm: 本次抬起/移动的臂
    - idle_arm: 本次作为吸附支撑的臂
    """
    # 1. 支撑臂开启风扇 -> 等待 0.5s
    print(f"  [吸附] 臂{idle_arm} 风扇开启 ({FAN_SPEED_ON}us)...")
    arm_client.set_fan(idle_arm, FAN_SPEED_ON)
    if not sleep_interruptible(0.5): return False

    # 2. 动作臂关闭风扇 -> 移动至过渡角 -> 等待 Done -> 等待 0.5s
    print(f"  [释放] 臂{active_arm} 风扇关闭")
    arm_client.fan_off(active_arm)

    print(f"  [抬臂1阶段] 臂{active_arm} 移动至 {MID_ANGLES}...")
    arm_client.move_joint(active_arm, MID_ANGLES)
    if not wait_for_move(arm_client): return False
    if not sleep_interruptible(0.5): return False

    # 3. 动作臂移动至目标角 -> 等待 Done -> 等待 0.5s -> 打开动作臂风扇
    print(f"  [抬臂2阶段] 臂{active_arm} 移动至 {TARGET_ANGLES}...")
    arm_client.move_joint(active_arm, TARGET_ANGLES)
    if not wait_for_move(arm_client): return False
    if not sleep_interruptible(0.5): return False

    print(f"  [吸附] 臂{active_arm} 风扇开启 ({FAN_SPEED_ON}us)")
    arm_client.set_fan(active_arm, FAN_SPEED_ON)

    # 阶段完成后统一等待 1.5 秒
    print("  状态完成，等待 1.5s ...")
    return sleep_interruptible(1.5)


def run_loop(arm_client: ArmClient, fc, max_cycles: int = 0):
    """
    [K 指令逻辑] State 1 -> State 2 -> State 3 状态机循环
    State 3 (双臂同步动作) 期间自动解除定高, 完成后恢复。
    fc=None (PX4 已关闭 / --no-fc) 时退化为纯机械臂循环。
    """
    global loop_active

    # 连接了飞控就必须先接管, 否则 State 3 的定高解除无从谈起
    if fc is not None and not fc.in_offboard:
        print("✕ 飞控尚未接管, State 3 无法解除定高。")
        print("  请先输入 't' 接管飞控; 纯机械臂测试请用 --no-fc 重启脚本")
        return

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

            if not sleep_interruptible(0.5):
                break

            # ---- State 3: 双臂同步上抬/复位 (PX4 已关闭, 纯机械臂动作) ----
            print(f"\n--> State 3: 双臂同时移动至 {INIT_ANGLES}")
            alt_released = False
            if fc is not None:
                if not fc.in_offboard:
                    print("  ! [飞控] 已不在 OFFBOARD, 无法解除定高, 终止循环")
                    break
                print("  [飞控] 解除定高 -> X/Y保持 + Z固定推力 "
                      f"(thrust={fc.hover_thrust:.3f})")
                if not fc.release_altitude_hold():
                    print("  ! [飞控] 无姿态数据, 解除定高失败, 终止循环")
                    break
                alt_released = True
                # 等 10Hz 设定点流完成类型切换 (2~3 个周期)
                if not sleep_interruptible(0.3):
                    fc.resume_altitude_hold()
                    break

            arm_client.move_joint(1, INIT_ANGLES)
            arm_client.move_joint(2, INIT_ANGLES)

            move_ok = wait_for_move(arm_client)

            # 无论移动成功/中断/超时, 先恢复定高再决定是否退出循环
            if alt_released:
                print("  [飞控] 恢复定高 -> 当前位置定点保持")
                fc.resume_altitude_hold()

            if not move_ok:
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
    parser = argparse.ArgumentParser(description="双机械臂爬升测试 (climb_test, PX4 已关闭)")
    parser.add_argument("--port", default=None, help="机械臂串口设备地址, 默认自动匹配")
    parser.add_argument("--fc-port", default=None,
                        help="(已停用) 飞控串口 (TELEM2), PX4_ENABLED=False 时无效")
    parser.add_argument("--fc-baud", type=int, default=None,
                        help="(已停用) 飞控波特率, PX4_ENABLED=False 时无效")
    parser.add_argument("--no-fc", action="store_true",
                        help="纯机械臂模式: 不连接飞控 (PX4 已关闭, 此参数当前无实际作用)")
    parser.add_argument("--thrust", type=float, default=None,
                        help="(已停用) 手动指定悬停推力 0~1, PX4_ENABLED=False 时无效")
    parser.add_argument("--cycles", type=int, default=0, help="运行循环次数, 0 表示无限循环")
    args = parser.parse_args()

    port = args.port or find_port()
    if not port:
        print("✕ 未找到 Arduino 串口，请检查连线或通过 --port 指定！")
        sys.exit(1)

    print("==============================================")
    print("  8fly 双机械臂 - 爬升测试 (PX4 交互已关闭)    ")
    print("==============================================")
    print("控制指令说明:")
    print("  l : 执行初始化姿态 (电机到达指定角度, 不开风扇)")
    print("  t : (已关闭) 接管飞控 - PX4 交互已全部关闭, 此指令无效")
    print("  k : 开始爬升循环 (State 1 -> State 2 -> State 3)")
    print("  s : 停止循环 / 急停退出")
    print("----------------------------------------------")
    print(f"正在连接机械臂: {port} ...")

    with ArmClient(port) as arm:
        # ---- 连接飞控 (PX4_ENABLED=False 已全部关闭, 跳过) ----
        fc = None
        if PX4_ENABLED and not args.no_fc:
            try:
                from fc_client import FCClient, DEFAULT_PORT, DEFAULT_BAUD
            except ImportError as e:
                print(f"✕ 无法导入 fc_client ({e})")
                print("  请先 pip install pymavlink, 或加 --no-fc 进行纯机械臂测试")
                sys.exit(1)

            if not hasattr(FCClient, 'fetch_hover_thrust'):
                print("✕ 检测到 fc_client.py 是旧版本 (缺少 fetch_hover_thrust)")
                print("  请把最新的 fc_client.py 一并拷贝到本目录后再运行!")
                sys.exit(1)

            def _on_mode_lost(old, new):
                """飞手把模式切离 OFFBOARD -> 立即中止爬升"""
                print(f"\n!!! 飞手切离 OFFBOARD ({old} -> {new}), 中止爬升并急停 !!!")
                stop_flag.set()

            fc_port = args.fc_port or DEFAULT_PORT
            fc_baud = args.fc_baud or DEFAULT_BAUD
            print(f"正在连接飞控: {fc_port} @ {fc_baud} ...")
            try:
                fc = FCClient(fc_port, baud=fc_baud, on_mode_lost=_on_mode_lost)
            except Exception as e:
                print(f"✕ 飞控连接失败: {e}")
                print("  检查接线/波特率/MAV_1 参数, 或加 --no-fc 进行纯机械臂测试")
                sys.exit(1)
            print(f"飞控已连接, 当前模式: {fc.mode_name}")
            print("请先输入 't' 接管飞控, 再输入 'k' 开始爬升")
        else:
            print("PX4 交互已全部关闭: 纯机械臂模式 (不连接飞控, 无接管/定高解除)")

        # 启动键盘监听后台线程
        listener = threading.Thread(target=listen_command, daemon=True)
        listener.start()

        print("设备连接成功, 随时可以输入指令 (l/t/k/s)...")

        try:
            # 主线程事件循环
            while not stop_flag.is_set():
                time.sleep(0.05)

                # 触发初始化
                if start_init_event.is_set():
                    start_init_event.clear()
                    do_init(arm)

                # 触发接管飞控 (PX4 已关闭时提示无效)
                if start_takeover_event.is_set():
                    start_takeover_event.clear()
                    if fc is not None:
                        try:
                            do_takeover(fc, args.thrust)
                        except Exception as e:
                            print(f"✕ 接管流程异常中断: {e} (程序未退出, 可重试 't')")
                    else:
                        print("PX4 交互已全部关闭 (纯机械臂模式), 't' 指令无效")

                # 触发爬升循环
                if start_loop_event.is_set():
                    start_loop_event.clear()
                    run_loop(arm, fc, args.cycles)
        finally:
            # 退出清理
            print("\n正在停止并清理系统安全状态...")
            arm.fan_off(1)
            arm.fan_off(2)
            arm.stop()
            print("风扇已关，命令已停止。")
            if fc is not None:
                print("!!! 飞控设定点即将停止, 飞手请立即接管遥控器 !!!")
                fc.close()
            print("程序结束。")


if __name__ == '__main__':
    main()
