#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
8fly · 定高解除/恢复 飞行测试 + T265 定位桥 (alt_hold_test.py) — 一体化版
==========================================================================
单进程独占飞控串口, 同时干两件事:
  1) T265 位姿 -> VISION_POSITION_ESTIMATE 30Hz 发给 PX4 (替代独立桥接脚本)
  2) OFFBOARD 设定点控制: 定点 / 关闭Z定高 (X/Y保持) / 恢复 / 自动循环

测试流程 (飞手主导起降):
  1. 飞手手动起飞, 自己切 POSCTL 悬停
  2. 操作员按 'a' -> 脚本接管 OFFBOARD 并关闭 Z 定高
     (X/Y 仍定点: 树莓派外环 PD; Z 轴固定推力 MPC_THR_HOVER)
  3. 观察高度与水平位置, 按 'h' 恢复定高
  4. 降落由飞手完成 (按 's' 退出脚本, 或飞手直接拨模式开关接管)

控制命令:
  t  接管飞控 (切 OFFBOARD 定点保持; 'a'/'g' 未接管时会自动先执行)
  a  关闭 Z 定高 (X/Y保持 + Z固定推力)
  h  恢复 Z 定高 (全轴定点保持)
  g  自动 N 轮循环: 上升 -> 定高关 -> 定高开 -> 回到起始高度
  s  急停退出 (自动恢复定高, 飞手接管)

安全机制:
  - 漂移看门狗: 定高关期间高度漂移超 --max-drift (默认 0.3m),
    立即自动恢复定高并报警
  - 'a'/'g' 会检查飞机已解锁 (飞手已起飞), 地面上拒绝执行
  - 飞手拨模式开关切离 OFFBOARD: 脚本立即中止
  - 任何中断/异常: 兜底确保先恢复定高再退出

依赖: pip install pymavlink pyrealsense2 numpy
PX4 参数: 沿用原 t265_px4_bridge.py 的 EKF 视觉融合配置 (EKF2_AID_MASK 等)

用法:
  python3 alt_hold_test.py                          # T265桥 + 飞控一体 (默认 /dev/ttyUSB0)
  python3 alt_hold_test.py --thrust 0.55            # 手动指定悬停推力
  python3 alt_hold_test.py --cycles 5 --release-time 3
  python3 alt_hold_test.py --no-t265                # 不启动T265桥 (仅调试指令/户外GPS)
"""

import argparse
import math
import sys
import threading
import time

import numpy as np

try:
    import pyrealsense2 as rs
except ImportError:
    rs = None

# ============================================================
# T265 桥接配置 (沿用原 t265_px4_bridge.py 的参数与坐标映射)
# ============================================================
T265_SEND_RATE_HZ = 30.0        # T265 输出 200Hz, 降频发给飞控
T265_MIN_CONFIDENCE = 2         # tracker_confidence: 0失败 1低 2中 3高

# 线程通信与控制标志
stop_flag = threading.Event()
start_takeover_event = threading.Event()
start_release_event = threading.Event()
start_resume_event = threading.Event()
start_test_event = threading.Event()

# 漂移看门狗状态 (定高关期间生效)
watchdog = {'active': False, 'z_entry': 0.0}

# 命令行参数 (main 里填充, 各处理函数共用)
ARGS = None


# ============================================================
# T265 坐标变换 (与原桥接脚本一致, 未做任何改动)
# ============================================================
def quat_mul(q1, q2):
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    return np.array([
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2,
        w1*w2 - x1*x2 - y1*y2 - z1*z2
    ])


def quat_inverse(q):
    x, y, z, w = q
    norm2 = x*x + y*y + z*z + w*w
    return np.array([-x, -y, -z, w]) / norm2


def remap_pos(p_t265):
    x, y, z = p_t265
    # 相机水平面内偏转90°安装: 新X=-旧Y, 新Y=旧X 叠加基础映射
    return np.array([-x, -z, -y])


def remap_quat(q_t265):
    x, y, z, w = q_t265
    return np.array([-z, x, -y, w])


def quat_to_euler_rad(q):
    x, y, z, w = q
    pitch = -math.asin(np.clip(2.0 * (x * z - w * y), -1, 1))
    roll  = math.atan2(2.0 * (w * x + y * z), w * w - x * x - y * y + z * z)
    yaw   = math.atan2(2.0 * (w * z + x * y), w * w + x * x - y * y - z * z)
    return -pitch, roll, yaw


class T265Bridge:
    """
    T265 位姿 -> VISION_POSITION_ESTIMATE 桥 (后台线程)。
    与 FCClient 共用同一条 MAVLink 链路:
      - 发送经 fc.send_vision_position_estimate() (串口写互斥)
      - TIMESYNC 应答由 FCClient 的唯一接收线程处理 (不在此读串口!)
    跟踪置信度低时暂停发送, 避免毒化 EKF。
    """

    def __init__(self, fc, send_rate=T265_SEND_RATE_HZ, min_confidence=T265_MIN_CONFIDENCE):
        self.fc = fc
        self.send_rate = send_rate
        self.min_confidence = min_confidence
        self.pipe = None
        self._running = False
        self._thread = None
        self.send_count = 0
        self.drop_count = 0
        self.ready = False              # 基准已设定, 开始发送

    def start(self):
        cfg = rs.config()
        cfg.enable_stream(rs.stream.pose)
        self.pipe = rs.pipeline()
        self.pipe.start(cfg)
        self._running = True
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()
        print("T265 已启动, 等待位姿数据 ...")

    def stop(self):
        self._running = False
        if self._thread is not None:
            self._thread.join(timeout=2.0)
            self._thread = None
        if self.pipe is not None:
            try:
                self.pipe.stop()
            except Exception:
                pass
            self.pipe = None

    def _loop(self):
        last_send = 0.0
        last_status = 0.0
        initial_pos = None
        initial_quat = None
        inv_init_quat = None

        while self._running and not stop_flag.is_set():
            try:
                frames = self.pipe.wait_for_frames()
            except Exception as e:
                print(f"\n! [T265] 读取异常: {e}")
                time.sleep(0.5)
                continue
            pose_frame = frames.get_pose_frame()
            if not pose_frame:
                continue
            pose = pose_frame.get_pose_data()

            now = time.time()
            if now - last_send < 1.0 / self.send_rate:
                continue
            last_send = now

            # 跟踪质量检查: 丢失跟踪时不发
            if pose.tracker_confidence < self.min_confidence:
                self.drop_count += 1
                if now - last_status >= 2.0:
                    print(f"\r[T265] 置信度低({pose.tracker_confidence}), "
                          f"暂停发送 (已丢 {self.drop_count})   ", end="")
                    last_status = now
                continue

            pos = remap_pos(np.array([
                pose.translation.x, pose.translation.y, pose.translation.z]))
            quat = remap_quat(np.array([
                pose.rotation.x, pose.rotation.y, pose.rotation.z, pose.rotation.w]))

            # 首帧设为基准 (与原桥接脚本一致: 位置不扣基准, 姿态取相对)
            if initial_pos is None:
                initial_pos = pos.copy()
                initial_quat = quat.copy()
                inv_init_quat = quat_inverse(initial_quat)
                self.ready = True
                print(f"\n[T265] 基准已设定: X前={initial_pos[0]:.3f} "
                      f"Y右={initial_pos[1]:.3f} Z下={initial_pos[2]:.3f} m")
                continue

            quat = quat_mul(inv_init_quat, quat)
            roll, pitch, yaw = quat_to_euler_rad(quat)

            try:
                self.fc.send_vision_position_estimate(
                    pos[0], pos[1], pos[2], roll, pitch, yaw,
                    timestamp_us=int(now * 1e6))
                self.send_count += 1
            except Exception:
                pass                        # 串口瞬断, 下一帧重试

            if now - last_status >= 5.0:
                print(f"\r[T265] 已发 {self.send_count} | "
                      f"X前={pos[0]:+.2f} Y右={pos[1]:+.2f} Z下={pos[2]:+.2f} | "
                      f"conf={pose.tracker_confidence}   ", end="")
                last_status = now


# ============================================================
# 工具函数
# ============================================================
def sleep_interruptible(seconds: float) -> bool:
    """可被打断的睡眠等待。收到停止信号立即返回 False。"""
    t0 = time.time()
    while time.time() - t0 < seconds:
        if stop_flag.is_set():
            return False
        time.sleep(0.05)
    return True


def wait_z(fc, target_z: float, tol: float = 0.06, timeout: float = 10.0) -> bool:
    """等待到达目标高度 (NED z, 向下为正)。返回 True=到位"""
    t0 = time.time()
    while time.time() - t0 < timeout:
        if stop_flag.is_set():
            return False
        pos = fc.get_position()
        if pos is not None and abs(pos[2] - target_z) <= tol:
            return True
        time.sleep(0.1)
    print(f"  ! 等待高度到位超时 (目标 z={target_z:.2f}m)")
    return False


def drift_watchdog(fc):
    """
    后台看门狗线程: 定高关期间每 0.1s 检查高度漂移,
    超过 --max-drift 立即自动恢复定高并报警。
    """
    link_lost_reported = False
    while not stop_flag.is_set():
        if watchdog['active'] and fc.in_offboard:
            if not fc.link_alive():
                # 链路中断: 位置缓存不可信, 看门狗判据失效, 只能报警靠飞手
                if not link_lost_reported:
                    print("\n!!! [看门狗] 飞控链路中断, 位置不可信 "
                          "—— 飞手立即接管! !!!")
                    link_lost_reported = True
            else:
                link_lost_reported = False
                pos = fc.get_position()
                if pos is not None:
                    dz = pos[2] - watchdog['z_entry']   # NED: 正=下降
                    if abs(dz) > ARGS.max_drift:
                        print(f"\n!!! [看门狗] 定高关期间漂移 {dz:+.2f}m "
                              f"超限({ARGS.max_drift}m), 自动恢复定高 !!!")
                        fc.resume_altitude_hold()
                        watchdog['active'] = False
        time.sleep(0.1)


def release_phase(fc, seconds: float) -> bool:
    """
    定高关阶段: 持续 seconds 秒, 漂移由看门狗线程兜底。
    返回 True=平安度过; False=被中断/看门狗触发。
    """
    pos = fc.get_position()
    if pos is None:
        print("  ! 无位置数据, 取消定高关")
        return False
    if not fc.release_altitude_hold():
        print("  ! 无位置/姿态数据, 解除失败")
        return False
    watchdog['z_entry'] = pos[2]
    watchdog['active'] = True

    t0 = time.time()
    while time.time() - t0 < seconds:
        if stop_flag.is_set():
            return False
        if not watchdog['active']:      # 看门狗已触发并自动恢复
            return False
        time.sleep(0.1)
    return True


def listen_command():
    """后台线程：解析键盘输入 (t / a / h / g / s)"""
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
        elif cmd == 't':
            print(">>> 收到接管飞控指令 't'")
            start_takeover_event.set()
        elif cmd == 'a':
            print(">>> 收到关闭定高指令 'a'")
            start_release_event.set()
        elif cmd == 'h':
            print(">>> 收到恢复定高指令 'h'")
            start_resume_event.set()
        elif cmd == 'g':
            print(">>> 收到自动循环指令 'g'")
            start_test_event.set()


# ============================================================
# 指令处理
# ============================================================
def do_takeover(fc) -> bool:
    """[T] 接管飞控: 悬停推力 -> 命令通道自检 -> 切 OFFBOARD 定点"""
    print("\n=== 接管飞控 (T) ===")
    if fc.in_offboard:
        print("已处于 OFFBOARD 定点保持")
        return True

    if ARGS.thrust is not None:
        fc.hover_thrust = ARGS.thrust
        print(f"悬停推力: 手动指定 = {ARGS.thrust:.3f}")
    else:
        h = fc.fetch_hover_thrust()
        if h is not None:
            print(f"悬停推力: MPC_THR_HOVER = {h:.3f}")
        else:
            print(f"! 读取 MPC_THR_HOVER 超时, 使用当前值 {fc.hover_thrust:.3f}")

    if not fc.attitude_fresh():
        print("✕ 姿态数据已停止刷新 (链路中断或接收线程异常), 拒绝接管")
        print("  → 飞行中才出现 = 典型接触不良/EMI: 落地后 dmesg 查 USB 掉线记录")
        return False

    if not fc.tx_link_check():
        print("✕ 命令通道不通: 飞控收不到我们的指令, 无法接管")
        print("  → 检查串口连线 (尤其 USB-TTL TXD -> TELEM2 RX)/是否被其他进程占用")
        print("  → 飞行中才出现 = 典型接触不良/EMI: 落地后 dmesg 查 USB 掉线记录")
        return False

    if fc.takeover_hold():
        print("✓ 已接管: OFFBOARD 定点保持当前位置")
        return True
    print("✕ 接管失败 (检查设定点流/RC 标定/COM_RC_IN_MODE)")
    return False


def do_release_z(fc):
    """[A] 关闭 Z 定高: (未接管则先接管) -> X/Y保持 + Z固定推力"""
    print("\n=== 关闭 Z 定高 (A) ===")
    if not fc.armed:
        print("✕ 飞机未解锁, 拒绝执行 — 请先由飞手起飞并切 POSCTL 悬停")
        return
    if not fc.in_offboard:
        print("尚未接管, 先执行接管流程 ...")
        if not do_takeover(fc):
            print("✕ 接管失败, 无法关闭定高")
            return

    pos = fc.get_position()
    if pos is None:
        print("✕ 无位置数据, 拒绝执行")
        return
    if not fc.release_altitude_hold():
        print("✕ 无姿态数据, 关闭失败")
        return

    watchdog['z_entry'] = pos[2]
    watchdog['active'] = True
    print(f"✓ Z 定高已关: X/Y 定点 + Z 固定推力 {fc.hover_thrust:.3f}")
    print(f"  (看门狗值守中: 漂移超 {ARGS.max_drift}m 自动恢复) 输入 'h' 恢复定高")


def do_resume_z(fc):
    """[H] 恢复 Z 定高: 全轴定点保持"""
    print("\n=== 恢复 Z 定高 (H) ===")
    watchdog['active'] = False
    if not fc.in_offboard:
        print("不在 OFFBOARD (飞手可能已接管)")
        return
    if fc.resume_altitude_hold():
        print("✓ 已恢复全轴定点保持")
    else:
        print("✕ 无位置数据, 恢复失败! 飞手立即接管!")


def run_test(fc, cycles: int, rise: float, release_time: float):
    """[G] 自动 N 轮循环: 上升 -> 定高关 -> 定高开 -> 回到起始高度"""
    if not fc.armed:
        print("✕ 飞机未解锁, 请先由飞手起飞并切 POSCTL 悬停")
        return
    if not fc.in_offboard:
        print("尚未接管, 先执行接管流程 ...")
        if not do_takeover(fc):
            return

    pos0 = fc.get_position()
    if pos0 is None:
        print("✕ 无位置数据, 无法开始")
        return
    x0, y0, z0 = pos0
    print(f"\n起始位置: x={x0:.2f} y={y0:.2f} z={z0:.2f} (NED)")
    print(f"测试参数: {cycles} 轮, 上升 {rise}m, 定高关 {release_time}s, "
          f"漂移限 {ARGS.max_drift}m")

    completed = 0
    try:
        for i in range(1, cycles + 1):
            if stop_flag.is_set():
                break
            print(f"\n========== 第 {i}/{cycles} 轮 ==========")

            # ---- 1. 原地上升 ----
            print(f"  [上升] 目标 z={z0 - rise:.2f}m ...")
            fc.set_position_local(x0, y0, z0 - rise)
            if not wait_z(fc, z0 - rise):
                break
            print("  到位")

            # ---- 2. 定高关 ----
            print(f"  [定高关] X/Y保持 + Z固定推力 {release_time}s "
                  f"(thrust={fc.hover_thrust:.3f}) ...")
            ok = release_phase(fc, release_time)

            # ---- 3. 定高开 (无论结果如何都必须恢复) ----
            print("  [定高开] 恢复全轴定点保持")
            fc.resume_altitude_hold()
            watchdog['active'] = False

            if stop_flag.is_set():
                break
            if not ok:
                print("  !! 定高关阶段异常, 中止循环 (已恢复定高)")
                break

            if not sleep_interruptible(1.0):
                break

            # ---- 4. 回到起始高度 ----
            print(f"  [下降] 回到起始高度 z={z0:.2f}m ...")
            fc.set_position_local(x0, y0, z0)
            if not wait_z(fc, z0):
                break
            print("  已回到起始高度")

            completed += 1
            if not sleep_interruptible(0.5):
                break

        print(f"\n循环结束: 完成 {completed}/{cycles} 轮, 保持定点中")
        print("输入 'g' 再测, 'h/a' 手动关开, 's' 退出 (飞手接管降落)")
    finally:
        watchdog['active'] = False
        # 兜底: 确保不停留在 姿态+推力 模式
        if fc.in_offboard:
            for _ in range(10):
                if fc.resume_altitude_hold():
                    break
                time.sleep(0.2)
            else:
                print("!!! 恢复定高失败 (无位置数据), 飞手请立即接管 !!!")


# ============================================================
# 主流程
# ============================================================
def main():
    global ARGS
    parser = argparse.ArgumentParser(description="定高解除/恢复 飞行测试 + T265桥 (一体化)")
    parser.add_argument("--fc-port", default=None,
                        help="飞控串口, 默认 fc_client.DEFAULT_PORT (/dev/ttyUSB0)")
    parser.add_argument("--fc-baud", type=int, default=None,
                        help="飞控波特率, 默认 fc_client.DEFAULT_BAUD (921600)")
    parser.add_argument("--thrust", type=float, default=None,
                        help="手动指定悬停推力 0~1 (默认读取 PX4 参数 MPC_THR_HOVER)")
    parser.add_argument("--cycles", type=int, default=3, help="'g' 循环轮数 (默认 3)")
    parser.add_argument("--rise", type=float, default=0.2, help="每轮上升高度 m (默认 0.2)")
    parser.add_argument("--release-time", type=float, default=2.0,
                        help="每轮定高关持续时间 s (默认 2.0)")
    parser.add_argument("--max-drift", type=float, default=0.3,
                        help="定高关期间允许的最大高度漂移 m, 超限自动恢复 (默认 0.3)")
    parser.add_argument("--no-t265", action="store_true",
                        help="不启动 T265 桥 (仅调试指令, 或户外 GPS 定位)")
    args = parser.parse_args()
    ARGS = args

    print("==============================================")
    print("  8fly - 定高解除/恢复 飞行测试 + T265 定位桥  ")
    print("==============================================")
    print("流程: 飞手手动起飞并切 POSCTL 悬停")
    print("      -> 按 'a' 关闭 Z 定高 (X/Y 保持)")
    print("      -> 按 'h' 恢复定高; 降落由飞手完成")
    print("指令:  t=接管  a=关定高  h=恢复  g=自动循环  s=急停退出")
    print("----------------------------------------------")

    from fc_client import FCClient, DEFAULT_PORT, DEFAULT_BAUD
    if not hasattr(FCClient, 'send_vision_position_estimate'):
        print("✕ 检测到 fc_client.py 是旧版本 (缺少 send_vision_position_estimate)")
        print("  请把最新的 fc_client.py 一并拷贝到本目录后再运行!")
        sys.exit(1)

    def _on_mode_lost(old, new):
        """飞手把模式切离 OFFBOARD -> 立即中止"""
        print(f"\n!!! 飞手切离 OFFBOARD ({old} -> {new}), 中止测试 !!!")
        stop_flag.set()

    fc_port = args.fc_port or DEFAULT_PORT
    fc_baud = args.fc_baud or DEFAULT_BAUD
    print(f"正在连接飞控: {fc_port} @ {fc_baud} ...")
    try:
        # 设定点 20Hz: X/Y 保持外环更平滑
        fc = FCClient(fc_port, baud=fc_baud, on_mode_lost=_on_mode_lost,
                      setpoint_rate=20)
    except Exception as e:
        print(f"✕ 飞控连接失败: {e}")
        print("  检查接线/波特率/串口是否被占用")
        sys.exit(1)
    print(f"飞控已连接, 当前模式: {fc.mode_name}")

    # ---- 启动 T265 桥 (定位源) ----
    t265 = None
    if not args.no_t265:
        if rs is None:
            print("✕ 未安装 pyrealsense2 (pip install pyrealsense2), 或用 --no-t265")
            sys.exit(1)
        try:
            t265 = T265Bridge(fc)
            t265.start()
        except Exception as e:
            print(f"✕ T265 启动失败: {e}")
            print("  检查相机连接, 或用 --no-t265 跳过 (仅调试)")
            sys.exit(1)
    else:
        print("--no-t265: 不启动视觉桥 (确认另有定位源, 如 GPS)")

    # ---- 等待 EKF 位置就绪 ----
    print("等待 EKF 位置数据 (视觉融合需要几秒) ...")
    if fc.wait_position(10.0):
        pos = fc.get_position()
        print(f"✓ 位置有效: x={pos[0]:.2f} y={pos[1]:.2f} z={pos[2]:.2f} (NED)")
    else:
        print("! 10s 内未收到有效位置 — 检查 T265 数据流 / EKF2_AID_MASK 配置")
        print("  (继续运行, 但接管/定高关将无法执行)")

    # ---- 后台线程: 键盘 + 看门狗 ----
    threading.Thread(target=listen_command, daemon=True).start()
    threading.Thread(target=drift_watchdog, args=(fc,), daemon=True).start()

    print("\n飞手起飞并 POSCTL 悬停后, 随时可以输入指令 (t/a/h/g/s)...")

    try:
        while not stop_flag.is_set():
            time.sleep(0.05)

            if start_takeover_event.is_set():
                start_takeover_event.clear()
                try:
                    do_takeover(fc)
                except Exception as e:
                    print(f"✕ 接管流程异常: {e} (程序未退出, 可重试)")

            if start_release_event.is_set():
                start_release_event.clear()
                try:
                    do_release_z(fc)
                except Exception as e:
                    print(f"✕ 关闭定高异常: {e}")

            if start_resume_event.is_set():
                start_resume_event.clear()
                try:
                    do_resume_z(fc)
                except Exception as e:
                    print(f"✕ 恢复定高异常: {e}")

            if start_test_event.is_set():
                start_test_event.clear()
                try:
                    run_test(fc, args.cycles, args.rise, args.release_time)
                except Exception as e:
                    print(f"✕ 循环测试异常: {e} (已触发兜底恢复)")
                    watchdog['active'] = False
                    if fc.in_offboard:
                        fc.resume_altitude_hold()
    finally:
        print("\n正在清理并退出...")
        watchdog['active'] = False
        if fc.in_offboard:
            fc.resume_altitude_hold()
        print("!!! 飞控设定点即将停止, 飞手请立即接管遥控器 !!!")
        if t265 is not None:
            t265.stop()
        fc.close()
        print("程序结束。")


if __name__ == '__main__':
    main()
