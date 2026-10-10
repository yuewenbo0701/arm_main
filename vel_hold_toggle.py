# -*- coding: utf-8 -*-
"""
T265 -> PX4 桥接 + Offboard 速度/位置设定值发送（xyz全掩切换版）
====================================================================

功能 1 (与 alt_hold_toggle.py 相同):
    读取 T265 位姿, 转换到 NED 系 (X前/Y右/Z下), 降频到 30Hz
    通过 VISION_POSITION_ESTIMATE 发给 Pixhawk, 供 EKF2 融合定点。

功能 2 (本脚本改动):
    以 10Hz 间隔定时发送 SET_POSITION_TARGET_LOCAL_NED 设定值,
    定点目标 = 启动时 / 按键切换时 T265 测得的当前位置,
    键盘实时切换"xyz 位置全掩"模式:
        按 a : xyz 位置全掩 (忽略 x/y/z 位置, 改用 vx=vy=vz=0 三轴零速保持,
               飞机就地刹车悬停, 不再锚定任何位置点)
        按 b : 恢复位置定点 (以当前位置为新定点, 恢复位置掩码)
        按 q : 退出脚本

    注意 (PX4 v1.17 源码行为, 与 alt_hold_toggle.py 同一约束):
    位置控制器要求每个轴至少有一个有效设定点 (位置/速度/加速度三选一)。
    xyz 位置全掩且速度也全忽略 -> 输入无效, 200ms 后 PX4 内部 failsafe
    刹车。所以全掩位置不能只加 IGNORE_PX/PY/PZ, 必须同时放开速度环:
        3576 | XYZ_IGNORE(7) - V_IGNORE(56) = 3527
    与 Z_OFF_MASK=3548 (+4 掩z位置, -32 放开vz) 是同一思路推广到三轴。

依赖: pip install pyrealsense2 pymavlink numpy
运行: python vel_hold_toggle.py
"""

import sys
import time
import math
import threading
import numpy as np
import pyrealsense2 as rs
from pymavlink import mavutil

# ========== 参数区 ==========
MAVLINK_ADDR   = '/dev/ttyUSB0'   # Pixhawk USB 直连
MAVLINK_BAUD   = 921600
SEND_RATE_HZ   = 30.0             # 视觉定位发送频率 (T265 输出 200Hz, 降频发给飞控)
MIN_CONFIDENCE = 2                # tracker_confidence: 0失败 1低 2中 3高

SETPOINT_HZ    = 10.0             # 设定值发送频率 (PX4 Offboard 要求 >= 2Hz)

# 位置掩码: 忽略速度(56) + 忽略加速度(448) + 忽略Yaw/Yaw_rate(3072) = 3576
BASE_MASK = 3576
# xyz全掩掩码: BASE_MASK 上忽略 xyz 位置(+7), 同时放开速度环(-56) 并给
# vx=vy=vz=0 -> 3527: 三轴零速保持 (PX4 速度环闭环调姿态/推力就地刹车)
VEL_HOLD_MASK = 3527

# ========== 全局状态 ==========
pos_enabled = True      # True = xyz 位置定点; False = xyz全掩 (vx=vy=vz=0 零速保持)
running     = True
lock        = threading.Lock()

cur_pos   = None        # T265 最新 NED 位置 (x前/y右/z下), 视觉线程更新
hold_xyz  = None        # 当前定点目标; 首帧位置初始化, 每次按键切换时刷新

# ========== 四元数工具 ==========
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

# ========== T265 坐标系 -> NED 重映射 ==========
def remap_pos(p_t265):
    x, y, z = p_t265
    # 基础映射旧结果为 (-z, x, -y)，叠加90°旋转后:
    return np.array([-x, -z, -y])

def remap_quat(q_t265):
    x, y, z, w = q_t265
    return np.array([-z, x, -y, w])

# ========== 四元数 -> 欧拉角（ZYX，输出单位: 弧度） ==========
def quat_to_euler_rad(q):
    x, y, z, w = q
    pitch = -math.asin(np.clip(2.0 * (x * z - w * y), -1, 1))
    roll  = math.atan2(2.0 * (w * x + y * z), w * w - x * x - y * y + z * z)
    yaw   = math.atan2(2.0 * (w * z + x * y), w * w + x * x - y * y - z * z)
    return -pitch, roll, yaw

# ========== 键盘监听线程: a xyz全掩, b 恢复位置定点, q 退出 ==========
def keyboard_listener():
    global pos_enabled, running, hold_xyz

    restore_termios = None
    try:
        import msvcrt                                  # Windows
        def read_key():
            if msvcrt.kbhit():
                return msvcrt.getch().decode('utf-8', errors='ignore').lower()
            return None
    except ImportError:                                # Linux / macOS
        import tty, termios, select
        fd = sys.stdin.fileno()
        old = termios.tcgetattr(fd)
        tty.setcbreak(fd)
        def restore_termios():                         # 退出时恢复终端, 否则 shell 会乱
            termios.tcsetattr(fd, termios.TCSADRAIN, old)
        def read_key():
            if select.select([sys.stdin], [], [], 0)[0]:
                return sys.stdin.read(1).lower()
            return None

    print("\n[键盘] a=xyz位置全掩(vx=vy=vz=0 零速保持)  "
          "b=恢复位置定点(取当前位置)  q=退出")
    try:
        while running:
            try:
                key = read_key()
            except Exception:
                break
            if key == 'a':
                with lock:
                    pos_enabled = False
                print("\n>>> [a] xyz位置已全掩: mask=%d "
                      "(忽略xyz位置, vx=vy=vz=0 零速保持)" % VEL_HOLD_MASK)
            elif key == 'b':
                with lock:
                    if cur_pos is not None:
                        hold_xyz = cur_pos.copy()      # 以当前位置为新定点
                    pos_enabled = True
                print("\n>>> [b] 位置定点已恢复: mask=%d (定点=当前位置)" % BASE_MASK)
            elif key == 'q':
                running = False
                break
            time.sleep(0.05)
    finally:
        if restore_termios:
            restore_termios()

# ========== 发送设定值 (带内部定时, 由主循环调用) ==========
def send_setpoint(master):
    """根据当前 pos_enabled 状态计算掩码并发送一条设定值。
    位置模式: 定点 = 最近一次启动/按键时锁定的当前位置。
    全掩模式: 三轴速度置 0, 不依赖任何位置点 (T265 尚无位置时也能发,
              上电即可让飞机先刹车稳住)。"""
    with lock:
        pos   = None if hold_xyz is None else hold_xyz.copy()
        p_on  = pos_enabled
    if p_on:
        if pos is None:
            return
        mask = BASE_MASK                             # 3576: x/y/z 位置
        x, y, z = pos[0], pos[1], pos[2]
        vx = vy = vz = 0.0                           # 速度被忽略, 值随意
    else:
        mask = VEL_HOLD_MASK                         # 3527: xyz全掩 + 三轴零速
        x = y = z = 0.0                              # 位置被忽略, 值随意
        vx = vy = vz = 0.0                           # 零速保持, 就地刹车
    master.mav.set_position_target_local_ned_send(
        0,                                   # time_boot_ms
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,
        mask,                                # type_mask
        x, y, z,                             # 位置 (仅在 BASE_MASK 下生效)
        vx, vy, vz,                          # 速度 (仅在 VEL_HOLD_MASK 下生效)
        0, 0, 0,                             # afx, afy, afz (被忽略)
        0, 0                                 # yaw, yaw_rate (被忽略)
    )

# ========== 主流程 ==========
def main():
    global running, cur_pos, hold_xyz

    # ---- 连接飞控 ----
    print(f"连接飞控 {MAVLINK_ADDR} @ {MAVLINK_BAUD} ...")
    master = mavutil.mavlink_connection(MAVLINK_ADDR, baud=MAVLINK_BAUD)
    master.wait_heartbeat()
    print(f"已连接: sysid={master.target_system} compid={master.target_component}")

    # ---- 启动 T265 ----
    pipe = rs.pipeline()
    cfg = rs.config()
    cfg.enable_stream(rs.stream.pose)
    pipe.start(cfg)
    print("T265 已启动，等待位姿数据...")

    # ---- 启动键盘监听 ----
    t = threading.Thread(target=keyboard_listener, daemon=True)
    t.start()

    print(f"[发送] 视觉定位 {SEND_RATE_HZ:.0f}Hz + 设定值 {SETPOINT_HZ:.0f}Hz "
          f"(定点=启动/按键时的当前位置)")

    # ---- 定时器 ----
    last_vision_send   = 0.0
    next_setpoint_send = time.time()

    send_count = 0
    drop_count = 0

    initial_pos  = None
    inv_init_quat = None

    try:
        while running:
            # 取帧带 100ms 超时: 即使相机帧没来, 循环也能准时走,
            # 保证设定值和 TIMESYNC 应答不会停摆
            try:
                frames = pipe.wait_for_frames(100)
            except RuntimeError:
                frames = None

            # ---- 应答 PX4 的 TIMESYNC（不做这步两边时钟对不上，EKF 会拒收数据） ----
            while True:
                msg = master.recv_match(blocking=False)
                if msg is None:
                    break
                if msg.get_type() == 'TIMESYNC':
                    master.mav.timesync_send(int(time.time_ns()), msg.ts1)

            now = time.time()

            # ---- 任务1: 到点就发设定值 (SETPOINT_HZ 间隔定时) ----
            if now >= next_setpoint_send:
                send_setpoint(master)
                next_setpoint_send = now + 1.0 / SETPOINT_HZ

            if frames is None:
                continue
            pose_frame = frames.get_pose_frame()
            if not pose_frame:
                continue
            pose = pose_frame.get_pose_data()

            # ---- 视觉定位降频 ----
            if now - last_vision_send < 1.0 / SEND_RATE_HZ:
                continue
            last_vision_send = now

            # ---- 跟踪质量检查：丢失跟踪时不发，避免毒化 EKF ----
            if pose.tracker_confidence < MIN_CONFIDENCE:
                drop_count += 1
                print(f"\r[警告] 跟踪置信度低({pose.tracker_confidence})，暂停发送视觉数据，已丢 {drop_count} 帧",
                      end="")
                continue

            # ---- 坐标转换 ----
            pos = remap_pos(np.array([
                pose.translation.x,
                pose.translation.y,
                pose.translation.z
            ]))
            quat = remap_quat(np.array([
                pose.rotation.x,
                pose.rotation.y,
                pose.rotation.z,
                pose.rotation.w
            ]))

            # ---- 更新当前位置; 首个有效位置即为初始定点目标 ----
            with lock:
                cur_pos = pos.copy()
                if hold_xyz is None:
                    hold_xyz = pos.copy()

            if initial_pos is None:
                initial_pos = pos.copy()
                inv_init_quat = quat_inverse(quat.copy())
                rel_euler = quat_to_euler_rad(quat)
                print("基准已设定")
                print(f"初始位置: X前={initial_pos[0]:.3f} "
                      f"Y右={initial_pos[1]:.3f} Z下={initial_pos[2]:.3f} m")
                print(f"初始姿态: Roll={rel_euler[0]:.1f} "
                      f"Pitch={rel_euler[1]:.1f} Yaw={rel_euler[2]:.1f}°")
                continue

            quat = quat_mul(inv_init_quat, quat)
            roll, pitch, yaw = quat_to_euler_rad(quat)

            # ---- 任务2: 发送 VISION_POSITION_ESTIMATE ----
            # 协方差第一元素置 NaN -> PX4 使用 EKF2_EVP_NOISE / EKF2_EVA_NOISE 默认噪声
            cov = [float('nan')] * 21
            master.mav.vision_position_estimate_send(
                int(now * 1e6),                  # UNIX 时间戳(us)，靠 TIMESYNC 对齐
                float(pos[0]), float(pos[1]), float(pos[2]),
                float(roll), float(pitch), float(yaw),
                cov
            )
            send_count += 1

            with lock:
                p_on = pos_enabled
            print(
                f"\r已发送 {send_count} | "
                f"X前={pos[0]:+.3f} Y右={pos[1]:+.3f} Z下={pos[2]:+.3f} m | "
                f"R={math.degrees(roll):+.1f} P={math.degrees(pitch):+.1f} "
                f"Y={math.degrees(yaw):+.1f}° | conf={pose.tracker_confidence} | "
                f"模式={'位置定点' if p_on else 'xyz全掩(v=0)'} "
                f"mask={BASE_MASK if p_on else VEL_HOLD_MASK}",
                end=""
            )

    except KeyboardInterrupt:
        print("\n停止。")

    finally:
        running = False
        t.join(timeout=1.0)     # 让键盘线程恢复终端设置
        pipe.stop()
        master.close()

if __name__ == '__main__':
    main()
