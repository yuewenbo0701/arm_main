"""
T265 -> PX4 桥接脚本
读取 T265 位姿，转换到 NED 系（X前/Y右/Z下），
通过 MAVLink VISION_POSITION_ESTIMATE 发给 Pixhawk，供 EKF2 融合定点。

相机安装朝向：水平面内偏转 90°（镜头朝机身侧面），
对应关系（实测）：新X=-旧Y，新Y=旧X，ROLL=-旧PITCH，PITCH=旧ROLL。

依赖: pip install pyrealsense2 pymavlink numpy
运行: python t265_px4_bridge.py
"""

import time
import math
import numpy as np
import pyrealsense2 as rs
from pymavlink import mavutil

# ========== 参数区 ==========
MAVLINK_ADDR   = '/dev/ttyUSB0'   # Pixhawk USB 直连；走 TELEM2 改成 '/dev/ttyUSB0'
MAVLINK_BAUD   = 921600
SEND_RATE_HZ   = 30.0             # T265 输出 200Hz，降频发给飞控
MIN_CONFIDENCE = 2                # tracker_confidence: 0失败 1低 2中 3高

# ========== 四元数乘法 ==========
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
    #return roll, pitch, yaw
    return -pitch,roll,yaw

# ========== 连接飞控 ==========
print(f"连接飞控 {MAVLINK_ADDR} @ {MAVLINK_BAUD} ...")
master = mavutil.mavlink_connection(MAVLINK_ADDR, baud=MAVLINK_BAUD)
master.wait_heartbeat()
print(f"已连接: sysid={master.target_system} compid={master.target_component}")

# ========== 启动 T265 ==========
pipe = rs.pipeline()
cfg = rs.config()
cfg.enable_stream(rs.stream.pose)
pipe.start(cfg)
print("T265 已启动，等待位姿数据...")

last_send = 0.0
send_count = 0
drop_count = 0

initial_pos = None
initial_quat = None
inv_init_quat = None

try:
    while True:
        frames = pipe.wait_for_frames()
        pose_frame = frames.get_pose_frame()
        if not pose_frame:
            continue
        pose = pose_frame.get_pose_data()

        # ---- 应答 PX4 的 TIMESYNC（不做这步两边时钟对不上，EKF 会拒收数据） ----
        while True:
            msg = master.recv_match(blocking=False)
            if msg is None:
                break
            if msg.get_type() == 'TIMESYNC':
                master.mav.timesync_send(int(time.time_ns()), msg.ts1)

        # ---- 降频 ----
        now = time.time()
        if now - last_send < 1.0 / SEND_RATE_HZ:
            continue
        last_send = now

        # ---- 跟踪质量检查：丢失跟踪时不发，避免毒化 EKF ----
        if pose.tracker_confidence < MIN_CONFIDENCE:
            drop_count += 1
            print(f"\r[警告] 跟踪置信度低({pose.tracker_confidence})，暂停发送，已丢 {drop_count} 帧", end="")
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

        if initial_pos is None:

            initial_pos = pos.copy()
            initial_quat = quat.copy()

            inv_init_quat = quat_inverse(initial_quat)

            rel_euler = quat_to_euler_rad(initial_quat)

            print("基准已设定")
            print(
                f"初始位置: "
                f"X前={initial_pos[0]:.3f} "
                f"Y右={initial_pos[1]:.3f} "
                f"Z下={initial_pos[2]:.3f} m"
            )

            print(
                f"初始姿态: "
                f"Roll={rel_euler[0]:.1f} "
                f"Pitch={rel_euler[1]:.1f} "
                f"Yaw={rel_euler[2]:.1f}°"
            )

            continue

        quat = quat_mul(
            inv_init_quat,
            quat
        )

        roll, pitch, yaw = quat_to_euler_rad(quat)

        # ---- 发送 VISION_POSITION_ESTIMATE ----
        # 协方差第一元素置 NaN -> PX4 使用 EKF2_EVP_NOISE / EKF2_EVA_NOISE 默认噪声
        cov = [float('nan')] * 21
        master.mav.vision_position_estimate_send(
            int(now * 1e6),                      # UNIX 时间戳(us)，靠 TIMESYNC 对齐
            float(pos[0]), float(pos[1]), float(pos[2]),
            float(roll), float(pitch), float(yaw),
            cov
        )
        send_count += 1

        print(
            f"\r已发送 {send_count} | "
            f"X前={pos[0]:+.3f} Y右={pos[1]:+.3f} Z下={pos[2]:+.3f} m | "
            f"R={math.degrees(roll):+.1f} P={math.degrees(pitch):+.1f} "
            f"Y={math.degrees(yaw):+.1f}° | conf={pose.tracker_confidence}",
            end=""
        )

except KeyboardInterrupt:
    print("\n停止。")

finally:
    pipe.stop()
    master.close()
