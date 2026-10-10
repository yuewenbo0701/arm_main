#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
8fly · 贴墙吸附任务 (树莓派总调度)
==================================
流程:
  1. 飞手遥控器手动起飞, 飞到墙面前 (本脚本等待用户回车确认)
  2. 接管: Pixhawk(PX4) 切 OFFBOARD, 定点保持当前位置
  3. 闭环接近: 读 VL53L1X 激光距离, P 控制机体系前向速度 vx, 接近至 30cm
  4. 伸出机械臂: 预备位 -> 下压位
  5. 开涵道风扇吸附, 监测 FSR 原始值饱和 -> 保持吸附
中止: 任意时刻 飞手切模式 / 用户按 q+回车 / Ctrl+C
      -> 停飞控设定点 + 机械臂急停

用法:
    python3 mission.py                              # 串口用文件头默认值
    python3 mission.py --arm /dev/ttyUSB0 --fc /dev/ttyUSB1

真机前请先按 README「真机前 checklist」逐项确认。
"""

import argparse
import sys
import threading
import time

from arm_client import ArmClient, find_port

# ------------------------------------------------------------
# 任务参数 (所有可调参数集中在此)
# ------------------------------------------------------------
ARM_PORT = '/dev/ttyUSB0'      # 机械臂 Arduino 串口 (None=自动查找)
ARM_BAUD = 115200
FC_PORT = '/dev/ttyUSB1'       # Pixhawk TELEM2 串口 (USB-TTL 3.3V)
FC_BAUD = 921600               # 已确认: PX4 + TELEM2 + 921600

# ---- 接近控制 ----
TARGET_DISTANCE_MM = 300       # 目标贴墙距离 (激光读数)
APPROACH_KP = 0.002            # P 增益: vx[m/s] = KP * (d - target)[mm]
APPROACH_VX_MAX = 0.30         # 接近速度上限 (m/s)
APPROACH_TOL_MM = 20           # 到位判据: |d - target| <= 容差
APPROACH_OK_FRAMES = 5         # 连续 N 个控制周期在容差内才算到位
APPROACH_TIMEOUT_S = 30.0      # 接近阶段总超时

LASER_MAX_VALID_MM = 4000      # 激光量程上限 (VL53L1X Long 约 4m)
STATUS_TIMEOUT_S = 1.0         # 超过此时间没收到状态帧 -> 数据失效 -> 中止
CTRL_PERIOD_S = 0.1            # 接近控制周期 (10Hz)

# ---- 伸臂 / 吸附 (角度待用户确认, 默认沿用固件自动序列, 臂1) ----
ARM_NUM = 1
POSE_PREPARE = (0, 20, 20, -40)    # 预备位
POSE_PRESS = (-40, 20, 20, -40)    # 下压位
MOVE_TIMEOUT_S = 20.0          # 单次移动 wait_done 超时
FAN_THROTTLE = 1200            # 吸附油门 (us)

# ---- FSR 饱和判定 (fsr_raw = ADC 原始值 0~1023, 固件每 500ms 更新) ----
FSR_JUDGE_DELAY_S = 2.0        # 开风扇后延时再判定 (跳过 ramp 上升段)
FSR_SAT_FRAMES = 8             # 连续 N 帧 fsr_raw 不增长 -> 判定饱和
FSR_SAT_MIN_RAW = 50           # 饱和值下限, 防止传感器断线 (恒 0) 误判
FSR_ADC_THRESHOLD = None       # 阈值判定: fsr_raw >= 此值即饱和; None=禁用
FSR_TIMEOUT_S = 20.0           # 吸附判定总超时
FSR_POLL_S = 0.2               # FSR 判定轮询周期 (状态帧 200ms 一帧)
HOLD_PERIOD_S = 2.0            # 吸附保持阶段状态打印周期


class Mission:
    """贴墙吸附任务状态机 (arm/fc 可注入 mock 以便测试)"""

    def __init__(self, arm, fc):
        self.arm = arm
        self.fc = fc
        self.abort_event = threading.Event()
        self.abort_reason = ''

    # ---------------- 中止 ----------------
    def abort(self, reason):
        if not self.abort_event.is_set():
            self.abort_reason = reason
            print('!! 中止: %s' % reason)
            self.abort_event.set()

    def _on_mode_lost(self, old, new):
        self.abort('飞手切模式: %s -> %s' % (old, new))

    def _start_key_watcher(self):
        """后台线程: 用户输入 q+回车 -> 中止"""
        def watch():
            try:
                for line in sys.stdin:
                    if line.strip().lower() == 'q':
                        self.abort('用户按键 q')
                        return
            except Exception:
                pass
        threading.Thread(target=watch, daemon=True).start()

    # ---------------- 阶段 0: 等飞手 ----------------
    def wait_pilot_ready(self):
        print('=' * 56)
        print(' 8fly 贴墙吸附任务')
        print(' 1) 确认真机 checklist 已逐项完成 (见 README)')
        print(' 2) 飞手遥控器手动起飞, 飞到墙面前悬停')
        print(' 3) 回车后树莓派接管 (OFFBOARD 定点 -> 接近 -> 吸附)')
        print('    接管后按 q+回车 或 飞手切模式 可随时中止')
        print('=' * 56)
        input('>>> 就绪后按回车开始自动阶段 ...')

    # ---------------- 阶段 1: 接近 ----------------
    def _read_laser(self):
        """返回有效激光距离 mm; 无效(0/超量程/状态过期)返回 None"""
        s = self.arm.status
        if s is None:
            return None
        if time.time() - self.arm.status_time > STATUS_TIMEOUT_S:
            return None
        if s.laser_mm <= 0 or s.laser_mm > LASER_MAX_VALID_MM:
            return None
        return s.laser_mm

    def approach(self):
        """P 控制闭环接近至 TARGET_DISTANCE_MM。返回 True=到位, False=中止"""
        print('[接近] 目标 %dmm, vx 上限 %.2fm/s' % (TARGET_DISTANCE_MM, APPROACH_VX_MAX))
        ok_frames = 0
        t0 = time.time()
        last_print = 0.0
        while not self.abort_event.is_set():
            if time.time() - t0 > APPROACH_TIMEOUT_S:
                self.abort('接近超时 (%.0fs)' % APPROACH_TIMEOUT_S)
                break
            d = self._read_laser()
            if d is None:
                self.abort('激光数据无效/超量程/超时')
                break
            err = d - TARGET_DISTANCE_MM
            if abs(err) <= APPROACH_TOL_MM:
                ok_frames += 1
                if ok_frames >= APPROACH_OK_FRAMES:
                    print('[接近] 到位: %dmm' % d)
                    self.fc.set_velocity_body(0)
                    self.fc.hold_current_position()
                    return True
            else:
                ok_frames = 0
            vx = max(-APPROACH_VX_MAX, min(APPROACH_VX_MAX, APPROACH_KP * err))
            self.fc.set_velocity_body(vx)
            if time.time() - last_print >= 1.0:
                print('[接近] d=%dmm  vx=%+.2fm/s' % (d, vx))
                last_print = time.time()
            time.sleep(CTRL_PERIOD_S)
        # 中止 -> 立即悬停
        self.fc.set_velocity_body(0)
        self.fc.hold_current_position()
        return False

    # ---------------- 阶段 2: 伸臂 + 吸附 ----------------
    def extend_and_attach(self):
        """伸臂 -> 开风扇 -> 等 FSR 饱和。返回 True=已吸附, False=中止"""
        print('[伸臂] 臂%d 预备位 %s' % (ARM_NUM, POSE_PREPARE))
        self.arm.move_joint(ARM_NUM, POSE_PREPARE)
        if not self.arm.wait_done(MOVE_TIMEOUT_S):
            self.abort('预备位移动超时')
            return False
        if self.abort_event.is_set():
            return False

        print('[伸臂] 臂%d 下压位 %s' % (ARM_NUM, POSE_PRESS))
        self.arm.move_joint(ARM_NUM, POSE_PRESS)
        if not self.arm.wait_done(MOVE_TIMEOUT_S):
            self.abort('下压位移动超时')
            return False
        if self.abort_event.is_set():
            return False

        print('[吸附] 臂%d 风扇开 (%dus), 等待 FSR 饱和 ...' % (ARM_NUM, FAN_THROTTLE))
        self.arm.fan_on(ARM_NUM, FAN_THROTTLE)
        t_delay = time.time() + FSR_JUDGE_DELAY_S
        while time.time() < t_delay and not self.abort_event.is_set():
            time.sleep(0.05)

        last = None
        stable = 0
        t0 = time.time()
        while not self.abort_event.is_set():
            if time.time() - t0 > FSR_TIMEOUT_S:
                self.abort('FSR 饱和判定超时 (%.0fs)' % FSR_TIMEOUT_S)
                break
            s = self.arm.status
            raw = None
            if s is not None:
                raw = s.fsr1_raw if ARM_NUM == 1 else s.fsr2_raw
            if raw is None:
                # 旧固件无 fsr_raw 字段, 会一直等到 FSR_TIMEOUT_S 中止
                time.sleep(FSR_POLL_S)
                continue
            if FSR_ADC_THRESHOLD is not None and raw >= FSR_ADC_THRESHOLD:
                print('[吸附] FSR 达到阈值: %d >= %d' % (raw, FSR_ADC_THRESHOLD))
                return True
            if last is not None and raw <= last:
                stable += 1
                if stable >= FSR_SAT_FRAMES and raw >= FSR_SAT_MIN_RAW:
                    print('[吸附] FSR 饱和: %d (连续%d帧未增长)' % (raw, stable))
                    return True
            else:
                stable = 0
            last = raw
            time.sleep(FSR_POLL_S)
        return False

    # ---------------- 阶段 3: 吸附保持 ----------------
    def hold_loop(self):
        """定点悬停 + 保持风扇, 直到中止"""
        print('[保持] 吸附保持中 (q+回车 / 飞手切模式 / Ctrl+C 中止)')
        while not self.abort_event.is_set():
            s = self.arm.status
            if s is not None:
                print('[保持] 激光=%dmm  fan%d=%dus  fsr_raw=%s,%s  模式=%s'
                      % (s.laser_mm, ARM_NUM,
                         s.fan1 if ARM_NUM == 1 else s.fan2,
                         s.fsr1_raw, s.fsr2_raw, self.fc.mode_name))
            time.sleep(HOLD_PERIOD_S)

    # ---------------- 主流程 ----------------
    def run(self):
        try:
            self.wait_pilot_ready()
            self._start_key_watcher()

            print('[接管] 切 OFFBOARD 定点保持 ...')
            if not self.fc.takeover_hold():
                self.abort('接管失败 (切 OFFBOARD 未确认)')
            elif self.approach():
                if self.extend_and_attach():
                    self.hold_loop()
        except KeyboardInterrupt:
            self.abort('Ctrl+C')
        finally:
            print('[收尾] 停飞控设定点 + 机械臂急停 ...')
            try:
                self.fc.release()
            except Exception:
                pass
            try:
                self.arm.stop()
            except Exception:
                pass
            if self.abort_reason:
                print('[收尾] 中止原因: %s' % self.abort_reason)
            print('[收尾] 完成, 请飞手接管降落。')


def main():
    ap = argparse.ArgumentParser(description='8fly 贴墙吸附任务')
    ap.add_argument('--arm', default=ARM_PORT, help='机械臂串口 (默认 %(default)s)')
    ap.add_argument('--fc', default=FC_PORT, help='飞控串口 (默认 %(default)s)')
    ap.add_argument('--fc-baud', type=int, default=FC_BAUD, help='飞控波特率')
    args = ap.parse_args()

    from fc_client import FCClient   # 延迟导入: pymavlink 仅真机需要

    arm_port = args.arm or find_port()
    if not arm_port:
        print('未找到机械臂串口, 用 --arm 指定')
        sys.exit(1)

    print('连接机械臂 %s ...' % arm_port)
    with ArmClient(arm_port, ARM_BAUD) as arm:
        print('连接飞控 %s @%d ...' % (args.fc, args.fc_baud))
        with FCClient(args.fc, args.fc_baud) as fc:
            print('飞控已连接, 当前模式: %s' % fc.mode_name)
            m = Mission(arm, fc)
            fc.on_mode_lost = m._on_mode_lost   # 飞手切模式 -> 中止
            m.run()


if __name__ == '__main__':
    main()
