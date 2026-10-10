#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
8fly · 机械臂惯量扰动定点测试
==================================
目的: 检测机械臂活动造成的整机惯量/质心变化, 是否影响无人机定点保持。

说明: T265 桥接进程占用飞控串口 (TELEM2), 本脚本不连接飞控,
      只执行机械臂动作序列。每个阶段打印墙钟时间戳, 可事后与
      T265 桥接端的位置记录按时间对齐分析。

流程 (风扇全程不动):
  1. 两臂归零静止 BASELINE_S 秒 (基线段)
  2. 循环 CYCLES 轮, 每轮对每个摆动关节:
       两臂同名关节同时: +ANGLE 度 -> 保持 -> 回零(复位) -> 保持
       两臂同名关节同时: -ANGLE 度 -> 保持 -> 回零(复位) -> 保持

中止: 任意时刻 用户按 q+回车 / Ctrl+C -> 机械臂急停 + 归零
      (本脚本不接管飞控, 定点由飞手/PX4 负责)

用法:
    python3 test_inertia.py                          # 串口用文件头默认值
    python3 test_inertia.py --arm /dev/ttyUSB1
    python3 test_inertia.py --joints 1,2,3,4 --cycles 5

真机前请先按 README「真机前 checklist」逐项确认。
"""

import argparse
import sys
import threading
import time

from arm_client import ArmClient, find_port

# ------------------------------------------------------------
# 测试参数 (所有可调参数集中在此)
# ------------------------------------------------------------
ARM_PORT = '/dev/ttyUSB1'      # 机械臂 Arduino 串口 (None=自动查找)
ARM_BAUD = 115200

# ---- 动作序列 ----
ARMS = (1, 2)                  # 参与测试的臂号, 同名关节同时活动
SWING_JOINTS = (1, 2, 3, 4)    # 摆动的关节号 (1-based); 全部4个关节参与
ANGLE_DEG = 30.0               # 摆动幅度 ±度
CYCLES = 3                     # 重复轮数
BASELINE_S = 8.0               # 基线时长 (两臂归零静止)
HOLD_S = 1.5                   # 每个姿态到位后的保持时间 (供观察扰动后稳态)
MOVE_TIMEOUT_S = 20.0          # 单次移动 wait_done 超时


def _stamp():
    """墙钟时间戳 HH:MM:SS.mmm, 用于与 T265 记录对齐"""
    return time.strftime('%H:%M:%S') + ('.%03d' % int(time.time() * 1000 % 1000))


class InertiaTest:
    """惯量扰动定点测试 (arm 可注入 mock 以便测试)"""

    def __init__(self, arm, angle=ANGLE_DEG, cycles=CYCLES, hold_s=HOLD_S,
                 baseline_s=BASELINE_S, arms=ARMS, swing_joints=SWING_JOINTS):
        self.arm = arm
        self.angle = angle
        self.cycles = cycles
        self.hold_s = hold_s
        self.baseline_s = baseline_s
        self.arms = arms
        self.swing_joints = swing_joints      # 1-based 关节号

        self.abort_event = threading.Event()
        self.abort_reason = ''

    # ---------------- 中止 ----------------
    def abort(self, reason):
        if not self.abort_event.is_set():
            self.abort_reason = reason
            print('!! 中止: %s' % reason)
            self.abort_event.set()

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

    # ---------------- 动作执行 ----------------
    def _target_angles(self, joint, sign):
        """构造 4 关节目标角: 指定关节 = sign*ANGLE, 其余 0"""
        angles = [0.0, 0.0, 0.0, 0.0]
        angles[joint - 1] = sign * self.angle
        return angles

    def _sleep_check(self, seconds):
        """睡眠 but 可中止。返回 False=已中止"""
        t0 = time.time()
        while time.time() - t0 < seconds and not self.abort_event.is_set():
            time.sleep(0.05)
        return not self.abort_event.is_set()

    def _move_and_hold(self, joint, angles, cycle, direction, label):
        """两臂同名关节同时下发目标角, 一起等完成, 保持 HOLD_S。返回 False=已中止"""
        if self.abort_event.is_set():
            return False
        print('[%s] 轮%d 关节%d(两臂同时) %s: %s'
              % (_stamp(), cycle, joint, label,
                 ['%+.1f' % a for a in angles]))
        for arm_num in self.arms:
            self.arm.move_joint(arm_num, angles)
        # 固件单一 moving 标志, 两臂到位后只回一次 DONE
        if not self.arm.wait_done(MOVE_TIMEOUT_S):
            self.abort('关节%d 移动超时 (%s)' % (joint, label))
            return False
        if self.arm.last_error is not None:
            self.abort('关节%d 报错 0x%02X (%s)' % (joint, self.arm.last_error, label))
            return False
        print('[%s] 到位, 保持 %.0fs ...' % (_stamp(), self.hold_s))
        return self._sleep_check(self.hold_s)

    def run_sequence(self):
        """主序列: CYCLES 轮 x 每个摆动关节(两臂同时) +A/复位/-A/复位"""
        zero = [0.0, 0.0, 0.0, 0.0]
        for cycle in range(1, self.cycles + 1):
            for joint in self.swing_joints:
                # +ANGLE -> 复位
                if not self._move_and_hold(joint, self._target_angles(joint, +1),
                                           cycle, +1, 'swing_pos'):
                    return False
                if not self._move_and_hold(joint, zero, cycle, 0, 'reset'):
                    return False
                # -ANGLE -> 复位
                if not self._move_and_hold(joint, self._target_angles(joint, -1),
                                           cycle, -1, 'swing_neg'):
                    return False
                if not self._move_and_hold(joint, zero, cycle, 0, 'reset'):
                    return False
        return True

    # ---------------- 主流程 ----------------
    def run(self):
        try:
            self._start_key_watcher()
            print('[提示] 飞手保持定点悬停; q+回车 / Ctrl+C 可随时中止')
            print('[提示] 各阶段已打墙钟时间戳, 可与 T265 位置记录对齐')

            print('[%s] [基线] 两臂归零, 静止 %.0fs ...'
                  % (_stamp(), self.baseline_s))
            self.arm.home()
            if not self.arm.wait_done(MOVE_TIMEOUT_S):
                self.abort('归零超时')
                return
            if not self._sleep_check(self.baseline_s):
                return

            self.run_sequence()
            if not self.abort_event.is_set():
                print('[%s] [完成] 测试序列结束' % _stamp())
        except KeyboardInterrupt:
            self.abort('Ctrl+C')
        finally:
            print('[%s] [收尾] 机械臂急停 + 归零 (风扇全程未动) ...' % _stamp())
            try:
                self.arm.stop()
                self.arm.home()
            except Exception:
                pass
            if self.abort_reason:
                print('[收尾] 中止原因: %s' % self.abort_reason)


def main():
    ap = argparse.ArgumentParser(description='8fly 机械臂惯量扰动定点测试')
    ap.add_argument('--arm', default=ARM_PORT, help='机械臂串口 (默认 %(default)s)')
    ap.add_argument('--angle', type=float, default=ANGLE_DEG,
                    help='摆动幅度 ±度 (默认 %(default)s)')
    ap.add_argument('--cycles', type=int, default=CYCLES,
                    help='重复轮数 (默认 %(default)s)')
    ap.add_argument('--hold', type=float, default=HOLD_S,
                    help='每个姿态保持秒数 (默认 %(default)s)')
    ap.add_argument('--baseline', type=float, default=BASELINE_S,
                    help='基线静止秒数 (默认 %(default)s)')
    ap.add_argument('--joints', default=','.join(str(j) for j in SWING_JOINTS),
                    help='摆动关节号, 逗号分隔 1~4 (默认 %(default)s)')
    args = ap.parse_args()

    swing_joints = tuple(int(j) for j in args.joints.split(',') if j.strip())
    if not swing_joints or any(j < 1 or j > 4 for j in swing_joints):
        print('--joints 需为 1~4 的关节号, 逗号分隔')
        sys.exit(1)

    arm_port = args.arm or find_port()
    if not arm_port:
        print('未找到机械臂串口, 用 --arm 指定')
        sys.exit(1)

    print('连接机械臂 %s ...' % arm_port)
    with ArmClient(arm_port, ARM_BAUD) as arm:
        InertiaTest(arm, angle=args.angle, cycles=args.cycles,
                    hold_s=args.hold, baseline_s=args.baseline,
                    swing_joints=swing_joints).run()


if __name__ == '__main__':
    main()
