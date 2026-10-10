#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
8fly · 机械臂随机组合扰动测试
================================
目的: 八个电机同时以随机角度活动, 制造不规则的惯量/质心扰动,
      观察对无人机定点的综合影响。

说明: 每次生成一个随机姿态: 两臂各 4 个关节独立在 [0, +LIM] 度内随机,
      八个电机同时下发、同时到位, 保持 HOLD_S 后继续下一个姿态。
      所有角度非负, 不超过 +30°。

用法:
    python3 test_random_pose.py
    python3 test_random_pose.py --arm /dev/ttyUSB1 --count 30 --limit 25

中止: 任意时刻 q+回车 / Ctrl+C -> 急停 + 归零
"""

import argparse
import random
import sys
import threading
import time

from arm_client import ArmClient, find_port

# ------------------------------------------------------------
# 测试参数
# ------------------------------------------------------------
ARM_PORT = '/dev/ttyUSB1'
ARM_BAUD = 115200

POS_LIMIT = 30.0               # 每个关节上限 +度 (下限 0)
POSE_COUNT = 20                # 随机姿态数量
HOLD_S = 0.5                   # 到位后保持时间
MOVE_TIMEOUT_S = 20.0          # 单次移动超时


def _stamp():
    return time.strftime('%H:%M:%S') + ('.%03d' % int(time.time() * 1000 % 1000))


class RandomPoseTest:
    def __init__(self, arm, pos_limit=POS_LIMIT, pose_count=POSE_COUNT,
                 hold_s=HOLD_S):
        self.arm = arm
        self.pos_limit = pos_limit
        self.pose_count = pose_count
        self.hold_s = hold_s

        self.abort_event = threading.Event()
        self.abort_reason = ''

    def abort(self, reason):
        if not self.abort_event.is_set():
            self.abort_reason = reason
            print('!! 中止: %s' % reason)
            self.abort_event.set()

    def _start_key_watcher(self):
        def watch():
            try:
                for line in sys.stdin:
                    if line.strip().lower() == 'q':
                        self.abort('用户按键 q')
                        return
            except Exception:
                pass
        threading.Thread(target=watch, daemon=True).start()

    def _random_angles(self):
        """生成 4 关节随机角度, 范围 [0, pos_limit]"""
        return [random.uniform(0.0, self.pos_limit) for _ in range(4)]

    def _sleep_check(self, seconds):
        t0 = time.time()
        while time.time() - t0 < seconds and not self.abort_event.is_set():
            time.sleep(0.05)
        return not self.abort_event.is_set()

    def run(self):
        try:
            self._start_key_watcher()
            print('[提示] q+回车 / Ctrl+C 可随时中止')
            print('[%s] 起始归零 ...' % _stamp())
            self.arm.home()
            if not self.arm.wait_done(MOVE_TIMEOUT_S):
                self.abort('归零超时')
                return

            for i in range(1, self.pose_count + 1):
                if self.abort_event.is_set():
                    break

                a1 = self._random_angles()
                a2 = self._random_angles()
                print('[%s] 姿态 %d/%d  臂1:%s  臂2:%s'
                      % (_stamp(), i, self.pose_count,
                         ['%+.1f' % a for a in a1],
                         ['%+.1f' % a for a in a2]))

                self.arm.move_joint(1, a1)
                self.arm.move_joint(2, a2)

                if not self.arm.wait_done(MOVE_TIMEOUT_S):
                    self.abort('姿态 %d 移动超时' % i)
                    return
                if self.arm.last_error is not None:
                    self.abort('姿态 %d 报错 0x%02X' % (i, self.arm.last_error))
                    return

                if not self._sleep_check(self.hold_s):
                    return

            if not self.abort_event.is_set():
                print('[%s] [完成] 随机序列结束' % _stamp())
        except KeyboardInterrupt:
            self.abort('Ctrl+C')
        finally:
            print('[%s] [收尾] 急停 + 归零 ...' % _stamp())
            try:
                self.arm.stop()
                self.arm.home()
            except Exception:
                pass
            if self.abort_reason:
                print('[收尾] 中止原因: %s' % self.abort_reason)


def main():
    ap = argparse.ArgumentParser(description='8fly 随机组合扰动测试')
    ap.add_argument('--arm', default=ARM_PORT, help='机械臂串口')
    ap.add_argument('--count', type=int, default=POSE_COUNT,
                    help='随机姿态数量 (默认 %(default)s)')
    ap.add_argument('--limit', type=float, default=POS_LIMIT,
                    help='单关节上限 +度 (默认 %(default)s)')
    ap.add_argument('--hold', type=float, default=HOLD_S,
                    help='到位保持秒数 (默认 %(default)s)')
    args = ap.parse_args()

    arm_port = args.arm or find_port()
    if not arm_port:
        print('未找到机械臂串口, 用 --arm 指定')
        sys.exit(1)

    random.seed()  # 用系统时间初始化
    print('连接机械臂 %s ...' % arm_port)
    with ArmClient(arm_port, ARM_BAUD) as arm:
        RandomPoseTest(arm, pos_limit=args.limit, pose_count=args.count,
                       hold_s=args.hold).run()


if __name__ == '__main__':
    main()
