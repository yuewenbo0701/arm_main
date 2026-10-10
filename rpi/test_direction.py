#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
8fly · 舵机正方向标定 (地面测试)
==================================
目的: 确定每个舵机的运动学正方向 (+) 对应实机的哪个转向。

流程 (仅地面进行, 风扇全程不动):
  1. 所有舵机先归零
  2. 臂1 关节1~4、臂2 关节1~4 轮流:
       当前关节 0° -> +ANGLE 度 -> 保持 -> 回 0° -> 保持
  3. 保持期间观察并记录实机转向 (俯视顺时针/逆时针、抬头/低头等)

中止: 任意时刻 q+回车 / Ctrl+C -> 机械臂急停 + 归零

用法:
    python3 test_direction.py                        # 每关节保持 3s 自动走下一个
    python3 test_direction.py --wait                 # 每关节按键回车才走下一个
    python3 test_direction.py --angle 10 --hold 5    # 自定义角度/保持秒数
    python3 test_direction.py --arm 2                # 只测臂2
    python3 test_direction.py --joints 2,3           # 只测指定关节

注意: 地面测试请卸下桨叶/不装负载, 确认臂活动范围内无障碍。
"""

import argparse
import sys
import threading
import time

from arm_client import ArmClient, find_port

# ------------------------------------------------------------
# 测试参数 (所有可调参数集中在此)
# ------------------------------------------------------------
ARM_PORT = '/dev/ttyUSB0'      # 机械臂 Arduino 串口 (None=自动查找)
ARM_BAUD = 115200

ANGLE_DEG = 10.0               # 正转幅度 (度, 从归零位起)
HOLD_S = 3.0                   # 每个姿态保持秒数 (--wait 时改为按键继续)
ARMS = (1, 2)                  # 参与测试的臂号
JOINTS = (1, 2, 3, 4)          # 参与测试的关节号 (1-based)
MOVE_TIMEOUT_S = 20.0          # 单次移动 wait_done 超时


class DirectionTest:
    """舵机正方向标定 (arm 可注入 mock 以便测试)"""

    def __init__(self, arm, angle=ANGLE_DEG, hold_s=HOLD_S,
                 arms=ARMS, joints=JOINTS, wait_key=False):
        self.arm = arm
        self.angle = angle
        self.hold_s = hold_s
        self.arms = arms
        self.joints = joints
        self.wait_key = wait_key
        self.abort_event = threading.Event()
        self.abort_reason = ''

    # ---------------- 中止 ----------------
    def abort(self, reason):
        if not self.abort_event.is_set():
            self.abort_reason = reason
            print('!! 中止: %s' % reason)
            self.abort_event.set()

    def _start_key_watcher(self):
        """后台线程: q+回车 -> 中止 (--wait 模式下回车=继续, q=中止)"""
        def watch():
            try:
                for line in sys.stdin:
                    if line.strip().lower() == 'q':
                        self.abort('用户按键 q')
                        return
            except Exception:
                pass
        threading.Thread(target=watch, daemon=True).start()

    # ---------------- 保持 ----------------
    def _hold(self, arm_num, joint, label):
        """保持当前姿态: 默认计时 HOLD_S, --wait 模式下等回车。返回 False=已中止"""
        if self.wait_key:
            sys.stdout.write('    >>> 观察臂%d 关节%d %s, 回车继续 (q=中止) ...'
                             % (arm_num, joint, label))
            sys.stdout.flush()
            for line in sys.stdin:
                if self.abort_event.is_set():
                    return False
                if line.strip().lower() == 'q':
                    return False        # watcher 已置位
                return True
            return False                # stdin 关闭
        t0 = time.time()
        while time.time() - t0 < self.hold_s and not self.abort_event.is_set():
            time.sleep(0.05)
        return not self.abort_event.is_set()

    # ---------------- 动作 ----------------
    def _angles_for(self, joint, value):
        """4 关节全 0, 指定关节 = value"""
        angles = [0.0, 0.0, 0.0, 0.0]
        angles[joint - 1] = value
        return angles

    def _move(self, arm_num, angles, desc):
        """下发关节角并等完成。返回 False=已中止"""
        if self.abort_event.is_set():
            return False
        self.arm.move_joint(arm_num, angles)
        if not self.arm.wait_done(MOVE_TIMEOUT_S):
            self.abort('臂%d 移动超时 (%s)' % (arm_num, desc))
            return False
        if self.arm.last_error is not None:
            self.abort('臂%d 报错 0x%02X (%s)' % (arm_num, self.arm.last_error, desc))
            return False
        return True

    # ---------------- 主流程 ----------------
    def run(self):
        try:
            self._start_key_watcher()
            print('[归零] 所有舵机回初始位 ...')
            self.arm.home()
            if not self.arm.wait_done(MOVE_TIMEOUT_S):
                self.abort('归零超时')
                return
            time.sleep(1.0)

            for arm_num in self.arms:
                for joint in self.joints:
                    if self.abort_event.is_set():
                        return
                    print('[正转] 臂%d 关节%d: 0° -> %+.1f°'
                          % (arm_num, joint, self.angle))
                    if not self._move(arm_num, self._angles_for(joint, self.angle),
                                      '关节%d 正转' % joint):
                        return
                    if not self._hold(arm_num, joint, '正转姿态'):
                        return

                    print('[复位] 臂%d 关节%d: %+.1f° -> 0°'
                          % (arm_num, joint, self.angle))
                    if not self._move(arm_num, self._angles_for(joint, 0.0),
                                      '关节%d 复位' % joint):
                        return
                    if not self._hold(arm_num, joint, '复位'):
                        return
            print('[完成] 全部舵机正方向测试结束')
        except KeyboardInterrupt:
            self.abort('Ctrl+C')
        finally:
            print('[收尾] 机械臂急停 + 归零 (风扇全程未动) ...')
            try:
                self.arm.stop()
                self.arm.home()
            except Exception:
                pass
            if self.abort_reason:
                print('[收尾] 中止原因: %s' % self.abort_reason)


def main():
    ap = argparse.ArgumentParser(description='8fly 舵机正方向标定 (地面)')
    ap.add_argument('--port', default=ARM_PORT, help='机械臂串口 (默认 %(default)s)')
    ap.add_argument('--angle', type=float, default=ANGLE_DEG,
                    help='正转幅度 度 (默认 %(default)s)')
    ap.add_argument('--hold', type=float, default=HOLD_S,
                    help='每姿态保持秒数 (默认 %(default)s)')
    ap.add_argument('--wait', action='store_true',
                    help='按键模式: 每关节回车才继续 (q=中止)')
    ap.add_argument('--arm', type=int, choices=(1, 2), default=None,
                    help='只测指定臂 (默认两臂都测)')
    ap.add_argument('--joints', default=','.join(str(j) for j in JOINTS),
                    help='关节号, 逗号分隔 1~4 (默认 %(default)s)')
    args = ap.parse_args()

    joints = tuple(int(j) for j in args.joints.split(',') if j.strip())
    if not joints or any(j < 1 or j > 4 for j in joints):
        print('--joints 需为 1~4 的关节号, 逗号分隔')
        sys.exit(1)
    arms = (args.arm,) if args.arm else ARMS

    port = args.port or find_port()
    if not port:
        print('未找到机械臂串口, 用 --port 指定')
        sys.exit(1)

    print('连接机械臂 %s ...' % port)
    with ArmClient(port, ARM_BAUD) as arm:
        DirectionTest(arm, angle=args.angle, hold_s=args.hold,
                      arms=arms, joints=joints, wait_key=args.wait).run()


if __name__ == '__main__':
    main()
