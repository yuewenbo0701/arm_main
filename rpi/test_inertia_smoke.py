#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""test_inertia.py 的冒烟测试 (mock 机械臂, 无需硬件)"""
import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from test_inertia import InertiaTest


class MockArm:
    def __init__(self):
        self.moves = []          # [(arm, angles)]
        self.homed = 0
        self.stopped = 0
        self.last_error = None

    def home(self):
        self.homed += 1

    def move_joint(self, arm, angles):
        self.moves.append((arm, list(angles)))

    def wait_done(self, timeout=30):
        return True

    def stop(self):
        self.stopped += 1


class TestInertiaSmoke(unittest.TestCase):
    def test_sequence(self):
        arm = MockArm()
        InertiaTest(arm, angle=15.0, cycles=1, hold_s=0.05, baseline_s=0.05).run()

        # 动作序列: 每关节 两臂同时 x (+/复位/-/复位) = 8 次,
        # 默认关节 (2,3) 共 16 次
        self.assertEqual(len(arm.moves), 16)
        # 顺序: 关节2 两臂 +15 -> 两臂 0 -> 两臂 -15 -> 两臂 0, 再关节3
        self.assertEqual(arm.moves[0], (1, [0.0, 15.0, 0.0, 0.0]))
        self.assertEqual(arm.moves[1], (2, [0.0, 15.0, 0.0, 0.0]))
        self.assertEqual(arm.moves[2], (1, [0.0, 0.0, 0.0, 0.0]))
        self.assertEqual(arm.moves[3], (2, [0.0, 0.0, 0.0, 0.0]))
        self.assertEqual(arm.moves[4], (1, [0.0, -15.0, 0.0, 0.0]))
        self.assertEqual(arm.moves[5], (2, [0.0, -15.0, 0.0, 0.0]))
        self.assertEqual(arm.moves[8], (1, [0.0, 0.0, 15.0, 0.0]))
        self.assertEqual(arm.moves[9], (2, [0.0, 0.0, 15.0, 0.0]))
        # 每次移动两臂同名关节的角度完全一致
        for i in range(0, 16, 2):
            self.assertEqual(arm.moves[i][1], arm.moves[i + 1][1])
        # 收尾: 急停 + 归零
        self.assertEqual(arm.stopped, 1)
        self.assertGreaterEqual(arm.homed, 1)

    def test_single_joint(self):
        arm = MockArm()
        InertiaTest(arm, angle=10.0, cycles=1, hold_s=0.05, baseline_s=0.05,
                    swing_joints=(1,)).run()
        # 单关节: 4 步 x 两臂 = 8 次
        self.assertEqual(len(arm.moves), 8)
        self.assertEqual(arm.moves[0], (1, [10.0, 0.0, 0.0, 0.0]))
        self.assertEqual(arm.moves[1], (2, [10.0, 0.0, 0.0, 0.0]))

    def test_move_timeout_aborts(self):
        class TimeoutArm(MockArm):
            def wait_done(self, timeout=30):
                return False
        arm = TimeoutArm()
        t = InertiaTest(arm, hold_s=0.05, baseline_s=0.05)
        t.run()
        self.assertTrue(t.abort_event.is_set())
        self.assertEqual(arm.stopped, 1)   # 中止后仍急停+归零


if __name__ == '__main__':
    unittest.main(verbosity=2)
