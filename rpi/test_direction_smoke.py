#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""test_direction.py 的冒烟测试 (mock 机械臂, 无需硬件)"""
import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from test_direction import DirectionTest


class MockArm:
    def __init__(self):
        self.moves = []
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


class TestDirectionSmoke(unittest.TestCase):
    def test_full_sequence(self):
        arm = MockArm()
        DirectionTest(arm, angle=10.0, hold_s=0.01).run()
        # 2 臂 x 4 关节 x (正转+复位) = 16 次移动
        self.assertEqual(len(arm.moves), 16)
        # 第一组: 臂1 关节1 +10 -> 0
        self.assertEqual(arm.moves[0], (1, [10.0, 0.0, 0.0, 0.0]))
        self.assertEqual(arm.moves[1], (1, [0.0, 0.0, 0.0, 0.0]))
        # 臂1 关节4 之后接臂2 关节1
        self.assertEqual(arm.moves[8], (2, [10.0, 0.0, 0.0, 0.0]))
        # 最后一组: 臂2 关节4
        self.assertEqual(arm.moves[14], (2, [0.0, 0.0, 0.0, 10.0]))
        # 收尾急停+归零
        self.assertEqual(arm.stopped, 1)
        self.assertGreaterEqual(arm.homed, 1)
        # 全程无负角度, 且任意时刻只有一个关节非零
        for _, angles in arm.moves:
            self.assertTrue(all(a >= 0 for a in angles))
            self.assertLessEqual(sum(1 for a in angles if a != 0), 1)

    def test_single_arm_joint_filter(self):
        arm = MockArm()
        DirectionTest(arm, angle=10.0, hold_s=0.01,
                      arms=(2,), joints=(3,)).run()
        self.assertEqual(len(arm.moves), 2)
        self.assertEqual(arm.moves[0], (2, [0.0, 0.0, 10.0, 0.0]))


if __name__ == '__main__':
    unittest.main(verbosity=2)
