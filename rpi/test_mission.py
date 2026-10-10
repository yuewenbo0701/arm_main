#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
mission 接近控制器 / 吸附判定 / 中止逻辑单元测试 (mock arm+fc, 无需硬件)
运行: cd rpi && python3 -m unittest test_mission -v

FakeArm 内置一个简单物理仿真: 按 FakeFC 下发的 vx 实时更新激光距离,
因此 test_converges_to_target 是真正的闭环收敛测试。
"""
import threading
import time
import unittest
from types import SimpleNamespace

import mission
from mission import Mission


class FakeArm:
    """模拟 Arduino: 状态帧 + 距离物理仿真 (vx>0 前向接近 -> 距离减小)"""

    def __init__(self, dist0=1000.0, fsr_script=None, fresh=True):
        self.dist = float(dist0)
        self.vx = 0.0                  # 由 FakeFC.set_velocity_body 写入
        self.fresh = fresh             # False 时状态时间戳不更新 (模拟数据 stale)
        self._fsr_script = list(fsr_script or [])
        self._fsr = None
        self.moves = []                # move_joint 记录
        self.fan_calls = []            # fan_on 记录
        self.stopped = False
        self._lock = threading.Lock()
        self.status = None
        self.status_time = time.time()
        self._update_status()          # 先造一帧, 避免启动瞬间 status=None
        self._running = True
        self._last = time.time()
        self._thread = threading.Thread(target=self._sim_loop, daemon=True)
        self._thread.start()

    def _next_fsr(self):
        with self._lock:
            if self._fsr_script:
                self._fsr = self._fsr_script.pop(0)
            return self._fsr

    def _update_status(self):
        with self._lock:
            d = max(0, int(self.dist))
        self.status = SimpleNamespace(
            laser_mm=d, fsr1_raw=self._fsr, fsr2_raw=None,
            fan1=1200, fan2=1000)
        if self.fresh:
            self.status_time = time.time()

    def _sim_loop(self):
        while self._running:
            now = time.time()
            dt = now - self._last
            self._last = now
            with self._lock:
                self.dist -= self.vx * dt * 1000.0   # vx m/s -> mm
            self._next_fsr()
            self._update_status()
            time.sleep(0.01)

    # ---- ArmClient 同款 API ----
    def move_joint(self, arm, angles, wait=False, timeout=30):
        self.moves.append((arm, tuple(angles)))

    def wait_done(self, timeout=30):
        return True

    def fan_on(self, arm, throttle=1200):
        self.fan_calls.append((arm, throttle))

    def stop(self):
        self.stopped = True

    def close(self):
        self._running = False


class FakeFC:
    """模拟 FCClient: 记录设定点, 把 vx 回写给 FakeArm 驱动物理仿真"""

    def __init__(self, takeover_ok=True):
        self.vel_cmds = []
        self.hold_calls = 0
        self.released = False
        self.mode_name = 'OFFBOARD'
        self.takeover_ok = takeover_ok
        self.arm = None                # 连接后回写 vx

    def takeover_hold(self, timeout=6.0):
        return self.takeover_ok

    def set_velocity_body(self, vx, vy=0.0, vz=0.0):
        self.vel_cmds.append(vx)
        if self.arm:
            self.arm.vx = vx

    def hold_current_position(self):
        self.hold_calls += 1
        if self.arm:
            self.arm.vx = 0.0
        return True

    def release(self):
        self.released = True


def make_pair(dist0=1000.0, **arm_kw):
    arm = FakeArm(dist0=dist0, **arm_kw)
    fc = FakeFC()
    fc.arm = arm
    return Mission(arm, fc), arm, fc


class TestApproach(unittest.TestCase):
    def setUp(self):
        mission.CTRL_PERIOD_S = 0.01
        mission.APPROACH_OK_FRAMES = 3
        mission.APPROACH_TIMEOUT_S = 10.0
        mission.STATUS_TIMEOUT_S = 1.0

    def test_converges_to_target(self):
        """闭环收敛: d 从 1000mm 接近到 300mm, vx 始终不超限"""
        m, arm, fc = make_pair(dist0=1000.0)
        ok = m.approach()
        self.assertTrue(ok, '应到位, 中止原因: %s' % m.abort_reason)
        self.assertAlmostEqual(arm.dist, mission.TARGET_DISTANCE_MM,
                               delta=mission.APPROACH_TOL_MM + 30)
        self.assertGreaterEqual(fc.hold_calls, 1)          # 到位后转定点
        self.assertTrue(all(abs(v) <= mission.APPROACH_VX_MAX + 1e-9
                            for v in fc.vel_cmds))
        # 到位前 vx 应为正 (前向接近)
        self.assertGreater(fc.vel_cmds[0], 0)

    def test_abort_on_zero_laser(self):
        """激光读数 0 (无效) -> 立即中止并悬停"""
        m, arm, fc = make_pair(dist0=0.0)
        self.assertFalse(m.approach())
        self.assertIn('激光', m.abort_reason)
        self.assertGreaterEqual(fc.hold_calls, 1)

    def test_abort_on_over_range(self):
        """激光超量程 (>4000mm) -> 中止"""
        m, arm, fc = make_pair(dist0=5000.0)
        self.assertFalse(m.approach())
        self.assertIn('激光', m.abort_reason)

    def test_abort_on_stale_status(self):
        """状态帧过期 -> 中止"""
        m, arm, fc = make_pair(dist0=800.0, fresh=False)
        arm.status_time = time.time() - 100                # 100s 前的旧帧
        self.assertFalse(m.approach())
        self.assertIn('激光', m.abort_reason)

    def test_abort_event_stops_approach(self):
        """外部置中止 (如飞手切模式回调) -> 接近退出并悬停"""
        m, arm, fc = make_pair(dist0=1500.0)
        threading.Timer(0.3, m.abort, ('模拟飞手切模式',)).start()
        self.assertFalse(m.approach())
        self.assertEqual(m.abort_reason, '模拟飞手切模式')
        self.assertGreaterEqual(fc.hold_calls, 1)
        self.assertEqual(fc.vel_cmds[-1], 0)               # 退出前已刹停


class TestAttach(unittest.TestCase):
    def setUp(self):
        mission.FSR_JUDGE_DELAY_S = 0.02
        mission.FSR_POLL_S = 0.01
        mission.FSR_SAT_FRAMES = 3
        mission.FSR_TIMEOUT_S = 0.5
        mission.FSR_ADC_THRESHOLD = None
        mission.MOVE_TIMEOUT_S = 5.0

    def test_extend_sequence_and_saturation(self):
        """伸臂序列正确, FSR 上升到平台后判定饱和"""
        fsr = [100, 200, 350, 500, 580, 600] + [600] * 20
        m, arm, fc = make_pair(fsr_script=fsr)
        self.assertTrue(m.extend_and_attach())
        self.assertEqual(arm.moves,
                         [(mission.ARM_NUM, mission.POSE_PREPARE),
                          (mission.ARM_NUM, mission.POSE_PRESS)])
        self.assertEqual(arm.fan_calls, [(mission.ARM_NUM, mission.FAN_THROTTLE)])

    def test_zero_fsr_no_false_saturation(self):
        """FSR 恒 0 (传感器断线) -> 不误判饱和, 超时中止"""
        m, arm, fc = make_pair(fsr_script=[0] * 100)
        self.assertFalse(m.extend_and_attach())
        self.assertIn('FSR', m.abort_reason)

    def test_threshold_trigger(self):
        """启用阈值判定后, 到达阈值立即判定"""
        mission.FSR_ADC_THRESHOLD = 500
        try:
            m, arm, fc = make_pair(fsr_script=[100, 300, 499, 500])
            self.assertTrue(m.extend_and_attach())
        finally:
            mission.FSR_ADC_THRESHOLD = None

    def test_abort_between_moves(self):
        """移动之间收到中止 -> 不再开风扇"""
        m, arm, fc = make_pair(fsr_script=[600] * 50)
        orig_wait = arm.wait_done
        def wait_and_abort(timeout=30):
            r = orig_wait(timeout)
            m.abort('模拟中止')
            return r
        arm.wait_done = wait_and_abort
        self.assertFalse(m.extend_and_attach())
        self.assertEqual(arm.fan_calls, [])                # 风扇未被打开


class TestRunFlow(unittest.TestCase):
    def setUp(self):
        mission.CTRL_PERIOD_S = 0.01
        mission.APPROACH_OK_FRAMES = 3
        mission.APPROACH_TIMEOUT_S = 10.0
        mission.STATUS_TIMEOUT_S = 1.0
        mission.FSR_JUDGE_DELAY_S = 0.02
        mission.FSR_POLL_S = 0.01
        mission.FSR_SAT_FRAMES = 3
        mission.FSR_TIMEOUT_S = 5.0
        mission.HOLD_PERIOD_S = 0.02

    def test_full_mission_flow(self):
        """全流程: 接管 -> 接近 -> 伸臂 -> 吸附 -> 保持 -> 中止收尾"""
        import builtins
        orig_input = builtins.input
        builtins.input = lambda *a: ''                     # 跳过回车确认
        fsr = [100, 300, 600] + [600] * 200
        m, arm, fc = make_pair(dist0=400.0, fsr_script=fsr)
        threading.Timer(2.0, m.abort, ('测试结束',)).start()  # 保持阶段中止
        try:
            m.run()
        finally:
            builtins.input = orig_input
            arm.close()
        self.assertEqual(m.abort_reason, '测试结束')
        self.assertTrue(fc.released)                       # 收尾: 停设定点
        self.assertTrue(arm.stopped)                       # 收尾: 机械臂急停
        self.assertEqual(arm.fan_calls, [(mission.ARM_NUM, mission.FAN_THROTTLE)])

    def test_takeover_failure_aborts(self):
        """接管失败 -> 不进入接近, 直接收尾"""
        import builtins
        orig_input = builtins.input
        builtins.input = lambda *a: ''
        m, arm, fc = make_pair(dist0=800.0)
        fc.takeover_ok = False
        try:
            m.run()
        finally:
            builtins.input = orig_input
            arm.close()
        self.assertIn('接管失败', m.abort_reason)
        self.assertEqual(fc.vel_cmds, [])                  # 从未下发速度
        self.assertTrue(fc.released)
        self.assertTrue(arm.stopped)


if __name__ == '__main__':
    unittest.main()
