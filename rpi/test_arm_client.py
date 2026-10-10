#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
arm_client 协议编解码 + 状态帧兼容性单元测试 (无需硬件)
运行: cd rpi && python3 -m unittest test_arm_client -v
"""
import struct
import unittest

from arm_client import ArmStatus, FrameParser, build_frame, RSP_STATUS, CMD_HOME


def make_status_payload(laser=300, moving=1, fsr_raw=None):
    """构造状态帧负载: fsr_raw=None -> 18 字节旧格式; 否则 22 字节新格式"""
    p = bytearray(18)
    p[0] = moving
    p[1] = 0                              # auto
    struct.pack_into('<HH', p, 2, 1200, 1000)   # fan1, fan2
    p[6] = 1                              # fsr1 触发
    p[7] = 0
    struct.pack_into('<H', p, 8, laser)   # 激光
    struct.pack_into('<4b', p, 10, 0, 20, 20, -40)   # arm1 角度
    struct.pack_into('<4b', p, 14, 1, 2, 3, 4)       # arm2 角度
    if fsr_raw is not None:
        p += struct.pack('<HH', *fsr_raw)
    return bytes(p)


class TestFrameCodec(unittest.TestCase):
    """帧构建/解析回环 (对应固件 arm_protocol.cpp)"""

    def collect(self):
        frames = []
        parser = FrameParser(lambda cmd, payload: frames.append((cmd, payload)))
        return frames, parser

    def test_build_frame_checksum(self):
        # [0xAA][0x55][CMD][LEN][PAYLOAD][XOR=CMD^LEN^PAYLOAD]
        f = build_frame(CMD_HOME)
        self.assertEqual(f, bytes([0xAA, 0x55, CMD_HOME, 0x00, CMD_HOME ^ 0]))
        f = build_frame(0x02, bytes([0x01, 0x02]))
        self.assertEqual(f[:4], bytes([0xAA, 0x55, 0x02, 0x02]))
        self.assertEqual(f[-1], 0x02 ^ 0x02 ^ 0x01 ^ 0x02)

    def test_parse_single(self):
        frames, parser = self.collect()
        payload = make_status_payload(laser=300, fsr_raw=(500, 600))
        for b in build_frame(RSP_STATUS, payload):
            parser.feed(b)
        self.assertEqual(len(frames), 1)
        cmd, p = frames[0]
        self.assertEqual(cmd, RSP_STATUS)
        self.assertEqual(p, payload)

    def test_parse_with_noise(self):
        # 噪声(含伪帧头/坏校验)之后紧跟一帧好帧, 应只解析出好帧
        frames, parser = self.collect()
        payload = make_status_payload()
        noise = bytes([0x00, 0xFF, 0xAA, 0xAA, 0x55, 0x99, 0x01, 0x00, 0x55])
        data = noise + build_frame(RSP_STATUS, payload)
        for b in data:
            parser.feed(b)
        self.assertEqual(len(frames), 1)
        self.assertEqual(frames[0][1], payload)

    def test_parse_sticky_frames(self):
        # 两帧粘连一次喂入, 应解析出两帧
        frames, parser = self.collect()
        p1 = make_status_payload(laser=100)
        p2 = make_status_payload(laser=200)
        data = build_frame(RSP_STATUS, p1) + build_frame(RSP_STATUS, p2)
        for b in data:
            parser.feed(b)
        self.assertEqual(len(frames), 2)
        self.assertEqual(struct.unpack_from('<H', frames[0][1], 8)[0], 100)
        self.assertEqual(struct.unpack_from('<H', frames[1][1], 8)[0], 200)

    def test_bad_checksum_dropped(self):
        frames, parser = self.collect()
        f = bytearray(build_frame(RSP_STATUS, make_status_payload()))
        f[-1] ^= 0xFF                       # 破坏校验
        for b in f:
            parser.feed(b)
        self.assertEqual(len(frames), 0)


class TestArmStatus(unittest.TestCase):
    """状态帧 18/22 字节后向兼容"""

    def test_legacy_18_bytes(self):
        s = ArmStatus.from_payload(make_status_payload(laser=456))
        self.assertTrue(s.moving)
        self.assertEqual((s.fan1, s.fan2), (1200, 1000))
        self.assertEqual(s.laser_mm, 456)
        self.assertEqual(s.arm1_angles, (0, 20, 20, -40))
        self.assertEqual(s.arm2_angles, (1, 2, 3, 4))
        self.assertIsNone(s.fsr1_raw)       # 旧帧无原始值
        self.assertIsNone(s.fsr2_raw)

    def test_new_22_bytes(self):
        s = ArmStatus.from_payload(make_status_payload(laser=300, fsr_raw=(777, 888)))
        self.assertEqual(s.laser_mm, 300)
        self.assertEqual(s.fsr1_raw, 777)
        self.assertEqual(s.fsr2_raw, 888)
        # 旧字段位置不变
        self.assertEqual(s.arm1_angles, (0, 20, 20, -40))

    def test_loopback_22_bytes(self):
        # 构建 22 字节状态帧 -> 解析 -> ArmStatus, 端到端一致
        frames = []
        parser = FrameParser(lambda c, p: frames.append((c, p)))
        payload = make_status_payload(laser=301, fsr_raw=(1023, 0))
        for b in build_frame(RSP_STATUS, payload):
            parser.feed(b)
        s = ArmStatus.from_payload(frames[0][1])
        self.assertEqual(s.laser_mm, 301)
        self.assertEqual(s.fsr1_raw, 1023)
        self.assertEqual(s.fsr2_raw, 0)


if __name__ == '__main__':
    unittest.main()
