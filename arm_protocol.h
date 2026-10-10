#ifndef ARM_PROTOCOL_H
#define ARM_PROTOCOL_H

#include <Arduino.h>

// ============================================================
// 帧格式: [0xAA] [0x55] [CMD] [LEN] [PAYLOAD...] [XOR_CHECKSUM]
// 与 T265 协议风格一致
// ============================================================

#define ARM_FRAME_HEADER1   0xAA
#define ARM_FRAME_HEADER2   0x55
#define ARM_MAX_PAYLOAD     32

// ============================================================
// 下行命令 (树莓派 → Arm)
// ============================================================
#define CMD_HOME            0x01  // 归零所有舵机
#define CMD_MOVE_JOINT      0x02  // 手动关节控制 [arm_num:1B][angles:4xfloat]
                                  // (兼容旧格式: 只发3个float时 theta4 = -(theta2+theta3))
// 0x03 已废弃: 原逆解位姿控制 (正/逆运动学已移除, 位姿解算由树莓派端完成)
#define CMD_FAN_CTRL        0x04  // 风扇控制       [arm_num:1B][throttle:uint16]
                                  // arm_num: 1=臂1末端吸附风扇(引脚8)  2=臂2末端吸附风扇(引脚9)
                                  //          3=左前臂涵道风扇(引脚33)  4=右前臂涵道风扇(引脚32)
                                  //          5=臂3末端吸附风扇(引脚26) 6=臂4末端吸附风扇(引脚27)
#define CMD_AUTO_START      0x05  // 启动工作序列
#define CMD_STOP            0x06  // 急停
#define CMD_STATUS_REQ      0x07  // 请求立即上报一次状态

// ============================================================
// 上行响应 (Arm → 树莓派)
// ============================================================
#define RSP_ACK             0x80  // 通用确认
#define RSP_STATUS          0x81  // 状态上报
#define RSP_DONE            0x82  // 动作完成
#define RSP_ERROR           0x83  // 错误

// ============================================================
// 错误码
// ============================================================
#define ERR_NONE             0x00
#define ERR_ANGLE_OUT_RANGE  0x01
// 0x02 已废弃: 原 IK 无解错误 (运动学已移除)
#define ERR_TIMEOUT          0x03
#define ERR_BUSY             0x04

// ============================================================
// 解析用结构体
// ============================================================
typedef struct {
    uint8_t cmd;
    uint8_t len;
    uint8_t payload[ARM_MAX_PAYLOAD];
    uint8_t checksum;
    bool valid;
} ArmFrame;

// ============================================================
// 协议解析函数
// ============================================================

// 逐字节喂入，返回 true 表示收完一帧（帧在 frame 中）
bool arm_parse_byte(uint8_t b, ArmFrame* frame);

// 构建响应帧到 buf，返回帧总长度
// rsp_id: 响应命令ID
// payload / len: 响应数据
uint8_t arm_build_response(uint8_t* buf, uint8_t rsp_id,
                           const uint8_t* payload, uint8_t len);

// 快捷：构建简单ACK (无数据)
uint8_t arm_build_ack(uint8_t* buf);

// 快捷：构建DONE (无数据)
uint8_t arm_build_done(uint8_t* buf);

// 快捷：构建ERROR
uint8_t arm_build_error(uint8_t* buf, uint8_t err_code);

#endif
