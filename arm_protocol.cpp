#include "arm_protocol.h"

// ============================================================
// 帧解析状态机 (与 T265Receiver 风格一致)
// ============================================================

bool arm_parse_byte(uint8_t b, ArmFrame* frame) {
    static uint8_t state = 0;
    static uint8_t idx = 0;
    static uint8_t cmd = 0;
    static uint8_t len = 0;
    static uint8_t buf[ARM_MAX_PAYLOAD];
    static uint8_t chk = 0;

    switch (state) {
        case 0:
            if (b == ARM_FRAME_HEADER1) state = 1;
            break;
        case 1:
            if (b == ARM_FRAME_HEADER2) {
                state = 2;
                idx = 0;
                chk = 0;
            } else if (b != ARM_FRAME_HEADER1) {
                state = 0;
            }
            break;
        case 2:
            cmd = b;
            chk ^= b;
            state = 3;
            break;
        case 3:
            len = b;
            chk ^= b;
            if (len > ARM_MAX_PAYLOAD) {
                state = 0;  // 长度非法，丢弃
            } else if (len == 0) {
                state = 4;  // 无负载，直接等校验
            } else {
                state = 5;  // 有负载，进入读取
                idx = 0;
            }
            break;
        case 4:
            // 校验比对
            if (chk == b) {
                frame->cmd = cmd;
                frame->len = len;
                memcpy(frame->payload, buf, len);  // 复制负载数据
                frame->checksum = b;
                frame->valid = true;
            }
            state = 0;
            return frame->valid;
        case 5:
            buf[idx++] = b;
            chk ^= b;
            if (idx >= len) {
                state = 4;  // 负载读完，等校验
            }
            break;
    }
    return false;
}

// ============================================================
// 构建响应帧
// ============================================================

uint8_t arm_build_response(uint8_t* buf, uint8_t rsp_id,
                           const uint8_t* payload, uint8_t len) {
    buf[0] = ARM_FRAME_HEADER1;
    buf[1] = ARM_FRAME_HEADER2;
    buf[2] = rsp_id;
    buf[3] = len;

    uint8_t chk = rsp_id ^ len;

    for (uint8_t i = 0; i < len; i++) {
        buf[4 + i] = payload[i];
        chk ^= payload[i];
    }

    buf[4 + len] = chk;
    return 5 + len;  // 总长度
}

// ============================================================
// 快捷构建函数
// ============================================================

uint8_t arm_build_ack(uint8_t* buf) {
    return arm_build_response(buf, RSP_ACK, nullptr, 0);
}

uint8_t arm_build_done(uint8_t* buf) {
    return arm_build_response(buf, RSP_DONE, nullptr, 0);
}

uint8_t arm_build_error(uint8_t* buf, uint8_t err_code) {
    return arm_build_response(buf, RSP_ERROR, &err_code, 1);
}
