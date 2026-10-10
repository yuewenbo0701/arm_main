// ============================================================
// 8fly 机械臂控制程序 (树莓派上位机版) —— 四臂版
// ------------------------------------------------------------
// 通信: 树莓派5 --TX/RX--> Arduino Serial1 (Mega2560 引脚18=TX1 / 19=RX1, 115200)
//       二进制帧协议与原版一致, 见 arm_protocol.h
// 调试: DEBUG_ENABLED=0 时走 Serial (USB 口); 不要开成 1 —— 调试信息
//       会打进 Serial1 污染树莓派链路帧
// 说明: 正/逆运动学已移除 —— 关节角由树莓派直接下发,
//       CMD_MOVE_POSE 指令与串口交互菜单一并移除,
//       其余功能 (平滑移动/风扇/FSR/激光/状态上报) 保持不变
// 更新: VL53L1X 已在 setup() 中启用; 状态帧扩展为 22 字节
//       (末尾追加 fsr1/fsr2 的 ADC 原始值, 旧 18 字节字段不变)
// 更新: 新增前臂涵道风扇 (左前=引脚33, 右前=引脚32),
//       CMD_FAN_CTRL 风扇号扩展为 1~6 (1/2/5/6=臂1~4末端吸附[8/9/26/27], 3/4=前臂涵道)
// 更新: 扩展为四机械臂 (臂号 1~4):
//       臂1=左上(4,5,6,7) 臂2=右上(10,11,12,13)
//       臂3=左下(22,23,24,25) 臂4=右下(28,29,30,31)
//       CMD_MOVE_JOINT 接受臂号 1~4; CMD_HOME/急停 覆盖全部四臂;
//       状态帧扩展为 30 字节 (末尾追加 臂3/臂4 当前角度, 旧字段位置不变)
//       !!! 臂3/臂4 的舵机标定值目前是占位值, 装车后必须重新标定 !!!
// ============================================================

#include "arm_config.h"
#include "arm_protocol.h"
#include <VL53L1X.h>
#include <Wire.h>

// ============================================================
// 串口分配
// ============================================================
#define LINK_SERIAL   Serial1    // 与树莓派通信 (Serial1: 引脚18=TX1 / 19=RX1)
#define DEBUG_ENABLED 0       // 1=往 Serial1 打调试信息 (会污染链路, 仅脱机调试用), 0=走 USB Serial

#if DEBUG_ENABLED
  #define DBG_BEGIN(x)   Serial1.begin(x)
  #define DBG_PRINT(x)   Serial1.print(x)
  #define DBG_PRINTLN(x) Serial1.println(x)
#else
  #define DBG_BEGIN(x)   Serial.begin(x)
  #define DBG_PRINT(x)   Serial.print(x)
  #define DBG_PRINTLN(x) Serial.println(x)

#endif

// ============================================================
// 目标缓存 & 平滑移动
// ============================================================
static float target_pwm[ARM_COUNT + 1][4];   // [臂号1~4][关节0~3] 目标 PWM
static float start_pwm[ARM_COUNT + 1][4];    // 本次移动的起始位置 (PWM)
static unsigned long moveStartMs = 0;        // 本次移动的开始时刻
static float moveDurationMs = 0;             // 本次移动的总时长 (按最长路程折算)
static unsigned long lastUpdate = 0;
static bool moving = false;

// 指令完成标志 (move 完成后发送 RSP_DONE)
static bool pending_done = false;
static bool was_moving = false;

// ============================================================
// 自动工作序列 (非阻塞)
// ============================================================
static bool auto_work_active = false;
static short auto_work_state = 0;
static unsigned long auto_work_timer = 0;

// ============================================================
// 状态上报计时
// ============================================================
static unsigned long lastStatusTime = 0;
#define STATUS_PERIOD_MS 200   // 每200ms上报一次状态

// ============================================================
// 角度检查
// ============================================================
bool checkAngleRange(float angle, short idx, short arm_num) {
    if (arm_num < 1 || arm_num > ARM_COUNT) return false;
    if (angle < SERVO_ANGLE_MIN[arm_num][idx]) return false;
    if (angle > SERVO_ANGLE_MAX[arm_num][idx]) return false;
    return true;
}

// ============================================================
// VL53L1X 激光测距传感器
// I2C 引脚: Mega2560 = 20(SDA)/21(SCL); Uno = A4(SDA)/A5(SCL)
// (Wire 库自动选用对应板型的硬件 I2C 引脚, 代码无需区分)
// ============================================================
VL53L1X laser_sensor;
uint16_t laser_distance = 0;       // 最新测距值 (mm)
bool laser_available = false;      // 传感器是否就绪
unsigned long lastLaserRead = 0;
#define LASER_READ_PERIOD_MS 100   // 100ms 读取一次 (10Hz)

// ============================================================
// 压力传感器
// ============================================================
const short fsr_pins[2] = { A0, A1 };
const short fsr_times = 20;
volatile short fsrs[2];
const short fsr_threshold = 25;
volatile bool fsrs_cur[2];

void readfsr() {
    for (short i = 0; i < 2; i++) {
        short samples[fsr_times];
        for (short j = 0; j < fsr_times; j++) {
            samples[j] = analogRead(fsr_pins[i]);
        }
        for (short a = 0; a < fsr_times - 1; a++) {
            for (short b = a + 1; b < fsr_times; b++) {
                if (samples[a] > samples[b]) {
                    short tmp = samples[a];
                    samples[a] = samples[b];
                    samples[b] = tmp;
                }
            }
        }
        fsrs[i] = samples[fsr_times / 2];
        if (fsrs[i] >= fsr_threshold) fsrs_cur[i] = true;
        else fsrs_cur[i] = false;
    }
}

// ============================================================
// 平滑移动 (非阻塞 — 每循环调用一次)
// 同步插补: 所有关节按同一时间进度走, 无论各转多少角度
// 都同时开始、同时到达; 整体速度由路程最长的关节决定
// (该关节速度不超过 SPEED_US_PER_SEC, 其余关节自动减速配合)
// 返回 true 表示移动完成
// ============================================================
bool updateSmoothMove() {
    if (!moving) return true;

    unsigned long now = millis();
    if (now - lastUpdate < UPDATE_PERIOD_MS) return false;
    lastUpdate = now;

    // 时间进度 0.0 ~ 1.0
    float t = (moveDurationMs <= 0) ? 1.0f
              : (float)(now - moveStartMs) / moveDurationMs;
    if (t >= 1.0f) t = 1.0f;

    for (short a = 1; a <= ARM_COUNT; a++) {
        for (short i = 0; i < 4; i++) {
            SERVO_PWM[a][i] = start_pwm[a][i] + (target_pwm[a][i] - start_pwm[a][i]) * t;
        }
    }
    writeAllServo();

    bool finished = (t >= 1.0f);
    if (finished) {
        // 终点直接写死目标值, 消除浮点累计误差
        for (short a = 1; a <= ARM_COUNT; a++) {
            for (short i = 0; i < 4; i++) {
                SERVO_PWM[a][i] = target_pwm[a][i];
            }
        }
        writeAllServo();
        moving = false;
    }
    return finished;
}

// ============================================================
// 加载目标角度 (不立即移动)
// ============================================================
void loadTarget(float degree, short idx, short arm_num) {
    if (arm_num < 1 || arm_num > ARM_COUNT) return;
    float clamped = degree;
    if (clamped < SERVO_ANGLE_MIN[arm_num][idx]) clamped = SERVO_ANGLE_MIN[arm_num][idx];
    if (clamped > SERVO_ANGLE_MAX[arm_num][idx]) clamped = SERVO_ANGLE_MAX[arm_num][idx];
    target_pwm[arm_num][idx] = Angle_to_PWM(clamped,
        SERVO_CALIB_ZERO[arm_num][idx], SERVO_CALIB_PLUS90[arm_num][idx]);
}

// ============================================================
// 开始平滑移动 (非阻塞)
// 记录起始位置, 并按全部 (4臂 x 4关节) 中最长的路程折算总时长,
// 使所有关节同时到达目标 (同步插补)
// ============================================================
void startMove() {
    float maxDist = 0;
    for (short a = 1; a <= ARM_COUNT; a++) {
        for (short i = 0; i < 4; i++) {
            start_pwm[a][i] = SERVO_PWM[a][i];
            float d = fabs(target_pwm[a][i] - start_pwm[a][i]);
            if (d > maxDist) maxDist = d;
        }
    }
    moveDurationMs = maxDist / SPEED_US_PER_SEC * 1000.0f;
    moveStartMs = millis();
    moving = true;
    lastUpdate = 0;  // 立即开始第一步
}

// ============================================================
// 舵机归零
// ============================================================
void initAllServo() {
    for (short a = 1; a <= ARM_COUNT; a++) {
        for (short i = 0; i < 4; i++) {
            loadTarget(0, i, a);
        }
    }
    startMove();
}

// ============================================================
// 风扇控制 (臂端吸附风扇: 1=臂1[引脚8] 2=臂2[引脚9] 5=臂3[引脚26] 6=臂4[引脚27];
//            前臂涵道风扇: 3=左前[引脚33] 4=右前[引脚32])
// ============================================================
int getFanThrottle(short arm) {
    switch (arm) {
        case 1: return fanThrottle[0];
        case 2: return fanThrottle[1];
        case 3: return frontFanThrottle[0];
        case 4: return frontFanThrottle[1];
        case 5: return fanThrottle[2];
        case 6: return fanThrottle[3];
    }
    return FAN_THROTTLE_MIN;
}

void setFanThrottle(short arm, int target) {
    target = constrain(target, FAN_THROTTLE_MIN, FAN_THROTTLE_MAX);
    Servo* esc = NULL;
    int*   cur = NULL;
    switch (arm) {
        case 1: esc = &fan_esc_arm1;    cur = &fanThrottle[0];      break;
        case 2: esc = &fan_esc_arm2;    cur = &fanThrottle[1];      break;
        case 3: esc = &fan_esc_front_l; cur = &frontFanThrottle[0]; break;
        case 4: esc = &fan_esc_front_r; cur = &frontFanThrottle[1]; break;
        case 5: esc = &fan_esc_arm3;    cur = &fanThrottle[2];      break;
        case 6: esc = &fan_esc_arm4;    cur = &fanThrottle[3];      break;
        default: return;
    }

    if (target > *cur) {
        for (int t = *cur; t <= target; t += FAN_RAMP_STEP) {
            esc->writeMicroseconds(t);
            delay(FAN_RAMP_DELAY);
        }
    } else if (target < *cur) {
        for (int t = *cur; t >= target; t -= FAN_RAMP_STEP) {
            esc->writeMicroseconds(t);
            delay(FAN_RAMP_DELAY);
        }
    }
    esc->writeMicroseconds(target);
    *cur = target;
}

void fanOn(short arm)  { if (getFanThrottle(arm) < FAN_THROTTLE_ON) setFanThrottle(arm, FAN_THROTTLE_ON); }
void fanOff(short arm) { if (getFanThrottle(arm) > FAN_THROTTLE_MIN) setFanThrottle(arm, FAN_THROTTLE_MIN); }

void initFan() {
    fan_esc_arm1.attach(FAN_ESC_PIN_ARM1);
    fan_esc_arm1.writeMicroseconds(FAN_THROTTLE_MIN);
    fanThrottle[0] = FAN_THROTTLE_MIN;

    fan_esc_arm2.attach(FAN_ESC_PIN_ARM2);
    fan_esc_arm2.writeMicroseconds(FAN_THROTTLE_MIN);
    fanThrottle[1] = FAN_THROTTLE_MIN;

    // 臂3/臂4 末端涵道风扇: 左下=引脚26, 右下=引脚27
    fan_esc_arm3.attach(FAN_ESC_PIN_ARM3);
    fan_esc_arm3.writeMicroseconds(FAN_THROTTLE_MIN);
    fanThrottle[2] = FAN_THROTTLE_MIN;

    fan_esc_arm4.attach(FAN_ESC_PIN_ARM4);
    fan_esc_arm4.writeMicroseconds(FAN_THROTTLE_MIN);
    fanThrottle[3] = FAN_THROTTLE_MIN;

    // 前臂涵道风扇: 左前=引脚33, 右前=引脚32 (上电先给 1000us 让电调完成解锁)
    fan_esc_front_l.attach(FAN_ESC_PIN_FRONT_L);
    fan_esc_front_l.writeMicroseconds(FAN_THROTTLE_MIN);
    frontFanThrottle[0] = FAN_THROTTLE_MIN;

    fan_esc_front_r.attach(FAN_ESC_PIN_FRONT_R);
    fan_esc_front_r.writeMicroseconds(FAN_THROTTLE_MIN);
    frontFanThrottle[1] = FAN_THROTTLE_MIN;

    delay(200);
    DBG_PRINTLN("Fan ESCs initialized. (Arm1: pin8, Arm2: pin9, Arm3: pin26, Arm4: pin27, FrontL: pin33, FrontR: pin32)");
}

// ============================================================
// 树莓派指令处理
// ============================================================
void process_command(ArmFrame* frame) {
    switch (frame->cmd) {
        // ---- HOME: 归零所有舵机 ----
        case CMD_HOME: {
            DBG_PRINTLN("[PI] HOME");
            auto_work_active = false;  // 取消自动序列
            initAllServo();
            pending_done = true;
            break;
        }

        // ---- MOVE_JOINT: [arm_num:1B][angles:4xfloat] ----
        // 运动学已移除, 4 个关节角全部由树莓派直接下发;
        // 兼容旧格式: 只发 3 个角时自动取 theta4 = -(theta2+theta3)
        // arm_num: 1=左上 2=右上 3=左下 4=右下
        case CMD_MOVE_JOINT: {
            if (frame->len < 1 + 3 * 4) break;   // 至少 arm + 3 floats
            short arm = frame->payload[0];
            if (arm < 1 || arm > ARM_COUNT) break;

            float angles[4];
            if (frame->len >= 1 + 4 * 4) {       // 16 bytes: 4 个角全给定
                memcpy(angles, &frame->payload[1], 16);
            } else {                             // 12 bytes: 兼容旧格式
                memcpy(angles, &frame->payload[1], 12);
                angles[3] = -(angles[1] + angles[2]);
            }

            // 角度范围检查, 超限拒绝并回报错误
            bool ok = true;
            for (short i = 0; i < 4; i++) {
                if (!checkAngleRange(angles[i], i, arm)) { ok = false; break; }
            }
            if (!ok) {
                uint8_t txBuf[32];
                uint8_t len = arm_build_error(txBuf, ERR_ANGLE_OUT_RANGE);
                LINK_SERIAL.write(txBuf, len);
                DBG_PRINTLN("[PI] MOVE_JOINT angle out of range!");
                break;
            }

            for (short i = 0; i < 4; i++) {
                loadTarget(angles[i], i, arm);
            }

            DBG_PRINT("[PI] MOVE_JOINT arm="); DBG_PRINT(arm);
            DBG_PRINT(" a1="); DBG_PRINT(angles[0]);
            DBG_PRINT(" a2="); DBG_PRINT(angles[1]);
            DBG_PRINT(" a3="); DBG_PRINT(angles[2]);
            DBG_PRINT(" a4="); DBG_PRINTLN(angles[3]);

            auto_work_active = false;
            startMove();
            pending_done = true;
            break;
        }

        // ---- FAN_CTRL: [arm_num:1B][throttle:uint16] ----
        // arm_num: 1=臂1末端(引脚8) 2=臂2末端(引脚9) 5=臂3末端(引脚26) 6=臂4末端(引脚27),
        //          3=左前臂涵道(引脚33) 4=右前臂涵道(引脚32)
        case CMD_FAN_CTRL: {
            if (frame->len >= 3) {
                short arm = frame->payload[0];
                uint16_t throttle;
                memcpy(&throttle, &frame->payload[1], 2);
                DBG_PRINT("[PI] FAN_CTRL arm="); DBG_PRINT(arm);
                DBG_PRINT(" throttle="); DBG_PRINTLN(throttle);
                if (arm >= 1 && arm <= 6) {
                    setFanThrottle(arm, throttle);
                }
            }
            uint8_t txBuf[32];
            uint8_t len = arm_build_ack(txBuf);
            LINK_SERIAL.write(txBuf, len);
            break;
        }

        // ---- AUTO_START: 启动自动工作序列 ----
        case CMD_AUTO_START: {
            DBG_PRINTLN("[PI] AUTO_START");
            auto_work_active = true;
            auto_work_state = 0;
            auto_work_timer = 0;
            uint8_t txBuf[32];
            uint8_t len = arm_build_ack(txBuf);
            LINK_SERIAL.write(txBuf, len);
            break;
        }

        // ---- STOP (急停) ----
        case CMD_STOP: {
            DBG_PRINTLN("[PI] STOP");
            moving = false;
            auto_work_active = false;
            pending_done = false;
            fanOff(1);   // 臂1末端涵道风扇
            fanOff(2);   // 臂2末端涵道风扇
            fanOff(3);   // 左前臂涵道风扇
            fanOff(4);   // 右前臂涵道风扇
            fanOff(5);   // 臂3末端涵道风扇
            fanOff(6);   // 臂4末端涵道风扇
            uint8_t txBuf[32];
            uint8_t len = arm_build_ack(txBuf);
            LINK_SERIAL.write(txBuf, len);
            break;
        }

        // ---- STATUS_REQ: 立即上报一次状态 ----
        case CMD_STATUS_REQ: {
            sendStatus();
            break;
        }

        default:
            DBG_PRINT("[PI] Unknown cmd: 0x");
            if (DEBUG_ENABLED) Serial1.println(frame->cmd, HEX);
            break;
    }
}

// ============================================================
// 发送状态上报到树莓派
// ============================================================
void sendStatus() {
    // payload: [moving:1B][auto:1B][fan1:2B][fan2:2B][fsr:2B][laser:2B]
    //          [arm1_angles:4B][arm2_angles:4B][fsr1_raw:2B][fsr2_raw:2B]
    //          [arm3_angles:4B][arm4_angles:4B] = 30 bytes
    // (18 字节之后的字段均为后追加, 旧字段位置不变, 后向兼容)
    uint8_t payload[30];
    payload[0] = moving ? 1 : 0;
    payload[1] = auto_work_active ? 1 : 0;

    uint16_t fan1 = fanThrottle[0];
    uint16_t fan2 = fanThrottle[1];
    memcpy(&payload[2], &fan1, 2);
    memcpy(&payload[4], &fan2, 2);

    payload[6] = fsrs_cur[0] ? 1 : 0;
    payload[7] = fsrs_cur[1] ? 1 : 0;

    memcpy(&payload[8], &laser_distance, 2);

    // 当前舵机角度 (PWM → 角度, 粗略)
    // 各臂在 payload 中的起始偏移: 臂1→10, 臂2→14, 臂3→22, 臂4→26
    // (18~21 已被 FSR 原始值占用, 故臂3/臂4 跳到最后)
    const short angle_offset[ARM_COUNT + 1] = {0, 10, 14, 22, 26};
    for (short a = 1; a <= ARM_COUNT; a++) {
        for (short i = 0; i < 4; i++) {
            int8_t angle = (int8_t)constrain(
                PWM_to_Angle(SERVO_PWM[a][i],
                    SERVO_CALIB_ZERO[a][i], SERVO_CALIB_PLUS90[a][i]),
                -127, 127);
            payload[angle_offset[a] + i] = angle;
        }
    }

    // FSR ADC 原始值 (0~1023), 供树莓派端做饱和/阈值判定
    uint16_t fsr1_raw = (uint16_t)fsrs[0];
    uint16_t fsr2_raw = (uint16_t)fsrs[1];
    memcpy(&payload[18], &fsr1_raw, 2);
    memcpy(&payload[20], &fsr2_raw, 2);

    uint8_t txBuf[40];
    uint8_t len = arm_build_response(txBuf, RSP_STATUS, payload, 30);
    LINK_SERIAL.write(txBuf, len);
}

// ============================================================
// 自动工作序列 (非阻塞状态机)
// ============================================================
void updateAutoWork() {
    if (!auto_work_active) return;

    // 等上一个移动完成
    if (moving) return;

    // 状态间延时 (500ms)
    if (auto_work_timer != 0 && millis() - auto_work_timer < 500) return;

    switch (auto_work_state) {
        case 0:  // 抬起 → 风扇关
            fanOff(1);
            loadTarget(0, 0, 1); loadTarget(0, 1, 1);
            loadTarget(0, 2, 1); loadTarget(0, 3, 1);
            startMove();
            auto_work_state = 1;
            auto_work_timer = 0;
            DBG_PRINTLN("[AUTO] State 0: Lift up, fan off");
            break;

        case 1:  // 调整到预备位置
            loadTarget(0, 0, 1); loadTarget(20, 1, 1);
            loadTarget(20, 2, 1); loadTarget(-40, 3, 1);
            startMove();
            auto_work_state = 2;
            auto_work_timer = 0;
            DBG_PRINTLN("[AUTO] State 1: Pre-position");
            break;

        case 2:  // 下压 → 风扇开
            fanOn(1);
            loadTarget(-40, 0, 1); loadTarget(20, 1, 1);
            loadTarget(20, 2, 1); loadTarget(-40, 3, 1);
            startMove();
            auto_work_state = 3;
            auto_work_timer = 0;
            DBG_PRINTLN("[AUTO] State 2: Press, fan on");
            break;

        case 3:  // 复原
            loadTarget(-40, 0, 1); loadTarget(0, 1, 1);
            loadTarget(0, 2, 1); loadTarget(0, 3, 1);
            startMove();
            auto_work_state = 0;
            auto_work_timer = millis();  // 循环开始计时
            DBG_PRINTLN("[AUTO] State 3: Retract");
            break;
    }
}

// ============================================================
// 初始化
// ============================================================
void setup() {
    LINK_SERIAL.begin(115200);   // 树莓派 (Serial1: 引脚18=TX1 / 19=RX1)
    DBG_BEGIN(115200);           // 调试口: DEBUG_ENABLED=0 时走 USB(Serial)

    initAttachServo();
    initFan();

    // ---- VL53L1X 激光传感器初始化 ----
    Wire.begin();
    Wire.setClock(400000);  // 400kHz I2C
    laser_sensor.setTimeout(500);
    if (laser_sensor.init()) {
        laser_sensor.setDistanceMode(VL53L1X::Long);
        laser_sensor.setMeasurementTimingBudget(50000);  // 50ms budget
        laser_sensor.startContinuous(50);  // 50ms 采样周期 (20Hz)
        laser_available = true;
        DBG_PRINTLN("VL53L1X laser sensor ready.");
    } else {
        laser_available = false;
        DBG_PRINTLN("ERROR: VL53L1X not detected! Check wiring.");
    }

    initAllServo();

    DBG_PRINTLN("Arms ready. Waiting for Raspberry Pi on Serial(USB).");
}

// ============================================================
// 主循环
// ============================================================
void loop() {
    // ---------- 1. 处理树莓派串口指令 (Serial/USB) ----------
    while (LINK_SERIAL.available()) {
        uint8_t b = LINK_SERIAL.read();
        ArmFrame frame;
        if (arm_parse_byte(b, &frame)) {
            process_command(&frame);
        }
    }

    // ---------- 2. 驱动平滑移动 (非阻塞) ----------
    was_moving = moving;
    bool finished = updateSmoothMove();

    // 移动完成 → 通知树莓派
    if (was_moving && finished && pending_done) {
        pending_done = false;
        uint8_t txBuf[32];
        uint8_t len = arm_build_done(txBuf);
        LINK_SERIAL.write(txBuf, len);
        DBG_PRINTLN("[PI] Move done.");
    }

    // ---------- 3. 自动工作序列 ----------
    updateAutoWork();

    // ---------- 4. 定期读取 VL53L1X 激光 ----------
    if (laser_available && millis() - lastLaserRead >= LASER_READ_PERIOD_MS) {
        if (laser_sensor.dataReady()) {
            laser_distance = laser_sensor.read();
        }
        lastLaserRead = millis();
    }

    // ---------- 5. 定期读取 FSR ----------
    static unsigned long lastFSR = 0;
    if (millis() - lastFSR >= 500) {
        readfsr();
        lastFSR = millis();
    }

    // ---------- 6. 定期状态上报 ----------
    if (millis() - lastStatusTime >= STATUS_PERIOD_MS) {
        sendStatus();
        lastStatusTime = millis();
    }
}
