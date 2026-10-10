// ============================================================
// 8fly 机械臂控制程序
// 双输入源: Serial(USB调试) + Serial1(FC指令)
// 非阻塞舵机平滑移动 + 自动工作序列
// ============================================================

#include "arm_config.h"
#include "arm_protocol.h"
#include <VL53L1X.h>
#include <Wire.h>
#include <math.h>
#ifndef PI
#define PI 3.1415926
#endif

// ============================================================
// 状态列表
// ============================================================
#define IDLE          0
#define INIT          1
#define TEST_SELECT   2
#define TEST_INPUT    3
#define SOLVE_SELECT  4
#define SOLVE_INPUT   5
#define WORK_IDLE     6
#define WORKING       7
#define FAN_DEBUG     8

short state = IDLE;

// ============================================================
// 读取数据 (交互模式)
// ============================================================
static short idx_test = 0;
static short idx_solve = 0;
static float input_receiver[10];

// ============================================================
// 目标缓存 & 平滑移动
// ============================================================
static float target1[4], target2[4];
static unsigned long lastUpdate = 0;
static bool moving = false;
static short arm_num;

// FC 命令完成标志 (move完成后发送 RSP_DONE)
static bool pending_fc_done = false;
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
// 运动学结构体
// ============================================================
struct Point3D { float x; float y; float z; };
struct JointAngles { float t1; float t2; float t3; float t4; };

// ============================================================
// 连杆参数
// ============================================================
const float L1_x = 84,  L1_y = 48,  L1_z = 0;
const float L2_x = 20,  L2_y = 18,  L2_z = 36;
const float L3_x = 80,  L3_y = 0,   L3_z = 0;
const float L4_x = 20,  L4_y = 0,   L4_z = 0;
const float L5_x = 30,  L5_y = 136, L5_z = -28;

/* -------- 简要示意图（朝机头方向） --------

    y
    ^
    |                                       o P
    |                                       |
    |                                       |
    |             S2            S3   S4     |
    |         ----o--------------o----o------
    |         |
    |         |
    |---------o
    |         S1
    |
----o-------------------------------------------->  x
    |
    |

  计算过程中，假设各舵机的运动方向如下：
  S1: 沿y轴旋转，逆时针为正方向；
  S2: 沿z轴旋转，逆时针为正方向；
  S3: 沿z轴旋转，逆时针为正方向；
  S4: 沿z轴旋转，逆时针为正方向；
  另一条腿安装与旋转方向腿镜像对称
  为保证腿部垂直于墙面落下，程序中固定theta4 = -theta2 - theta3
*/

// ============================================================
// 角度检查
// ============================================================
bool checkAngleRange(float angle, short idx, short arm_num) {
    switch (arm_num) {
        case 1:
            if (angle < SERVO_ARM1_ANGLE_MIN[idx] || angle > SERVO_ARM1_ANGLE_MAX[idx]) return false;
            break;
        case 2:
            if (angle < SERVO_ARM2_ANGLE_MIN[idx] || angle > SERVO_ARM2_ANGLE_MAX[idx]) return false;
            break;
    }
    return true;
}

float Rad_to_Deg(float rad) { return rad * (180.0 / PI); }
float Deg_to_Rad(float deg) { return deg * (PI / 180.0); }

// ============================================================
// 正运动学
// ============================================================
Point3D getPosition(float tar[], short arm_num) {
    float theta1 = tar[0];
    float theta2 = tar[1];
    float theta3 = tar[2];

    float rad1 = Deg_to_Rad(theta1);
    float rad2 = Deg_to_Rad(theta2);
    float rad3 = Deg_to_Rad(theta3);

    Point3D pos;
    float offset_x = L2_x + L3_x * cos(rad2) + L4_x * cos(rad2 + rad3) + L5_x;
    pos.y = L1_y + L2_y + L3_x * sin(rad2) + L4_x * sin(rad2 + rad3) + L5_y;
    pos.z = offset_x * sin(rad1) + (L2_z + L5_z) * cos(rad1);
    pos.x = L1_x + offset_x * cos(rad1) - (L2_z + L5_z) * sin(rad1);
    return pos;
}

// ============================================================
// 逆运动学
// ============================================================
const short MAX_SOLUTIONS = 50;
JointAngles solutions[MAX_SOLUTIONS];
Point3D point;

short solve(Point3D target, JointAngles outputArray[], short arm_num) {
    short count = 0;
    float offset_x = sqrt(target.z * target.z + (target.x - L1_x) * (target.x - L1_x) - (L2_z + L5_z) * (L2_z + L5_z));
    float rad1 = atan2(target.z, (target.x - L1_x)) - atan2((L2_z + L5_z), offset_x);
    float theta1 = Rad_to_Deg(rad1);

    float inter_x = offset_x - L2_x - L5_x;
    float inter_y = target.y - L1_y - L2_y - L5_y;

    float D = sqrt(inter_y * inter_y + inter_x * inter_x);
    if (D > (L3_x + L4_x) || D < abs(L3_x - L4_x)) { return count; }

    float cos_rad3 = (D * D - L3_x * L3_x - L4_x * L4_x) / (2 * L3_x * L4_x);
    if (cos_rad3 > 1.0) cos_rad3 = 1.0;
    if (cos_rad3 < -1.0) cos_rad3 = -1.0;

    float rad3_pos[2];
    rad3_pos[0] = acos(cos_rad3);
    rad3_pos[1] = -acos(cos_rad3);

    for (float rad3 : rad3_pos) {
        float k1 = L3_x + L4_x * cos(rad3);
        float k2 = L4_x * sin(rad3);
        float rad2 = atan2(inter_y, inter_x) - atan2(k2, k1);
        float theta2 = Rad_to_Deg(rad2);
        float theta3 = Rad_to_Deg(rad3);
        float theta4 = -theta2 - theta3;

        if (checkAngleRange(theta1, 0, arm_num) && checkAngleRange(theta2, 1, arm_num) &&
            checkAngleRange(theta3, 2, arm_num) && checkAngleRange(theta4, 3, arm_num)) {
            outputArray[count].t1 = theta1;
            outputArray[count].t2 = theta2;
            outputArray[count].t3 = theta3;
            outputArray[count].t4 = theta4;
            count++;
        }
    }
    return count;
}

// ============================================================
// VL53L1X 激光测距传感器 (I2C: SDA=A4, SCL=A5)
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
// 返回 true 表示移动完成
// ============================================================
bool updateSmoothMove() {
    if (!moving) return true;

    unsigned long now = millis();
    if (now - lastUpdate < UPDATE_PERIOD_MS) return false;
    lastUpdate = now;

    bool finished = true;

    auto step = [&](float cur[], float dst[]) {
        for (short i = 0; i < 4; i++) {
            long dist = dst[i] - cur[i];
            if (dist == 0) continue;
            short dir = (dist > 0) ? 1 : -1;
            long maxStep = SPEED_US_PER_SEC * UPDATE_PERIOD_MS / 1000;
            long step = min(abs(dist), maxStep) * dir;
            long next = cur[i] + step;
            if (next != dst[i]) finished = false;
            else next = dst[i];
            cur[i] = next;
        }
    };

    step(SERVO_ARM1, target1);
    step(SERVO_ARM2, target2);
    writeAllServo();

    if (finished) {
        moving = false;
    }
    return finished;
}

// ============================================================
// 加载目标角度 (不立即移动)
// ============================================================
void loadTarget(float degree, short idx, short arm_num) {
    float clamped = degree;
    switch (arm_num) {
        case 1:
            if (clamped < SERVO_ARM1_ANGLE_MIN[idx]) clamped = SERVO_ARM1_ANGLE_MIN[idx];
            if (clamped > SERVO_ARM1_ANGLE_MAX[idx]) clamped = SERVO_ARM1_ANGLE_MAX[idx];
            target1[idx] = Angle_to_PWM(clamped, SERVO_ARM1_CALIB_ZERO[idx], SERVO_ARM1_CALIB_PLUS90[idx]);
            break;
        case 2:
            if (clamped < SERVO_ARM2_ANGLE_MIN[idx]) clamped = SERVO_ARM2_ANGLE_MIN[idx];
            if (clamped > SERVO_ARM2_ANGLE_MAX[idx]) clamped = SERVO_ARM2_ANGLE_MAX[idx];
            target2[idx] = Angle_to_PWM(clamped, SERVO_ARM2_CALIB_ZERO[idx], SERVO_ARM2_CALIB_PLUS90[idx]);
            break;
    }
}

// ============================================================
// 开始平滑移动 (非阻塞)
// ============================================================
void startMove() {
    moving = true;
    lastUpdate = 0;  // 立即开始第一步
}

// ============================================================
// 舵机归零
// ============================================================
void initAllServo() {
    for (short i = 0; i < 4; i++) {
        loadTarget(0, i, 1);
        loadTarget(0, i, 2);
    }
    startMove();
}

// ============================================================
// 风扇控制 (每臂独立)
// ============================================================
void setFanThrottle(short arm, int target) {
    target = constrain(target, FAN_THROTTLE_MIN, FAN_THROTTLE_MAX);
    Servo& esc = (arm == 1) ? fan_esc_arm1 : fan_esc_arm2;
    int& cur = fanThrottle[arm - 1];

    if (target > cur) {
        for (int t = cur; t <= target; t += FAN_RAMP_STEP) {
            esc.writeMicroseconds(t);
            delay(FAN_RAMP_DELAY);
        }
    } else if (target < cur) {
        for (int t = cur; t >= target; t -= FAN_RAMP_STEP) {
            esc.writeMicroseconds(t);
            delay(FAN_RAMP_DELAY);
        }
    }
    esc.writeMicroseconds(target);
    cur = target;
}

void fanOn(short arm)  { if (fanThrottle[arm - 1] < FAN_THROTTLE_ON) setFanThrottle(arm, FAN_THROTTLE_ON); }
void fanOff(short arm) { if (fanThrottle[arm - 1] > FAN_THROTTLE_MIN) setFanThrottle(arm, FAN_THROTTLE_MIN); }

void initFan() {
    fan_esc_arm1.attach(FAN_ESC_PIN_ARM1);
    fan_esc_arm1.writeMicroseconds(FAN_THROTTLE_MIN);
    fanThrottle[0] = FAN_THROTTLE_MIN;

    fan_esc_arm2.attach(FAN_ESC_PIN_ARM2);
    fan_esc_arm2.writeMicroseconds(FAN_THROTTLE_MIN);
    fanThrottle[1] = FAN_THROTTLE_MIN;

    delay(200);
    Serial.println("Fan ESCs initialized. (Arm1: pin8, Arm2: pin9)");
}

// ============================================================
// FC 命令处理
// ============================================================
void process_fc_command(ArmFrame* frame) {
    switch (frame->cmd) {
        // ---- HOME ----
        case CMD_HOME: {
            Serial.println("[FC] HOME");
            auto_work_active = false;  // 取消自动序列
            initAllServo();
            pending_fc_done = true;
            break;
        }

        // ---- MOVE_JOINT: [arm_num:1B][angles:4xfloat] ----
        // 注意: 第4关节由运动学约束决定 theta4 = -(theta2+theta3)
        case CMD_MOVE_JOINT: {
            if (frame->len < 1) break;
            short arm = frame->payload[0];
            if (arm != 1 && arm != 2) break;

            float angles[4];
            if (frame->len >= 1 + 4 * 4) {  // 16 bytes for 4 floats
                memcpy(angles, &frame->payload[1], 16);
            } else if (frame->len >= 1 + 3 * 4) {  // 12 bytes for 3 floats
                memcpy(angles, &frame->payload[1], 12);
                angles[3] = 0;  // 下面会重算
            }
            // 强制运动学约束: theta4 = -(theta2 + theta3)
            angles[3] = -(angles[1] + angles[2]);

            for (short i = 0; i < 4; i++) {
                loadTarget(angles[i], i, arm);
            }

            Serial.print("[FC] MOVE_JOINT arm="); Serial.print(arm);
            Serial.print(" a1="); Serial.print(angles[0]);
            Serial.print(" a2="); Serial.print(angles[1]);
            Serial.print(" a3="); Serial.print(angles[2]);
            Serial.print(" a4="); Serial.print(angles[3]);
            Serial.println(" (auto)");

            auto_work_active = false;
            startMove();
            pending_fc_done = true;
            break;
        }

        // ---- MOVE_POSE: [arm_num:1B][x:float][y:float][z:float] ----
        case CMD_MOVE_POSE: {
            if (frame->len < 13) break;  // 1 + 3*4 = 13
            short arm = frame->payload[0];
            if (arm != 1 && arm != 2) break;

            Point3D target;
            memcpy(&target.x, &frame->payload[1], 4);
            memcpy(&target.y, &frame->payload[5], 4);
            memcpy(&target.z, &frame->payload[9], 4);

            Serial.print("[FC] MOVE_POSE arm="); Serial.print(arm);
            Serial.print(" x="); Serial.print(target.x);
            Serial.print(" y="); Serial.print(target.y);
            Serial.print(" z="); Serial.println(target.z);

            short cnt = solve(target, solutions, arm);
            if (cnt >= 1) {
                loadTarget(solutions[0].t1, 0, arm);
                loadTarget(solutions[0].t2, 1, arm);
                loadTarget(solutions[0].t3, 2, arm);
                loadTarget(solutions[0].t4, 3, arm);
                auto_work_active = false;
                startMove();
                pending_fc_done = true;
            } else {
                // 无解
                uint8_t txBuf[32];
                uint8_t len = arm_build_error(txBuf, ERR_IK_NO_SOLUTION);
                Serial1.write(txBuf, len);
                Serial.println("[FC] IK no solution!");
            }
            break;
        }

        // ---- FAN_CTRL: [arm_num:1B][throttle:uint16] ----
        case CMD_FAN_CTRL: {
            if (frame->len >= 3) {
                short arm = frame->payload[0];
                uint16_t throttle;
                memcpy(&throttle, &frame->payload[1], 2);
                Serial.print("[FC] FAN_CTRL arm="); Serial.print(arm);
                Serial.print(" throttle="); Serial.println(throttle);
                if (arm == 1 || arm == 2) {
                    setFanThrottle(arm, throttle);
                }
            }
            uint8_t txBuf[32];
            uint8_t len = arm_build_ack(txBuf);
            Serial1.write(txBuf, len);
            break;
        }

        // ---- AUTO_START ----
        case CMD_AUTO_START: {
            Serial.println("[FC] AUTO_START");
            auto_work_active = true;
            auto_work_state = 0;
            auto_work_timer = 0;
            uint8_t txBuf[32];
            uint8_t len = arm_build_ack(txBuf);
            Serial1.write(txBuf, len);
            break;
        }

        // ---- STOP (急停) ----
        case CMD_STOP: {
            Serial.println("[FC] STOP");
            moving = false;
            auto_work_active = false;
            pending_fc_done = false;
            fanOff(1);
            fanOff(2);
            uint8_t txBuf[32];
            uint8_t len = arm_build_ack(txBuf);
            Serial1.write(txBuf, len);
            break;
        }

        default:
            Serial.print("[FC] Unknown cmd: 0x");
            Serial.println(frame->cmd, HEX);
            break;
    }
}

// ============================================================
// 发送状态上报到 FC
// ============================================================
void sendStatus() {
    // payload v3: [moving:1B][auto:1B][fan1:2B][fan2:2B][fsr:2B][laser:2B]
    //              [arm1_angles:4B][arm2_angles:4B] = 18 bytes
    uint8_t payload[18];
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
    for (short i = 0; i < 4; i++) {
        int8_t angle1 = (int8_t)constrain(
            PWM_to_Angle(SERVO_ARM1[i], SERVO_ARM1_CALIB_ZERO[i], SERVO_ARM1_CALIB_PLUS90[i]),
            -127, 127);
        int8_t angle2 = (int8_t)constrain(
            PWM_to_Angle(SERVO_ARM2[i], SERVO_ARM2_CALIB_ZERO[i], SERVO_ARM2_CALIB_PLUS90[i]),
            -127, 127);
        payload[10 + i] = angle1;
        payload[14 + i] = angle2;
    }

    uint8_t txBuf[32];
    uint8_t len = arm_build_response(txBuf, RSP_STATUS, payload, 18);
    Serial1.write(txBuf, len);
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
            Serial.println("[AUTO] State 0: Lift up, fan off");
            break;

        case 1:  // 调整到预备位置
            loadTarget(0, 0, 1); loadTarget(20, 1, 1);
            loadTarget(20, 2, 1); loadTarget(-40, 3, 1);
            startMove();
            auto_work_state = 2;
            auto_work_timer = 0;
            Serial.println("[AUTO] State 1: Pre-position");
            break;

        case 2:  // 下压 → 风扇开
            fanOn(1);
            loadTarget(-40, 0, 1); loadTarget(20, 1, 1);
            loadTarget(20, 2, 1); loadTarget(-40, 3, 1);
            startMove();
            auto_work_state = 3;
            auto_work_timer = 0;
            Serial.println("[AUTO] State 2: Press, fan on");
            break;

        case 3:  // 复原
            loadTarget(-40, 0, 1); loadTarget(0, 1, 1);
            loadTarget(0, 2, 1); loadTarget(0, 3, 1);
            startMove();
            auto_work_state = 0;
            auto_work_timer = millis();  // 循环开始计时
            Serial.println("[AUTO] State 3: Retract");
            break;
    }
}

// ============================================================
// 交互模式 — IDLE
// ============================================================
void to_idle() {
    state = IDLE;
    Serial.println("Arms ready. FC on Serial1. t=test s=solve w=auto f=fan i=init");
}

void idleMode() {
    if (!Serial.available()) return;
    String input = Serial.readStringUntil('\n');
    input.trim();
    if (input == "i") {
        state = INIT;
        Serial.println("Entering init mode.");
        return;
    }
    if (input == "t") {
        state = TEST_SELECT;
        Serial.println("Choose arm to test. Input 1/2 or q to quit.");
        return;
    }
    if (input == "s") {
        state = SOLVE_SELECT;
        Serial.println("Choose arm to solve. Input 1/2 or q to quit.");
        return;
    }
    if (input == "w") {
        state = WORK_IDLE;
        Serial.println("Press s to start, p to stop, q to quit.");
        return;
    }
    if (input == "f") {
        state = FAN_DEBUG;
        Serial.println("Fan debug. 1/2=arm, on/off/0-2000, q=quit.");
        return;
    }
    else {
        Serial.println("Unknown cmd");
        return;
    }
}

// ============================================================
// 交互模式 — INIT
// ============================================================
void initMode() {
    initAllServo();
    Serial.println("Initialize complete.");
    to_idle();
}

// ============================================================
// 交互模式 — FAN DEBUG
// ============================================================
void fanDebugMode() {
    if (!Serial.available()) return;
    String input = Serial.readStringUntil('\n');
    input.trim();
    if (input == "q") { to_idle(); return; }

    // 解析: <arm> <cmd>  如 "1 on", "2 1500", "1 off"
    short sp = input.indexOf(' ');
    if (sp == -1) {
        Serial.println("Usage: <1|2> <on|off|0-2000>  q=quit");
        return;
    }
    short arm = input.substring(0, sp).toInt();
    String cmd = input.substring(sp + 1);
    cmd.trim();

    if (arm != 1 && arm != 2) {
        Serial.println("Arm must be 1 or 2.");
        return;
    }

    if (cmd == "on") {
        fanOn(arm);
        Serial.print("Fan"); Serial.print(arm);
        Serial.print(" ON  (thr="); Serial.print(fanThrottle[arm - 1]);
        Serial.println(")");
    } else if (cmd == "off") {
        fanOff(arm);
        Serial.print("Fan"); Serial.print(arm);
        Serial.print(" OFF (thr="); Serial.print(fanThrottle[arm - 1]);
        Serial.println(")");
    } else {
        int thr = cmd.toInt();
        if (thr >= FAN_THROTTLE_MIN && thr <= FAN_THROTTLE_MAX) {
            setFanThrottle(arm, thr);
            Serial.print("Fan"); Serial.print(arm);
            Serial.print(" -> "); Serial.println(thr);
        } else {
            Serial.println("Bad throttle. Range 1000-2000.");
        }
    }
}

// ============================================================
// 交互模式 — TEST
// ============================================================
void testSelectMode() {
    if (!Serial.available()) return;
    String input = Serial.readStringUntil('\n');
    input.trim();
    if (input == "1") { state = TEST_INPUT; arm_num = 1; Serial.println("Adjusting arm1. Input 3 floats."); }
    else if (input == "2") { state = TEST_INPUT; arm_num = 2; Serial.println("Adjusting arm2. Input 3 floats."); }
    else if (input == "q") { to_idle(); }
    else { Serial.println("Choose arm by entering 1/2."); }
}

void testInputMode() {
    if (!Serial.available()) return;
    String line = Serial.readStringUntil('\n');
    line.trim();
    if (line.length() == 0) return;
    short head = 0;
    while (head < line.length() && idx_test < 3) {
        short tail = line.indexOf(' ', head);
        if (tail == -1) tail = line.length();
        String seg = line.substring(head, tail);
        seg.trim();
        float ang = seg.toFloat();
        bool ok = seg.length() > 0 && checkAngleRange(ang, idx_test, arm_num);
        if (!ok) {
            Serial.println("Bad angle: " + seg);
            return;
        }
        input_receiver[idx_test] = ang;
        head = tail + 1;
        loadTarget(ang, idx_test, arm_num);
        if (idx_test == 2) {
            ang = -input_receiver[1] - input_receiver[2];
            loadTarget(ang, 3, arm_num);
        }
        idx_test++;
    }
    if (idx_test < 3) {
        Serial.print("Got "); Serial.print(idx_test); Serial.println("/3 angles, keep sending...");
        return;
    }
    point = getPosition(input_receiver, arm_num);
    Serial.print("Moving to ("); Serial.print(point.x, 3); Serial.print(", ");
    Serial.print(point.y, 3); Serial.print(", "); Serial.print(point.z, 3); Serial.println(")");
    startMove();
    idx_test = 0;
    state = TEST_SELECT;
    Serial.println("You can test another arm.");
}

// ============================================================
// 交互模式 — SOLVE
// ============================================================
void solveSelectMode() {
    if (!Serial.available()) return;
    String input = Serial.readStringUntil('\n');
    input.trim();
    if (input == "1") { state = SOLVE_INPUT; arm_num = 1; Serial.println("Solving arm1. Input 3 floats."); }
    else if (input == "2") { state = SOLVE_INPUT; arm_num = 2; Serial.println("Solving arm2. Input 3 floats."); }
    else if (input == "q") { to_idle(); }
    else { Serial.println("Choose arm by entering 1/2."); }
}

void solveInputMode() {
    if (!Serial.available()) return;
    String line = Serial.readStringUntil('\n');
    line.trim();
    if (line.length() == 0) return;
    short head = 0;
    while (head < line.length() && idx_solve < 3) {
        short tail = line.indexOf(' ', head);
        if (tail == -1) tail = line.length();
        String seg = line.substring(head, tail);
        seg.trim();
        float len = seg.toFloat();
        bool ok = seg.length() > 0 && len >= -300 && len <= 300;
        if (!ok) {
            Serial.println("Bad length: " + seg);
            return;
        }
        input_receiver[idx_solve] = len;
        head = tail + 1;
        idx_solve++;
    }
    if (idx_solve < 3) {
        Serial.print("Got "); Serial.print(idx_solve); Serial.println("/3 axis, keep sending...");
        return;
    }
    point.x = input_receiver[0]; point.y = input_receiver[1]; point.z = input_receiver[2];
    short solution_cnt = solve(point, solutions, arm_num);
    Serial.print("Found "); Serial.print(solution_cnt); Serial.println(" solutions.");
    if (solution_cnt >= 1) {
        Serial.println("Moving to first solution.");
        loadTarget(solutions[0].t1, 0, arm_num);
        loadTarget(solutions[0].t2, 1, arm_num);
        loadTarget(solutions[0].t3, 2, arm_num);
        loadTarget(solutions[0].t4, 3, arm_num);
        startMove();
    }
    idx_solve = 0;
    state = SOLVE_SELECT;
    Serial.println("You can solve another arm.");
}

// ============================================================
// 交互模式 — WORK
// ============================================================
void workidleMode() {
    if (Serial.available()) {
        String input = Serial.readStringUntil('\n');
        input.trim();
        if (input == "q") { auto_work_active = false; state = INIT; return; }
        else if (input == "s" && !auto_work_active) {
            auto_work_active = true; auto_work_state = 0; auto_work_timer = 0;
            Serial.println("Auto work started.");
        }
        else if (input == "p" && auto_work_active) {
            auto_work_active = false;
            Serial.println("Paused. s=restart, q=quit.");
        }
        else { Serial.println("s=start, p=stop, q=quit."); }
    }
    if (auto_work_active) state = WORKING;
}

void workingMode() {
    // 运行自动序列的同时，允许串口输入退出/暂停
    if (Serial.available()) {
        String input = Serial.readStringUntil('\n');
        input.trim();
        if (input == "q") {
            auto_work_active = false;
            moving = false;
            pending_fc_done = false;
            initAllServo();
            to_idle();
            return;
        }
        else if (input == "p") {
            auto_work_active = false;
            state = WORK_IDLE;
            Serial.println("Paused. s=restart, q=quit.");
            return;
        }
    }

    updateAutoWork();
    if (!auto_work_active) {
        state = WORK_IDLE;
        Serial.println("Auto work done.");
    }
}

// ============================================================
// 初始化
// ============================================================
void setup() {
    Serial.begin(115200);
    Serial1.begin(115200);   // FC 通信

    initAttachServo();
    initFan();

    // ---- VL53L1X 激光传感器初始化 ----
    // Wire.begin();
    // Wire.setClock(400000);  // 400kHz I2C
    // laser_sensor.setTimeout(500);
    // if (laser_sensor.init()) {
    //     laser_sensor.setDistanceMode(VL53L1X::Long);
    //     laser_sensor.setMeasurementTimingBudget(50000);  // 50ms budget
    //     laser_sensor.startContinuous(50);  // 50ms 采样周期 (20Hz)
    //     laser_available = true;
    //     Serial.println("VL53L1X laser sensor ready.");
    // } else {
    //     laser_available = false;
    //     Serial.println("ERROR: VL53L1X not detected! Check wiring.");
    // }

    initAllServo();

    Serial.println("Arms ready. FC on Serial1, debug on Serial.");
    Serial.println("t=test s=solve w=auto f=fan i=init");
}

// ============================================================
// 主循环
// ============================================================
void loop() {
    // ---------- 1. 处理 FC 串口指令 (Serial1) ----------
    while (Serial1.available()) {
        uint8_t b = Serial1.read();
        ArmFrame frame;
        if (arm_parse_byte(b, &frame)) {
            process_fc_command(&frame);
        }
    }

    // ---------- 2. 处理 USB 交互指令 (Serial) ----------
    switch (state) {
        case IDLE:         idleMode();         break;
        case INIT:         initMode();         break;
        case TEST_SELECT:  testSelectMode();   break;
        case TEST_INPUT:   testInputMode();    break;
        case SOLVE_SELECT: solveSelectMode();  break;
        case SOLVE_INPUT:  solveInputMode();   break;
        case WORK_IDLE:    workidleMode();     break;
        case WORKING:      workingMode();      break;
        case FAN_DEBUG:    fanDebugMode();     break;
        default:           state = IDLE;       break;
    }

    // ---------- 3. 驱动平滑移动 (非阻塞) ----------
    was_moving = moving;
    bool finished = updateSmoothMove();

    // 移动完成 → 通知FC
    if (was_moving && finished && pending_fc_done) {
        pending_fc_done = false;
        uint8_t txBuf[32];
        uint8_t len = arm_build_done(txBuf);
        Serial1.write(txBuf, len);
        Serial.println("[FC] Move done.");
    }

    // ---------- 4. 自动工作序列 ----------
    updateAutoWork();

    // ---------- 5. 定期读取 VL53L1X 激光 ----------
    if (laser_available && millis() - lastLaserRead >= LASER_READ_PERIOD_MS) {
        if (laser_sensor.dataReady()) {
            laser_distance = laser_sensor.read();
        }
        lastLaserRead = millis();
    }

    // ---------- 6. 定期读取 FSR ----------
    static unsigned long lastFSR = 0;
    if (millis() - lastFSR >= 500) {
        readfsr();
        lastFSR = millis();

        // ---- 传感器实时输出 (激光 + 压力) ----
        // Serial.print("[SENSOR] Laser: ");
        // Serial.print(laser_distance);
        // Serial.print(" mm | FSR0: ");
        // Serial.print(fsrs[0]);
        // Serial.print(" (");
        // Serial.print(fsrs_cur[0] ? "ON" : "OFF");
        // Serial.print(") | FSR1: ");
        // Serial.print(fsrs[1]);
        // Serial.print(" (");
        // Serial.print(fsrs_cur[1] ? "ON" : "OFF");
        // Serial.println(")");
    }

    // ---------- 7. 定期状态上报 ----------
    if (millis() - lastStatusTime >= STATUS_PERIOD_MS) {
        sendStatus();
        lastStatusTime = millis();
    }
}
