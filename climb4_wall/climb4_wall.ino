// ============================================================
// 8fly 四机械臂 · 爬墙程序 (climb4_wall.ino) —— 脱机独立版
// ------------------------------------------------------------
// 基于 fan.ino 的串口交互风格重写, 功能与上位机脚本
// climb4_wall_test.py 完全一致: 不再需要树莓派,
// 打开串口监视器 (115200) 发送单键命令即可:
//   l  初始化: 四臂同时到 G1 (低位贴墙), 臂端风扇启动到 1300us 吸附
//   k  开始爬墙循环 (左上->右上->左下->右下->四臂同时向下)
//   s  急停: 停止当前动作/循环, 全部风扇渐弱关闭
//   q  同 s (脚本版是退出, 单片机上等价于急停回到待命)
//   h  帮助
//
// 机械臂引脚 (观察者面对机头方向, 根部->末端 4 个关节):
//   臂1 = 左上: 引脚 4, 5, 6, 7    臂端涵道风扇 = 引脚 8
//   臂2 = 右上: 引脚 10, 11, 12, 13 臂端涵道风扇 = 引脚 9
//   臂3 = 左下: 引脚 22, 23, 24, 25 臂端涵道风扇 = 引脚 26
//   臂4 = 右下: 引脚 28, 29, 30, 31 臂端涵道风扇 = 引脚 27
//
// 步态四组关节角 (度, 根部 -> 末端):
//   G1 = {-30,  10,  10, -20}   低位贴墙 (循环终点"向下"的位置, INIT)
//   G2 = {-30, -10, -10,  20}   低位离墙
//   G3 = { 30, -10, -10,  20}   高位离墙 (抬升)
//   G4 = { 30,  10,  10, -20}   高位贴墙
//
// 臂端涵道风扇联动 (与 climb4_wall_test.py 相同):
//   贴墙到位后 -> 直跳 1200us 启动再补到 1300us 吸附
//   离墙之前   -> 渐弱至 1000us 关闭
//   !!! 升油门不能慢爬: 电调低速蠕动会触发失速保护锁死,
//       必须先直跳到 1200 让电机转起来, 再补到目标 !!!
//
// 循环逻辑 (每轮):
//   Step 1: 左上(臂1) 风扇关 -> 离墙(G2) -> 抬升(G3) -> 贴墙(G4) -> 风扇吸附
//   Step 2: 右上(臂2) 同上
//   Step 3: 左下(臂3) 同上
//   Step 4: 右下(臂4) 同上
//   Step 5: 四个臂同时向下 (G4 -> G1), 风扇保持吸附
//
// 适用板卡: Arduino Mega 2560
// ============================================================

#include <Servo.h>

// ============================================================
// 机械臂数量与编号 (数组下标 1-based, 下标 0 闲置)
// ============================================================
#define ARM_COUNT 4

// 当前实时 PWM 值
float SERVO_PWM[ARM_COUNT + 1][4] = {
  {1500, 1500, 1500, 1500},   // 下标0 闲置
  {1500, 1500, 1500, 1500},   // Arm1 左上
  {1500, 1500, 1500, 1500},   // Arm2 右上
  {1500, 1500, 1500, 1500},   // Arm3 左下
  {1500, 1500, 1500, 1500},   // Arm4 右下
};

// ============================================================
// 舵机校准数据 (三点标定法) —— 与 arm_config.h 保持一致
// !!! 臂3(左下)/臂4(右下) 的标定值目前是占位值 (复制自臂1/臂2),
//     装车后必须重新做三点标定 !!!
// ============================================================
const float SERVO_CALIB_ZERO[ARM_COUNT + 1][4] = {
  {1500, 1500, 1500, 1500},
  {1500, 1500, 1500, 1500},   // Arm1 左上
  {1450, 1500, 1500, 1500},   // Arm2 右上
  {1500, 1550, 1600, 1400},   // Arm3 左下 (占位=Arm1, 待标定)
  {1450, 1550, 1500, 1500},   // Arm4 右下 (占位=Arm2, 待标定)
};
const float SERVO_CALIB_PLUS90[ARM_COUNT + 1][4] = {
  {1500, 1500, 1500, 1500},
  {2450, 2400, 2450,  600},   // Arm1 左上
  { 550,  550,  550, 2350},   // Arm2 右上
  {2450, 2400, 2450,  550},   // Arm3 左下 (占位=Arm1, 待标定)
  { 550,  650,  550, 2400},   // Arm4 右下 (占位=Arm2, 待标定)
};

// 角度限位 (运动学角度, 单位: 度), 四臂一致
const float SERVO_ANGLE_MIN[4] = { -60, -30, -30, -75 };
const float SERVO_ANGLE_MAX[4] = {  60,  75,  75,  75 };

// 引脚绑定
const uint8_t SERVO_PINS[ARM_COUNT + 1][4] = {
  { 0,  0,  0,  0},
  { 4,  5,  6,  7},           // Arm1 左上
  {10, 11, 12, 13},           // Arm2 右上
  {22, 23, 24, 25},           // Arm3 左下
  {28, 29, 30, 31},           // Arm4 右下
};

Servo servos[ARM_COUNT + 1][4];   // [臂号1~4][关节0~3]

// 平滑移动速度 (与固件 arm_main.ino 一致)
#define SPEED_US_PER_SEC 300
#define UPDATE_PERIOD_MS 10

// ============================================================
// 臂端涵道风扇 (ESC): 臂1=引脚8 臂2=引脚9 臂3=引脚26 臂4=引脚27
// ============================================================
const uint8_t FAN_ESC_PIN[ARM_COUNT + 1] = { 0, 8, 9, 26, 40 };
const int FAN_THROTTLE_MIN  = 1000;   // 关闭
const int FAN_THROTTLE_MAX  = 2000;
const int TIP_FAN_ATTACH    = 1100;   // 贴墙吸附目标油门 (us)
const int TIP_FAN_START     = 1020;   // 电调启动油门 (直跳, 低于此值可能锁死)
const unsigned long TIP_FAN_START_HOLD_MS = 300;   // 启动油门保持时间
const int TIP_FAN_STEP      = 50;     // 降油门渐变每步 -50us
const unsigned long TIP_FAN_INTERVAL_MS = 100;     // 降油门每步间隔

Servo fanEsc[ARM_COUNT + 1];                       // 下标 1~4 = 臂1~臂4
int   fanCur[ARM_COUNT + 1] = {0, 1000, 1000, 1000, 1000};   // 各臂风扇当前油门

// ============================================================
// 步态关节角 (度, 根部 -> 末端, 4 个角全部下发)
// ============================================================
const float G1_DOWN_ATTACH[4] = {-30.0,  10.0,  10.0, -20.0};   // 低位贴墙 (INIT)
const float G2_DOWN_DETACH[4] = {-30.0, -10.0, -10.0,  20.0};   // 低位离墙
const float G3_UP_DETACH[4]   = { 30.0, -10.0, -10.0,  20.0};   // 高位离墙 (抬升)
const float G4_UP_ATTACH[4]   = { 30.0,  10.0,  10.0, -20.0};   // 高位贴墙

// 四臂定义: (协议臂号, 中文名) —— 循环动作顺序
const char* ARM_NAME[ARM_COUNT + 1] = {"", "左上", "右上", "左下", "右下"};

// ============================================================
// 时序参数 (与 climb4_wall_test.py 一致)
// ============================================================
const unsigned long MOVE_SETTLE_MS  = 500;    // 每段移动到位后的稳定等待
const unsigned long STEP_SETTLE_MS  = 1000;   // 每个臂完成动作后的等待
const unsigned long CYCLE_SETTLE_MS = 1000;   // 每轮循环结束后的等待
const int  MAX_CYCLES = 0;                    // 循环次数, 0 = 无限 (直到按 s/q)

// ============================================================
// 控制标志
// ============================================================
volatile bool stopFlag = false;   // s/q 急停: 中断当前动作/循环
bool loopActive = false;          // 爬墙循环进行中

// ============================================================
// 角度 -> PWM (三点标定, 方向蕴含在 plus90-zero 符号中)
// ============================================================
float Angle_to_PWM(float angle, short arm, short joint) {
    float clamped = constrain(angle, SERVO_ANGLE_MIN[joint], SERVO_ANGLE_MAX[joint]);
    if (clamped != angle) {
        Serial.print(F("⚠ 臂")); Serial.print(arm);
        Serial.print(F(" 关节")); Serial.print(joint + 1);
        Serial.print(F(" 角度 ")); Serial.print(angle);
        Serial.print(F(" 超限, 已钳位到 ")); Serial.println(clamped);
    }
    return clamped * (SERVO_CALIB_PLUS90[arm][joint] - SERVO_CALIB_ZERO[arm][joint])
           / 90.0 + SERVO_CALIB_ZERO[arm][joint];
}

// ============================================================
// 串口急停检查: 在阻塞动作中调用, 收到 s/q 置 stopFlag
// ============================================================
void pollStopKey() {
    while (Serial.available()) {
        char c = tolower(Serial.read());
        if (c == 's' || c == 'q') {
            Serial.print(F("\n>>> 收到急停指令 '"));
            Serial.print(c); Serial.println(F("'"));
            stopFlag = true;
        }
        // 动作期间忽略其它键 (l/k 需等当前动作结束后再按)
    }
}

// 可被打断的延时, 被打断返回 false
bool delayInterruptible(unsigned long ms) {
    unsigned long t0 = millis();
    while (millis() - t0 < ms) {
        pollStopKey();
        if (stopFlag) return false;
        delay(10);
    }
    return true;
}

// ============================================================
// 平滑移动 (阻塞, 可被 s/q 打断)
// 同步插补: 所有参与的关节按同一时间进度走, 同时开始同时到达;
// 整体速度由路程最长的关节决定 (不超过 SPEED_US_PER_SEC)
// armMask: 位掩码, bit0=臂1 ... bit3=臂4 (如 0b1111 = 四臂同时)
// 返回 true = 到位; false = 被急停打断
// ============================================================
bool smoothMove(const float angles[4], uint8_t armMask) {
    float startPwm[ARM_COUNT + 1][4];
    float targetPwm[ARM_COUNT + 1][4];
    float maxDist = 0;

    for (short a = 1; a <= ARM_COUNT; a++) {
        if (!(armMask & (1 << (a - 1)))) continue;
        for (short j = 0; j < 4; j++) {
            startPwm[a][j]  = SERVO_PWM[a][j];
            targetPwm[a][j] = Angle_to_PWM(angles[j], a, j);
            float d = fabs(targetPwm[a][j] - startPwm[a][j]);
            if (d > maxDist) maxDist = d;
        }
    }

    unsigned long duration = (unsigned long)(maxDist / SPEED_US_PER_SEC * 1000.0f);
    unsigned long t0 = millis();
    unsigned long lastUpdate = 0;

    while (true) {
        pollStopKey();
        if (stopFlag) return false;

        unsigned long now = millis();
        float t = (duration == 0) ? 1.0f : (float)(now - t0) / duration;
        if (t >= 1.0f) t = 1.0f;

        if (now - lastUpdate >= UPDATE_PERIOD_MS || t >= 1.0f) {
            lastUpdate = now;
            for (short a = 1; a <= ARM_COUNT; a++) {
                if (!(armMask & (1 << (a - 1)))) continue;
                for (short j = 0; j < 4; j++) {
                    SERVO_PWM[a][j] = startPwm[a][j] + (targetPwm[a][j] - startPwm[a][j]) * t;
                    servos[a][j].writeMicroseconds(SERVO_PWM[a][j]);
                }
            }
        }
        if (t >= 1.0f) break;
    }
    return true;
}

bool moveOneArm(short arm, const float angles[4]) {
    return smoothMove(angles, 1 << (arm - 1));
}

bool moveAllArms(const float angles[4]) {
    return smoothMove(angles, 0b1111);
}

// ============================================================
// 臂端涵道风扇渐变 (阻塞, 可被打断)
// 升油门: 先直跳 TIP_FAN_START 让电机转起来, 再补到 target
//         (慢爬会触发电调失速保护锁死: 渐变不转/直跳能转)
// 降油门: 渐变至 target (电机已在转, 无失速问题, 缓慢降更平顺)
// ============================================================
bool tipFanRamp(short arm, int target) {
    target = constrain(target, FAN_THROTTLE_MIN, FAN_THROTTLE_MAX);
    int cur = fanCur[arm];
    if (cur == target) return true;

    if (target > cur) {
        // ---- 升油门: 直跳启动油门 -> 补到目标 ----
        if (cur < TIP_FAN_START) {
            Serial.print(F("  [臂端风扇] 臂")); Serial.print(arm);
            Serial.print(F(" ")); Serial.print(cur);
            Serial.print(F(" -> ")); Serial.print(TIP_FAN_START);
            Serial.println(F("us (直跳启动)"));
            fanEsc[arm].writeMicroseconds(TIP_FAN_START);
            fanCur[arm] = TIP_FAN_START;
            if (!delayInterruptible(TIP_FAN_START_HOLD_MS)) return false;
        }
        if (target != TIP_FAN_START) {
            Serial.print(F("  [臂端风扇] 臂")); Serial.print(arm);
            Serial.print(F(" ")); Serial.print(fanCur[arm]);
            Serial.print(F(" -> ")); Serial.print(target);
            Serial.println(F("us"));
            fanEsc[arm].writeMicroseconds(target);
            fanCur[arm] = target;
        }
        return true;
    }

    // ---- 降油门: 渐变至 target ----
    Serial.print(F("  [臂端风扇] 臂")); Serial.print(arm);
    Serial.print(F(" ")); Serial.print(cur);
    Serial.print(F(" -> ")); Serial.print(target);
    Serial.print(F("us (每 ")); Serial.print(TIP_FAN_INTERVAL_MS);
    Serial.print(F("ms -")); Serial.print(TIP_FAN_STEP);
    Serial.println(F("us)"));
    int t = cur;
    while (t != target) {
        pollStopKey();
        if (stopFlag) return false;
        t -= TIP_FAN_STEP;
        if (t < target) t = target;
        fanEsc[arm].writeMicroseconds(t);
        fanCur[arm] = t;
        if (t != target && !delayInterruptible(TIP_FAN_INTERVAL_MS)) return false;
    }
    return true;
}

// ============================================================
// 急停: 立即停止所有动作, 全部臂端风扇渐弱关闭
// ============================================================
void emergencyStop() {
    Serial.println(F("\n!!! 急停: 停止动作, 关闭全部臂端风扇 !!!"));
    for (short a = 1; a <= ARM_COUNT; a++) {
        // 快速渐弱关闭 (每 5ms -20us, 与固件 fanOff 一致)
        while (fanCur[a] > FAN_THROTTLE_MIN) {
            int t = fanCur[a] - 20;
            if (t < FAN_THROTTLE_MIN) t = FAN_THROTTLE_MIN;
            fanEsc[a].writeMicroseconds(t);
            fanCur[a] = t;
            delay(5);
        }
    }
    Serial.println(F("已停止。输入 'l' 重新初始化, 'k' 重新开始爬墙。"));
}

// ============================================================
// [L 指令] 初始化: 四臂同时到 G1 (低位贴墙), 风扇启动到 1300us 吸附
// ============================================================
void doInit() {
    Serial.println(F("\n=== 执行初始化 (L) ==="));
    Serial.println(F("同时移动 臂1(左上)/臂2(右上)/臂3(左下)/臂4(右下) 至 G1 {-30, 10, 10, -20}"));

    if (moveAllArms(G1_DOWN_ATTACH)) {
        Serial.print(F("✓ 初始化位置到达成功, 臂端风扇启动至 "));
        Serial.print(TIP_FAN_ATTACH);
        Serial.println(F("us 吸附 ..."));
        for (short a = 1; a <= ARM_COUNT; a++) {
            if (!tipFanRamp(a, TIP_FAN_ATTACH)) break;
        }
    } else {
        Serial.println(F("✕ 初始化移动被中断"));
    }
}

// ============================================================
// 单臂爬墙步骤: 风扇渐弱关 -> 离墙(G2) -> 抬升(G3) -> 贴墙(G4) -> 风扇吸附
// 其余三个臂保持不动作为支撑
// ============================================================
bool stepArmLift(short arm) {
    // 0. 离墙前解除吸附: 臂端风扇渐弱至 1000us (关闭)
    if (!tipFanRamp(arm, FAN_THROTTLE_MIN)) return false;

    // 1. 离墙: 低位贴墙(G1) -> 低位离墙(G2)
    Serial.print(F("  [离墙] 臂")); Serial.print(arm);
    Serial.print(F("(")); Serial.print(ARM_NAME[arm]);
    Serial.println(F(") 移动至 G2 {-30, -10, -10, 20}..."));
    if (!moveOneArm(arm, G2_DOWN_DETACH)) return false;
    if (!delayInterruptible(MOVE_SETTLE_MS)) return false;

    // 2. 抬升: 低位离墙(G2) -> 高位离墙(G3)
    Serial.print(F("  [抬升] 臂")); Serial.print(arm);
    Serial.print(F("(")); Serial.print(ARM_NAME[arm]);
    Serial.println(F(") 移动至 G3 {30, -10, -10, 20}..."));
    if (!moveOneArm(arm, G3_UP_DETACH)) return false;
    if (!delayInterruptible(MOVE_SETTLE_MS)) return false;

    // 3. 贴墙: 高位离墙(G3) -> 高位贴墙(G4)
    Serial.print(F("  [贴墙] 臂")); Serial.print(arm);
    Serial.print(F("(")); Serial.print(ARM_NAME[arm]);
    Serial.println(F(") 移动至 G4 {30, 10, 10, -20}..."));
    if (!moveOneArm(arm, G4_UP_ATTACH)) return false;
    if (!delayInterruptible(MOVE_SETTLE_MS)) return false;

    // 4. 贴墙完成: 臂端风扇直跳启动至 1300us 吸附
    if (!tipFanRamp(arm, TIP_FAN_ATTACH)) return false;

    Serial.print(F("  臂")); Serial.print(arm);
    Serial.print(F("(")); Serial.print(ARM_NAME[arm]);
    Serial.print(F(") 完成，等待 ")); Serial.print(STEP_SETTLE_MS / 1000.0);
    Serial.println(F("s ..."));
    return delayInterruptible(STEP_SETTLE_MS);
}

// ============================================================
// [K 指令] 爬墙循环: Step 1~4 依次抬四个臂 -> Step 5 四臂同时向下
// ============================================================
void runLoop() {
    loopActive = true;
    int cycle = 0;

    while (!stopFlag) {
        cycle++;
        Serial.print(F("\n================ [爬墙循环第 "));
        Serial.print(cycle);
        Serial.println(F(" 次] ================"));

        // ---- Step 1~4: 左上 -> 右上 -> 左下 -> 右下 依次 离墙/抬升/贴墙 ----
        for (short step = 1; step <= ARM_COUNT; step++) {
            Serial.print(F("\n--> Step ")); Serial.print(step);
            Serial.print(F(": 臂")); Serial.print(step);
            Serial.print(F("(")); Serial.print(ARM_NAME[step]);
            Serial.println(F(") 离墙 -> 抬升 -> 贴墙"));
            if (!stepArmLift(step)) goto loop_end;
        }

        // ---- Step 5: 四个臂同时向下 (G4 -> G1), 风扇保持吸附 ----
        Serial.println(F("\n--> Step 5: 四个臂同时向下移动至 G1 {-30, 10, 10, -20}"));
        if (!moveAllArms(G1_DOWN_ATTACH)) break;

        Serial.print(F("  Step 5 完成，等待 "));
        Serial.print(CYCLE_SETTLE_MS / 1000.0);
        Serial.println(F("s ..."));
        if (!delayInterruptible(CYCLE_SETTLE_MS)) break;

        if (MAX_CYCLES > 0 && cycle >= MAX_CYCLES) {
            Serial.print(F("\n到达指定循环次数 ("));
            Serial.print(MAX_CYCLES);
            Serial.println(F(")，停止循环"));
            break;
        }
    }

loop_end:
    loopActive = false;
    if (stopFlag) Serial.println(F("循环被急停中断。"));
}

// ============================================================
// 打印帮助
// ============================================================
void printHelp() {
    Serial.println(F("\n────────── 命令列表 ──────────"));
    Serial.println(F(" l  初始化: 四臂同时到 G1 低位贴墙, 风扇渐大吸附"));
    Serial.println(F(" k  开始爬墙循环 (左上->右上->左下->右下->四臂同时向下)"));
    Serial.println(F(" s  急停: 停止动作/循环, 全部风扇渐弱关闭"));
    Serial.println(F(" q  同 s"));
    Serial.println(F(" h  显示本帮助"));
    Serial.println(F("──────────────────────────────"));
    Serial.println(F("步态: G1低位贴墙 -> G2低位离墙 -> G3高位离墙 -> G4高位贴墙 -> 四臂同时回G1"));
    Serial.println(F("风扇: 贴墙直跳1200再补到1300吸附 / 离墙渐弱至1000关闭"));
}

// ============================================================
// 初始化
// ============================================================
void setup() {
    Serial.begin(115200);
    delay(500);

    Serial.println(F("\n╔══════════════════════════════════════╗"));
    Serial.println(F("║   8fly 四机械臂 · 爬墙程序 v1.0     ║"));
    Serial.println(F("║   板卡: Arduino Mega 2560 (脱机版)  ║"));
    Serial.println(F("╚══════════════════════════════════════╝"));
    Serial.println(F("臂号: 1=左上 2=右上 3=左下 4=右下 (臂端风扇: 8/9/26/27)"));

    // ---- 舵机挂载, 先写到中位 ----
    for (short a = 1; a <= ARM_COUNT; a++) {
        for (short j = 0; j < 4; j++) {
            servos[a][j].attach(SERVO_PINS[a][j]);
            servos[a][j].writeMicroseconds(SERVO_PWM[a][j]);
        }
    }

    // ---- 臂端涵道风扇 ESC 挂载, 先发最低油门让电调完成解锁 ----
    for (short a = 1; a <= ARM_COUNT; a++) {
        fanEsc[a].attach(FAN_ESC_PIN[a]);
        fanEsc[a].writeMicroseconds(FAN_THROTTLE_MIN);
        fanCur[a] = FAN_THROTTLE_MIN;
    }
    Serial.println(F("→ 发送解锁信号 (全部风扇最低油门 1000us)..."));
    delay(3000);   // 等电调完成上电自检/解锁

    Serial.println(F("→ 就绪! 输入 'h' 查看命令列表。"));
    printHelp();
}

// ============================================================
// 主循环: 单键命令 (串口监视器发送, 无需回车符也兼容)
// ============================================================
void loop() {
    if (Serial.available()) {
        char c = tolower(Serial.read());

        switch (c) {
            case 'l':
                Serial.println(F("\n>>> 收到初始化指令 'l'"));
                stopFlag = false;   // 清除上次急停残留, 允许新动作
                doInit();
                break;

            case 'k':
                Serial.println(F("\n>>> 收到循环启动指令 'k'"));
                if (loopActive) {
                    Serial.println(F("循环正在进行中，如需重启请先按 's' 急停。"));
                } else {
                    stopFlag = false;
                    runLoop();
                }
                break;

            case 's':
            case 'q':
                stopFlag = true;
                emergencyStop();
                break;

            case 'h':
                printHelp();
                break;

            case '\n':
            case '\r':
            case ' ':
                break;   // 忽略换行/空格

            default:
                Serial.print(F("⚠ 未知命令: "));
                Serial.println(c);
                Serial.println(F("  输入 'h' 查看命令列表"));
                break;
        }
    }
}
