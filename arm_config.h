#ifndef __ARM_CONFIG_H__
#define __ARM_CONFIG_H__

#include <Servo.h>

/* ---------- 机械臂数量与编号 ----------
 * 观察者面对机头方向:
 *   臂1 = 左上 (引脚 4,5,6,7)     臂2 = 右上 (引脚 10,11,12,13)
 *   臂3 = 左下 (引脚 22,23,24,25) 臂4 = 右下 (引脚 28,29,30,31)
 * 数组下标按 1-based 使用 (下标 0 闲置), 与协议臂号一一对应 */
#define ARM_COUNT 4

/* ---------- 当前实时 PWM 值 ---------- */
float SERVO_PWM[ARM_COUNT + 1][4] = {
  {1500, 1500, 1500, 1500},   // 下标0 闲置
  {1500, 1500, 1500, 1500},   // Arm1 左上
  {1500, 1500, 1500, 1500},   // Arm2 右上
  {1500, 1500, 1500, 1500},   // Arm3 左下
  {1500, 1500, 1500, 1500},   // Arm4 右下
};

/* ---------- 舵机校准数据 (三点标定法) ----------
 * 标定步骤:
 *   1. 将关节置于运动学 0° 位置, 记录 PWM → 填入 CALIB_ZERO
 *   2. 将关节置于运动学 +90° 位置, 记录 PWM → 填入 CALIB_PLUS90
 *   3. 将关节置于运动学 -90° 位置, 记录 PWM → 填入 CALIB_MINUS90
 *
 * !!! 注意: 臂3(左下)/臂4(右下) 的标定值目前是占位值,
 *     分别直接复制自 臂1(左上)/臂2(右上), 装车后必须重新做三点标定 !!! */
const float SERVO_CALIB_ZERO[ARM_COUNT + 1][4] = {
  {1500, 1500, 1500, 1500},   // 下标0 闲置
  {1500, 1500, 1500, 1500},   // Arm1 左上
  {1450, 1500, 1500, 1500},   // Arm2 右上
  {1500, 1550, 1600, 1400},   // Arm3 左下 (占位=Arm1, 待标定)
  {1450, 1550, 1500, 1500},   // Arm4 右下 (占位=Arm2, 待标定)
};
const float SERVO_CALIB_PLUS90[ARM_COUNT + 1][4] = {
  {1500, 1500, 1500, 1500},   // 下标0 闲置
  {2450, 2400, 2450,  600},   // Arm1 左上
  { 550,  550,  550, 2350},   // Arm2 右上
  {2450, 2400, 2450,  550},   // Arm3 左下 (占位=Arm1, 待标定)
  { 550,  650,  550, 2400},   // Arm4 右下 (占位=Arm2, 待标定)
};
const float SERVO_CALIB_MINUS90[ARM_COUNT + 1][4] = {
  {1500, 1500, 1500, 1500},   // 下标0 闲置
  { 550,  600,  550, 2400},   // Arm1 左上
  {2300, 2450, 2450,  650},   // Arm2 右上
  { 550,  600,  550, 2450},   // Arm3 左下 (占位=Arm1, 待标定)
  {2300, 2400, 2450,  600},   // Arm4 右下 (占位=Arm2, 待标定)
};

/* ---------- 角度限位 (运动学角度, 单位: 度), 四臂一致 ---------- */
const float SERVO_ANGLE_MIN[ARM_COUNT + 1][4] = {
  {   0,   0,   0,   0},      // 下标0 闲置
  { -60, -30, -30, -75},
  { -60, -30, -30, -75},
  { -60, -30, -30, -75},
  { -60, -30, -30, -75},
};
const float SERVO_ANGLE_MAX[ARM_COUNT + 1][4] = {
  {   0,   0,   0,   0},      // 下标0 闲置
  {  60,  75,  75,  75},
  {  60,  75,  75,  75},
  {  60,  75,  75,  75},
  {  60,  75,  75,  75},
};

/* ---------- 舵机对象 ---------- */
Servo servos[ARM_COUNT + 1][4];   // [臂号1~4][关节0~3]

/* ---------- 固定速度：us / s ---------- */
#define SPEED_US_PER_SEC 300
#define UPDATE_PERIOD_MS 10

/* ---------- 引脚绑定 ---------- */
const uint8_t SERVO_PINS[ARM_COUNT + 1][4] = {
  { 0,  0,  0,  0},           // 下标0 闲置
  { 4,  5,  6,  7},           // Arm1 左上
  {10, 11, 12, 13},           // Arm2 右上
  {22, 23, 24, 25},           // Arm3 左下
  {28, 29, 30, 31},           // Arm4 右下
};

void initAttachServo() {
  for (short a = 1; a <= ARM_COUNT; a++) {
    for (short j = 0; j < 4; j++) {
      servos[a][j].attach(SERVO_PINS[a][j]);
    }
  }
}

/* ---------- 一次性写入 ---------- */
void writeAllServo() {
  for (short a = 1; a <= ARM_COUNT; a++) {
    for (short j = 0; j < 4; j++) {
      servos[a][j].writeMicroseconds(SERVO_PWM[a][j]);
    }
  }
}

/* --------- 角度与 PWM 互转 (三点标定, 无 FLAG) --------- */
// 用 0° 和 +90° 两个校准点确定直线
// 角度 = (PWM - zero) * 90 / (plus90 - zero)
// PWM  = 角度 * (plus90 - zero) / 90 + zero
// 方向自动蕴含在 (plus90 - zero) 的符号中

float PWM_to_Angle(float pwm, float calib_zero, float calib_plus90) {
  return (pwm - calib_zero) * 90.0 / (calib_plus90 - calib_zero);
}

float Angle_to_PWM(float angle, float calib_zero, float calib_plus90) {
  return angle * (calib_plus90 - calib_zero) / 90.0 + calib_zero;
}

/* --------- 涵道风扇 (ESC, 每臂末端一个) --------- */
// 协议风扇号: 1=臂1左上(引脚8) 2=臂2右上(引脚9) 5=臂3左下(引脚26) 6=臂4右下(引脚27)
const int FAN_ESC_PIN_ARM1   = 8;
const int FAN_ESC_PIN_ARM2   = 9;
const int FAN_ESC_PIN_ARM3   = 26;
const int FAN_ESC_PIN_ARM4   = 27;
const int FAN_THROTTLE_MIN   = 1000;
const int FAN_THROTTLE_MAX   = 2000;
const int FAN_THROTTLE_ON    = 1200;
const int FAN_RAMP_DELAY     = 5;
const int FAN_RAMP_STEP      = 20;

Servo fan_esc_arm1;
Servo fan_esc_arm2;
Servo fan_esc_arm3;
Servo fan_esc_arm4;
int fanThrottle[4] = {1000, 1000, 1000, 1000};   // [0..3] = Arm1..Arm4 末端风扇

/* --------- 前臂涵道风扇 (推进/辅助) --------- */
// 左前臂 -> 引脚33 (协议风扇号 3), 右前臂 -> 引脚32 (协议风扇号 4)
const int FAN_ESC_PIN_FRONT_L = 33;
const int FAN_ESC_PIN_FRONT_R = 32;

Servo fan_esc_front_l;
Servo fan_esc_front_r;
int frontFanThrottle[2] = {1000, 1000};   // [0]=左前臂, [1]=右前臂

#endif
