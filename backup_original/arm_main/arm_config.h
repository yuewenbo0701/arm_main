#ifndef __ARM_CONFIG_H__
#define __ARM_CONFIG_H__

#include <Servo.h>

/* ---------- 当前实时 PWM 值 ---------- */
float SERVO_ARM1[4] = {1500, 1500, 1500, 1500};
float SERVO_ARM2[4] = {1500, 1500, 1500, 1500};

/* ---------- 舵机校准数据 (三点标定法) ---------- */
// 标定步骤:
//   1. 将关节置于运动学 0° 位置, 记录 PWM → 填入 CALIB_ZERO
//   2. 将关节置于运动学 +90° 位置, 记录 PWM → 填入 CALIB_PLUS90
//   3. 将关节置于运动学 -90° 位置, 记录 PWM → 填入 CALIB_MINUS90

// Arm1: 4 个关节的标定 PWM
const float SERVO_ARM1_CALIB_ZERO[4]    = {1500, 1500, 1500, 1500};
const float SERVO_ARM1_CALIB_PLUS90[4]  = {2450,  550, 2400,  650};
const float SERVO_ARM1_CALIB_MINUS90[4] = { 550, 2450,  600, 2350};

// Arm2: 4 个关节的标定 PWM
const float SERVO_ARM2_CALIB_ZERO[4]    = {1450, 1500, 1500, 1500};
const float SERVO_ARM2_CALIB_PLUS90[4]  = { 550, 2300,  650, 2350};
const float SERVO_ARM2_CALIB_MINUS90[4] = {2300,  700, 2300,  650};

/* ---------- 角度限位 (运动学角度, 单位: 度) ---------- */
const float SERVO_ARM1_ANGLE_MIN[4] = {-60, -30, -30, -75};
const float SERVO_ARM1_ANGLE_MAX[4] = { 60,  75,  75,  75};
const float SERVO_ARM2_ANGLE_MIN[4] = {-60, -30, -30, -75};
const float SERVO_ARM2_ANGLE_MAX[4] = { 60,  75,  75,  75};

/* ---------- 舵机对象 ---------- */
Servo servo_1_1; Servo servo_1_2; Servo servo_1_3; Servo servo_1_4;
Servo servo_2_1; Servo servo_2_2; Servo servo_2_3; Servo servo_2_4;

/* ---------- 固定速度：us / s ---------- */
#define SPEED_US_PER_SEC 400
#define UPDATE_PERIOD_MS 10

/* ---------- 引脚绑定 ---------- */
void initAttachServo() {
  servo_1_1.attach(4); servo_1_2.attach(5); servo_1_3.attach(6); servo_1_4.attach(7);
  servo_2_1.attach(10); servo_2_2.attach(11); servo_2_3.attach(12); servo_2_4.attach(13);
}

/* ---------- 一次性写入 ---------- */
void writeAllServo() {
  servo_1_1.writeMicroseconds(SERVO_ARM1[0]); servo_1_2.writeMicroseconds(SERVO_ARM1[1]);
  servo_1_3.writeMicroseconds(SERVO_ARM1[2]); servo_1_4.writeMicroseconds(SERVO_ARM1[3]);
  servo_2_1.writeMicroseconds(SERVO_ARM2[0]); servo_2_2.writeMicroseconds(SERVO_ARM2[1]);
  servo_2_3.writeMicroseconds(SERVO_ARM2[2]); servo_2_4.writeMicroseconds(SERVO_ARM2[3]);
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

/* --------- 涵道风扇 (ESC, 每臂一个) --------- */
const int FAN_ESC_PIN_ARM1   = 8;
const int FAN_ESC_PIN_ARM2   = 9;
const int FAN_THROTTLE_MIN   = 1000;
const int FAN_THROTTLE_MAX   = 2000;
const int FAN_THROTTLE_ON    = 1200;
const int FAN_RAMP_DELAY     = 5;
const int FAN_RAMP_STEP      = 20;

Servo fan_esc_arm1;
Servo fan_esc_arm2;
int fanThrottle[2] = {1000, 1000};   // [0]=Arm1, [1]=Arm2

#endif
