#ifndef H_SERVO
#define H_SERVO

#include <stdint.h>

/*
 * 摆杆舵机抽象层。
 *
 * PA8 / TIMA0 CCP0：1 MHz、20 ms PWM。角度采用绝对表示，
 * 90 度仅作为上电初始位置；角度增加表示 PWM 脉宽增加，机械上对应“右端降低”。
 * 机械允许范围已确认为 5~175 度；脉宽换算仍沿用 0~180 度对应
 * 1100~2100 us 的本机实测关系。
 */
#define SERVO_ANGLE_MIN_DEG      (5.0f)
#define SERVO_ANGLE_MAX_DEG      (175.0f)
#define SERVO_ANGLE_NEUTRAL_DEG  (90.0f)
#define SERVO_PULSE_MIN_US       (1100U)
#define SERVO_PULSE_NEUTRAL_US   (1600U)
#define SERVO_PULSE_MAX_US       (2100U)
#define SERVO_US_PER_DEG         ((float)(SERVO_PULSE_MAX_US - SERVO_PULSE_MIN_US) / 180.0f)

void Servo_Init(void);
void Servo_SetTargetAngle(float angle_deg);
float Servo_GetTargetAngle(void);
uint16_t Servo_GetPulseUs(void);
void Servo_ApplyHardware(void);

#endif
