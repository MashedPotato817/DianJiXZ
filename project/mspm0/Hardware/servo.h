#ifndef H_SERVO
#define H_SERVO

#include <stdint.h>

/*
 * 摆杆舵机抽象层。
 *
 * PA8 / TIMA0 CCP0：1 MHz、20 ms PWM。正角度表示 PWM 脉宽增加；
 * 当前机械定义为“右端降低”。实测安全脉宽为 1050~2050 us，中位为 1550 us；
 * 首轮闭环仍由角度限幅限制在中位两侧约 50 us。
 */
#define SERVO_ANGLE_MIN_DEG      (-5.0f)
#define SERVO_ANGLE_MAX_DEG      (5.0f)
#define SERVO_ANGLE_NEUTRAL_DEG  (0.0f)
#define SERVO_PULSE_MIN_US       (1100U)
#define SERVO_PULSE_NEUTRAL_US   (1600U)
#define SERVO_PULSE_MAX_US       (2100U)
#define SERVO_US_PER_DEG         (10.0f)

void Servo_Init(void);
void Servo_SetTargetAngle(float angle_deg);
float Servo_GetTargetAngle(void);
uint16_t Servo_GetPulseUs(void);
void Servo_ApplyHardware(void);

#endif
