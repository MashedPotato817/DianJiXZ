#include "servo.h"

static float g_servo_target_angle_deg = SERVO_ANGLE_NEUTRAL_DEG;

void Servo_Init(void)
{
    g_servo_target_angle_deg = SERVO_ANGLE_NEUTRAL_DEG;
}

void Servo_SetTargetAngle(float angle_deg)
{
    if (angle_deg > SERVO_ANGLE_MAX_DEG) {
        angle_deg = SERVO_ANGLE_MAX_DEG;
    } else if (angle_deg < SERVO_ANGLE_MIN_DEG) {
        angle_deg = SERVO_ANGLE_MIN_DEG;
    }
    g_servo_target_angle_deg = angle_deg;
}

float Servo_GetTargetAngle(void)
{
    return g_servo_target_angle_deg;
}

void Servo_ApplyHardware(void)
{
    /*
     * TODO(硬件接线确认后)：将目标角度映射为 50 Hz PWM 脉宽，
     * 并写入 SysConfig 新增的舵机 TimerA 比较通道。
     */
}
