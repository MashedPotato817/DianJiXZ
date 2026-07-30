#include "servo.h"
#include "ti_msp_dl_config.h"

static float g_servo_target_angle_deg = SERVO_ANGLE_NEUTRAL_DEG;
static uint16_t g_servo_pulse_us = SERVO_PULSE_NEUTRAL_US;

void Servo_Init(void)
{
    g_servo_target_angle_deg = SERVO_ANGLE_NEUTRAL_DEG;
    g_servo_pulse_us = SERVO_PULSE_NEUTRAL_US;
    /* 新比较值仅在周期边界载入，避免产生畸形舵机脉冲。 */
    DL_TimerA_setCaptCompUpdateMethod(PWM_1_INST,
        DL_TIMER_CC_UPDATE_METHOD_ZERO_EVT, DL_TIMER_CC_0_INDEX);
    Servo_ApplyHardware();
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

uint16_t Servo_GetPulseUs(void)
{
    return g_servo_pulse_us;
}

void Servo_ApplyHardware(void)
{
    float pulse = (float)SERVO_PULSE_NEUTRAL_US +
                  g_servo_target_angle_deg * SERVO_US_PER_DEG;

    if (pulse > (float)SERVO_PULSE_MAX_US) {
        pulse = (float)SERVO_PULSE_MAX_US;
    } else if (pulse < (float)SERVO_PULSE_MIN_US) {
        pulse = (float)SERVO_PULSE_MIN_US;
    }
    g_servo_pulse_us = (uint16_t)(pulse + 0.5f);

    /* TIMA0 为向下计数 PWM：高电平脉宽 = period - compare。 */
    DL_TimerA_setCaptureCompareValue(PWM_1_INST,
        20000U - (uint32_t)g_servo_pulse_us, DL_TIMER_CC_0_INDEX);
}
