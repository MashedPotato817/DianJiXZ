#include "ball_control.h"
#include "servo.h"

static Ball_Control g_ball_control;

void Ball_Control_Init(void)
{
    g_ball_control.kp = BALL_CONTROL_KP_DEG_PER_MM;
    g_ball_control.kd = BALL_CONTROL_KD_DEG_S_PER_MM;
    g_ball_control.target_x_mm = 0.0f;
    g_ball_control.last_error_mm = 0.0f;
    g_ball_control.last_timestamp_ms = 0U;
    g_ball_control.enabled = BALL_CONTROL_ENABLE_DEFAULT;
    g_ball_control.has_last_sample = 0U;
    Servo_SetTargetAngle(SERVO_ANGLE_NEUTRAL_DEG);
    Servo_ApplyHardware();
}

void Ball_Control_SetTarget(float target_x_mm)
{
    g_ball_control.target_x_mm = target_x_mm;
    g_ball_control.last_error_mm = 0.0f;
    g_ball_control.has_last_sample = 0U;
}

void Ball_Control_SetEnabled(uint8_t enabled)
{
    g_ball_control.enabled = (enabled != 0U) ? 1U : 0U;
    Ball_Control_Reset();
}

uint8_t Ball_Control_IsEnabled(void)
{
    return g_ball_control.enabled;
}

void Ball_Control_Reset(void)
{
    g_ball_control.last_error_mm = 0.0f;
    g_ball_control.last_timestamp_ms = 0U;
    g_ball_control.has_last_sample = 0U;
    Servo_SetTargetAngle(SERVO_ANGLE_NEUTRAL_DEG);
    Servo_ApplyHardware();
}

void Ball_Control_Step(const K230_BallPosition *position, float period_s)
{
    float error;
    float derivative;
    float output;
    float sample_period_s;

    if ((g_ball_control.enabled == 0U) || (position == 0) ||
        (position->valid == 0U) || (period_s <= 0.0f)) {
        Ball_Control_Reset();
        return;
    }

    /* K230 帧率低于 200 Hz；同一帧不可在每个 5 ms 周期重复做 D 项。 */
    if ((g_ball_control.has_last_sample != 0U) &&
        (position->timestamp_ms == g_ball_control.last_timestamp_ms)) {
        return;
    }

    error = g_ball_control.target_x_mm - position->x_mm;
    if ((error < BALL_CONTROL_DEADBAND_MM) &&
        (error > -BALL_CONTROL_DEADBAND_MM)) {
        error = 0.0f;
    }
    sample_period_s = period_s;
    if (g_ball_control.has_last_sample != 0U) {
        sample_period_s = (float)(position->timestamp_ms -
                          g_ball_control.last_timestamp_ms) / 1000.0f;
        if ((sample_period_s < period_s) || (sample_period_s > 0.1f)) {
            sample_period_s = period_s;
        }
    }
    derivative = (g_ball_control.has_last_sample != 0U) ?
                 (error - g_ball_control.last_error_mm) / sample_period_s : 0.0f;
    output = g_ball_control.kp * error + g_ball_control.kd * derivative;
    Servo_SetTargetAngle(output);
    Servo_ApplyHardware();
    g_ball_control.last_error_mm = error;
    g_ball_control.last_timestamp_ms = position->timestamp_ms;
    g_ball_control.has_last_sample = 1U;
}

const Ball_Control *Ball_Control_Get(void)
{
    return &g_ball_control;
}
