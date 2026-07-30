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
    g_ball_control.edge_recovery_start_ms = 0U;
    g_ball_control.enabled = BALL_CONTROL_ENABLE_DEFAULT;
    g_ball_control.has_last_sample = 0U;
    g_ball_control.edge_recovery_active = 0U;
    g_ball_control.edge_recovery_timed_out = 0U;
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
    g_ball_control.edge_recovery_start_ms = 0U;
    g_ball_control.edge_recovery_active = 0U;
    g_ball_control.edge_recovery_timed_out = 0U;
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
        (period_s <= 0.0f)) {
        Ball_Control_Reset();
        return;
    }

    if (position->valid == 0U) {
        /* 只有 K230 明确报告从画面左右边缘离开，才执行向中心的恢复。 */
        if (position->edge_direction == 0) {
            Ball_Control_Reset();
            return;
        }
        if (g_ball_control.edge_recovery_timed_out != 0U) {
            Servo_SetTargetAngle(SERVO_ANGLE_NEUTRAL_DEG);
            Servo_ApplyHardware();
            return;
        }
        if (g_ball_control.edge_recovery_active == 0U) {
            g_ball_control.edge_recovery_active = 1U;
            g_ball_control.edge_recovery_start_ms = position->timestamp_ms;
        }
        if ((uint32_t)(position->timestamp_ms -
                       g_ball_control.edge_recovery_start_ms) >
                       BALL_CONTROL_EDGE_RECOVERY_TIMEOUT_MS) {
            g_ball_control.edge_recovery_active = 0U;
            g_ball_control.edge_recovery_timed_out = 1U;
            Servo_SetTargetAngle(SERVO_ANGLE_NEUTRAL_DEG);
            Servo_ApplyHardware();
            return;
        }
        /* 实测：低 PWM 使球向右（X 增大）。左侧离开时降低 PWM 拉回右侧；右侧反之。 */
        output = (float)position->edge_direction *
                 BALL_CONTROL_EDGE_RECOVERY_OFFSET_DEG;
        Servo_SetTargetAngle(SERVO_ANGLE_NEUTRAL_DEG + output);
        Servo_ApplyHardware();
        g_ball_control.has_last_sample = 0U;
        return;
    }

    g_ball_control.edge_recovery_active = 0U;
    g_ball_control.edge_recovery_timed_out = 0U;

    /* K230 帧率低于 200 Hz；同一帧不可在每个 5 ms 周期重复做 D 项。 */
    if ((g_ball_control.has_last_sample != 0U) &&
        (position->timestamp_ms == g_ball_control.last_timestamp_ms)) {
        return;
    }

    /* 实测方向：X 偏右时提高 PWM，右端降低后将球拉回左侧。 */
    error = position->x_mm - g_ball_control.target_x_mm;
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
    if (output > BALL_CONTROL_MAX_OFFSET_DEG) {
        output = BALL_CONTROL_MAX_OFFSET_DEG;
    } else if (output < -BALL_CONTROL_MAX_OFFSET_DEG) {
        output = -BALL_CONTROL_MAX_OFFSET_DEG;
    }
    /* 控制器输出是相对中位的偏移，舵机驱动接口使用 0~180 度绝对角度。 */
    Servo_SetTargetAngle(SERVO_ANGLE_NEUTRAL_DEG + output);
    Servo_ApplyHardware();
    g_ball_control.last_error_mm = error;
    g_ball_control.last_timestamp_ms = position->timestamp_ms;
    g_ball_control.has_last_sample = 1U;
}

const Ball_Control *Ball_Control_Get(void)
{
    return &g_ball_control;
}
