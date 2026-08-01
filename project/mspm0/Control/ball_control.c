#include "ball_control.h"
#include "servo.h"

static Ball_Control g_ball_control;

static float Ball_Control_Abs(float value)
{
    return (value < 0.0f) ? -value : value;
}

static float Ball_Control_Limit(float value, float limit)
{
    if (value > limit) {
        return limit;
    }
    if (value < -limit) {
        return -limit;
    }
    return value;
}

static float Ball_Control_Clamp(float value, float minimum, float maximum)
{
    if (value < minimum) {
        return minimum;
    }
    if (value > maximum) {
        return maximum;
    }
    return value;
}

static uint32_t Ball_Control_PeriodMs(float period_s)
{
    uint32_t period_ms = (uint32_t)(period_s * 1000.0f + 0.5f);

    return (period_ms == 0U) ? 1U : period_ms;
}

static void Ball_Control_ApplyOutput(float output_deg, float limit_deg)
{
    float command_deg;

    output_deg = Ball_Control_Limit(output_deg, limit_deg);
    command_deg = Ball_Control_Clamp(
        g_ball_control.trim_angle_deg + output_deg,
        SERVO_ANGLE_MIN_DEG, SERVO_ANGLE_MAX_DEG);
    g_ball_control.last_command_angle_deg = command_deg;
    g_ball_control.last_output_deg =
        command_deg - g_ball_control.trim_angle_deg;
    Servo_SetTargetAngle(command_deg);
    Servo_ApplyHardware();
}

static void Ball_Control_ResetSamples(void)
{
    g_ball_control.last_error_mm = 0.0f;
    g_ball_control.filtered_velocity_mm_s = 0.0f;
    g_ball_control.last_timestamp_ms = 0U;
    g_ball_control.last_valid_control_ms = 0U;
    g_ball_control.has_last_sample = 0U;
}

static void Ball_Control_HandleEdge(const K230_BallPosition *position)
{
    /* 本次离场恢复已超时后保持平衡，直到重新收到有效视觉位置。 */
    if (g_ball_control.phase == BALL_CONTROL_PHASE_LOST) {
        return;
    }
    if (g_ball_control.phase != BALL_CONTROL_PHASE_EDGE) {
        g_ball_control.phase = BALL_CONTROL_PHASE_EDGE;
        g_ball_control.edge_start_ms = g_ball_control.control_now_ms;
        g_ball_control.edge_direction =
            (position->edge_direction > 0) ? 1 : -1;
        /*
         * 只在进入 EDGE 时下发一次，随后锁存首次离场方向；避免视觉边缘
         * 符号抖动导致舵机在两个大角度之间按 5ms 周期来回切换。
         */
        Ball_Control_ApplyOutput(
            (g_ball_control.edge_direction > 0) ?
            BALL_CONTROL_EDGE_OUTPUT_DEG :
            -BALL_CONTROL_EDGE_OUTPUT_DEG,
            BALL_CONTROL_EDGE_OUTPUT_DEG);
    }
    if ((uint32_t)(g_ball_control.control_now_ms -
                   g_ball_control.edge_start_ms) >=
        BALL_CONTROL_EDGE_TIMEOUT_MS) {
        g_ball_control.phase = BALL_CONTROL_PHASE_LOST;
        Ball_Control_ApplyOutput(0.0f, BALL_CONTROL_MAX_OUTPUT_DEG);
        Ball_Control_ResetSamples();
        return;
    }
    /* +X 离场时保持增大脉宽使球向 -X 返回，-X 离场时相反。 */
    Ball_Control_ResetSamples();
}

void Ball_Control_Init(void)
{
    g_ball_control.kp = BALL_CONTROL_KP_DEG_PER_MM;
    g_ball_control.kd = BALL_CONTROL_KD_DEG_S_PER_MM;
    g_ball_control.target_x_mm = 0.0f;
    g_ball_control.trim_angle_deg = BALL_CONTROL_TRIM_INITIAL_DEG;
    g_ball_control.last_command_angle_deg = BALL_CONTROL_TRIM_INITIAL_DEG;
    g_ball_control.last_output_deg = 0.0f;
    g_ball_control.control_now_ms = 0U;
    g_ball_control.last_apply_ms = 0U - BALL_CONTROL_UPDATE_MS;
    g_ball_control.edge_start_ms = 0U;
    g_ball_control.enabled = BALL_CONTROL_ENABLE_DEFAULT;
    g_ball_control.servo_hold = 0U;
    g_ball_control.edge_direction = 0;
    Ball_Control_Reset();
}

void Ball_Control_SetTarget(float target_x_mm)
{
    g_ball_control.target_x_mm = target_x_mm;
    Ball_Control_Reset();
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

void Ball_Control_SetServoHold(uint8_t hold)
{
    g_ball_control.servo_hold = (hold != 0U) ? 1U : 0U;
}

void Ball_Control_SetTrimAngle(float angle_deg)
{
    g_ball_control.trim_angle_deg = Ball_Control_Clamp(
        angle_deg, SERVO_ANGLE_MIN_DEG, SERVO_ANGLE_MAX_DEG);
    Ball_Control_Reset();
}

void Ball_Control_SetGains(float kp, float kd)
{
    if (kp > 0.0f) {
        g_ball_control.kp = kp;
    }
    if (kd >= 0.0f) {
        g_ball_control.kd = kd;
    }
}

void Ball_Control_Reset(void)
{
    Ball_Control_ResetSamples();
    g_ball_control.edge_start_ms = 0U;
    g_ball_control.edge_direction = 0;
    g_ball_control.phase = (g_ball_control.enabled != 0U) ?
                           BALL_CONTROL_PHASE_TRACK :
                           BALL_CONTROL_PHASE_OFF;
    g_ball_control.last_apply_ms =
        g_ball_control.control_now_ms - BALL_CONTROL_UPDATE_MS;
    Ball_Control_ApplyOutput(0.0f, BALL_CONTROL_MAX_OUTPUT_DEG);
}

void Ball_Control_Step(const K230_BallPosition *position, float period_s)
{
    float error;
    float raw_velocity;
    float sample_period_s;
    float output_deg;
    uint32_t sample_period_ms;
    uint32_t valid_age_ms;
    uint8_t new_sample;

    if ((position == 0) || (period_s <= 0.0f)) {
        return;
    }
    g_ball_control.control_now_ms += Ball_Control_PeriodMs(period_s);

    if (g_ball_control.servo_hold != 0U) {
        g_ball_control.phase = BALL_CONTROL_PHASE_HOLD;
        return;
    }
    if (g_ball_control.enabled == 0U) {
        g_ball_control.phase = BALL_CONTROL_PHASE_OFF;
        return;
    }

    if (position->valid == 0U) {
        if (position->edge_direction != 0) {
            Ball_Control_HandleEdge(position);
            return;
        }
        valid_age_ms = (g_ball_control.has_last_sample != 0U) ?
                       (g_ball_control.control_now_ms -
                        g_ball_control.last_valid_control_ms) :
                       (BALL_CONTROL_LOST_HOLD_MS + 1U);
        if ((g_ball_control.has_last_sample != 0U) &&
            (valid_age_ms <= BALL_CONTROL_LOST_HOLD_MS)) {
            g_ball_control.phase = BALL_CONTROL_PHASE_HOLD;
            return;
        }
        if (g_ball_control.phase != BALL_CONTROL_PHASE_LOST) {
            g_ball_control.phase = BALL_CONTROL_PHASE_LOST;
            Ball_Control_ApplyOutput(0.0f,
                                     BALL_CONTROL_MAX_OUTPUT_DEG);
            Ball_Control_ResetSamples();
        }
        return;
    }

    new_sample = ((g_ball_control.has_last_sample == 0U) ||
                  (position->timestamp_ms !=
                   g_ball_control.last_timestamp_ms)) ? 1U : 0U;
    if (new_sample == 0U) {
        return;
    }

    error = position->x_mm - g_ball_control.target_x_mm;
    sample_period_ms = 0U;
    if (g_ball_control.has_last_sample != 0U) {
        sample_period_ms = position->timestamp_ms -
                           g_ball_control.last_timestamp_ms;
    }
    if ((g_ball_control.has_last_sample == 0U) ||
        (sample_period_ms == 0U) ||
        (sample_period_ms > BALL_CONTROL_MAX_SAMPLE_PERIOD_MS)) {
        g_ball_control.filtered_velocity_mm_s = 0.0f;
    } else {
        sample_period_s = (float)sample_period_ms / 1000.0f;
        raw_velocity =
            (error - g_ball_control.last_error_mm) / sample_period_s;
        g_ball_control.filtered_velocity_mm_s +=
            BALL_CONTROL_VELOCITY_FILTER_ALPHA *
            (raw_velocity - g_ball_control.filtered_velocity_mm_s);
    }
    g_ball_control.last_error_mm = error;
    g_ball_control.last_timestamp_ms = position->timestamp_ms;
    g_ball_control.last_valid_control_ms = g_ball_control.control_now_ms;
    g_ball_control.has_last_sample = 1U;
    g_ball_control.phase = BALL_CONTROL_PHASE_TRACK;

    if ((uint32_t)(g_ball_control.control_now_ms -
                   g_ball_control.last_apply_ms) <
        BALL_CONTROL_UPDATE_MS) {
        return;
    }
    g_ball_control.last_apply_ms = g_ball_control.control_now_ms;

    output_deg = g_ball_control.kp * error +
                 g_ball_control.kd *
                 g_ball_control.filtered_velocity_mm_s;
    if ((Ball_Control_Abs(error) > BALL_CONTROL_STABLE_BAND_MM) &&
        (Ball_Control_Abs(g_ball_control.filtered_velocity_mm_s) <=
         BALL_CONTROL_STALL_VELOCITY_MM_S)) {
        if ((error > 0.0f) && (output_deg < BALL_CONTROL_MIN_MOVE_DEG)) {
            output_deg = BALL_CONTROL_MIN_MOVE_DEG;
        } else if ((error < 0.0f) &&
                   (output_deg > -BALL_CONTROL_MIN_MOVE_DEG)) {
            output_deg = -BALL_CONTROL_MIN_MOVE_DEG;
        }
    }
    Ball_Control_ApplyOutput(output_deg, BALL_CONTROL_MAX_OUTPUT_DEG);
}

const Ball_Control *Ball_Control_Get(void)
{
    return &g_ball_control;
}
