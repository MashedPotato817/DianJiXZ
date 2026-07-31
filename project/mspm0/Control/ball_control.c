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
    if (value > maximum) {
        return maximum;
    }
    if (value < minimum) {
        return minimum;
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
    float command_angle_deg;

    output_deg = Ball_Control_Limit(output_deg, limit_deg);
    command_angle_deg = Ball_Control_Clamp(
        g_ball_control.trim_angle_deg + output_deg,
        SERVO_ANGLE_MIN_DEG,
        SERVO_ANGLE_MAX_DEG);
    Servo_SetTargetAngle(command_angle_deg);
    Servo_ApplyHardware();
    g_ball_control.last_command_angle_deg = command_angle_deg;
    g_ball_control.last_output_deg =
        command_angle_deg - g_ball_control.trim_angle_deg;
}

static void Ball_Control_UpdateTrim(float error, float velocity,
                                    float sample_period_s)
{
    float trim_step;

    if ((sample_period_s <= 0.0f) ||
        (Ball_Control_Abs(error) >
         BALL_CONTROL_TRIM_LEARN_ERROR_MM) ||
        (Ball_Control_Abs(velocity) >
         BALL_CONTROL_TRIM_LEARN_VELOCITY_MM_S) ||
        (g_ball_control.phase == BALL_CONTROL_PHASE_ACCEL) ||
        (g_ball_control.phase == BALL_CONTROL_PHASE_EDGE) ||
        (g_ball_control.phase == BALL_CONTROL_PHASE_HOLD) ||
        (g_ball_control.phase == BALL_CONTROL_PHASE_LOST) ||
        (g_ball_control.phase == BALL_CONTROL_PHASE_PASS) ||
        (g_ball_control.phase == BALL_CONTROL_PHASE_FAULT)) {
        return;
    }

    trim_step = BALL_CONTROL_TRIM_KI_DEG_PER_MM_S *
                error * sample_period_s;
    trim_step = Ball_Control_Limit(
        trim_step, BALL_CONTROL_TRIM_MAX_STEP_DEG);
    g_ball_control.trim_angle_deg = Ball_Control_Clamp(
        g_ball_control.trim_angle_deg + trim_step,
        BALL_CONTROL_TRIM_MIN_DEG,
        BALL_CONTROL_TRIM_MAX_DEG);
}

static void Ball_Control_ClearMotion(Ball_ControlPhase phase)
{
    g_ball_control.phase = phase;
    g_ball_control.phase_before_hold = phase;
    g_ball_control.phase_start_ms = g_ball_control.control_now_ms;
    g_ball_control.toward_velocity_mm_s = 0.0f;
    g_ball_control.stopping_distance_mm = 0.0f;
    g_ball_control.velocity_confirm_frames = 0U;
    g_ball_control.accel_level = 0U;
    g_ball_control.motion_direction = 0;
    g_ball_control.correction_start_direction = 0;
    g_ball_control.pass_target_abs_mm = 0.0f;
}

static void Ball_Control_StartAccel(float error)
{
    float start_abs;

    if (g_ball_control.correction_start_direction == 0) {
        g_ball_control.correction_start_direction =
            (error > 0.0f) ? 1 : -1;
        start_abs = Ball_Control_Abs(error);
        g_ball_control.pass_target_abs_mm = Ball_Control_Clamp(
            BALL_CONTROL_PASS_TARGET_RATIO * start_abs,
            BALL_CONTROL_PASS_TARGET_MIN_MM,
            BALL_CONTROL_PASS_TARGET_MAX_MM);
    }
    g_ball_control.phase = BALL_CONTROL_PHASE_ACCEL;
    g_ball_control.phase_before_hold = BALL_CONTROL_PHASE_ACCEL;
    g_ball_control.phase_start_ms = g_ball_control.control_now_ms;
    g_ball_control.velocity_confirm_frames = 0U;
    g_ball_control.accel_level = 0U;
    g_ball_control.motion_direction = (error > 0.0f) ? 1 : -1;
}

static float Ball_Control_GetAccelOutput(void)
{
    uint32_t elapsed_ms =
        g_ball_control.control_now_ms - g_ball_control.phase_start_ms;
    uint32_t level = elapsed_ms / BALL_CONTROL_ACCEL_STEP_MS;
    float output;

    if (level > 2U) {
        level = 2U;
    }
    g_ball_control.accel_level = (uint8_t)level;
    output = BALL_CONTROL_ACCEL_FIRST_DEG +
             BALL_CONTROL_ACCEL_STEP_DEG * (float)level;
    if (output > BALL_CONTROL_ACCEL_MAX_DEG) {
        output = BALL_CONTROL_ACCEL_MAX_DEG;
    }
    return (float)g_ball_control.motion_direction * output;
}

static void Ball_Control_StartPass(void)
{
    g_ball_control.phase = BALL_CONTROL_PHASE_PASS;
    g_ball_control.phase_before_hold = BALL_CONTROL_PHASE_PASS;
    g_ball_control.phase_start_ms = g_ball_control.control_now_ms;
    g_ball_control.velocity_confirm_frames = 0U;
    g_ball_control.accel_level = 0U;
    g_ball_control.motion_direction = 0;
}

static void Ball_Control_HandlePass(float error, float error_abs,
                                    uint8_t new_sample)
{
    float pass_velocity;
    float output;
    float output_limit;
    uint32_t pass_elapsed_ms;

    pass_velocity =
        -(float)g_ball_control.correction_start_direction *
        g_ball_control.filtered_velocity_mm_s;
    pass_elapsed_ms =
        g_ball_control.control_now_ms - g_ball_control.phase_start_ms;

    /*
     * 仍沿原方向穿过中心时，只做速度阻尼，不允许立即施加反向20度ACC。
     * 达到规划反侧距离后把阻尼上限从4度提高到6度，避免高速跑远。
     */
    output_limit =
        (error_abs >= g_ball_control.pass_target_abs_mm) ?
        BALL_CONTROL_PASS_DAMP_HARD_MAX_DEG :
        BALL_CONTROL_PASS_DAMP_SOFT_MAX_DEG;
    output =
        BALL_CONTROL_PASS_DAMP_KD_DEG_S_PER_MM *
        g_ball_control.filtered_velocity_mm_s;
    if ((g_ball_control.filtered_velocity_mm_s > 0.0f) &&
        (output < BALL_CONTROL_PASS_DAMP_MIN_DEG)) {
        output = BALL_CONTROL_PASS_DAMP_MIN_DEG;
    } else if ((g_ball_control.filtered_velocity_mm_s < 0.0f) &&
               (output > -BALL_CONTROL_PASS_DAMP_MIN_DEG)) {
        output = -BALL_CONTROL_PASS_DAMP_MIN_DEG;
    }
    output = Ball_Control_Limit(output, output_limit);

    /*
     * 至少保持150ms，覆盖当前85ms中位视觉间隔。确认原方向速度已经
     * 降到8mm/s以内后，才从反侧重新起动回中，形成一次小幅过零。
     */
    if ((new_sample != 0U) &&
        (pass_elapsed_ms >= BALL_CONTROL_PASS_MIN_HOLD_MS) &&
        (pass_velocity <= BALL_CONTROL_VELOCITY_STOP_MM_S)) {
        g_ball_control.correction_start_direction = 0;
        g_ball_control.pass_target_abs_mm = 0.0f;
        Ball_Control_StartAccel(error);
        Ball_Control_ApplyOutput(
            Ball_Control_GetAccelOutput(),
            BALL_CONTROL_ACCEL_MAX_DEG);
        return;
    }

    Ball_Control_ApplyOutput(
        output, BALL_CONTROL_PASS_DAMP_HARD_MAX_DEG);
}

static void Ball_Control_ResetSamples(void)
{
    g_ball_control.last_error_mm = 0.0f;
    g_ball_control.last_timestamp_ms = 0U;
    g_ball_control.last_valid_control_ms = 0U;
    g_ball_control.filtered_velocity_mm_s = 0.0f;
    g_ball_control.has_last_sample = 0U;
}

static void Ball_Control_EnterLost(void)
{
    Ball_Control_ResetSamples();
    Ball_Control_ClearMotion(BALL_CONTROL_PHASE_LOST);
    Ball_Control_ApplyOutput(0.0f, BALL_CONTROL_NORMAL_MAX_DEG);
}

static void Ball_Control_HandleEdge(const K230_BallPosition *position)
{
    float output;

    if (g_ball_control.phase != BALL_CONTROL_PHASE_EDGE) {
        g_ball_control.phase = BALL_CONTROL_PHASE_EDGE;
        g_ball_control.phase_before_hold = BALL_CONTROL_PHASE_EDGE;
        g_ball_control.edge_recovery_start_ms =
            g_ball_control.control_now_ms;
        g_ball_control.edge_recovery_timed_out = 0U;
    }
    if ((g_ball_control.control_now_ms -
         g_ball_control.edge_recovery_start_ms) >
        BALL_CONTROL_EDGE_RECOVERY_TIMEOUT_MS) {
        g_ball_control.edge_recovery_timed_out = 1U;
        Ball_Control_ApplyOutput(0.0f, BALL_CONTROL_NORMAL_MAX_DEG);
        return;
    }

    output = (float)position->edge_direction *
             BALL_CONTROL_EDGE_RECOVERY_OFFSET_DEG;
    Ball_Control_ApplyOutput(output,
                            BALL_CONTROL_EDGE_RECOVERY_OFFSET_DEG);
    Ball_Control_ResetSamples();
}

void Ball_Control_Init(void)
{
    g_ball_control.kp = BALL_CONTROL_KP_DEG_PER_MM;
    g_ball_control.kd = BALL_CONTROL_KD_DEG_S_PER_MM;
    g_ball_control.target_x_mm = 0.0f;
    g_ball_control.trim_angle_deg =
        BALL_CONTROL_TRIM_INITIAL_DEG;
    g_ball_control.last_command_angle_deg =
        BALL_CONTROL_TRIM_INITIAL_DEG;
    g_ball_control.control_now_ms = 0U;
    g_ball_control.edge_recovery_start_ms = 0U;
    g_ball_control.edge_recovery_timed_out = 0U;
    g_ball_control.enabled = BALL_CONTROL_ENABLE_DEFAULT;
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

void Ball_Control_Reset(void)
{
    Ball_Control_ResetSamples();
    g_ball_control.edge_recovery_start_ms = 0U;
    g_ball_control.edge_recovery_timed_out = 0U;
    Ball_Control_ClearMotion((g_ball_control.enabled != 0U) ?
                             BALL_CONTROL_PHASE_CAPTURE :
                             BALL_CONTROL_PHASE_OFF);
    Ball_Control_ApplyOutput(0.0f, BALL_CONTROL_NORMAL_MAX_DEG);
}

void Ball_Control_Step(const K230_BallPosition *position, float period_s)
{
    float error;
    float error_abs;
    float raw_velocity = 0.0f;
    float output;
    float sample_period_s;
    float new_sample_period_s = 0.0f;
    float toward_velocity;
    float stopping_distance;
    uint32_t sample_period_ms;
    uint32_t valid_age_ms;
    uint32_t accel_elapsed_ms;
    uint8_t new_sample;
    int8_t error_direction;

    if ((position == 0) || (period_s <= 0.0f)) {
        return;
    }

    g_ball_control.control_now_ms += Ball_Control_PeriodMs(period_s);
    if (g_ball_control.enabled == 0U) {
        if (g_ball_control.phase != BALL_CONTROL_PHASE_OFF) {
            Ball_Control_Reset();
        }
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
                       (BALL_CONTROL_LOST_HOLD_TOTAL_MS + 1U);
        if ((g_ball_control.has_last_sample != 0U) &&
            (valid_age_ms <= BALL_CONTROL_LOST_HOLD_TOTAL_MS)) {
            if (g_ball_control.phase != BALL_CONTROL_PHASE_HOLD) {
                g_ball_control.phase_before_hold = g_ball_control.phase;
                g_ball_control.phase = BALL_CONTROL_PHASE_HOLD;
            }
            /* 短时普通丢帧保持上一安全倾角，不重置运动阶段。 */
            return;
        }
        if (g_ball_control.phase != BALL_CONTROL_PHASE_LOST) {
            Ball_Control_EnterLost();
        }
        return;
    }

    g_ball_control.edge_recovery_timed_out = 0U;
    if (g_ball_control.phase == BALL_CONTROL_PHASE_HOLD) {
        g_ball_control.phase = g_ball_control.phase_before_hold;
    } else if ((g_ball_control.phase == BALL_CONTROL_PHASE_EDGE) ||
               (g_ball_control.phase == BALL_CONTROL_PHASE_LOST)) {
        Ball_Control_ClearMotion(BALL_CONTROL_PHASE_CAPTURE);
    }

    new_sample = ((g_ball_control.has_last_sample == 0U) ||
                  (position->timestamp_ms !=
                   g_ball_control.last_timestamp_ms)) ? 1U : 0U;
    if (new_sample != 0U) {
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
            new_sample_period_s = sample_period_s;
            raw_velocity =
                (error - g_ball_control.last_error_mm) / sample_period_s;
            g_ball_control.filtered_velocity_mm_s +=
                BALL_CONTROL_VELOCITY_FILTER_ALPHA *
                (raw_velocity -
                 g_ball_control.filtered_velocity_mm_s);
        }

        g_ball_control.last_error_mm = error;
        g_ball_control.last_timestamp_ms = position->timestamp_ms;
        g_ball_control.last_valid_control_ms =
            g_ball_control.control_now_ms;
        g_ball_control.has_last_sample = 1U;
    } else {
        error = g_ball_control.last_error_mm;
    }

    error_abs = Ball_Control_Abs(error);
    error_direction = (error > 0.0f) ? 1 : ((error < 0.0f) ? -1 : 0);
    toward_velocity =
        -(float)error_direction *
        g_ball_control.filtered_velocity_mm_s;
    g_ball_control.toward_velocity_mm_s = toward_velocity;

    if (toward_velocity > 0.0f) {
        stopping_distance =
            toward_velocity * toward_velocity /
            (2.0f * BALL_CONTROL_BRAKE_ACCEL_MM_S2);
    } else {
        stopping_distance = 0.0f;
    }
    g_ball_control.stopping_distance_mm = stopping_distance;
    if (new_sample != 0U) {
        Ball_Control_UpdateTrim(
            error,
            g_ball_control.filtered_velocity_mm_s,
            new_sample_period_s);
    }

    if (error_abs <= BALL_CONTROL_TARGET_TOLERANCE_MM) {
        /*
         * 中心容差内不再套用 RUN 的 12 度动摩擦下限。
         * 2~8 度阻尼随速度连续增长：低速不反推过度，高速仍能制动。
         */
        if (Ball_Control_Abs(
                g_ball_control.filtered_velocity_mm_s) <=
            BALL_CONTROL_VELOCITY_STOP_MM_S) {
            Ball_Control_ClearMotion(BALL_CONTROL_PHASE_CAPTURE);
            Ball_Control_ApplyOutput(
                0.0f, BALL_CONTROL_NORMAL_MAX_DEG);
        } else {
            g_ball_control.phase = BALL_CONTROL_PHASE_BRAKE;
            g_ball_control.phase_before_hold =
                BALL_CONTROL_PHASE_BRAKE;
            output =
                BALL_CONTROL_CAPTURE_DAMP_KD_DEG_S_PER_MM *
                g_ball_control.filtered_velocity_mm_s;
            if ((g_ball_control.filtered_velocity_mm_s > 0.0f) &&
                (output < BALL_CONTROL_CAPTURE_DAMP_MIN_DEG)) {
                output = BALL_CONTROL_CAPTURE_DAMP_MIN_DEG;
            } else if (
                (g_ball_control.filtered_velocity_mm_s < 0.0f) &&
                (output > -BALL_CONTROL_CAPTURE_DAMP_MIN_DEG)) {
                output = -BALL_CONTROL_CAPTURE_DAMP_MIN_DEG;
            }
            Ball_Control_ApplyOutput(
                output, BALL_CONTROL_CAPTURE_DAMP_MAX_DEG);
        }
        return;
    }

    if (g_ball_control.phase == BALL_CONTROL_PHASE_FAULT) {
        Ball_Control_ApplyOutput(0.0f, BALL_CONTROL_NORMAL_MAX_DEG);
        return;
    }

    /*
     * 记录本轮从哪一侧开始。首次越过中心并离开±2mm后进入PASS，
     * 防止误差符号翻转把仍在原方向运动误判成需要完整反向ACC。
     */
    if ((g_ball_control.phase != BALL_CONTROL_PHASE_PASS) &&
        (g_ball_control.correction_start_direction != 0) &&
        (error_direction ==
         -g_ball_control.correction_start_direction)) {
        Ball_Control_StartPass();
    }
    if (g_ball_control.phase == BALL_CONTROL_PHASE_PASS) {
        Ball_Control_HandlePass(error, error_abs, new_sample);
        return;
    }

    if (g_ball_control.phase == BALL_CONTROL_PHASE_ACCEL) {
        if (g_ball_control.motion_direction != error_direction) {
            Ball_Control_StartAccel(error);
        }

        if (new_sample != 0U) {
            if (toward_velocity >=
                BALL_CONTROL_VELOCITY_CONFIRM_MM_S) {
                if (g_ball_control.velocity_confirm_frames <
                    BALL_CONTROL_VELOCITY_CONFIRM_FRAMES) {
                    g_ball_control.velocity_confirm_frames++;
                }
            } else {
                g_ball_control.velocity_confirm_frames = 0U;
            }
        }

        if (g_ball_control.velocity_confirm_frames >=
            BALL_CONTROL_VELOCITY_CONFIRM_FRAMES) {
            g_ball_control.phase = BALL_CONTROL_PHASE_RUN;
            g_ball_control.phase_before_hold = BALL_CONTROL_PHASE_RUN;
            g_ball_control.phase_start_ms =
                g_ball_control.control_now_ms;
            g_ball_control.velocity_confirm_frames = 0U;
        } else {
            accel_elapsed_ms =
                g_ball_control.control_now_ms -
                g_ball_control.phase_start_ms;
            if (accel_elapsed_ms >=
                BALL_CONTROL_ACCEL_TIMEOUT_MS) {
                Ball_Control_ClearMotion(
                    BALL_CONTROL_PHASE_FAULT);
                Ball_Control_ApplyOutput(
                    0.0f, BALL_CONTROL_NORMAL_MAX_DEG);
                return;
            }
            Ball_Control_ApplyOutput(
                Ball_Control_GetAccelOutput(),
                BALL_CONTROL_ACCEL_MAX_DEG);
            return;
        }
    }

    /*
     * 非目标静止或反向运动：立即重新加速，不再等待500ms。
     * 只有已经向中心运动时才允许进入 RUN/BRAKE。
     */
    if ((error_abs > BALL_CONTROL_TARGET_TOLERANCE_MM) &&
        ((toward_velocity < BALL_CONTROL_VELOCITY_STALL_MM_S) ||
         (error_direction == 0))) {
        Ball_Control_StartAccel(error);
        Ball_Control_ApplyOutput(Ball_Control_GetAccelOutput(),
                                 BALL_CONTROL_ACCEL_MAX_DEG);
        return;
    }

    if ((toward_velocity > 0.0f) &&
        (error_abs <= stopping_distance +
                      BALL_CONTROL_BRAKE_MARGIN_MM)) {
        g_ball_control.phase = BALL_CONTROL_PHASE_BRAKE;
    } else {
        g_ball_control.phase = BALL_CONTROL_PHASE_RUN;
    }
    g_ball_control.phase_before_hold = g_ball_control.phase;

    output = g_ball_control.kp * error +
             g_ball_control.kd *
             g_ball_control.filtered_velocity_mm_s;

    if (g_ball_control.phase == BALL_CONTROL_PHASE_RUN) {
        /*
         * RUN_MIN 为 0：不再用固定 12 度维持推动。
         * 这里只阻止尚未到达制动切换面时提前反向，允许输出自然降到 0；
         * 若球在中心外重新停住，前面的 STALL 分支会再次进入 ACC。
         */
        if ((error_direction > 0) &&
            (output < BALL_CONTROL_RUN_MIN_DEG)) {
            output = BALL_CONTROL_RUN_MIN_DEG;
        } else if ((error_direction < 0) &&
                   (output > -BALL_CONTROL_RUN_MIN_DEG)) {
            output = -BALL_CONTROL_RUN_MIN_DEG;
        }
    } else {
        /* 到达制动切换面后，确保杆角确实反向，不只回到中位。 */
        if (error_direction > 0) {
            if (output > -BALL_CONTROL_BRAKE_MIN_DEG) {
                output = -BALL_CONTROL_BRAKE_MIN_DEG;
            }
        } else if (error_direction < 0) {
            if (output < BALL_CONTROL_BRAKE_MIN_DEG) {
                output = BALL_CONTROL_BRAKE_MIN_DEG;
            }
        }
    }

    Ball_Control_ApplyOutput(output, BALL_CONTROL_NORMAL_MAX_DEG);
}

const Ball_Control *Ball_Control_Get(void)
{
    return &g_ball_control;
}
