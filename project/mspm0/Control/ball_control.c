#include "ball_control.h"
#include "servo.h"

static Ball_Control g_ball_control;

static float Ball_Control_Abs(float value)
{
    return (value < 0.0f) ? -value : value;
}

static void Ball_Control_ResetBreakaway(void)
{
    g_ball_control.stationary_start_ms = 0U;
    g_ball_control.kick_start_ms = 0U;
    g_ball_control.kick_cooldown_start_ms = 0U;
    g_ball_control.drive_start_ms = 0U;
    g_ball_control.stationary_reference_x_mm = 0.0f;
    g_ball_control.kick_offset_deg = 0.0f;
    g_ball_control.has_stationary_reference = 0U;
    g_ball_control.breakaway_active = 0U;
    g_ball_control.kick_cooldown_active = 0U;
    g_ball_control.drive_active = 0U;
    g_ball_control.kick_fault_active = 0U;
    g_ball_control.kick_attempt_count = 0U;
    g_ball_control.kick_motion_confirm_frames = 0U;
    g_ball_control.kick_direction = 0;
}

static void Ball_Control_StartKick(float error, uint32_t timestamp_ms)
{
    float kick_offset;

    if (g_ball_control.kick_attempt_count < BALL_CONTROL_KICK_MAX_ATTEMPTS) {
        g_ball_control.kick_attempt_count++;
    }
    kick_offset = BALL_CONTROL_KICK_FIRST_OFFSET_DEG +
                  BALL_CONTROL_KICK_STEP_DEG *
                  (float)(g_ball_control.kick_attempt_count - 1U);
    if (kick_offset > BALL_CONTROL_KICK_MAX_OFFSET_DEG) {
        kick_offset = BALL_CONTROL_KICK_MAX_OFFSET_DEG;
    }

    g_ball_control.kick_offset_deg = kick_offset;
    g_ball_control.kick_start_ms = timestamp_ms;
    g_ball_control.kick_direction = (error > 0.0f) ? 1 : -1;
    g_ball_control.kick_motion_confirm_frames = 0U;
    g_ball_control.breakaway_active = 1U;
    g_ball_control.kick_cooldown_active = 0U;
    g_ball_control.drive_active = 0U;
}

static void Ball_Control_EndKickFailed(uint32_t timestamp_ms)
{
    g_ball_control.breakaway_active = 0U;
    g_ball_control.kick_motion_confirm_frames = 0U;
    g_ball_control.has_stationary_reference = 0U;
    g_ball_control.kick_direction = 0;

    if (g_ball_control.kick_attempt_count >= BALL_CONTROL_KICK_MAX_ATTEMPTS) {
        g_ball_control.kick_fault_active = 1U;
        g_ball_control.kick_cooldown_active = 0U;
    } else {
        g_ball_control.kick_cooldown_active = 1U;
        g_ball_control.kick_cooldown_start_ms = timestamp_ms;
    }
}

static void Ball_Control_StartDrive(uint32_t timestamp_ms)
{
    g_ball_control.breakaway_active = 0U;
    g_ball_control.kick_motion_confirm_frames = 0U;
    g_ball_control.drive_active = 1U;
    g_ball_control.drive_start_ms = timestamp_ms;
    g_ball_control.kick_cooldown_active = 0U;
    g_ball_control.kick_attempt_count = 0U;
    g_ball_control.has_stationary_reference = 0U;
}

static void Ball_Control_EndDrive(void)
{
    g_ball_control.drive_active = 0U;
    g_ball_control.drive_start_ms = 0U;
    g_ball_control.kick_direction = 0;
    g_ball_control.has_stationary_reference = 0U;
    g_ball_control.stationary_start_ms = 0U;
    g_ball_control.stationary_reference_x_mm = 0.0f;
}

void Ball_Control_Init(void)
{
    g_ball_control.kp = BALL_CONTROL_KP_DEG_PER_MM;
    g_ball_control.kd = BALL_CONTROL_KD_DEG_S_PER_MM;
    g_ball_control.target_x_mm = 0.0f;
    g_ball_control.last_error_mm = 0.0f;
    g_ball_control.last_timestamp_ms = 0U;
    g_ball_control.edge_recovery_start_ms = 0U;
    g_ball_control.stationary_start_ms = 0U;
    g_ball_control.kick_start_ms = 0U;
    g_ball_control.kick_cooldown_start_ms = 0U;
    g_ball_control.drive_start_ms = 0U;
    g_ball_control.stationary_reference_x_mm = 0.0f;
    g_ball_control.filtered_velocity_mm_s = 0.0f;
    g_ball_control.last_output_deg = 0.0f;
    g_ball_control.kick_offset_deg = 0.0f;
    g_ball_control.enabled = BALL_CONTROL_ENABLE_DEFAULT;
    g_ball_control.has_last_sample = 0U;
    g_ball_control.has_stationary_reference = 0U;
    g_ball_control.edge_recovery_active = 0U;
    g_ball_control.edge_recovery_timed_out = 0U;
    g_ball_control.breakaway_active = 0U;
    g_ball_control.kick_cooldown_active = 0U;
    g_ball_control.drive_active = 0U;
    g_ball_control.kick_fault_active = 0U;
    g_ball_control.kick_attempt_count = 0U;
    g_ball_control.kick_motion_confirm_frames = 0U;
    g_ball_control.kick_direction = 0;
    Servo_SetTargetAngle(SERVO_ANGLE_NEUTRAL_DEG);
    Servo_ApplyHardware();
}

void Ball_Control_SetTarget(float target_x_mm)
{
    g_ball_control.target_x_mm = target_x_mm;
    g_ball_control.last_error_mm = 0.0f;
    g_ball_control.filtered_velocity_mm_s = 0.0f;
    g_ball_control.last_output_deg = 0.0f;
    g_ball_control.has_last_sample = 0U;
    Ball_Control_ResetBreakaway();
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
    g_ball_control.filtered_velocity_mm_s = 0.0f;
    g_ball_control.last_output_deg = 0.0f;
    g_ball_control.has_last_sample = 0U;
    g_ball_control.edge_recovery_start_ms = 0U;
    g_ball_control.edge_recovery_active = 0U;
    g_ball_control.edge_recovery_timed_out = 0U;
    Ball_Control_ResetBreakaway();
    Servo_SetTargetAngle(SERVO_ANGLE_NEUTRAL_DEG);
    Servo_ApplyHardware();
}

void Ball_Control_Step(const K230_BallPosition *position, float period_s)
{
    float error;
    float error_abs;
    float raw_velocity;
    float output;
    float sample_period_s;
    float movement;
    float toward_movement;
    float away_movement;
    float run_floor;
    float run_weight;
    uint32_t kick_elapsed_ms;

    if ((g_ball_control.enabled == 0U) || (position == 0) ||
        (period_s <= 0.0f)) {
        Ball_Control_Reset();
        return;
    }

    if (position->valid == 0U) {
        Ball_Control_ResetBreakaway();
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
        g_ball_control.filtered_velocity_mm_s = 0.0f;
        g_ball_control.last_output_deg = output;
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
    error_abs = Ball_Control_Abs(error);

    sample_period_s = period_s;
    if (g_ball_control.has_last_sample != 0U) {
        sample_period_s = (float)(position->timestamp_ms -
                          g_ball_control.last_timestamp_ms) / 1000.0f;
        if ((sample_period_s < period_s) || (sample_period_s > 0.1f)) {
            sample_period_s = period_s;
        }
    }

    raw_velocity = (g_ball_control.has_last_sample != 0U) ?
                   (error - g_ball_control.last_error_mm) /
                   sample_period_s : 0.0f;
    if (g_ball_control.has_last_sample == 0U) {
        g_ball_control.filtered_velocity_mm_s = 0.0f;
    } else {
        g_ball_control.filtered_velocity_mm_s +=
            BALL_CONTROL_VELOCITY_FILTER_ALPHA *
            (raw_velocity - g_ball_control.filtered_velocity_mm_s);
    }

    /*
     * 混合控制状态：
     * PD 静止确认 -> 分级 KICK -> DRIVE 平滑维持 -> 滤波 PD 捕获。
     * KICK 只根据新的有效视觉帧推进，任何视觉失效都由前面的安全分支复位。
     */
    if (error_abs <= BALL_CONTROL_BREAKAWAY_EXIT_ERROR_MM) {
        Ball_Control_ResetBreakaway();
    } else if (g_ball_control.kick_fault_active != 0U) {
        /* 三次 KICK 均未可靠起动，保持中位，等待人工移动或复位。 */
    } else if (g_ball_control.breakaway_active != 0U) {
        movement = position->x_mm -
                   g_ball_control.stationary_reference_x_mm;
        toward_movement = -(float)g_ball_control.kick_direction * movement;
        away_movement = -toward_movement;
        kick_elapsed_ms = (uint32_t)(position->timestamp_ms -
                                     g_ball_control.kick_start_ms);

        if (((g_ball_control.kick_direction > 0) && (error <= 0.0f)) ||
            ((g_ball_control.kick_direction < 0) && (error >= 0.0f))) {
            Ball_Control_ResetBreakaway();
        } else if (away_movement >= BALL_CONTROL_BREAKAWAY_MOVEMENT_MM) {
            Ball_Control_EndKickFailed(position->timestamp_ms);
        } else {
            if (toward_movement >= BALL_CONTROL_BREAKAWAY_MOVEMENT_MM) {
                if (g_ball_control.kick_motion_confirm_frames <
                    BALL_CONTROL_KICK_CONFIRM_FRAMES) {
                    g_ball_control.kick_motion_confirm_frames++;
                }
            } else {
                g_ball_control.kick_motion_confirm_frames = 0U;
            }

            if ((kick_elapsed_ms >= BALL_CONTROL_KICK_MIN_DURATION_MS) &&
                (g_ball_control.kick_motion_confirm_frames >=
                 BALL_CONTROL_KICK_CONFIRM_FRAMES)) {
                Ball_Control_StartDrive(position->timestamp_ms);
            } else if (kick_elapsed_ms >= BALL_CONTROL_KICK_MAX_DURATION_MS) {
                Ball_Control_EndKickFailed(position->timestamp_ms);
            }
        }
    } else if (g_ball_control.drive_active != 0U) {
        if ((error_abs <= BALL_CONTROL_DRIVE_EXIT_ERROR_MM) ||
            ((g_ball_control.kick_direction > 0) && (error <= 0.0f)) ||
            ((g_ball_control.kick_direction < 0) && (error >= 0.0f)) ||
            ((uint32_t)(position->timestamp_ms -
                         g_ball_control.drive_start_ms) >=
             BALL_CONTROL_DRIVE_DURATION_MS)) {
            Ball_Control_EndDrive();
        }
    } else if (g_ball_control.kick_cooldown_active != 0U) {
        if ((uint32_t)(position->timestamp_ms -
                        g_ball_control.kick_cooldown_start_ms) >=
            BALL_CONTROL_KICK_COOLDOWN_MS) {
            g_ball_control.kick_cooldown_active = 0U;
            g_ball_control.has_stationary_reference = 0U;
        }
    } else if (error_abs >= BALL_CONTROL_BREAKAWAY_ERROR_MM) {
        if (g_ball_control.has_stationary_reference == 0U) {
            g_ball_control.stationary_reference_x_mm = position->x_mm;
            g_ball_control.stationary_start_ms = position->timestamp_ms;
            g_ball_control.has_stationary_reference = 1U;
        } else if (Ball_Control_Abs(position->x_mm -
                                    g_ball_control.stationary_reference_x_mm) >=
                   BALL_CONTROL_BREAKAWAY_MOVEMENT_MM) {
            g_ball_control.stationary_reference_x_mm = position->x_mm;
            g_ball_control.stationary_start_ms = position->timestamp_ms;
        } else if ((uint32_t)(position->timestamp_ms -
                              g_ball_control.stationary_start_ms) >=
                   BALL_CONTROL_BREAKAWAY_WAIT_MS) {
            Ball_Control_StartKick(error, position->timestamp_ms);
        }
    }

    output = g_ball_control.kp * error +
             g_ball_control.kd * g_ball_control.filtered_velocity_mm_s;
    if (g_ball_control.kick_fault_active != 0U) {
        output = 0.0f;
    }
    if (g_ball_control.breakaway_active != 0U) {
        if (g_ball_control.kick_direction > 0) {
            if (output < g_ball_control.kick_offset_deg) {
                output = g_ball_control.kick_offset_deg;
            }
        } else if (g_ball_control.kick_direction < 0) {
            if (output > -g_ball_control.kick_offset_deg) {
                output = -g_ball_control.kick_offset_deg;
            }
        }
    } else if (g_ball_control.drive_active != 0U) {
        if (error_abs >= BALL_CONTROL_DRIVE_FULL_ERROR_MM) {
            run_weight = 1.0f;
        } else if (error_abs <= BALL_CONTROL_DRIVE_EXIT_ERROR_MM) {
            run_weight = 0.0f;
        } else {
            run_weight = (error_abs - BALL_CONTROL_DRIVE_EXIT_ERROR_MM) /
                         (BALL_CONTROL_DRIVE_FULL_ERROR_MM -
                          BALL_CONTROL_DRIVE_EXIT_ERROR_MM);
        }
        run_floor = BALL_CONTROL_DRIVE_RUN_OFFSET_DEG * run_weight;

        /* PD 已要求反向制动时不施加维持下限；同方向输出才补足滚动维持力。 */
        if ((error > 0.0f) && (output >= 0.0f) && (output < run_floor)) {
            output = run_floor;
        } else if ((error < 0.0f) && (output <= 0.0f) &&
                   (output > -run_floor)) {
            output = -run_floor;
        }

        /* 同方向回落时逐帧减小，若 PD 要求反向制动则立即执行。 */
        if ((output * g_ball_control.last_output_deg) >= 0.0f) {
            if (output > g_ball_control.last_output_deg +
                         BALL_CONTROL_DRIVE_SLEW_DEG_PER_FRAME) {
                output = g_ball_control.last_output_deg +
                         BALL_CONTROL_DRIVE_SLEW_DEG_PER_FRAME;
            } else if (output < g_ball_control.last_output_deg -
                                BALL_CONTROL_DRIVE_SLEW_DEG_PER_FRAME) {
                output = g_ball_control.last_output_deg -
                         BALL_CONTROL_DRIVE_SLEW_DEG_PER_FRAME;
            }
        }
    }
    if (output > BALL_CONTROL_MAX_OFFSET_DEG) {
        output = BALL_CONTROL_MAX_OFFSET_DEG;
    } else if (output < -BALL_CONTROL_MAX_OFFSET_DEG) {
        output = -BALL_CONTROL_MAX_OFFSET_DEG;
    }
    /* 控制器输出是相对中位的偏移，舵机驱动接口使用 0~180 度绝对角度。 */
    Servo_SetTargetAngle(SERVO_ANGLE_NEUTRAL_DEG + output);
    Servo_ApplyHardware();
    g_ball_control.last_output_deg = output;
    g_ball_control.last_error_mm = error;
    g_ball_control.last_timestamp_ms = position->timestamp_ms;
    g_ball_control.has_last_sample = 1U;
}

const Ball_Control *Ball_Control_Get(void)
{
    return &g_ball_control;
}
