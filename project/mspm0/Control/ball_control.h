#ifndef H_BALL_CONTROL
#define H_BALL_CONTROL

#include "k230_link.h"

typedef enum {
    BALL_CONTROL_PHASE_OFF = 0,
    BALL_CONTROL_PHASE_CAPTURE = 1,
    BALL_CONTROL_PHASE_ACCEL = 2,
    BALL_CONTROL_PHASE_RUN = 3,
    BALL_CONTROL_PHASE_BRAKE = 4,
    BALL_CONTROL_PHASE_EDGE = 5,
    BALL_CONTROL_PHASE_FAULT = 6,
    BALL_CONTROL_PHASE_HOLD = 7,
    BALL_CONTROL_PHASE_LOST = 8
} Ball_ControlPhase;

typedef struct {
    float kp;
    float kd;
    float target_x_mm;
    float last_error_mm;
    float filtered_velocity_mm_s;
    float toward_velocity_mm_s;
    float stopping_distance_mm;
    float trim_angle_deg;
    float last_command_angle_deg;
    float last_output_deg;
    uint32_t last_timestamp_ms;
    uint32_t last_valid_control_ms;
    uint32_t control_now_ms;
    uint32_t phase_start_ms;
    uint32_t edge_recovery_start_ms;
    uint8_t enabled;
    uint8_t has_last_sample;
    uint8_t edge_recovery_timed_out;
    uint8_t velocity_confirm_frames;
    uint8_t accel_level;
    int8_t motion_direction;
    Ball_ControlPhase phase;
    Ball_ControlPhase phase_before_hold;
} Ball_Control;

#define BALL_CONTROL_ENABLE_DEFAULT   (1U)
#define BALL_CONTROL_KP_DEG_PER_MM          (0.24f)
#define BALL_CONTROL_KD_DEG_S_PER_MM        (0.040f)
#define BALL_CONTROL_TARGET_TOLERANCE_MM    (2.0f)
#define BALL_CONTROL_VELOCITY_FILTER_ALPHA  (0.45f)
#define BALL_CONTROL_VELOCITY_CONFIRM_MM_S (12.0f)
#define BALL_CONTROL_VELOCITY_STALL_MM_S    (8.0f)
#define BALL_CONTROL_VELOCITY_STOP_MM_S     (8.0f)
#define BALL_CONTROL_VELOCITY_CONFIRM_FRAMES (2U)
#define BALL_CONTROL_MAX_SAMPLE_PERIOD_MS  (250U)

/* 非目标静止时连续施力，不在档位之间回中。 */
#define BALL_CONTROL_ACCEL_FIRST_DEG        (20.0f)
#define BALL_CONTROL_ACCEL_STEP_DEG          (4.0f)
#define BALL_CONTROL_ACCEL_MAX_DEG          (28.0f)
#define BALL_CONTROL_ACCEL_STEP_MS          (300U)
#define BALL_CONTROL_ACCEL_TIMEOUT_MS      (1200U)

/* 已滚动后保持动摩擦，再按估算制动距离反向制动。 */
#define BALL_CONTROL_RUN_MIN_DEG            (12.0f)
#define BALL_CONTROL_BRAKE_MIN_DEG           (6.0f)
#define BALL_CONTROL_CAPTURE_DAMP_MIN_DEG    (2.0f)
#define BALL_CONTROL_CAPTURE_DAMP_MAX_DEG    (4.0f)
#define BALL_CONTROL_BRAKE_ACCEL_MM_S2     (250.0f)
#define BALL_CONTROL_BRAKE_MARGIN_MM         (2.0f)
#define BALL_CONTROL_NORMAL_MAX_DEG         (20.0f)

/*
 * 90 度只是上电种子，不是假定不变的物理平衡点。
 * trim 仅在中心附近低速时慢速学习，并预留至少 10 度快速纠偏余量。
 */
#define BALL_CONTROL_TRIM_INITIAL_DEG        (90.0f)
#define BALL_CONTROL_TRIM_MIN_DEG            (15.0f)
#define BALL_CONTROL_TRIM_MAX_DEG           (165.0f)
#define BALL_CONTROL_TRIM_KI_DEG_PER_MM_S     (0.18f)
#define BALL_CONTROL_TRIM_LEARN_ERROR_MM      (6.0f)
#define BALL_CONTROL_TRIM_LEARN_VELOCITY_MM_S (12.0f)
#define BALL_CONTROL_TRIM_MAX_STEP_DEG        (0.15f)

/* 链路自身100ms超时后再容忍到总计180ms，期间保持上一安全输出。 */
#define BALL_CONTROL_LOST_HOLD_TOTAL_MS    (180U)
#define BALL_CONTROL_EDGE_RECOVERY_OFFSET_DEG (75.0f) /* 90+/-80 度已确认安全，预留 5 度余量 */
#define BALL_CONTROL_EDGE_RECOVERY_TIMEOUT_MS (1000U)

void Ball_Control_Init(void);
void Ball_Control_SetTarget(float target_x_mm);
void Ball_Control_SetEnabled(uint8_t enabled);
uint8_t Ball_Control_IsEnabled(void);
void Ball_Control_Reset(void);
void Ball_Control_Step(const K230_BallPosition *position, float period_s);
const Ball_Control *Ball_Control_Get(void);

#endif
