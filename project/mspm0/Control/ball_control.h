#ifndef H_BALL_CONTROL
#define H_BALL_CONTROL

#include "k230_link.h"

typedef struct {
    float kp;
    float kd;
    float target_x_mm;
    float last_error_mm;
    uint32_t last_timestamp_ms;
    uint32_t edge_recovery_start_ms;
    uint32_t stationary_start_ms;
    uint32_t kick_start_ms;
    uint32_t kick_cooldown_start_ms;
    uint32_t drive_start_ms;
    float stationary_reference_x_mm;
    float filtered_velocity_mm_s;
    float last_output_deg;
    float kick_offset_deg;
    uint8_t enabled;
    uint8_t has_last_sample;
    uint8_t has_stationary_reference;
    uint8_t edge_recovery_active;
    uint8_t edge_recovery_timed_out;
    uint8_t breakaway_active; /* 当前是否处于单次 KICK 脉冲。 */
    uint8_t kick_cooldown_active;
    uint8_t drive_active;
    uint8_t kick_fault_active;
    uint8_t kick_attempt_count;
    uint8_t kick_motion_confirm_frames;
    int8_t kick_direction;
} Ball_Control;

#define BALL_CONTROL_ENABLE_DEFAULT   (1U)
#define BALL_CONTROL_KP_DEG_PER_MM    (0.24f)  /* 0~180 度映射下，PWM 等效增益较上一版提高约 33% */
#define BALL_CONTROL_KD_DEG_S_PER_MM  (0.006f) /* 与比例项同比增强，抑制位置快速变化时的过冲 */
#define BALL_CONTROL_DEADBAND_MM      (1.0f)
#define BALL_CONTROL_MAX_OFFSET_DEG   (20.0f)  /* 正常闭环仅允许在 90 度中位两侧小幅动作 */
#define BALL_CONTROL_BREAKAWAY_ERROR_MM       (8.0f)  /* 进入静止确认的误差门限。 */
#define BALL_CONTROL_BREAKAWAY_EXIT_ERROR_MM  (5.0f)  /* 迟滞退出门限，避免门限附近反复切换。 */
#define BALL_CONTROL_BREAKAWAY_MOVEMENT_MM    (2.0f)  /* 超过此累计位移即视为已重新运动。 */
#define BALL_CONTROL_BREAKAWAY_WAIT_MS      (500U)    /* 连续近静止半秒后才叠加脱困。 */
#define BALL_CONTROL_VELOCITY_FILTER_ALPHA   (0.45f)  /* 约 16 Hz 输入下的首轮速度低通系数。 */
#define BALL_CONTROL_KICK_FIRST_OFFSET_DEG  (16.0f)
#define BALL_CONTROL_KICK_STEP_DEG           (2.0f)
#define BALL_CONTROL_KICK_MAX_OFFSET_DEG    (20.0f)
#define BALL_CONTROL_KICK_MIN_DURATION_MS  (250U)
#define BALL_CONTROL_KICK_MAX_DURATION_MS  (600U)
#define BALL_CONTROL_KICK_COOLDOWN_MS      (400U)
#define BALL_CONTROL_KICK_MAX_ATTEMPTS       (3U)
#define BALL_CONTROL_KICK_CONFIRM_FRAMES     (2U)
#define BALL_CONTROL_DRIVE_DURATION_MS      (400U)
#define BALL_CONTROL_DRIVE_RUN_OFFSET_DEG   (10.0f)
#define BALL_CONTROL_DRIVE_FULL_ERROR_MM    (20.0f)
#define BALL_CONTROL_DRIVE_EXIT_ERROR_MM     (8.0f)
#define BALL_CONTROL_DRIVE_SLEW_DEG_PER_FRAME (2.0f)
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
