#ifndef H_BALL_CONTROL
#define H_BALL_CONTROL

#include "k230_link.h"

typedef enum {
    BALL_CONTROL_PHASE_OFF = 0,
    BALL_CONTROL_PHASE_TRACK = 1,
    BALL_CONTROL_PHASE_EDGE = 2,
    BALL_CONTROL_PHASE_HOLD = 3,
    BALL_CONTROL_PHASE_LOST = 4
} Ball_ControlPhase;

typedef struct {
    float kp;
    float kd;
    float target_x_mm;
    float last_error_mm;
    float filtered_velocity_mm_s;
    float trim_angle_deg;
    float last_command_angle_deg;
    float last_output_deg;
    uint32_t last_timestamp_ms;
    uint32_t last_valid_control_ms;
    uint32_t control_now_ms;
    uint32_t last_apply_ms;
    uint32_t edge_start_ms;
    uint8_t enabled;
    uint8_t has_last_sample;
    uint8_t servo_hold;
    Ball_ControlPhase phase;
} Ball_Control;

/*
 * 单一位置 PD：所有目标共用一套参数，每 200ms 最多更新一次舵机。
 * 中心 7mm 外且球已停住时给 8°最低纠偏，进入 7mm 内立即取消硬下限。
 */
#define BALL_CONTROL_ENABLE_DEFAULT       (K230_LINK_MAPPING_VALIDATED)
#define BALL_CONTROL_KP_DEG_PER_MM        (0.45f)
#define BALL_CONTROL_KD_DEG_S_PER_MM      (0.035f)
#define BALL_CONTROL_UPDATE_MS            (200U)
#define BALL_CONTROL_STABLE_BAND_MM       (7.0f)
#define BALL_CONTROL_STALL_VELOCITY_MM_S  (8.0f)
#define BALL_CONTROL_MIN_MOVE_DEG         (8.0f)
#define BALL_CONTROL_MAX_OUTPUT_DEG       (20.0f)
#define BALL_CONTROL_VELOCITY_FILTER_ALPHA (0.45f)
#define BALL_CONTROL_MAX_SAMPLE_PERIOD_MS (250U)

/* 短时丢帧保持上一输出；明确出视野时暂时向反方向拉回。 */
#define BALL_CONTROL_LOST_HOLD_MS         (180U)
#define BALL_CONTROL_EDGE_OUTPUT_DEG      (50.0f)
#define BALL_CONTROL_EDGE_TIMEOUT_MS      (1000U)

/* 无有效 Flash 标定记录时的最新实测无漂移基准：1701us。 */
#define BALL_CONTROL_TRIM_INITIAL_DEG     (108.1f)

void Ball_Control_Init(void);
void Ball_Control_SetTarget(float target_x_mm);
void Ball_Control_SetEnabled(uint8_t enabled);
uint8_t Ball_Control_IsEnabled(void);
/* 自动扫掠阶段置 1，禁止普通闭环覆盖校准舵机命令。 */
void Ball_Control_SetServoHold(uint8_t hold);
void Ball_Control_SetTrimAngle(float angle_deg);
void Ball_Control_SetGains(float kp, float kd);
void Ball_Control_Reset(void);
void Ball_Control_Step(const K230_BallPosition *position, float period_s);
const Ball_Control *Ball_Control_Get(void);

#endif
