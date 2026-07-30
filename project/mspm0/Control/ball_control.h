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
    uint8_t enabled;
    uint8_t has_last_sample;
    uint8_t edge_recovery_active;
    uint8_t edge_recovery_timed_out;
} Ball_Control;

#define BALL_CONTROL_ENABLE_DEFAULT   (1U)
#define BALL_CONTROL_KP_DEG_PER_MM    (0.24f)  /* 0~180 度映射下，PWM 等效增益较上一版提高约 33% */
#define BALL_CONTROL_KD_DEG_S_PER_MM  (0.006f) /* 与比例项同比增强，抑制位置快速变化时的过冲 */
#define BALL_CONTROL_DEADBAND_MM      (1.0f)
#define BALL_CONTROL_MAX_OFFSET_DEG   (20.0f)  /* 正常闭环仅允许在 90 度中位两侧小幅动作 */
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
