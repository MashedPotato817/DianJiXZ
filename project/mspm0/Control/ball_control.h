#ifndef H_BALL_CONTROL
#define H_BALL_CONTROL

#include "k230_link.h"

typedef struct {
    float kp;
    float kd;
    float target_x_mm;
    float last_error_mm;
    uint32_t last_timestamp_ms;
    uint8_t enabled;
    uint8_t has_last_sample;
} Ball_Control;

#define BALL_CONTROL_ENABLE_DEFAULT   (1U)
#define BALL_CONTROL_KP_DEG_PER_MM    (0.10f)  /* 首轮增强：50 mm 偏差对应 5 deg 限幅 */
#define BALL_CONTROL_KD_DEG_S_PER_MM  (0.0025f) /* 与 Kp 同比提高，抑制增强后的过冲 */
#define BALL_CONTROL_DEADBAND_MM      (1.0f)

void Ball_Control_Init(void);
void Ball_Control_SetTarget(float target_x_mm);
void Ball_Control_SetEnabled(uint8_t enabled);
uint8_t Ball_Control_IsEnabled(void);
void Ball_Control_Reset(void);
void Ball_Control_Step(const K230_BallPosition *position, float period_s);
const Ball_Control *Ball_Control_Get(void);

#endif
