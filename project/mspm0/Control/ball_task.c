#include "ball_task.h"
#include "ball_control.h"
#include "ball_calibrate.h"
#include "k230_link.h"

/* 题目要求3：+5 cm / -5 cm（K230 单位 mm），误差 ±1 cm=±10 mm。 */
#define BALL_TASK_ZERO_MM          (0.0f)
#define BALL_TASK_PLUS_MM          (50.0f)
#define BALL_TASK_MINUS_MM         (-50.0f)
/*
 * 位置容差 ±10mm（题目允许 ±1cm）+ 速度 ≤10mm/s（防高速穿过误判）。
 * +5cm 只需到达即切往 -5cm（不停留）；-5cm 需持续稳定 SETTLE_MS。
 */
#define BALL_TASK_TOLERANCE_MM     (10.0f)
#define BALL_TASK_MAX_VELOCITY_MM_S (10.0f)
#define BALL_TASK_SETTLE_MS        (500U)
#define BALL_TASK_SETTLE_MIN_SAMPLES (4U)
#define BALL_TASK_PHASE_TIMEOUT_MS (8000U)

typedef struct {
    Ball_TaskState state;
    uint32_t ms;
    uint32_t task_start_ms;
    uint32_t phase_start_ms;
    uint32_t settle_ms;
    uint32_t last_sample_ts;
    uint8_t settle_samples;
    uint8_t target_armed;
    uint32_t total_ms;
    float max_error_plus_mm;
    float max_error_minus_mm;
} Ball_Task;

static Ball_Task g_task;

static float Ball_Task_Max(float a, float b)
{
    return (a > b) ? a : b;
}

static void Ball_Task_ApplyTarget(float target_mm)
{
    K230_BallPosition position;

    Ball_Control_SetTarget(target_mm);
    K230_Link_GetPosition(&position);
    g_task.target_armed = 1U;
    g_task.phase_start_ms = g_task.ms;
    g_task.settle_ms = 0U;
    g_task.last_sample_ts = position.timestamp_ms;
    g_task.settle_samples = 0U;
}

void Ball_Task_Init(void)
{
    g_task.state = BALL_TASK_IDLE;
    g_task.ms = 0U;
    g_task.task_start_ms = 0U;
    g_task.phase_start_ms = 0U;
    g_task.settle_ms = 0U;
    g_task.last_sample_ts = 0U;
    g_task.settle_samples = 0U;
    g_task.target_armed = 0U;
    g_task.total_ms = 0U;
    g_task.max_error_plus_mm = 0.0f;
    g_task.max_error_minus_mm = 0.0f;
    /* MCU RESET 后明确把闭环目标恢复到杆中心 0 cm。 */
    Ball_Control_SetTarget(BALL_TASK_ZERO_MM);
}

void Ball_Task_StartPoint(void)
{
    /* 校准期间忽略短按，禁止两个状态机同时接管舵机。 */
    if (Ball_Calibrate_IsActive() != 0U) {
        return;
    }
    /* FAULT 后允许直接重启（避免 reset 清掉 RAM 中的标定 trim）。 */
    if ((g_task.state != BALL_TASK_IDLE) &&
        (g_task.state != BALL_TASK_POINT_DONE) &&
        (g_task.state != BALL_TASK_FAULT)) {
        return;
    }
    g_task.state = BALL_TASK_POINT_PLUS;
    g_task.task_start_ms = g_task.ms;
    g_task.total_ms = 0U;
    g_task.target_armed = 0U;
    g_task.settle_ms = 0U;
    g_task.last_sample_ts = 0U;
    g_task.settle_samples = 0U;
    g_task.max_error_plus_mm = 0.0f;
    g_task.max_error_minus_mm = 0.0f;
}

void Ball_Task_Tick5ms(void)
{
    K230_BallPosition position;
    const Ball_Control *control;
    float error_abs;
    float velocity_abs;

    g_task.ms += 5U;

    switch (g_task.state) {
    case BALL_TASK_POINT_PLUS:
        if (g_task.target_armed == 0U) {
            Ball_Task_ApplyTarget(BALL_TASK_PLUS_MM);
        }
        break;
    case BALL_TASK_POINT_MINUS:
        if (g_task.target_armed == 0U) {
            Ball_Task_ApplyTarget(BALL_TASK_MINUS_MM);
        }
        break;
    case BALL_TASK_IDLE:
    case BALL_TASK_POINT_DONE:
    case BALL_TASK_FAULT:
        return;
    }

    if ((uint32_t)(g_task.ms - g_task.phase_start_ms) >
        BALL_TASK_PHASE_TIMEOUT_MS) {
        /* 任一阶段超时都立即恢复零点目标。 */
        Ball_Control_SetTarget(BALL_TASK_ZERO_MM);
        g_task.state = BALL_TASK_FAULT;
        return;
    }

    K230_Link_GetPosition(&position);
    if (position.valid == 0U) {
        return;
    }
    /* 同一视觉帧只参与一次任务验收，不能按 5ms 中断重复累计稳定时间。 */
    if (position.timestamp_ms == g_task.last_sample_ts) {
        return;
    }
    g_task.last_sample_ts = position.timestamp_ms;

    {
        float target = (g_task.state == BALL_TASK_POINT_PLUS) ?
                       BALL_TASK_PLUS_MM : BALL_TASK_MINUS_MM;
        error_abs = position.x_mm - target;
    }
    if (error_abs < 0.0f) {
        error_abs = -error_abs;
    }

    /* 记录全程最大 |x-target|，DONE 后供验收对照 ±10mm。 */
    if (g_task.state == BALL_TASK_POINT_PLUS) {
        g_task.max_error_plus_mm = Ball_Task_Max(g_task.max_error_plus_mm,
                                                 error_abs);
    } else {
        g_task.max_error_minus_mm = Ball_Task_Max(g_task.max_error_minus_mm,
                                                  error_abs);
    }

    /*
     * +5cm 只按位置判定，到达后立即折返；-5cm 才要求低速、多帧稳定。
     */
    if (g_task.state == BALL_TASK_POINT_PLUS) {
        if (error_abs <= BALL_TASK_TOLERANCE_MM) {
            g_task.state = BALL_TASK_POINT_MINUS;
            g_task.target_armed = 0U;
            g_task.settle_ms = 0U;
            g_task.settle_samples = 0U;
        }
        return;
    }

    control = Ball_Control_Get();
    velocity_abs = control->filtered_velocity_mm_s;
    if (velocity_abs < 0.0f) {
        velocity_abs = -velocity_abs;
    }
    if ((error_abs <= BALL_TASK_TOLERANCE_MM) &&
        (velocity_abs <= BALL_TASK_MAX_VELOCITY_MM_S)) {
        if (g_task.settle_samples == 0U) {
            g_task.settle_ms = g_task.ms;
        }
        if (g_task.settle_samples < 255U) {
            g_task.settle_samples++;
        }
        if (((uint32_t)(g_task.ms - g_task.settle_ms) >=
             BALL_TASK_SETTLE_MS) &&
            (g_task.settle_samples >= BALL_TASK_SETTLE_MIN_SAMPLES)) {
            g_task.total_ms = g_task.ms - g_task.task_start_ms;
            g_task.state = BALL_TASK_POINT_DONE;
        }
    } else {
        g_task.settle_ms = 0U;
        g_task.settle_samples = 0U;
    }
}

Ball_TaskState Ball_Task_GetState(void)
{
    return g_task.state;
}

uint32_t Ball_Task_GetTotalMs(void)
{
    return g_task.total_ms;
}

float Ball_Task_GetMaxErrorPlusMm(void)
{
    return g_task.max_error_plus_mm;
}

float Ball_Task_GetMaxErrorMinusMm(void)
{
    return g_task.max_error_minus_mm;
}
