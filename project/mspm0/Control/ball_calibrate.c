#include "ball_calibrate.h"
#include "ball_control.h"
#include "servo.h"
#include "k230_link.h"
#include "calib_store.h"

#if BALL_CAL_ENABLE

typedef struct {
    Ball_CalibrateState state;
    float lo_deg;
    float hi_deg;
    float mid_deg;
    float result_deg;
    uint32_t ms;
    uint32_t phase_ms;
    uint32_t wait_start_ms;
    uint32_t start_ms;
    uint8_t iterations;
    float sample_x[BALL_CAL_MAX_SAMPLES];
    uint8_t sample_count;
    uint32_t last_sample_ts;
} Ball_Calibrate;

static Ball_Calibrate g_cal;

static float Ball_Calibrate_Abs(float value)
{
    return (value < 0.0f) ? -value : value;
}

static void Ball_Calibrate_StartIteration(void)
{
    g_cal.mid_deg = (g_cal.lo_deg + g_cal.hi_deg) * 0.5f;
    Servo_SetTargetAngle(g_cal.mid_deg);
    Servo_ApplyHardware();
    g_cal.state = BALL_CAL_STATE_SETTLE;
    g_cal.phase_ms = g_cal.ms;
}

static void Ball_Calibrate_Finish(float balance_deg)
{
    g_cal.result_deg = balance_deg;
    g_cal.state = BALL_CAL_STATE_DONE;
    Ball_Control_SetServoHold(0);
    /* 把舵机命令到标定结果并复位运动状态，随后闭环按新 trim 继续。 */
    Ball_Control_SetTrimAngle(balance_deg);
    /* 持久化：下次上电直接用该平衡角作为 trim 初值，不再依赖默认值。 */
    (void)CalibStore_Save(balance_deg);
}

static void Ball_Calibrate_Abort(void)
{
    g_cal.state = BALL_CAL_STATE_IDLE;
    Ball_Control_SetServoHold(0);
}

static void Ball_Calibrate_Decide(void)
{
    uint8_t n = g_cal.sample_count;
    uint8_t third;
    uint8_t i;
    float early = 0.0f;
    float late = 0.0f;
    float trend;

    if (n < 4U) {
        /* 样本不足：无法可靠判断方向，保守收敛到当前角度。 */
        Ball_Calibrate_Finish(g_cal.mid_deg);
        return;
    }

    third = n / 3U;
    for (i = 0U; i < third; i++) {
        early += g_cal.sample_x[i];
    }
    for (i = n - third; i < n; i++) {
        late += g_cal.sample_x[i];
    }
    early /= (float)third;
    late /= (float)third;
    trend = late - early;

    if ((g_cal.sample_x[0] > BALL_CAL_EDGE_X_MM) &&
        (g_cal.sample_x[n - 1U] > BALL_CAL_EDGE_X_MM) &&
        (Ball_Calibrate_Abs(trend) < BALL_CAL_TREND_THRESHOLD_MM)) {
        /* 球被卡在 +X 边缘：当前角度偏高（右端低），缩小上界。 */
        g_cal.hi_deg = g_cal.mid_deg;
    } else if ((g_cal.sample_x[0] < -BALL_CAL_EDGE_X_MM) &&
               (g_cal.sample_x[n - 1U] < -BALL_CAL_EDGE_X_MM) &&
               (Ball_Calibrate_Abs(trend) < BALL_CAL_TREND_THRESHOLD_MM)) {
        /* 球被卡在 -X 边缘：当前角度偏低（左端低），缩小下界。 */
        g_cal.lo_deg = g_cal.mid_deg;
    } else if (trend > BALL_CAL_TREND_THRESHOLD_MM) {
        /* 球向 +X 滚：当前角度偏高。 */
        g_cal.hi_deg = g_cal.mid_deg;
    } else if (trend < -BALL_CAL_TREND_THRESHOLD_MM) {
        /* 球向 -X 滚：当前角度偏低。 */
        g_cal.lo_deg = g_cal.mid_deg;
    } else {
        /* 球近乎静止：当前角度即为平衡角。 */
        Ball_Calibrate_Finish(g_cal.mid_deg);
        return;
    }

    g_cal.iterations++;
    if ((g_cal.iterations >= BALL_CAL_MAX_ITERATIONS) ||
        ((g_cal.hi_deg - g_cal.lo_deg) <= BALL_CAL_CONVERGE_SPAN_DEG) ||
        ((uint32_t)(g_cal.ms - g_cal.start_ms) > BALL_CAL_TOTAL_TIMEOUT_MS)) {
        Ball_Calibrate_Finish((g_cal.lo_deg + g_cal.hi_deg) * 0.5f);
        return;
    }

    Ball_Calibrate_StartIteration();
}

void Ball_Calibrate_Init(void)
{
    uint8_t i;

    g_cal.state = BALL_CAL_STATE_IDLE;
    g_cal.lo_deg = 0.0f;
    g_cal.hi_deg = 0.0f;
    g_cal.mid_deg = 0.0f;
    g_cal.result_deg = 0.0f;
    g_cal.ms = 0U;
    g_cal.phase_ms = 0U;
    g_cal.wait_start_ms = 0U;
    g_cal.start_ms = 0U;
    g_cal.iterations = 0U;
    g_cal.sample_count = 0U;
    g_cal.last_sample_ts = 0U;
    for (i = 0U; i < BALL_CAL_MAX_SAMPLES; i++) {
        g_cal.sample_x[i] = 0.0f;
    }
}

void Ball_Calibrate_Start(void)
{
    if ((g_cal.state != BALL_CAL_STATE_IDLE) &&
        (g_cal.state != BALL_CAL_STATE_DONE)) {
        return;
    }
    Ball_Control_SetServoHold(1);
    g_cal.state = BALL_CAL_STATE_WAIT_BALL;
    g_cal.wait_start_ms = g_cal.ms;
    g_cal.start_ms = g_cal.ms;
    g_cal.iterations = 0U;
    g_cal.result_deg = 0.0f;
}

void Ball_Calibrate_Tick5ms(void)
{
    K230_BallPosition position;

    g_cal.ms += 5U;

    switch (g_cal.state) {
    case BALL_CAL_STATE_IDLE:
    case BALL_CAL_STATE_DONE:
        break;

    case BALL_CAL_STATE_WAIT_BALL:
        if ((uint32_t)(g_cal.ms - g_cal.wait_start_ms) >
            BALL_CAL_WAIT_BALL_TIMEOUT_MS) {
            /* 球不在杆上：放弃标定，保持原 trim，闭环不受影响。 */
            Ball_Calibrate_Abort();
            break;
        }
        K230_Link_GetPosition(&position);
        if (position.valid != 0U) {
            g_cal.lo_deg = BALL_CAL_SEARCH_MIN_DEG;
            g_cal.hi_deg = BALL_CAL_SEARCH_MAX_DEG;
            Ball_Calibrate_StartIteration();
        }
        break;

    case BALL_CAL_STATE_SETTLE:
        if ((uint32_t)(g_cal.ms - g_cal.phase_ms) >= BALL_CAL_SETTLE_MS) {
            g_cal.state = BALL_CAL_STATE_OBSERVE;
            g_cal.phase_ms = g_cal.ms;
            g_cal.sample_count = 0U;
            g_cal.last_sample_ts = 0U;
        }
        break;

    case BALL_CAL_STATE_OBSERVE:
        K230_Link_GetPosition(&position);
        if ((position.valid != 0U) &&
            (position.timestamp_ms != g_cal.last_sample_ts)) {
            g_cal.last_sample_ts = position.timestamp_ms;
            if (g_cal.sample_count < BALL_CAL_MAX_SAMPLES) {
                g_cal.sample_x[g_cal.sample_count] = position.x_mm;
                g_cal.sample_count++;
            }
        }
        if ((uint32_t)(g_cal.ms - g_cal.phase_ms) >= BALL_CAL_OBSERVE_MS) {
            Ball_Calibrate_Decide();
        }
        break;
    }
}

uint8_t Ball_Calibrate_IsActive(void)
{
    return ((g_cal.state == BALL_CAL_STATE_WAIT_BALL) ||
            (g_cal.state == BALL_CAL_STATE_SETTLE) ||
            (g_cal.state == BALL_CAL_STATE_OBSERVE)) ? 1U : 0U;
}

float Ball_Calibrate_GetResultDeg(void)
{
    return g_cal.result_deg;
}

Ball_CalibrateState Ball_Calibrate_GetState(void)
{
    return g_cal.state;
}

#else /* BALL_CAL_ENABLE == 0：标定关闭，保留空实现以维持调用点不变。 */

void Ball_Calibrate_Init(void)
{
}

void Ball_Calibrate_Start(void)
{
}

void Ball_Calibrate_Tick5ms(void)
{
}

uint8_t Ball_Calibrate_IsActive(void)
{
    return 0U;
}

float Ball_Calibrate_GetResultDeg(void)
{
    return 0.0f;
}

Ball_CalibrateState Ball_Calibrate_GetState(void)
{
    return BALL_CAL_STATE_IDLE;
}

#endif /* BALL_CAL_ENABLE */
