#include "ball_calibrate.h"
#include "ball_control.h"
#include "ball_task.h"
#include "servo.h"
#include "k230_link.h"
#include "calib_store.h"

#if BALL_CAL_ENABLE

typedef enum {
    BALL_CAL_STAGE_BRACKET_LOW = 0,
    BALL_CAL_STAGE_BRACKET_HIGH,
    BALL_CAL_STAGE_BISECT,
    BALL_CAL_STAGE_VERIFY_LOW,
    BALL_CAL_STAGE_VERIFY_HIGH,
    BALL_CAL_STAGE_VERIFY_CENTER
} Ball_CalibrateStage;

typedef struct {
    Ball_CalibrateState state;
    Ball_CalibrateStage stage;
    float lo_deg;
    float hi_deg;
    float mid_deg;
    float candidate_deg;
    float probe_deg;
    float pending_deg;
    float last_valid_x_mm;
    float motion_score_mm;
    float lo_score_mm;
    float hi_score_mm;
    float original_trim_deg;
    float original_target_x_mm;
    float result_deg;
    uint32_t ms;
    uint32_t phase_ms;
    uint32_t trial_start_ms;
    uint32_t recovery_start_ms;
    uint32_t recovery_step_ms;
    uint32_t wait_start_ms;
    uint32_t start_ms;
    uint32_t center_start_ms;
    uint32_t center_stable_start_ms;
    uint8_t iterations;
    uint8_t center_stable;
    uint8_t save_pending;   /* 标定完成待写 Flash，主循环 ProcessSave 执行 */
    float sample_x[BALL_CAL_MAX_SAMPLES];
    uint8_t sample_count;
    uint8_t edge_positive_count;
    uint8_t edge_negative_count;
    int8_t trial_start_side;
    int8_t recovery_side;
    uint16_t recovery_pulse_us;
    uint16_t recovery_step_us;
    uint32_t last_sample_ts;
} Ball_Calibrate;

static Ball_Calibrate g_cal;

static float Ball_Calibrate_Abs(float value)
{
    return (value < 0.0f) ? -value : value;
}

static float Ball_Calibrate_Clamp(float value, float minimum, float maximum)
{
    if (value < minimum) {
        return minimum;
    }
    if (value > maximum) {
        return maximum;
    }
    return value;
}

static float Ball_Calibrate_PulseToDeg(uint16_t pulse_us)
{
    return ((float)pulse_us - (float)SERVO_PULSE_MIN_US) /
           SERVO_US_PER_DEG;
}

static float Ball_Calibrate_PulseSpanToDeg(uint16_t pulse_us)
{
    return (float)pulse_us / SERVO_US_PER_DEG;
}

static int8_t Ball_Calibrate_PositionSide(const K230_BallPosition *position)
{
    if (position->valid != 0U) {
        if (position->x_mm > BALL_CAL_EDGE_X_MM) {
            return 1;
        }
        if (position->x_mm < -BALL_CAL_EDGE_X_MM) {
            return -1;
        }
        return 0;
    }
    if (position->edge_direction > 0) {
        return 1;
    }
    if (position->edge_direction < 0) {
        return -1;
    }
    return 0;
}

static void Ball_Calibrate_ApplyTrial(float angle_deg,
                                      const K230_BallPosition *position)
{
    g_cal.mid_deg = angle_deg;
    g_cal.trial_start_side = Ball_Calibrate_PositionSide(position);
    g_cal.trial_start_ms = g_cal.ms;
    Servo_SetTargetAngle(angle_deg);
    Servo_ApplyHardware();
    g_cal.state = BALL_CAL_STATE_SETTLE;
    g_cal.phase_ms = g_cal.ms;
}

static void Ball_Calibrate_StartAt(float angle_deg)
{
    K230_BallPosition position;
    int8_t lost_side;

    K230_Link_GetPosition(&position);
    if (position.valid != 0U) {
        g_cal.last_valid_x_mm = position.x_mm;
        Ball_Calibrate_ApplyTrial(angle_deg, &position);
        return;
    }

    /* 优先按最后一次有效位置判断离开侧，edge 作为没有历史位置时的后备。 */
    if (g_cal.last_valid_x_mm > 0.0f) {
        lost_side = 1;
    } else if (g_cal.last_valid_x_mm < 0.0f) {
        lost_side = -1;
    } else {
        lost_side = Ball_Calibrate_PositionSide(&position);
    }
    if (lost_side == 0) {
        /* 只是无侧别的瞬时丢帧，仍可下发原试探点，由观察窗口继续判断。 */
        Ball_Calibrate_ApplyTrial(angle_deg, &position);
        return;
    }

    g_cal.pending_deg = angle_deg;
    g_cal.recovery_side = lost_side;
    g_cal.recovery_pulse_us = Servo_GetPulseUs();
    g_cal.recovery_step_us = BALL_CAL_RECOVER_START_US;
    g_cal.recovery_start_ms = g_cal.ms;
    g_cal.recovery_step_ms = g_cal.ms - BALL_CAL_RECOVER_STEP_MS;
    g_cal.state = BALL_CAL_STATE_RECOVER;
}

static void Ball_Calibrate_StartIteration(void)
{
    float lo_strength = Ball_Calibrate_Abs(g_cal.lo_score_mm);
    float hi_strength = Ball_Calibrate_Abs(g_cal.hi_score_mm);
    float ratio = 0.5f;

    /*
     * 用两端实测滚动强度线性估算零响应点。比例限制在 30%~70%，避免
     * 单次视觉跳变把试探点推到端点；证据不足时自然退回普通中点。
     */
    if ((g_cal.lo_score_mm > 0.0f) &&
        (g_cal.hi_score_mm < 0.0f) &&
        ((lo_strength + hi_strength) > 0.0f)) {
        ratio = lo_strength / (lo_strength + hi_strength);
        ratio = Ball_Calibrate_Clamp(
            ratio, BALL_CAL_NEXT_MIN_RATIO, BALL_CAL_NEXT_MAX_RATIO);
    }
    Ball_Calibrate_StartAt(
        g_cal.lo_deg + (g_cal.hi_deg - g_cal.lo_deg) * ratio);
}

static void Ball_Calibrate_Finish(float balance_deg)
{
    g_cal.result_deg = balance_deg;
    g_cal.state = BALL_CAL_STATE_DONE;
    Ball_Control_SetServoHold(0);
    /* CENTER 已按候选 trim 建立闭环，完成时保留其积分和制动状态。 */
#if BALL_CAL_SAVE_ENABLE
    /*
     * 持久化推迟到主循环（Ball_Calibrate_ProcessSave）：Flash 擦写毫秒级且
     * 需关全局中断，不能在 TIMER_0 中断里执行，否则会长时间阻塞 K230 接收。
     * 标定扇区由 scatter 文件从应用镜像中保留，保存后由主循环读回验证。
     */
    g_cal.save_pending = 1U;
#endif
}

static void Ball_Calibrate_Abort(void)
{
    g_cal.state = BALL_CAL_STATE_FAILED;
    Ball_Control_SetServoHold(0);
    /* 中止时恢复进入标定前的 trim 和目标，不能停留在最后一次试探角。 */
    Ball_Control_SetTrimAngle(g_cal.original_trim_deg);
    Ball_Control_SetTarget(g_cal.original_target_x_mm);
}

static int8_t Ball_Calibrate_GetDirection(uint8_t allow_same_edge,
                                          uint8_t *blocked_at_start_edge)
{
    uint8_t n = g_cal.sample_count;
    uint8_t third;
    uint8_t i;
    float early = 0.0f;
    float late = 0.0f;
    float trend = 0.0f;
    int8_t edge_side = 0;

    *blocked_at_start_edge = 0U;
    g_cal.motion_score_mm = 0.0f;
    if (g_cal.edge_positive_count >= BALL_CAL_EDGE_CONFIRM_FRAMES) {
        edge_side = 1;
    } else if (g_cal.edge_negative_count >= BALL_CAL_EDGE_CONFIRM_FRAMES) {
        edge_side = -1;
    }

    if (n >= BALL_CAL_MIN_SAMPLES) {
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
            edge_side = 1;
        } else if ((g_cal.sample_x[0] < -BALL_CAL_EDGE_X_MM) &&
                   (g_cal.sample_x[n - 1U] < -BALL_CAL_EDGE_X_MM) &&
                   (Ball_Calibrate_Abs(trend) <
                    BALL_CAL_TREND_THRESHOLD_MM)) {
            edge_side = -1;
        }
    }

    if (edge_side != 0) {
        if ((edge_side != g_cal.trial_start_side) ||
            (allow_same_edge != 0U)) {
            g_cal.motion_score_mm =
                (float)edge_side * BALL_CAL_TREND_SCORE_MAX_MM;
            return edge_side;
        }
        *blocked_at_start_edge = 1U;
        return 0;
    }
    if (n < BALL_CAL_MIN_SAMPLES) {
        return 0;
    }
    if (trend > BALL_CAL_TREND_THRESHOLD_MM) {
        g_cal.motion_score_mm = Ball_Calibrate_Clamp(
            trend, BALL_CAL_TREND_THRESHOLD_MM,
            BALL_CAL_TREND_SCORE_MAX_MM);
        return 1;
    }
    if (trend < -BALL_CAL_TREND_THRESHOLD_MM) {
        g_cal.motion_score_mm = Ball_Calibrate_Clamp(
            trend, -BALL_CAL_TREND_SCORE_MAX_MM,
            -BALL_CAL_TREND_THRESHOLD_MM);
        return -1;
    }
    return 0;
}

static void Ball_Calibrate_RetryObservation(void)
{
    /* 舵机命令保持不变，只重新收集一个窗口，等待视野外小球返回。 */
    g_cal.state = BALL_CAL_STATE_OBSERVE;
    g_cal.phase_ms = g_cal.ms;
    g_cal.sample_count = 0U;
    g_cal.edge_positive_count = 0U;
    g_cal.edge_negative_count = 0U;
    g_cal.last_sample_ts = 0U;
}

static uint8_t Ball_Calibrate_CanContinue(void)
{
    if ((g_cal.iterations >= BALL_CAL_MAX_ITERATIONS) ||
        ((uint32_t)(g_cal.ms - g_cal.start_ms) >
         BALL_CAL_TOTAL_TIMEOUT_MS)) {
        Ball_Calibrate_Abort();
        return 0U;
    }
    return 1U;
}

static void Ball_Calibrate_StartCenterCheck(void)
{
    g_cal.candidate_deg = (g_cal.lo_deg + g_cal.hi_deg) * 0.5f;
    g_cal.stage = BALL_CAL_STAGE_VERIFY_CENTER;
    Ball_Calibrate_StartAt(g_cal.candidate_deg);
}

static void Ball_Calibrate_StartCentering(void)
{
    /*
     * 夹逼值只证明摆杆近似无坡度，不能证明球已在中点。释放舵机接管，
     * 让低频零点闭环用该值作为 trim 把球送回 0±7mm，再决定是否保存。
     */
    g_cal.state = BALL_CAL_STATE_CENTER;
    g_cal.center_start_ms = g_cal.ms;
    g_cal.center_stable_start_ms = 0U;
    g_cal.center_stable = 0U;
    Ball_Control_SetTrimAngle(g_cal.candidate_deg);
    Ball_Control_SetTarget(0.0f);
    Ball_Control_SetServoHold(0);
}

static void Ball_Calibrate_ContinueBisect(void)
{
    float converge_deg =
        Ball_Calibrate_PulseSpanToDeg(BALL_CAL_CONVERGE_SPAN_US);

    if (Ball_Calibrate_CanContinue() == 0U) {
        return;
    }
    if ((g_cal.hi_deg - g_cal.lo_deg) <= converge_deg) {
        Ball_Calibrate_StartCenterCheck();
        return;
    }
    g_cal.stage = BALL_CAL_STAGE_BISECT;
    Ball_Calibrate_StartIteration();
}

static void Ball_Calibrate_StartVerify(void)
{
    float max_probe;

    g_cal.candidate_deg = g_cal.mid_deg;
    g_cal.probe_deg =
        Ball_Calibrate_PulseSpanToDeg(BALL_CAL_VERIFY_PROBE_US);
    max_probe = (g_cal.hi_deg - g_cal.lo_deg) * 0.5f;
    if (g_cal.probe_deg > max_probe) {
        g_cal.probe_deg = max_probe;
    }
    if (g_cal.probe_deg <=
        Ball_Calibrate_PulseSpanToDeg(BALL_CAL_CONVERGE_SPAN_US) * 0.5f) {
        Ball_Calibrate_StartCenterCheck();
        return;
    }
    g_cal.stage = BALL_CAL_STAGE_VERIFY_LOW;
    Ball_Calibrate_StartAt(g_cal.candidate_deg - g_cal.probe_deg);
}

static uint8_t Ball_Calibrate_ExpandProbe(uint8_t toward_high)
{
    float max_probe = (toward_high != 0U) ?
                      (g_cal.hi_deg - g_cal.candidate_deg) :
                      (g_cal.candidate_deg - g_cal.lo_deg);
    float next_probe = g_cal.probe_deg * 2.0f;

    if (next_probe > max_probe) {
        next_probe = max_probe;
    }
    if (next_probe <= g_cal.probe_deg + 0.01f) {
        return 0U;
    }
    g_cal.probe_deg = next_probe;
    Ball_Calibrate_StartAt(g_cal.candidate_deg +
                           ((toward_high != 0U) ? next_probe : -next_probe));
    return 1U;
}

static void Ball_Calibrate_Decide(void)
{
    int8_t direction;
    uint8_t blocked_at_start_edge;
    uint8_t trial_expired =
        ((uint32_t)(g_cal.ms - g_cal.trial_start_ms) >=
         BALL_CAL_TRIAL_TIMEOUT_MS) ? 1U : 0U;

    direction = Ball_Calibrate_GetDirection(
        trial_expired, &blocked_at_start_edge);

    if ((direction == 0) &&
        ((g_cal.sample_count < BALL_CAL_MIN_SAMPLES) ||
         (blocked_at_start_edge != 0U))) {
        if (trial_expired == 0U) {
            Ball_Calibrate_RetryObservation();
        } else {
            Ball_Calibrate_Abort();
        }
        return;
    }
    g_cal.iterations++;

    switch (g_cal.stage) {
    case BALL_CAL_STAGE_BRACKET_LOW:
        /* 最低PWM必须证明球能向+X滚动，静止不能当作平衡。 */
        if (direction != 1) {
            if (trial_expired == 0U) {
                Ball_Calibrate_RetryObservation();
            } else {
                Ball_Calibrate_Abort();
            }
            return;
        }
        g_cal.lo_deg = g_cal.mid_deg;
        g_cal.lo_score_mm = g_cal.motion_score_mm;
        g_cal.stage = BALL_CAL_STAGE_BRACKET_HIGH;
        Ball_Calibrate_StartAt(g_cal.hi_deg);
        return;

    case BALL_CAL_STAGE_BRACKET_HIGH:
        /* 最高PWM必须证明球能向-X滚动，至此才建立有效夹逼区间。 */
        if (direction != -1) {
            if (trial_expired == 0U) {
                Ball_Calibrate_RetryObservation();
            } else {
                Ball_Calibrate_Abort();
            }
            return;
        }
        g_cal.hi_deg = g_cal.mid_deg;
        g_cal.hi_score_mm = g_cal.motion_score_mm;
        Ball_Calibrate_ContinueBisect();
        return;

    case BALL_CAL_STAGE_BISECT:
        if (direction > 0) {
            g_cal.lo_deg = g_cal.mid_deg;
            g_cal.lo_score_mm = g_cal.motion_score_mm;
            Ball_Calibrate_ContinueBisect();
        } else if (direction < 0) {
            g_cal.hi_deg = g_cal.mid_deg;
            g_cal.hi_score_mm = g_cal.motion_score_mm;
            Ball_Calibrate_ContinueBisect();
        } else {
            /* 单点不动可能是静摩擦，必须在候选点两侧验证相反滚动。 */
            Ball_Calibrate_StartVerify();
        }
        return;

    case BALL_CAL_STAGE_VERIFY_LOW:
        if (direction > 0) {
            g_cal.lo_deg = g_cal.mid_deg;
            g_cal.lo_score_mm = g_cal.motion_score_mm;
            g_cal.stage = BALL_CAL_STAGE_VERIFY_HIGH;
            Ball_Calibrate_StartAt(g_cal.candidate_deg + g_cal.probe_deg);
        } else if (direction < 0) {
            g_cal.hi_deg = g_cal.mid_deg;
            g_cal.hi_score_mm = g_cal.motion_score_mm;
            Ball_Calibrate_ContinueBisect();
        } else if (Ball_Calibrate_ExpandProbe(0U) == 0U) {
            Ball_Calibrate_Abort();
        }
        return;

    case BALL_CAL_STAGE_VERIFY_HIGH:
        if (direction < 0) {
            g_cal.hi_deg = g_cal.mid_deg;
            g_cal.hi_score_mm = g_cal.motion_score_mm;
            g_cal.stage = BALL_CAL_STAGE_VERIFY_CENTER;
            Ball_Calibrate_StartAt(g_cal.candidate_deg);
        } else if (direction > 0) {
            g_cal.lo_deg = g_cal.mid_deg;
            g_cal.lo_score_mm = g_cal.motion_score_mm;
            Ball_Calibrate_ContinueBisect();
        } else if (Ball_Calibrate_ExpandProbe(1U) == 0U) {
            Ball_Calibrate_Abort();
        }
        return;

    case BALL_CAL_STAGE_VERIFY_CENTER:
        if (direction == 0) {
            Ball_Calibrate_StartCentering();
        } else {
            /* 复核仍有系统漂移，继续按实测方向缩小夹逼区间。 */
            if (direction > 0) {
                g_cal.lo_deg = g_cal.candidate_deg;
                g_cal.lo_score_mm = g_cal.motion_score_mm;
            } else {
                g_cal.hi_deg = g_cal.candidate_deg;
                g_cal.hi_score_mm = g_cal.motion_score_mm;
            }
            Ball_Calibrate_ContinueBisect();
        }
        return;
    }
}

void Ball_Calibrate_Init(void)
{
    uint8_t i;

    g_cal.state = BALL_CAL_STATE_IDLE;
    g_cal.stage = BALL_CAL_STAGE_BRACKET_LOW;
    g_cal.lo_deg = 0.0f;
    g_cal.hi_deg = 0.0f;
    g_cal.mid_deg = 0.0f;
    g_cal.candidate_deg = 0.0f;
    g_cal.probe_deg = 0.0f;
    g_cal.pending_deg = 0.0f;
    g_cal.last_valid_x_mm = 0.0f;
    g_cal.motion_score_mm = 0.0f;
    g_cal.lo_score_mm = 0.0f;
    g_cal.hi_score_mm = 0.0f;
    g_cal.original_trim_deg = BALL_CONTROL_TRIM_INITIAL_DEG;
    g_cal.original_target_x_mm = 0.0f;
    g_cal.result_deg = 0.0f;
    g_cal.ms = 0U;
    g_cal.phase_ms = 0U;
    g_cal.trial_start_ms = 0U;
    g_cal.recovery_start_ms = 0U;
    g_cal.recovery_step_ms = 0U;
    g_cal.wait_start_ms = 0U;
    g_cal.start_ms = 0U;
    g_cal.center_start_ms = 0U;
    g_cal.center_stable_start_ms = 0U;
    g_cal.iterations = 0U;
    g_cal.center_stable = 0U;
    g_cal.save_pending = 0U;
    g_cal.sample_count = 0U;
    g_cal.edge_positive_count = 0U;
    g_cal.edge_negative_count = 0U;
    g_cal.trial_start_side = 0;
    g_cal.recovery_side = 0;
    g_cal.recovery_pulse_us = SERVO_PULSE_NEUTRAL_US;
    g_cal.recovery_step_us = BALL_CAL_RECOVER_START_US;
    g_cal.last_sample_ts = 0U;
    for (i = 0U; i < BALL_CAL_MAX_SAMPLES; i++) {
        g_cal.sample_x[i] = 0.0f;
    }
}

/* 主循环调用：标定完成待写 Flash 时执行持久化（避免在 TIMER_0 中断里做）。
 * 返回：0=无待存，1=保存并读回验证成功，2=保存/读回失败。 */
uint8_t Ball_Calibrate_ProcessSave(void)
{
    float readback;

    if (g_cal.save_pending == 0U) {
        return 0U;
    }
    g_cal.save_pending = 0U;
    if (CalibStore_Save(g_cal.result_deg) == 0U) {
        g_cal.state = BALL_CAL_STATE_SAVE_FAILED;
        return 2U;
    }
    if ((CalibStore_Load(&readback) == 0U) ||
        (readback != g_cal.result_deg)) {
        g_cal.state = BALL_CAL_STATE_SAVE_FAILED;
        return 2U;
    }
    return 1U;
}

void Ball_Calibrate_Start(void)
{
    const Ball_Control *control;

    /* 标定依赖位置方向和尺度；五点映射未验证时不得移动舵机或写 Flash。 */
    if (K230_LINK_MAPPING_VALIDATED == 0U) {
        return;
    }
    {
        Ball_TaskState task_state = Ball_Task_GetState();
        if ((task_state == BALL_TASK_POINT_PLUS) ||
            (task_state == BALL_TASK_POINT_MINUS)) {
            return;
        }
    }
    if ((g_cal.state != BALL_CAL_STATE_IDLE) &&
        (g_cal.state != BALL_CAL_STATE_DONE) &&
        (g_cal.state != BALL_CAL_STATE_FAILED) &&
        (g_cal.state != BALL_CAL_STATE_SAVE_FAILED)) {
        return;
    }
    control = Ball_Control_Get();
    g_cal.original_trim_deg = control->trim_angle_deg;
    g_cal.original_target_x_mm = control->target_x_mm;
    Ball_Control_SetServoHold(1);
    g_cal.state = BALL_CAL_STATE_WAIT_BALL;
    g_cal.wait_start_ms = g_cal.ms;
    g_cal.start_ms = g_cal.ms;
    g_cal.iterations = 0U;
    g_cal.stage = BALL_CAL_STAGE_BRACKET_LOW;
    g_cal.candidate_deg = 0.0f;
    g_cal.probe_deg = 0.0f;
    g_cal.result_deg = 0.0f;
    g_cal.motion_score_mm = 0.0f;
    g_cal.lo_score_mm = 0.0f;
    g_cal.hi_score_mm = 0.0f;
    g_cal.center_stable = 0U;
}

void Ball_Calibrate_Tick5ms(void)
{
    K230_BallPosition position;
    uint16_t next_pulse;

    g_cal.ms += 5U;
    K230_Link_GetPosition(&position);
    if (position.valid != 0U) {
        g_cal.last_valid_x_mm = position.x_mm;
    }

    switch (g_cal.state) {
    case BALL_CAL_STATE_IDLE:
    case BALL_CAL_STATE_DONE:
    case BALL_CAL_STATE_FAILED:
    case BALL_CAL_STATE_SAVE_FAILED:
        break;

    case BALL_CAL_STATE_WAIT_BALL:
        if ((uint32_t)(g_cal.ms - g_cal.wait_start_ms) >
            BALL_CAL_WAIT_BALL_TIMEOUT_MS) {
            /* 完全没有有效位置或视野外侧别：保持原 trim，不盲目扫舵机。 */
            Ball_Calibrate_Abort();
            break;
        }
        /*
         * 宽范围标定会先验证两个端点，并能利用 edge 等待视野外小球返回，
         * 因此无需把球预先放进 ±10mm；否则 servo_hold 会冻结最后PWM，
         * 形成“等它进中心、但又不给它运动”的入口死锁。
         */
        if ((position.valid != 0U) || (position.edge_direction != 0)) {
            g_cal.lo_deg =
                Ball_Calibrate_PulseToDeg(BALL_CAL_SEARCH_MIN_US);
            g_cal.hi_deg =
                Ball_Calibrate_PulseToDeg(BALL_CAL_SEARCH_MAX_US);
            g_cal.stage = BALL_CAL_STAGE_BRACKET_LOW;
            Ball_Calibrate_StartAt(g_cal.lo_deg);
        }
        break;

    case BALL_CAL_STATE_RECOVER:
        if (position.valid != 0U) {
            /* 已重新进入视野，恢复此前尚未执行的校准试探点。 */
            Ball_Calibrate_ApplyTrial(g_cal.pending_deg, &position);
            break;
        }
        if ((uint32_t)(g_cal.ms - g_cal.recovery_start_ms) >=
            BALL_CAL_RECOVER_TIMEOUT_MS) {
            Ball_Calibrate_Abort();
            break;
        }
        if ((uint32_t)(g_cal.ms - g_cal.recovery_step_ms) >=
            BALL_CAL_RECOVER_STEP_MS) {
            g_cal.recovery_step_ms = g_cal.ms;
            next_pulse = g_cal.recovery_pulse_us;
            if (g_cal.recovery_side > 0) {
                /* 从+X离开：增大PWM，让球向-X返回。 */
                if ((uint32_t)next_pulse + g_cal.recovery_step_us >=
                    BALL_CAL_SEARCH_MAX_US) {
                    next_pulse = BALL_CAL_SEARCH_MAX_US;
                } else {
                    next_pulse = (uint16_t)(next_pulse +
                                           g_cal.recovery_step_us);
                }
            } else {
                /* 从-X离开：减小PWM，让球向+X返回。 */
                if (next_pulse <=
                    (uint16_t)(BALL_CAL_SEARCH_MIN_US +
                               g_cal.recovery_step_us)) {
                    next_pulse = BALL_CAL_SEARCH_MIN_US;
                } else {
                    next_pulse = (uint16_t)(next_pulse -
                                           g_cal.recovery_step_us);
                }
            }
            g_cal.recovery_pulse_us = next_pulse;
            Servo_SetTargetAngle(Ball_Calibrate_PulseToDeg(next_pulse));
            Servo_ApplyHardware();
            if (g_cal.recovery_step_us <
                BALL_CAL_RECOVER_MAX_STEP_US) {
                g_cal.recovery_step_us =
                    (uint16_t)(g_cal.recovery_step_us +
                               BALL_CAL_RECOVER_ADD_US);
                if (g_cal.recovery_step_us >
                    BALL_CAL_RECOVER_MAX_STEP_US) {
                    g_cal.recovery_step_us =
                        BALL_CAL_RECOVER_MAX_STEP_US;
                }
            }
        }
        break;

    case BALL_CAL_STATE_SETTLE:
        if ((uint32_t)(g_cal.ms - g_cal.phase_ms) >= BALL_CAL_SETTLE_MS) {
            g_cal.state = BALL_CAL_STATE_OBSERVE;
            g_cal.phase_ms = g_cal.ms;
            g_cal.sample_count = 0U;
            g_cal.edge_positive_count = 0U;
            g_cal.edge_negative_count = 0U;
            g_cal.last_sample_ts = 0U;
        }
        break;

    case BALL_CAL_STATE_OBSERVE:
        if (position.timestamp_ms != g_cal.last_sample_ts) {
            g_cal.last_sample_ts = position.timestamp_ms;
            if ((position.valid != 0U) &&
                (g_cal.sample_count < BALL_CAL_MAX_SAMPLES)) {
                g_cal.sample_x[g_cal.sample_count] = position.x_mm;
                g_cal.sample_count++;
            } else if (position.edge_direction > 0) {
                if (g_cal.edge_positive_count < 255U) {
                    g_cal.edge_positive_count++;
                }
            } else if (position.edge_direction < 0) {
                if (g_cal.edge_negative_count < 255U) {
                    g_cal.edge_negative_count++;
                }
            }
        }
        if ((uint32_t)(g_cal.ms - g_cal.phase_ms) >= BALL_CAL_OBSERVE_MS) {
            Ball_Calibrate_Decide();
        }
        break;

    case BALL_CAL_STATE_CENTER:
        if (((uint32_t)(g_cal.ms - g_cal.center_start_ms) >=
             BALL_CAL_CENTER_TIMEOUT_MS) ||
            ((uint32_t)(g_cal.ms - g_cal.start_ms) >=
             BALL_CAL_TOTAL_TIMEOUT_MS)) {
            Ball_Calibrate_Abort();
            break;
        }
        if ((position.valid != 0U) &&
            (Ball_Calibrate_Abs(position.x_mm) <=
             BALL_CAL_CENTER_TOLERANCE_MM) &&
            (Ball_Calibrate_Abs(
                Ball_Control_Get()->filtered_velocity_mm_s) <=
             BALL_CAL_CENTER_VELOCITY_MM_S)) {
            if (g_cal.center_stable == 0U) {
                g_cal.center_stable = 1U;
                g_cal.center_stable_start_ms = g_cal.ms;
            } else if ((uint32_t)(g_cal.ms -
                                  g_cal.center_stable_start_ms) >=
                       BALL_CAL_CENTER_STABLE_MS) {
                Ball_Calibrate_Finish(g_cal.candidate_deg);
            }
        } else {
            g_cal.center_stable = 0U;
        }
        break;
    }
}

uint8_t Ball_Calibrate_IsActive(void)
{
    return ((g_cal.state == BALL_CAL_STATE_WAIT_BALL) ||
            (g_cal.state == BALL_CAL_STATE_RECOVER) ||
            (g_cal.state == BALL_CAL_STATE_SETTLE) ||
            (g_cal.state == BALL_CAL_STATE_OBSERVE) ||
            (g_cal.state == BALL_CAL_STATE_CENTER)) ? 1U : 0U;
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
