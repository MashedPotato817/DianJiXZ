#include "line_follow.h"

/* ============================================================
 * 非线性加权位置表
 *
 * 设计原则：外侧传感器权重高、内侧权重低
 *   传感器 0,7（最外侧）→ ±9.0 → 大幅偏移时强力拉回
 *   传感器 1,6           → ±5.0
 *   传感器 2,5           → ±2.5
 *   传感器 3,4（中间）    → ±1.0 → 微小偏移时轻柔修正
 *
 * 外侧:内侧权重比 = 9:1（线性方案只有 7:1）
 * ============================================================ */
const float Gray_Weighted_Pos[8] = {
    -9.0f, -5.0f, -2.5f, -1.0f,
     1.0f,  2.5f,  5.0f,  9.0f
};

/* 中心传感器（sensor 3/4）位掩码，用于"居中直走"判定 */
#define CENTER_LEFT_MASK   (1U << 3)
#define CENTER_RIGHT_MASK  (1U << 4)
#define CENTER_MASK        (CENTER_LEFT_MASK | CENTER_RIGHT_MASK)

/* ---- 静态状态 ---- */
static float  g_last_error;
static int8_t g_last_dir;          /* -1=上次偏左, 1=上次偏右 */
static int8_t g_lost_search_dir;   /* 初始搜线方向: -1=右转搜线 */
static uint8_t g_lost_active;
static uint8_t g_lost_delay_ticks;
static uint8_t g_lost_recover_ticks;
static uint8_t g_lost_ramp_ticks;
static uint8_t g_cross_hold_ticks;

/* ---- 内部辅助 ---- */
static float LimitFloat(float val, float min, float max)
{
    if (val > max) return max;
    if (val < min) return min;
    return val;
}

void LineFollow_Reset(void)
{
    g_last_error        = 0.0f;
    g_last_dir          = 0;
    g_lost_search_dir   = -1;   /* 默认右转搜线 */
    g_lost_active       = 0;
    g_lost_delay_ticks  = 0;
    g_lost_recover_ticks = 0;
    g_lost_ramp_ticks   = 0;
    g_cross_hold_ticks  = 0;
}

void LineFollow_Update(const uint16_t gray_data[8], LineFollow_Result *result)
{
    uint8_t i;
    uint8_t raw;
    uint8_t count;
    float   weighted_sum;
    float   error;
    float   abs_error;
    float   error_delta;
    float   steer;
    float   base_speed;
    float   lost_diff;
    int8_t  search_dir;

    /* ---- 1. 传感器采样 → 位掩码 + 加权求和 ---- */
    raw          = 0;
    count        = 0;
    weighted_sum = 0.0f;

    for (i = 0; i < 8; i++) {
        if (gray_data[i]) {
            raw |= (uint8_t)(1U << i);
            count++;
            weighted_sum += Gray_Weighted_Pos[i];
        }
    }

    /* ---- 2. 丢线：延迟确认 → 沿最后方向搜线 ---- */
    if (count == 0) {
        if (!g_lost_active) {
            g_lost_active        = 1;
            g_lost_recover_ticks = 0;
            g_lost_ramp_ticks    = 0;
        }

        /* 延迟几 tick 保持直走，过滤瞬时丢线 */
        if (g_lost_delay_ticks < LF_LOST_DELAY_TICKS) {
            g_lost_delay_ticks++;
            goto out_straight_lost;
        }

        /* 差速爬坡：从低差速逐步增大，避免原地急转 */
        if (g_lost_ramp_ticks < LF_LOST_RAMP_TICKS) {
            lost_diff = LF_LOST_DIFF_START
                + (LF_LOST_DIFF_END - LF_LOST_DIFF_START)
                * ((float)g_lost_ramp_ticks / (float)LF_LOST_RAMP_TICKS);
            g_lost_ramp_ticks++;
        } else {
            lost_diff = LF_LOST_DIFF_END;
        }

        search_dir = (g_last_dir != 0) ? g_last_dir : g_lost_search_dir;
        if (search_dir > 0) {
            result->target_left  = LF_LOST_SEARCH_SPEED + lost_diff;
            result->target_right = LF_LOST_SEARCH_SPEED - lost_diff;
        } else {
            result->target_left  = LF_LOST_SEARCH_SPEED - lost_diff;
            result->target_right = LF_LOST_SEARCH_SPEED + lost_diff;
        }
        result->error   = 0.0f;
        result->is_lost = 1;
        return;
    }

    /* ---- 3. 十字路口：多灯同时亮 → 直走通过 ---- */
    if (count >= LF_CROSS_MIN_COUNT || g_cross_hold_ticks > 0) {
        if (count >= LF_CROSS_MIN_COUNT) {
            g_cross_hold_ticks = LF_CROSS_HOLD_TICKS;
        } else {
            g_cross_hold_ticks--;
        }
        g_last_error = 0.0f;
        goto out_straight;
    }

    /* ---- 4. 丢线恢复：找回线后继续按原方向转几 tick，确保稳定 ---- */
    if (g_lost_active) {
        if (g_lost_recover_ticks < LF_LOST_RECOVER_TICKS) {
            g_lost_recover_ticks++;
            search_dir = (g_last_dir != 0) ? g_last_dir : g_lost_search_dir;
            if (search_dir > 0) {
                result->target_left  = LF_LOST_SEARCH_SPEED + LF_LOST_DIFF_START;
                result->target_right = LF_LOST_SEARCH_SPEED - LF_LOST_DIFF_START;
            } else {
                result->target_left  = LF_LOST_SEARCH_SPEED - LF_LOST_DIFF_START;
                result->target_right = LF_LOST_SEARCH_SPEED + LF_LOST_DIFF_START;
            }
            result->error   = 0.0f;
            result->is_lost = 1;
            return;
        }
        g_lost_active        = 0;
        g_lost_recover_ticks = 0;
        g_lost_ramp_ticks    = 0;
    }
    g_lost_delay_ticks = 0;

    /* ---- 5. 中心灯检测：只有 sensor 3/4 亮 → 居中直走 ---- */
    if ((raw & ~CENTER_MASK) == 0) {
        g_last_error = 0.0f;
        goto out_straight;
    }

    /* ---- 6. 加权误差 ---- */
    error     = weighted_sum / (float)count;
    abs_error = (error >= 0.0f) ? error : -error;

    /* 记录偏线方向（仅当超出死区时更新，用于丢线搜线） */
    if (abs_error > LF_STRAIGHT_DEADBAND) {
        g_last_dir        = (error < 0.0f) ? -1 : 1;
        g_lost_search_dir = g_last_dir;
    }

    /* ---- 7. 死区：微小偏移 → 直走（直线稳定性核心）---- */
    if (abs_error <= LF_STRAIGHT_DEADBAND) {
        g_last_error = 0.0f;
        result->target_left  = LF_SPEED_STRAIGHT;
        result->target_right = LF_SPEED_STRAIGHT;
        result->error   = error;
        result->is_lost = 0;
        return;
    }

    /* ---- 8. PD 控制器（双档 KP）---- */
    error_delta = error - g_last_error;
    g_last_error = error;

    if (abs_error < LF_MID_THRESH) {
        /* 小偏差 — 软 KP，轻柔修正 */
        steer = LF_PD_KP_SOFT * error + LF_PD_KD * error_delta;
    } else {
        /* 大偏差 — 硬 KP，强力拉回 */
        steer = LF_PD_KP_HARD * error + LF_PD_KD * error_delta;
    }

    /* ---- 9. 分档降速（弯道越急、速度越低）---- */
    if (abs_error >= LF_HARD_THRESH) {
        base_speed = LF_SPEED_HARD;
    } else if (abs_error >= LF_MID_THRESH) {
        base_speed = LF_SPEED_MID;
    } else {
        base_speed = LF_SPEED_STRAIGHT;
    }

    /* ---- 10. 输出 ---- */
    result->target_left  = LimitFloat(base_speed + steer,
                              -LF_WHEEL_TARGET_MAX, LF_WHEEL_TARGET_MAX);
    result->target_right = LimitFloat(base_speed - steer,
                              -LF_WHEEL_TARGET_MAX, LF_WHEEL_TARGET_MAX);
    result->error   = error;
    result->is_lost = 0;
    return;

    /* ---- 共用出口 ---- */
out_straight:
    result->target_left  = LF_SPEED_STRAIGHT;
    result->target_right = LF_SPEED_STRAIGHT;
    result->error   = 0.0f;
    result->is_lost = 0;
    return;

out_straight_lost:
    result->target_left  = LF_SPEED_STRAIGHT;
    result->target_right = LF_SPEED_STRAIGHT;
    result->error   = 0.0f;
    result->is_lost = 1;
}
