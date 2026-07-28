#ifndef __LINE_FOLLOW_H
#define __LINE_FOLLOW_H

#include <stdint.h>

/* ============================================================
 * 8 路灰度巡线 — 非线性加权 PD 控制器
 *
 * 核心思路：
 *   外侧传感器（1、8 号灯）权重高 → 大幅偏移时强力拉回
 *   内侧传感器（4、5 号灯）权重低 → 微小偏移时轻柔修正
 *   死区内强制直走 → 直线稳定性
 *   双档 KP → 小偏差柔、大偏差刚
 *   弯道自动降速 → 过弯不甩尾
 * ============================================================ */

/* ---- 非线性权重位置表（单位：抽象位置值，非 mm）---- */
/* 外侧:内侧权重比 = 9:1，比线性权重 {-7,-5,-3,-1,1,3,5,7} 更激进 */
extern const float Gray_Weighted_Pos[8];

/* ---- PD 控制参数 ---- */
#define LF_PD_KP_SOFT   0.008f   /* 小偏差比例增益 (|error| < LF_MID_THRESH) */
#define LF_PD_KP_HARD   0.020f   /* 大偏差比例增益 (|error| >= LF_MID_THRESH) */
#define LF_PD_KD        0.004f   /* 微分增益，抑制振荡 */

/* ---- 阈值 ---- */
#define LF_STRAIGHT_DEADBAND  1.5f   /* |error| < 此值 → 强制直走 */
#define LF_MID_THRESH         4.0f   /* |error| >= 此值 → 中等弯，硬 KP */
#define LF_HARD_THRESH        6.5f   /* |error| >= 此值 → 急弯，硬 KP + 最低速 */

/* ---- 分档目标速度 (m/s) ---- */
/* 当前为保守值，验证后可参考对标组提升至 0.240 / 0.212 / 0.178 */
#define LF_SPEED_STRAIGHT  0.150f   /* 直道/微调 */
#define LF_SPEED_MID       0.120f   /* 中等弯道 */
#define LF_SPEED_HARD      0.090f   /* 急弯 */
#define LF_WHEEL_TARGET_MAX 0.200f  /* 单轮目标速度上限 */

/* ---- 丢线搜线 ---- */
#define LF_LOST_DELAY_TICKS    2U     /* 丢线后保持直走的 tick 数 */
#define LF_LOST_SEARCH_SPEED   0.105f /* 搜线基准速度 */
#define LF_LOST_DIFF_START     0.070f /* 搜线初始差速 */
#define LF_LOST_DIFF_END       0.110f /* 搜线最大差速 */
#define LF_LOST_RAMP_TICKS     4U     /* 差速爬坡 tick 数 */
#define LF_LOST_RECOVER_TICKS  2U     /* 找回线后维持转向的 tick 数 */

/* ---- 十字路口 ---- */
#define LF_CROSS_MIN_COUNT    4U     /* 同时看到 >= 此数黑灯 → 视为十字路口 */
#define LF_CROSS_HOLD_TICKS   8U     /* 十字路口直走保持 tick 数 */

/* ---- 输出 ---- */
typedef struct {
    float target_left;   /* 左轮目标速度 (m/s) */
    float target_right;  /* 右轮目标速度 (m/s) */
    float error;         /* 当前加权误差（调试/OLED 显示用） */
    uint8_t is_lost;     /* 是否处于丢线状态 */
} LineFollow_Result;

/* ---- API ---- */
void LineFollow_Update(const uint16_t gray_data[8], LineFollow_Result *result);
void LineFollow_Reset(void);

#endif /* __LINE_FOLLOW_H */
