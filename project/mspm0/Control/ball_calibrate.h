#ifndef H_BALL_CALIBRATE
#define H_BALL_CALIBRATE

#include <stdint.h>

/*
 * 自动扫掠标定：利用 K230 位置反馈，二分定位“摆杆水平”的舵机平衡角，
 * 消除硬件更换/重新安装后基准值漂移的影响。
 *
 * 物理原理：摆杆不水平时球滚向较低一侧。舵机角度低于平衡角时球滚向一侧，
 * 高于平衡角时滚向另一侧；观测 K230 的 x_mm 滚向哪侧即可缩小区间，
 * 最终收敛到球近乎静止的角度，即当前机构真实平衡角。
 *
 * 触发：按键长按。流程 IDLE→WAIT_BALL→SETTLE→OBSERVE→DONE。
 * 标定期间通过 Ball_Control_SetServoHold 冻结正常闭环，直接命令舵机。
 * 结果通过 Ball_Control_SetTrimAngle 写入 trim 初值并持久化到 Flash。
 *
 * 2026-08-01 重新启用：用户要求用自校准重新标定真实平衡基准（此前 1780us/
 * 122.4° 为人工试凑值，用户记忆中校准基准 PWM 约 1850）。方向逻辑已按实机
 * 确认修正：PWM 偏大→球往负方向滚，PWM 偏小→球往正方向滚。
 * 标定结果经 Ball_Control_SetTrimAngle 生效并写入 Flash，上电自动加载。
 */
#define BALL_CAL_ENABLE 1
#define BALL_CAL_SEARCH_MIN_DEG     (50.0f)
#define BALL_CAL_SEARCH_MAX_DEG     (150.0f)
#define BALL_CAL_CONVERGE_SPAN_DEG  (8.0f)
#define BALL_CAL_MAX_ITERATIONS     (8U)
#define BALL_CAL_SETTLE_MS          (400U)
#define BALL_CAL_OBSERVE_MS         (500U)
#define BALL_CAL_WAIT_BALL_TIMEOUT_MS (3000U)
#define BALL_CAL_TOTAL_TIMEOUT_MS   (20000U)
#define BALL_CAL_TREND_THRESHOLD_MM (2.0f)
#define BALL_CAL_EDGE_X_MM          (55.0f)
#define BALL_CAL_MAX_SAMPLES        (16U)

typedef enum {
    BALL_CAL_STATE_IDLE = 0,
    BALL_CAL_STATE_WAIT_BALL = 1,
    BALL_CAL_STATE_SETTLE = 2,
    BALL_CAL_STATE_OBSERVE = 3,
    BALL_CAL_STATE_DONE = 4
} Ball_CalibrateState;

void Ball_Calibrate_Init(void);
void Ball_Calibrate_Start(void);
/* 5 ms 控制中断内调用，非阻塞推进状态机。 */
void Ball_Calibrate_Tick5ms(void);
/* 标定激活期间为 1，供调用方跳过正常闭环。 */
uint8_t Ball_Calibrate_IsActive(void);
float Ball_Calibrate_GetResultDeg(void);
Ball_CalibrateState Ball_Calibrate_GetState(void);

#endif
