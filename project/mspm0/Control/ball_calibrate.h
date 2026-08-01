#ifndef H_BALL_CALIBRATE
#define H_BALL_CALIBRATE

#include <stdint.h>

/*
 * 自动扫掠标定：利用 K230 位置反馈，自适应夹逼“摆杆水平”的舵机平衡角，
 * 消除硬件更换/重新安装后基准值漂移的影响。
 *
 * 物理原理：摆杆不水平时球滚向较低一侧。舵机角度低于平衡角时球滚向一侧，
 * 高于平衡角时滚向另一侧；观测 K230 的 x_mm 滚向哪侧即可缩小区间，
 * 最终收敛到球近乎静止的角度，即当前机构真实平衡角。
 *
 * 触发：按键长按。流程先验证低/高PWM的相反滚动方向，再按实测位移趋势
 * 加权搜索并在候选点两侧主动探测；只有双向证据和候选点复核均通过才
 * 进入 DONE。
 * 标定期间通过 Ball_Control_SetServoHold 冻结正常闭环，直接命令舵机。
 * 结果通过 Ball_Control_SetTrimAngle 写入本次上电期间使用的 trim。
 *
 * 2026-08-01 当前固定上电基准更新为 1701us/108.1°。方向逻辑已按实机
 * 确认修正：PWM 偏大→球往负方向滚，PWM 偏小→球往正方向滚。
 * 标定完成后在主循环写入保留的 Flash 扇区，下次复位自动加载。
 */
#define BALL_CAL_ENABLE 1
/*
 * 标定结果写 Flash 持久化的开关。
 * 写入在主循环执行并读回验证，避免在 5ms 中断中阻塞 K230 接收。
 */
#define BALL_CAL_SAVE_ENABLE 1
/* 上电优先加载已通过 magic 和角度范围校验的标定结果。 */
#define BALL_CAL_LOAD_ENABLE 1
/*
 * 不预设平衡角：先在舵机完整脉宽范围建立正、反滚动证据，再夹逼平衡点。
 * 1400~1900us 的可能平衡位置被完整覆盖，同时允许机构重装后的更大漂移。
 */
#define BALL_CAL_SEARCH_MIN_US       (1100U)
#define BALL_CAL_SEARCH_MAX_US       (2100U)
#define BALL_CAL_CONVERGE_SPAN_US    (5U)
#define BALL_CAL_VERIFY_PROBE_US     (30U)
#define BALL_CAL_MAX_ITERATIONS      (20U)
#define BALL_CAL_SETTLE_MS          (400U)
#define BALL_CAL_OBSERVE_MS         (600U)
#define BALL_CAL_TRIAL_TIMEOUT_MS   (4000U)
#define BALL_CAL_RECOVER_TIMEOUT_MS (6000U)
#define BALL_CAL_RECOVER_STEP_MS    (250U)
#define BALL_CAL_RECOVER_START_US   (20U)
#define BALL_CAL_RECOVER_ADD_US     (10U)
#define BALL_CAL_RECOVER_MAX_STEP_US (100U)
#define BALL_CAL_WAIT_BALL_TIMEOUT_MS (3000U)
#define BALL_CAL_TOTAL_TIMEOUT_MS   (60000U)
#define BALL_CAL_TREND_THRESHOLD_MM (2.0f)
#define BALL_CAL_TREND_SCORE_MAX_MM (40.0f)
#define BALL_CAL_NEXT_MIN_RATIO     (0.30f)
#define BALL_CAL_NEXT_MAX_RATIO     (0.70f)
#define BALL_CAL_EDGE_X_MM          (55.0f)
#define BALL_CAL_EDGE_CONFIRM_FRAMES (3U)
#define BALL_CAL_MAX_SAMPLES        (16U)
#define BALL_CAL_MIN_SAMPLES        (6U)

typedef enum {
    BALL_CAL_STATE_IDLE = 0,
    BALL_CAL_STATE_WAIT_BALL = 1,
    BALL_CAL_STATE_RECOVER = 2,
    BALL_CAL_STATE_SETTLE = 3,
    BALL_CAL_STATE_OBSERVE = 4,
    BALL_CAL_STATE_DONE = 5,
    BALL_CAL_STATE_FAILED = 6,
    BALL_CAL_STATE_SAVE_FAILED = 7
} Ball_CalibrateState;

void Ball_Calibrate_Init(void);
void Ball_Calibrate_Start(void);
/* 5 ms 控制中断内调用，非阻塞推进状态机。 */
void Ball_Calibrate_Tick5ms(void);
/* 标定激活期间为 1，供调用方跳过正常闭环。 */
uint8_t Ball_Calibrate_IsActive(void);
/* 主循环调用：标定完成待写 Flash 时执行持久化，成功返回 1。 */
uint8_t Ball_Calibrate_ProcessSave(void);
float Ball_Calibrate_GetResultDeg(void);
Ball_CalibrateState Ball_Calibrate_GetState(void);

#endif
