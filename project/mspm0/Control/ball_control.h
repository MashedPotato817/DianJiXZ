#ifndef H_BALL_CONTROL
#define H_BALL_CONTROL

#include "k230_link.h"

typedef enum {
    BALL_CONTROL_PHASE_OFF = 0,
    BALL_CONTROL_PHASE_CAPTURE = 1,
    BALL_CONTROL_PHASE_ACCEL = 2,
    BALL_CONTROL_PHASE_RUN = 3,
    BALL_CONTROL_PHASE_BRAKE = 4,
    BALL_CONTROL_PHASE_EDGE = 5,
    BALL_CONTROL_PHASE_FAULT = 6,
    BALL_CONTROL_PHASE_HOLD = 7,
    BALL_CONTROL_PHASE_LOST = 8,
    BALL_CONTROL_PHASE_PASS = 9
} Ball_ControlPhase;

typedef struct {
    float kp;
    float kd;
    float target_x_mm;
    float last_error_mm;
    float filtered_velocity_mm_s;
    float toward_velocity_mm_s;
    float stopping_distance_mm;
    float integral_deg;                 /* 条件积分保持角：目标附近低速时累积，把球吸在目标点 */
    float trim_angle_deg;
    float last_command_angle_deg;
    float last_output_deg;
    float pass_target_abs_mm;
    float accel_scale;
    uint32_t last_timestamp_ms;
    uint32_t last_valid_control_ms;
    uint32_t control_now_ms;
    uint32_t phase_start_ms;
    uint32_t edge_recovery_start_ms;
    uint8_t enabled;
    uint8_t has_last_sample;
    uint8_t edge_recovery_timed_out;
    uint8_t servo_hold;
    uint8_t velocity_confirm_frames;
    uint8_t accel_level;
    int8_t motion_direction;
    int8_t correction_start_direction;
    Ball_ControlPhase phase;
    Ball_ControlPhase phase_before_hold;
} Ball_Control;

#define BALL_CONTROL_ENABLE_DEFAULT   (1U)
#define BALL_CONTROL_KP_DEG_PER_MM          (0.24f)   //Kp
#define BALL_CONTROL_KD_DEG_S_PER_MM        (0.040f)  //Kd
#define BALL_CONTROL_TARGET_TOLERANCE_MM    (2.0f)

/*
 * 条件积分（把球"吸"在目标点，抗管子斜坡/静摩擦）：
 * 目标附近（|error|≤BAND）且低速（|v|≤VEL）时累积位置误差，输出保持角。
 * 离开 BAND 清零；飞行中（高速）不积分但保留值。解决 PD 在目标点输出归 0、
 * 球到 -50 后被斜坡弹回的问题。
 */
#define BALL_CONTROL_INTEGRAL_BAND_MM        (12.0f)
#define BALL_CONTROL_INTEGRAL_VELOCITY_MM_S  (12.0f)
#define BALL_CONTROL_INTEGRAL_KI_DEG_PER_MM_S (0.30f)
#define BALL_CONTROL_INTEGRAL_MAX_DEG         (6.0f)
#define BALL_CONTROL_VELOCITY_FILTER_ALPHA  (0.45f)
#define BALL_CONTROL_VELOCITY_CONFIRM_MM_S (12.0f)
#define BALL_CONTROL_VELOCITY_STALL_MM_S    (8.0f)
#define BALL_CONTROL_VELOCITY_STOP_MM_S     (8.0f)
#define BALL_CONTROL_VELOCITY_CONFIRM_FRAMES (1U)
#define BALL_CONTROL_MAX_SAMPLE_PERIOD_MS  (250U)

/* 非目标静止时连续施力，不在档位之间回中。 */
/* 管子表面有凹陷（实测确认），起球需更大/更久的推力滚出坑沿。 */
#define BALL_CONTROL_ACCEL_FIRST_DEG        (24.0f)
#define BALL_CONTROL_ACCEL_STEP_DEG          (4.0f)
#define BALL_CONTROL_ACCEL_MAX_DEG          (36.0f)
#define BALL_CONTROL_ACCEL_STEP_MS          (300U)
#define BALL_CONTROL_ACCEL_TIMEOUT_MS      (2000U)

/*
 * 000006~000008 三组实测按进入ACC时的误差分桶统计：过冲幅度随进入误差
 * 近似线性增长（例如000007中<5mm入口过冲中位数4.5mm，20~40mm入口过冲
 * 中位数31.2mm）。说明固定20°首级起动力对贴近容差边缘的小误差过量，
 * 把球推过中心后又需要新一轮ACC。按误差比例缩放整条ACC坡度，
 * 误差≤2mm时用50%力度，≥20mm时保留原100%力度，二者间线性过渡。
 * 50%下限缺乏台架静摩擦实测数据，只是保守假设；若脱困成功率下降，
 * 需回到实测重新调整该下限。
 */
#define BALL_CONTROL_ACCEL_SCALE_MIN_ERROR_MM (2.0f)
#define BALL_CONTROL_ACCEL_SCALE_FULL_ERROR_MM (20.0f)
#define BALL_CONTROL_ACCEL_SCALE_MIN          (0.5f)

/*
 * 首帧确认回中运动后立即撤掉脱困强推力。
 * RUN 不设固定同向下限，只保留位置/速度 PD；若中心外再次停住，
 * 由 STALL 判断重新进入 ACC，而不是持续给球增加动能。
 */
#define BALL_CONTROL_RUN_MIN_DEG             (0.0f)
#define BALL_CONTROL_BRAKE_MIN_DEG           (6.0f)
#define BALL_CONTROL_CAPTURE_DAMP_KD_DEG_S_PER_MM (0.08f)
#define BALL_CONTROL_CAPTURE_DAMP_MIN_DEG    (2.0f)
#define BALL_CONTROL_CAPTURE_DAMP_MAX_DEG    (8.0f)
#define BALL_CONTROL_BRAKE_ACCEL_MM_S2     (250.0f)
#define BALL_CONTROL_BRAKE_MARGIN_MM         (2.0f)
#define BALL_CONTROL_NORMAL_MAX_DEG         (20.0f)

/*
 * 首次过零后不立即反向 ACC，而是先用有限阻尼完成一次受控过零。
 * 目标反侧距离取本轮起始误差的 20%，并限制在 3~8 mm。
 */
#define BALL_CONTROL_PASS_TARGET_RATIO        (0.20f)
#define BALL_CONTROL_PASS_TARGET_MIN_MM       (3.0f)
#define BALL_CONTROL_PASS_TARGET_MAX_MM       (8.0f)
#define BALL_CONTROL_PASS_DAMP_KD_DEG_S_PER_MM (0.07f)
#define BALL_CONTROL_PASS_DAMP_MIN_DEG        (2.0f)
#define BALL_CONTROL_PASS_DAMP_SOFT_MAX_DEG   (4.0f)
#define BALL_CONTROL_PASS_DAMP_HARD_MAX_DEG   (8.0f)
#define BALL_CONTROL_PASS_MIN_HOLD_MS       (150U)

/*
 * PWM 1660 us（=100.8°）是 2026-08-01 确认的当前机构平衡基准。
 * trim 在线学习已关闭（KI=0），避免运行中再次漂离基准；自动扫掠标定
 * 或 UART 调参仍可在本次上电期间更新 trim。Flash 旧值不在上电时加载。
 */
#define BALL_CONTROL_TRIM_INITIAL_DEG        (100.8f)
#define BALL_CONTROL_TRIM_MIN_DEG            (15.0f)
#define BALL_CONTROL_TRIM_MAX_DEG           (165.0f)
#define BALL_CONTROL_TRIM_KI_DEG_PER_MM_S     (0.00f)
#define BALL_CONTROL_TRIM_LEARN_ERROR_MM      (4.0f)
#define BALL_CONTROL_TRIM_LEARN_VELOCITY_MM_S (8.0f)
#define BALL_CONTROL_TRIM_MAX_STEP_DEG        (0.03f)

/* 链路自身100ms超时后再容忍到总计180ms，期间保持上一安全输出。 */
#define BALL_CONTROL_LOST_HOLD_TOTAL_MS    (180U)
/* 相对trim偏移；中位100.8°时可用上限为74.2°（175-100.8），取50°留余量 */
#define BALL_CONTROL_EDGE_RECOVERY_OFFSET_DEG (50.0f)
#define BALL_CONTROL_EDGE_RECOVERY_TIMEOUT_MS (1000U)

void Ball_Control_Init(void);
void Ball_Control_SetTarget(float target_x_mm);
void Ball_Control_SetEnabled(uint8_t enabled);
uint8_t Ball_Control_IsEnabled(void);
/* 自动标定期间置1：Step 完全返回，不 Reset 也不写舵机。 */
void Ball_Control_SetServoHold(uint8_t hold);
/* 自动标定结束调用：把 trim 设为标定结果并复位运动状态。 */
void Ball_Control_SetTrimAngle(float angle_deg);
/* 运行时调参（UART0 命令接口）：仅更新增益，不复位运动状态；0 表示不改。 */
void Ball_Control_SetGains(float kp, float kd);
void Ball_Control_Reset(void);
void Ball_Control_Step(const K230_BallPosition *position, float period_s);
const Ball_Control *Ball_Control_Get(void);

#endif
