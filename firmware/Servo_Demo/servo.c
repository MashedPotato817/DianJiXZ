#include "servo.h"
#include "ti_msp_dl_config.h"

/* ============================================================================
 * S20C 180° 舵机驱动 — TIMA0 PWM @ 50Hz
 *
 * 引脚: PA8 (IOMUX_PINCM19) → TIMA0_CCP0
 * 时钟: 80MHz / 8 / (7+1) = 1.25MHz, 周期 25000 ticks = 20ms = 50Hz
 *
 * 脉宽:
 *    600us =  750 ticks →   0°
 *   1500us = 1875 ticks →  90°
 *   2400us = 3000 ticks → 180°
 *
 * 机械:
 *   舵机 90° → 丝杆 25mm
 *   合页距丝杆触点 250mm → 平台倾角 = atan(h/250)
 * ============================================================================
 */

/* ---- 硬件映射 (换引脚只改这里) ---- */
#define SERVO_PORT              GPIOA
#define SERVO_PIN               DL_GPIO_PIN_8
#define SERVO_IOMUX             IOMUX_PINCM19
#define SERVO_IOMUX_FUNC        IOMUX_PINCM19_PF_TIMA0_CCP0
#define SERVO_TIMER             TIMA0
#define SERVO_CC_INDEX          DL_TIMER_CC_0_INDEX
#define SERVO_CC_OCTL_INDEX     DL_TIMERA_CAPTURE_COMPARE_0_INDEX

/* ---- PWM 参数 ---- */
#define SERVO_PERIOD_TICKS      25000U
#define SERVO_TICKS_PER_US_NUM  5U
#define SERVO_TICKS_PER_US_DEN  4U

static uint8_t g_servo_angle = 90;

/* ================================================================ */

static uint16_t pulse_us_to_ticks(uint16_t pulse_us)
{
    if (pulse_us < SERVO_PULSE_MIN_US) pulse_us = SERVO_PULSE_MIN_US;
    if (pulse_us > SERVO_PULSE_MAX_US) pulse_us = SERVO_PULSE_MAX_US;
    return (uint16_t)((pulse_us * SERVO_TICKS_PER_US_NUM) /
                       SERVO_TICKS_PER_US_DEN);
}

static uint16_t angle_to_pulse_us(uint8_t degree)
{
    if (degree > SERVO_ANGLE_MAX) degree = SERVO_ANGLE_MAX;
    return (uint16_t)(SERVO_PULSE_MIN_US +
        ((uint32_t)degree *
         (SERVO_PULSE_MAX_US - SERVO_PULSE_MIN_US)) / SERVO_ANGLE_MAX);
}

/* ================================================================ */

void Servo_SetPulseUs(uint16_t pulse_us)
{
    DL_TimerA_setCaptureCompareValue(SERVO_TIMER,
        pulse_us_to_ticks(pulse_us), SERVO_CC_INDEX);
}

void Servo_SetAngle(uint8_t degree)
{
    if (degree > SERVO_ANGLE_MAX) degree = SERVO_ANGLE_MAX;
    g_servo_angle = degree;
    DL_TimerA_setCaptureCompareValue(SERVO_TIMER,
        pulse_us_to_ticks(angle_to_pulse_us(degree)), SERVO_CC_INDEX);
}

void Servo_SetAngleFloat(float degree)
{
    if (degree < 0.0f) degree = 0.0f;
    if (degree > (float)SERVO_ANGLE_MAX) degree = (float)SERVO_ANGLE_MAX;
    Servo_SetAngle((uint8_t)(degree + 0.5f));
}

void Servo_SetHeight_mm(uint8_t height_mm)
{
    Servo_SetAngleFloat(SERVO_MM_TO_ANGLE((float)height_mm));
}

/* 通过目标平台倾角 (°) 设置舵机 */
void Servo_SetPlatformAngle(float plat_deg)
{
    /* 1. 计算所需丝杆高度: h = tan(plat_deg) * 250mm
     * 2. 高度 → 舵机角度 */
    float height_mm = tanf(plat_deg * 0.0174533f) * SERVO_HINGE_DISTANCE_MM;
    Servo_SetAngleFloat(SERVO_MM_TO_ANGLE(height_mm));
}

uint8_t Servo_GetAngle(void)
{
    return g_servo_angle;
}

/* 获取当前平台倾角 (°) */
float Servo_GetPlatformAngle(void)
{
    return SERVO_PLATFORM_ANGLE_DEG(g_servo_angle);
}

/* ================================================================ */

void Servo_Init(void)
{
    /* 1. 复位并使能 TIMA0 */
    DL_TimerA_reset(SERVO_TIMER);
    DL_TimerA_enablePower(SERVO_TIMER);
    delay_cycles(POWER_STARTUP_DELAY);

    /* 2. PA8 → TIMA0_CCP0 */
    DL_GPIO_initPeripheralOutputFunction(SERVO_IOMUX, SERVO_IOMUX_FUNC);
    DL_GPIO_enableOutput(SERVO_PORT, SERVO_PIN);

    /* 3. 时钟: 80MHz / 8 / 8 = 1.25MHz */
    static const DL_TimerA_ClockConfig clockCfg = {
        .clockSel    = DL_TIMER_CLOCK_BUSCLK,
        .divideRatio = DL_TIMER_CLOCK_DIVIDE_8,
        .prescale    = 7U
    };
    DL_TimerA_setClockConfig(SERVO_TIMER,
        (DL_TimerA_ClockConfig *)&clockCfg);

    /* 4. PWM: 边沿对齐, 25000 ticks → 20ms → 50Hz */
    static const DL_TimerA_PWMConfig pwmCfg = {
        .pwmMode           = DL_TIMER_PWM_MODE_EDGE_ALIGN_UP,
        .period            = SERVO_PERIOD_TICKS,
        .isTimerWithFourCC = false,
        .startTimer        = DL_TIMER_START
    };
    DL_TimerA_initPWMMode(SERVO_TIMER,
        (DL_TimerA_PWMConfig *)&pwmCfg);

    /* 5. 通道输出: 初始低、不反相、函数值源、立即更新 */
    DL_TimerA_setCaptureCompareOutCtl(SERVO_TIMER,
        DL_TIMER_CC_OCTL_INIT_VAL_LOW,
        DL_TIMER_CC_OCTL_INV_OUT_DISABLED,
        DL_TIMER_CC_OCTL_SRC_FUNCVAL,
        SERVO_CC_OCTL_INDEX);

    DL_TimerA_setCaptCompUpdateMethod(SERVO_TIMER,
        DL_TIMER_CC_UPDATE_METHOD_IMMEDIATE,
        SERVO_CC_OCTL_INDEX);

    /* 6. 启动 */
    DL_TimerA_enableClock(SERVO_TIMER);
    DL_TimerA_setCCPDirection(SERVO_TIMER, DL_TIMER_CC0_OUTPUT);

    /* 7. 默认中位: 舵机 90°, 丝杆 25mm, 平台 5.71° */
    Servo_SetAngle(90);
}
