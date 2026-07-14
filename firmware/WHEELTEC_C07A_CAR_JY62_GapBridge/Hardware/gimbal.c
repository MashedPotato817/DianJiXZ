#include "board.h"
#include "gimbal.h"

#define GIMBAL_PWM_PERIOD_TICKS      25000U
#define GIMBAL_TICKS_PER_US_NUM      5U
#define GIMBAL_TICKS_PER_US_DEN      4U
#define GIMBAL_PULSE_MIN_US          1000U
#define GIMBAL_PULSE_CENTER_US       1500U
#define GIMBAL_PULSE_MAX_US          2000U
#define GIMBAL_BOOT_TEST_ENABLE      1
#define GIMBAL_GPIO_DEBUG_ENABLE     1

static const DL_TimerA_ClockConfig g_gimbalClockConfig = {
    .clockSel = DL_TIMER_CLOCK_BUSCLK,
    .divideRatio = DL_TIMER_CLOCK_DIVIDE_8,
    .prescale = 7U
};

static const DL_TimerA_PWMConfig g_gimbalPwmConfig = {
    .pwmMode = DL_TIMER_PWM_MODE_EDGE_ALIGN_UP,
    .period = GIMBAL_PWM_PERIOD_TICKS,
    .isTimerWithFourCC = false,
    .startTimer = DL_TIMER_START
};

static uint16_t Gimbal_ClampPulseUs(uint16_t pulse_us)
{
    if (pulse_us < GIMBAL_PULSE_MIN_US) return GIMBAL_PULSE_MIN_US;
    if (pulse_us > GIMBAL_PULSE_MAX_US) return GIMBAL_PULSE_MAX_US;
    return pulse_us;
}

static uint16_t Gimbal_PulseUsToTicks(uint16_t pulse_us)
{
    return (uint16_t)((pulse_us * GIMBAL_TICKS_PER_US_NUM) /
                      GIMBAL_TICKS_PER_US_DEN);
}

void Gimbal_SetYawPulseUs(uint16_t pulse_us)
{
    pulse_us = Gimbal_ClampPulseUs(pulse_us);
    DL_TimerA_setCaptureCompareValue(TIMA0, Gimbal_PulseUsToTicks(pulse_us),
                                     DL_TIMER_CC_0_INDEX);
}

void Gimbal_SetPitchPulseUs(uint16_t pulse_us)
{
    pulse_us = Gimbal_ClampPulseUs(pulse_us);
    DL_TimerA_setCaptureCompareValue(TIMA0, Gimbal_PulseUsToTicks(pulse_us),
                                     DL_TIMER_CC_1_INDEX);
}

void Gimbal_SetYawSpeed(int16_t percent)
{
    int32_t pulse_us;

    if (percent > 100) percent = 100;
    if (percent < -100) percent = -100;

    pulse_us = GIMBAL_PULSE_CENTER_US +
               ((int32_t)percent * (GIMBAL_PULSE_MAX_US - GIMBAL_PULSE_CENTER_US)) / 100;
    Gimbal_SetYawPulseUs((uint16_t)pulse_us);
}

void Gimbal_StopYaw(void)
{
    Gimbal_SetYawPulseUs(GIMBAL_PULSE_CENTER_US);
}

void Gimbal_SetPitchDeg(int16_t degree)
{
    uint32_t pulse_us;

    if (degree < 0) degree = 0;
    if (degree > 180) degree = 180;

    pulse_us = GIMBAL_PULSE_MIN_US +
               ((uint32_t)degree * (GIMBAL_PULSE_MAX_US - GIMBAL_PULSE_MIN_US)) / 180U;
    Gimbal_SetPitchPulseUs((uint16_t)pulse_us);
}

void Gimbal_Init(void)
{
#if GIMBAL_GPIO_DEBUG_ENABLE
    uint8_t i;

    DL_GPIO_initDigitalOutput(IOMUX_PINCM1);
    DL_GPIO_clearPins(GPIOA, DL_GPIO_PIN_0);
    DL_GPIO_enableOutput(GPIOA, DL_GPIO_PIN_0);

    for (i = 0; i < 20U; ++i) {
        DL_GPIO_togglePins(GPIOA, DL_GPIO_PIN_0);
        delay_ms(200);
    }

    DL_GPIO_clearPins(GPIOA, DL_GPIO_PIN_0);
    return;
#endif

    DL_TimerA_reset(TIMA0);
    DL_TimerA_enablePower(TIMA0);
    delay_cycles(POWER_STARTUP_DELAY);

    DL_GPIO_initPeripheralOutputFunction(IOMUX_PINCM1, IOMUX_PINCM1_PF_TIMA0_CCP0);
    DL_GPIO_enableOutput(GPIOA, DL_GPIO_PIN_0);
    DL_GPIO_initPeripheralOutputFunction(IOMUX_PINCM2, IOMUX_PINCM2_PF_TIMA0_CCP1);
    DL_GPIO_enableOutput(GPIOA, DL_GPIO_PIN_1);

    DL_TimerA_setClockConfig(TIMA0, (DL_TimerA_ClockConfig *)&g_gimbalClockConfig);
    DL_TimerA_initPWMMode(TIMA0, (DL_TimerA_PWMConfig *)&g_gimbalPwmConfig);

    DL_TimerA_setCaptureCompareOutCtl(TIMA0, DL_TIMER_CC_OCTL_INIT_VAL_LOW,
        DL_TIMER_CC_OCTL_INV_OUT_DISABLED, DL_TIMER_CC_OCTL_SRC_FUNCVAL,
        DL_TIMERA_CAPTURE_COMPARE_0_INDEX);
    DL_TimerA_setCaptCompUpdateMethod(TIMA0, DL_TIMER_CC_UPDATE_METHOD_IMMEDIATE,
                                      DL_TIMERA_CAPTURE_COMPARE_0_INDEX);

    DL_TimerA_setCaptureCompareOutCtl(TIMA0, DL_TIMER_CC_OCTL_INIT_VAL_LOW,
        DL_TIMER_CC_OCTL_INV_OUT_DISABLED, DL_TIMER_CC_OCTL_SRC_FUNCVAL,
        DL_TIMERA_CAPTURE_COMPARE_1_INDEX);
    DL_TimerA_setCaptCompUpdateMethod(TIMA0, DL_TIMER_CC_UPDATE_METHOD_IMMEDIATE,
                                      DL_TIMERA_CAPTURE_COMPARE_1_INDEX);

    DL_TimerA_enableClock(TIMA0);
    DL_TimerA_setCCPDirection(TIMA0, DL_TIMER_CC0_OUTPUT | DL_TIMER_CC1_OUTPUT);

    Gimbal_StopYaw();
    Gimbal_SetPitchDeg(90);

#if GIMBAL_BOOT_TEST_ENABLE
    delay_ms(300);
    Gimbal_SetYawSpeed(35);
    delay_ms(800);
    Gimbal_StopYaw();
    delay_ms(300);
    Gimbal_SetYawSpeed(-35);
    delay_ms(800);
    Gimbal_StopYaw();
#endif
}
