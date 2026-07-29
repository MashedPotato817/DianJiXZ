#ifndef SERVO_H
#define SERVO_H

#include <stdint.h>
#include <math.h>

/* ============================================================================
 * S20C 180° 舵机驱动模块
 *
 * 硬件: PA8 → TIMA0_CCP0, 50Hz PWM, 600~2400us
 * 机械: 舵机→齿轮减速→丝杆顶升, 90°=25mm
 * 平台: 丝杆触点距合页铰链 250mm, 平台倾角 = atan(h/250)
 *
 * 依赖: ti_msp_dl_config.h (提供 DriverLib API + POWER_STARTUP_DELAY)
 * ============================================================================
 */

/* ---- 可调参数 ---- */
#define SERVO_PULSE_MIN_US      600U
#define SERVO_PULSE_MAX_US      2400U
#define SERVO_PULSE_CENTER_US   ((SERVO_PULSE_MIN_US + SERVO_PULSE_MAX_US) / 2U)
#define SERVO_ANGLE_MIN          0
#define SERVO_ANGLE_MAX          180

/* ---- 丝杆机械 (90° = 25mm) ---- */
#define SERVO_STROKE_90DEG_MM    25.0f
#define SERVO_ANGLE_TO_MM(deg)   ((float)(deg) * SERVO_STROKE_90DEG_MM / 90.0f)
#define SERVO_MM_TO_ANGLE(mm)    ((float)(mm) * 90.0f / SERVO_STROKE_90DEG_MM)

/* ---- 平台几何 (合页距丝杆触点 250mm) ---- */
#define SERVO_HINGE_DISTANCE_MM  250.0f

/* 舵机角度 → 平台倾角 (°) */
#define SERVO_PLATFORM_ANGLE_DEG(deg) \
    (atanf(SERVO_ANGLE_TO_MM(deg) / SERVO_HINGE_DISTANCE_MM) * 57.29578f)

/* 平台倾角 (°) → 舵机角度 */
#define SERVO_ANGLE_FROM_PLATFORM_DEG(plat_deg) \
    ((uint8_t)(SERVO_MM_TO_ANGLE(tanf((plat_deg) * 0.0174533f) * SERVO_HINGE_DISTANCE_MM) + 0.5f))

/* ---- API ---- */
void     Servo_Init(void);
void     Servo_SetAngle(uint8_t degree);
void     Servo_SetAngleFloat(float degree);
void     Servo_SetHeight_mm(uint8_t height_mm);
void     Servo_SetPlatformAngle(float plat_deg);
void     Servo_SetPulseUs(uint16_t pulse_us);
uint8_t  Servo_GetAngle(void);
float    Servo_GetPlatformAngle(void);

#endif
