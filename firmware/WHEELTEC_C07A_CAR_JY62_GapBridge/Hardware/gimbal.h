#ifndef GIMBAL_H
#define GIMBAL_H

#include <stdint.h>

/* PA0 is yaw (TIMA0 CCP0), PA1 is pitch (TIMA0 CCP1). */
void Gimbal_Init(void);

/* Continuous-rotation yaw servo: -100 is full left, 0 stops, +100 is full right. */
void Gimbal_SetYawSpeed(int16_t percent);
void Gimbal_StopYaw(void);

/* Pitch position servo: 0 to 180 degrees. */
void Gimbal_SetPitchDeg(int16_t degree);

/* Direct pulse control for calibrating a specific servo. Valid range: 1000 to 2000 us. */
void Gimbal_SetYawPulseUs(uint16_t pulse_us);
void Gimbal_SetPitchPulseUs(uint16_t pulse_us);

#endif
