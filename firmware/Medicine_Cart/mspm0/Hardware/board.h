#ifndef _BOARD_H_
#define _BOARD_H_
#include "stdio.h"
#include "string.h"
#include "ti_msp_dl_config.h"
#include "oled.h"
#include "led.h"
#include "key.h"
#include "motor.h"
#include "encoder.h"
#include "adc.h"
#include "control.h"

/* 送药车专用模块 */
#include "k230_link.h"
#include "line_follow.h"
#include "medicine_task.h"
#include "route.h"
#include "load_detect.h"
#include "indicator.h"

#define ABS(a)      (a>0 ? a:(-a))
typedef int32_t  s32;
typedef int16_t s16;
typedef int8_t  s8;

typedef const int32_t sc32;
typedef const int16_t sc16;
typedef const int8_t sc8;

typedef __IO int32_t  vs32;
typedef __IO int16_t  vs16;
typedef __IO int8_t   vs8;

typedef __I int32_t vsc32;
typedef __I int16_t vsc16;
typedef __I int8_t vsc8;

typedef uint32_t  u32;
typedef uint16_t u16;
typedef uint8_t  u8;

typedef const uint32_t uc32;
typedef const uint16_t uc16;
typedef const uint8_t uc8;

typedef __IO uint32_t  vu32;
typedef __IO uint16_t vu16;
typedef __IO uint8_t  vu8;

typedef __I uint32_t vuc32;
typedef __I uint16_t vuc16;
typedef __I uint8_t vuc8;

typedef enum {
    Mec_Car = 0,
    Omni_Car,
    Akm_Car,
    Diff_Car,
    FourWheel_Car,
    Tank_Car
} CarMode;

extern u8 Car_Mode;
extern u8 Flag_Stop;
extern float Voltage;
extern float Move_X, Move_Z;
extern int Motor_Left, Motor_Right;
extern float Velocity_Left, Velocity_Right;

#define SysTickMAX_COUNT 0xFFFFFF
#define SysTickFre 80000000
#define SysTick_MS(x)  ((SysTickFre/1000U)*(uint32_t)(x))
#define SysTick_US(x)  ((SysTickFre/1000000U)*(uint32_t)(x))

uint32_t Systick_getTick(void);
void delay_ms(uint32_t ms);
void delay_us(uint32_t us);
void delay_1us(unsigned long __us);
void delay_1ms(unsigned long ms);
#endif
