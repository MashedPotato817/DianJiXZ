#ifndef __LINE_FOLLOW_H
#define __LINE_FOLLOW_H
#include "ti_msp_dl_config.h"

/* ---- 8 路灰度传感器参数 ---- */
#define GRAY_BLACK_LEVEL             1
#define GRAY_BASE_SPEED_MM_S         45.0f
#define GRAY_SENSOR_SPAN_MM          85.0f
#define GRAY_SENSOR_PITCH_MM         (GRAY_SENSOR_SPAN_MM / 7.0f)
#define GRAY_SENSOR_FORWARD_MM       260.0f
#define GRAY_STEER_GAIN              1.70f
#define GRAY_MAX_ANGULAR_SPEED       0.40f
#define GRAY_LOST_SEARCH_ANGULAR_SPEED 0.50f
#define GRAY_LOST_SEARCH_MAX_ANGLE_RAD (2.0f * PI)

/* ---- 外部变量 ---- */
extern uint16_t Gray_Data[8];
extern uint16_t Gray_Raw[8];
extern float Gray_Line_Pos_mm;

/* ---- 初始化 ---- */
void Line_Follow_Init(void);

/* ---- 5ms ISR 中调用：读取传感器并更新 Move_X, Move_Z ---- */
void Line_Follow_Run(void);

/* ---- 读取 8 路原始值 ---- */
void Gray_Read_All(void);

#endif
