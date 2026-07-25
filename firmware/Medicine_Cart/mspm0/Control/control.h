#ifndef __CONTROL_H
#define __CONTROL_H
#include "board.h"

#define PI 3.1415926

/* 底盘几何参数 */
#define Frequency                    200.0f
#define WHEEL_DIAMETER_M             0.0768f
#define Perimeter                    (PI * WHEEL_DIAMETER_M)
#define Wheelspacing                 0.1780f

/* 编码器与速度环 */
#define ENCODER_LINES                13
#define MULTIPLY_FACTOR              2
#define GEAR_RATIO                   28
#define CPR                          (MULTIPLY_FACTOR * ENCODER_LINES * GEAR_RATIO)
#define SPEED_FILTER_ALPHA           0.4f
#define PI_DEADBAND                  0.005f
#define PWM_MAX                      7800

typedef struct {
    float Current_Encoder;
    float Motor_Pwm;
    float Target_Encoder;
    float Velocity;
} Motor_parameter;

typedef struct {
    int A;
    int B;
} Encoder;

extern float Move_X, Move_Z;
extern Encoder OriginalEncoder;
extern Motor_parameter MotorA, MotorB;
extern float Velocity_KP, Velocity_KI;
extern int Run_Mode;

/* 从编码器原始计数换算为速度 (m/s) */
void Get_Velocity_From_Encoder(int Encoder1, int Encoder2);

/* 运动学逆解：Vx(m/s) + Vz(rad/s) → 左右轮目标速度 */
void Get_Target_Encoder(float Vx, float Vz);

/* 增量 PI 控制器 → 返回 PWM 值 */
int Incremental_PI_Left(float Encoder, float Target);
int Incremental_PI_Right(float Encoder, float Target);

/* PWM 限幅 */
float PWM_Limit(float IN, float max, float min);

/* 绝对值 */
int myabs(int a);

/* 电机保护 */
int Turn_Off(void);

#endif
