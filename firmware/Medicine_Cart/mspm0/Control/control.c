#include "control.h"
#include "k230_link.h"
#include "key.h"
#include "line_follow.h"
#include "empty.h"

Encoder OriginalEncoder;
Motor_parameter MotorA, MotorB;
float Velocity_KP = 400, Velocity_KI = 300;
int Run_Mode = 1;       /* 1 = 巡线模式（送药车仅此模式） */
u8 Flag_Stop = 1;       /* 默认停车 */

void Get_Velocity_From_Encoder(int Encoder1, int Encoder2)
{
    static float Filtered_SpeedA = 0.0f, Filtered_SpeedB = 0.0f;
    float Encoder_A_pr, Encoder_B_pr, raw_speedA, raw_speedB;

    OriginalEncoder.A = Encoder1;
    OriginalEncoder.B = Encoder2;
    Encoder_A_pr = OriginalEncoder.A;
    Encoder_B_pr = -OriginalEncoder.B;

    raw_speedA = Encoder_A_pr * Frequency * Perimeter / CPR;
    raw_speedB = Encoder_B_pr * Frequency * Perimeter / CPR;

    Filtered_SpeedA = SPEED_FILTER_ALPHA * raw_speedA
                    + (1.0f - SPEED_FILTER_ALPHA) * Filtered_SpeedA;
    Filtered_SpeedB = SPEED_FILTER_ALPHA * raw_speedB
                    + (1.0f - SPEED_FILTER_ALPHA) * Filtered_SpeedB;

    MotorA.Current_Encoder = Filtered_SpeedA;
    MotorB.Current_Encoder = Filtered_SpeedB;
}

void Get_Target_Encoder(float Vx, float Vz)
{
    if (Vx < 0) Vz = -Vz;

    MotorA.Target_Encoder = Vx - Vz * Wheelspacing / 2.0f;
    MotorB.Target_Encoder = Vx + Vz * Wheelspacing / 2.0f;
}

int myabs(int a)
{
    return (a < 0) ? -a : a;
}

int Turn_Off(void)
{
    return 0;
}

float PWM_Limit(float IN, float max, float min)
{
    float OUT = IN;
    if (OUT > max) OUT = max;
    if (OUT < min) OUT = min;
    return OUT;
}

int Incremental_PI_Left(float Encoder, float Target)
{
    static float Bias, Pwm, Last_bias;
    float abs_bias;

    /* 停车时清零历史值，防止起步突跳 */
    if (Flag_Stop) {
        Pwm = 0;
        Bias = 0;
        Last_bias = 0;
        return 0;
    }

    Bias = Target - Encoder;
    abs_bias = (Bias > 0.0f) ? Bias : -Bias;

    if (abs_bias < PI_DEADBAND) { Last_bias = Bias; return (int)Pwm; }

    Pwm += Velocity_KP * (Bias - Last_bias) + Velocity_KI * Bias;
    Pwm = PWM_Limit(Pwm, PWM_MAX, -PWM_MAX);
    Last_bias = Bias;
    return (int)Pwm;
}

int Incremental_PI_Right(float Encoder, float Target)
{
    static float Bias, Pwm, Last_bias;
    float abs_bias;

    /* 停车时清零历史值，防止起步突跳 */
    if (Flag_Stop) {
        Pwm = 0;
        Bias = 0;
        Last_bias = 0;
        return 0;
    }

    Bias = Target - Encoder;
    abs_bias = (Bias > 0.0f) ? Bias : -Bias;

    if (abs_bias < PI_DEADBAND) { Last_bias = Bias; return (int)Pwm; }

    Pwm += Velocity_KP * (Bias - Last_bias) + Velocity_KI * Bias;
    Pwm = PWM_Limit(Pwm, PWM_MAX, -PWM_MAX);
    Last_bias = Bias;
    return (int)Pwm;
}

/* ========== 5ms 控制中断 ========== */
void TIMER_0_INST_IRQHandler(void)
{
    if (DL_TimerG_getPendingInterrupt(TIMER_0_INST) == DL_TIMERG_IIDX_ZERO) {

            g_sysTick5ms += 5U;
            K230_Tick5ms();

            Key();
            LED_Flash(100);

            Get_Velocity_From_Encoder(Get_Encoder_countA, Get_Encoder_countB);
            Get_Encoder_countA = Get_Encoder_countB = 0;

            /* 巡线计算由 line_follow 模块完成 */
            Line_Follow_Run();

            /* 运动学逆解 + PI + PWM */
            Get_Target_Encoder(Move_X, Move_Z);
            MotorA.Motor_Pwm = Incremental_PI_Left(
                MotorA.Current_Encoder, MotorA.Target_Encoder);
            MotorB.Motor_Pwm = Incremental_PI_Right(
                MotorB.Current_Encoder, MotorB.Target_Encoder);

            if (!Flag_Stop) {
                Set_PWM(-MotorA.Motor_Pwm, -MotorB.Motor_Pwm);
            } else {
                Set_PWM(0, 0);
            }
    }
}
