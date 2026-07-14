/*
 * Copyright (c) 2021, Texas Instruments Incorporated
 * All rights reserved.
 */
#include "board.h"
#include "oled.h"
u8 Car_Mode = Diff_Car;
int Motor_Left, Motor_Right;
u8 PID_Send;
float RC_Velocity = 200, RC_Turn_Velocity, Move_X, Move_Y, Move_Z, PS2_ON_Flag;
float Velocity_Left, Velocity_Right;
u16 test_num, show_cnt;
float Voltage = 0;

int main(void)
{
    SYSCFG_DL_init();
    OLED_Init();
    Gimbal_Init();
    Gimbal_StopYaw();
    Gimbal_SetPitchPulseUs(1500);
    OLED_Clear();
    OLED_ShowString(0, 0, "Pitch demo PA1");
    OLED_ShowString(0, 16, "1000us <-> 2000us");
    OLED_Refresh_Gram();
    delay_ms(1000);

    while (1) {
        LED_ON();
        Gimbal_SetPitchPulseUs(1000);
        delay_ms(1500);

        LED_OFF();
        Gimbal_SetPitchPulseUs(2000);
        delay_ms(1500);
    }
}
