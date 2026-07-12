/*
 * Copyright (c) 2021, Texas Instruments Incorporated
 * All rights reserved.
 */
#include "board.h"
#include "jy62.h"

u8 Car_Mode = Diff_Car;
int Motor_Left, Motor_Right;
u8 PID_Send;
float RC_Velocity = 200, RC_Turn_Velocity, Move_X, Move_Y, Move_Z, PS2_ON_Flag;
float Velocity_Left, Velocity_Right;
u16 test_num, show_cnt;
float Voltage = 0;

int main(void)
{
    u16 last_show_cnt = 0;

    SYSCFG_DL_init();

    NVIC_ClearPendingIRQ(ENCODERA_INT_IRQN);
    NVIC_ClearPendingIRQ(ENCODERB_INT_IRQN);
    NVIC_ClearPendingIRQ(UART_0_INST_INT_IRQN);
    NVIC_ClearPendingIRQ(UART_1_INST_INT_IRQN);

    NVIC_EnableIRQ(ENCODERA_INT_IRQN);
    NVIC_EnableIRQ(ENCODERB_INT_IRQN);
    NVIC_EnableIRQ(UART_0_INST_INT_IRQN);
    NVIC_EnableIRQ(UART_1_INST_INT_IRQN);

    NVIC_ClearPendingIRQ(TIMER_0_INST_INT_IRQN);
    NVIC_EnableIRQ(TIMER_0_INST_INT_IRQN);
    NVIC_EnableIRQ(ADC12_VOLTAGE_INST_INT_IRQN);

    OLED_Init();
    JY62_Init();

    while (1) {
        Voltage = Get_battery_volt();
        BTBufferHandler();
        BlackboxTelemetry_Process();
        if ((u16) (show_cnt - last_show_cnt) >= 8U) {
            oled_show();
            last_show_cnt = show_cnt;
        }
    }
}
