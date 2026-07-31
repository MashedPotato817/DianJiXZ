/*
 * Copyright (c) 2021, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
#include "board.h"
#include "k230_link.h"
#include "servo.h"
#include "ball_control.h"
#include "debug_telemetry.h"
u8 Car_Mode=Diff_Car;
int Motor_Left,Motor_Right;                 //电机PWM变量 应是Motor的
u8 PID_Send;            //延时和调参相关变量
float RC_Velocity=200,RC_Turn_Velocity,Move_X,Move_Y,Move_Z,PS2_ON_Flag;               //遥控控制的速度
float Velocity_Left,Velocity_Right; //车轮速度(mm/s)
u16 test_num,show_cnt;
float Voltage=0;

void UART1_IRQHandler(void)
{
    K230_Link_UART1_IRQHandler();
}

int main(void)
{
    // 系统初始化
    SYSCFG_DL_init();  // 初始化系统配置
    // 清除所有外设的中断挂起状态
    NVIC_ClearPendingIRQ(ENCODERA_INT_IRQN);    // 编码器A中断
    NVIC_ClearPendingIRQ(ENCODERB_INT_IRQN);    // 编码器B中断
    // 使能各外设的中断
    NVIC_EnableIRQ(ENCODERA_INT_IRQN);    // 开启编码器A中断
    NVIC_EnableIRQ(ENCODERB_INT_IRQN);    // 开启编码器B中断
    // 定时器和ADC相关中断配置
    NVIC_ClearPendingIRQ(TIMER_0_INST_INT_IRQN);  // 清除定时器0中断挂起
    NVIC_EnableIRQ(TIMER_0_INST_INT_IRQN);        // 开启定时器0中断
    NVIC_EnableIRQ(ADC12_VOLTAGE_INST_INT_IRQN);
    OLED_Init();  // 初始化OLED显示屏
    K230_Link_Init();  // UART1 轮询接收 K230 最小联调帧
    Servo_Init();      // 上电先输出 90 度初始种子（1600 us），运行中允许动态学习 trim
    Ball_Control_Init(); /* 当前默认使能，进入混合小球闭环。 */
    Debug_Telemetry_Init(); /* UART0 输出供串口助手/AI分析的时间对齐数据。 */
    // 主循环
    while (1) 
    {
		K230_Link_Process();
        Debug_Telemetry_Process();
		Voltage = Get_battery_volt();//采样小车当前电压
        oled_show();         //  OLED显示更新
    }
}



