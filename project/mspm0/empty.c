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
#include "ball_calibrate.h"
#include "ball_task.h"
#include "calib_store.h"
#include "debug_telemetry.h"
#include "control.h"
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
    /*
     * TIMER_0（5ms控制中断）内部会执行8路灰度Gray_Read_All，
     * 每路50us delay_us累计≥400us忙等待。UART1硬件RX FIFO仅4字节，
     * 115200bps下约347us即被填满；NVIC复位默认优先级相同，
     * 同优先级中断不能互相抢占，TIMER_0执行期间到达的K230字节
     * 会被硬件静默覆盖，且不会计入软件ring buffer的rx_overruns。
     * 把UART1设为更高优先级（数值更小），使其能抢占TIMER_0的忙等待，
     * 避免硬件层丢字节被误判为CRC/线路问题。
     */
    NVIC_SetPriority(UART_1_INST_INT_IRQN, 0);
    NVIC_SetPriority(TIMER_0_INST_INT_IRQN, 1);
    OLED_Init();  // 初始化OLED显示屏
    K230_Link_Init();  // UART1 轮询接收 K230 最小联调帧
    Servo_Init();      // 上电先输出最新实测平衡基准 108.1 度（1701 us）
    Ball_Control_Init(); /* 当前默认使能，进入混合小球闭环。 */
    Ball_Calibrate_Init(); /* 自动扫掠标定状态机，按键长按触发。 */
    Ball_Task_Init(); /* RESET 后目标为0；START(PA18)短按触发 0→+5→-5。 */
    {
        /* 上电优先恢复长按自动标定保存的平衡角，无有效数据才用108.1度。 */
        uint8_t trim_from_flash = 0U;
#if BALL_CAL_LOAD_ENABLE
        float stored_balance_deg;
        if (CalibStore_Load(&stored_balance_deg) != 0U) {
            Ball_Control_SetTrimAngle(stored_balance_deg);
            trim_from_flash = 1U;
        }
#endif
        Debug_Telemetry_Init(); /* UART0 输出供串口助手/AI分析的时间对齐数据。 */
        Debug_Telemetry_LogEvent("RESET"); /* 记录复位/上电时刻（t_ms=0）。 */
        Debug_Telemetry_LogEvent(trim_from_flash ? "TRIM,FLASH" : "TRIM,DEFAULT");
        Debug_Telemetry_LogEvent("TARGET,ZERO");
    }
    // 主循环
    while (1)
    {
		K230_Link_Process();
        Debug_Telemetry_Process();
        UART0_Command_Poll();  // 主循环轮询 UART0 RX，接收 $SET 运行时调参
        {
            uint8_t save_st = Ball_Calibrate_ProcessSave();  // 标定结果写 Flash（主循环安全上下文）
            if (save_st == 1U) {
                Debug_Telemetry_LogEvent("CAL,SAVED");
            } else if (save_st == 2U) {
                Debug_Telemetry_LogEvent("CAL,SAVEFAIL");
            }
        }
		Voltage = Get_battery_volt();//采样小车当前电压
        oled_show();         //  OLED显示更新

        /* 定点任务完成时一次性输出总耗时与各段最大误差，供现场与日志复核。 */
        static uint8_t point_reported = 0U;
        if (Ball_Task_GetState() == BALL_TASK_POINT_DONE) {
            if (point_reported == 0U) {
                printf("POINT DONE %ums e+%.1f e-%.1f\r\n",
                       (unsigned int)Ball_Task_GetTotalMs(),
                       (double)Ball_Task_GetMaxErrorPlusMm(),
                       (double)Ball_Task_GetMaxErrorMinusMm());
                point_reported = 1U;
            }
        } else {
            point_reported = 0U;
        }
    }
}



