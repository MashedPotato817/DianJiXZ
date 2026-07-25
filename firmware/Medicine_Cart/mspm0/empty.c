/*
 * 智能送药小车 — MSPM0G3507 主入口。
 *
 * 实时职责：
 *   5ms ISR (TIMG0) — 巡线、编码器速度换算、PI、PWM 输出
 *   主循环 — K230 串口、状态机、装载检测、OLED 刷新
 */
#include "empty.h"
#include "medicine_task.h"
#include "k230_link.h"
#include "line_follow.h"
#include "route.h"
#include "load_detect.h"
#include "indicator.h"

volatile uint32_t g_sysTick5ms;

u8 Car_Mode = Diff_Car;
int Motor_Left, Motor_Right;
float Voltage;
float Move_X, Move_Z;
float Velocity_Left, Velocity_Right;

/* ---- OLED 简易状态显示 ---- */
static void OLED_ShowStatus(void)
{
    char line[22];
    Medicine_State state = Medicine_Get_State();
    const char *state_names[] = {
        "WAIT_TARGET", "WAIT_LOAD", "OUTBOUND",
        "ARRIVED", "RETURN", "FINISHED", "FAULT"
    };

    OLED_ShowString(0, 0, "MedCart");

    /* 行2: 状态 */
    if (state <= TASK_FAULT) {
        OLED_ShowString(0, 10, state_names[state]);
    }

    /* 行3: K230 链路 */
    OLED_ShowString(0, 20, K230_Is_Online() ? "K230:ON " : "K230:OFF");

    /* 行4: 目标病房号 */
    OLED_ShowString(0, 30, "Ward:");
    if (K230_Is_TargetLocked()) {
        OLED_ShowNumber(50, 30, K230_Get_TargetWard(), 1, 12);
    } else {
        OLED_ShowString(50, 30, "-");
    }

    /* 行5: 灰度原始值 */
    {
        uint8_t i;
        OLED_ShowString(0, 40, "G:");
        for (i = 0; i < 8; i++) {
            OLED_ShowString(12 + i * 10, 40, Gray_Raw[i] ? "1" : "0");
        }
    }

    /* 行6: 线位置 */
    {
        int pos = (int)Gray_Line_Pos_mm;
        OLED_ShowString(0, 50, "P:");
        if (pos < 0)  OLED_ShowString(16, 50, "-");
        else          OLED_ShowString(16, 50, "+");
        OLED_ShowNumber(26, 50, myabs(pos), 3, 12);
    }

    OLED_Refresh_Gram();
}

int main(void)
{
    SYSCFG_DL_init();

    /* UART0 关闭 loopback（SysConfig 模板默认开启），使 printf 输出到 PA10 */
    DL_UART_Main_disableLoopbackMode(UART_0_INST);

    /* 清除中断挂起 */
    NVIC_ClearPendingIRQ(ENCODERA_INT_IRQN);
    NVIC_ClearPendingIRQ(ENCODERB_INT_IRQN);
    NVIC_ClearPendingIRQ(UART_0_INST_INT_IRQN);
    NVIC_ClearPendingIRQ(UART_1_INST_INT_IRQN);
    NVIC_ClearPendingIRQ(TIMER_0_INST_INT_IRQN);
    NVIC_ClearPendingIRQ(ADC12_VOLTAGE_INST_INT_IRQN);

    /* 使能中断 */
    NVIC_EnableIRQ(ENCODERA_INT_IRQN);
    NVIC_EnableIRQ(ENCODERB_INT_IRQN);
    NVIC_EnableIRQ(TIMER_0_INST_INT_IRQN);
    NVIC_EnableIRQ(ADC12_VOLTAGE_INST_INT_IRQN);
    /*
     * UART1 中断不使能 — K230 通信用轮询模式。
     * UART0 中断不使能 — 调试 printf 用阻塞发送。
     */

    OLED_Init();

    /* 模块初始化 */
    K230_Link_Init();
    Medicine_Task_Init();
    Line_Follow_Init();
    Route_Init();
    Load_Detect_Init();
    Indicator_Init();

    while (1) {
        /* 通信：接收 K230 帧、维护链路状态 */
        K230_Link_Process();

        /* 装载检测去抖（Task 3 接入后有效） */
        Load_Detect_Process();

        /* 送药任务状态机 */
        Medicine_Task_Run();

        /* 电池监测 */
        Voltage = Get_battery_volt();

        /* OLED 刷新 */
        OLED_ShowStatus();
    }
}
