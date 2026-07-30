/*
 * Copyright (c) 2020, Texas Instruments Incorporated
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

#include "ti_msp_dl_config.h"

/* 舵机高电平脉宽范围（单位：计数 = 微秒，PWM 时钟已分频到 1MHz）
 * 0.5ms(500) 对应 0°，2.5ms(2500) 对应 180° */
#define SERVO_PULSE_MIN 500U     /* 0°   高电平 0.5ms */
#define SERVO_PULSE_MAX 2500U    /* 180° 高电平 2.5ms */

/* PWM 周期计数（与 SysConfig 的 Period Count 一致）。
 * 本工程为 Edge-aligned DOWN counting，输出初值 Low：
 * 实际高电平时间 = PERIOD - 比较值(CC)，所以设置 CC 时要用 PERIOD 减去目标脉宽。 */
#define SERVO_PWM_PERIOD    20000U

/* 平滑转动参数 */
#define SERVO_ANGLE_STEP    1.0f        /* 每步角度增量 */
#define SERVO_STEP_DELAY    640000U     /* 每步延时 ≈ 20ms（32MHz × 0.02s） */

/* Keil/ARMCLANG 不支持 TI 内建的 __delay_cycles，用 DriverLib 的忙等延时代替。
 * 注意：入参不可为 0（0 表示最大延时）。 */
#define servo_delay_cycles(n)   DL_Common_delayCycles(n)

/* 角度(0~180) → 比较值，带边界钳位。
 * 先算目标高电平脉宽(500~2500)，再按向下计数极性取反：CC = PERIOD - 脉宽。 */
static uint32_t servo_angle_to_ccr(float angle)
{
    uint32_t pulse;
    if (angle < 0.0f)   angle = 0.0f;
    if (angle > 180.0f) angle = 180.0f;
    pulse = (uint32_t)(SERVO_PULSE_MIN +
            (angle / 180.0f) * (SERVO_PULSE_MAX - SERVO_PULSE_MIN));
    return SERVO_PWM_PERIOD - pulse;   /* Edge-aligned down counting 极性取反 */
}

/* 设置舵机角度 */
void servo_set_angle(float angle)
{
    uint32_t ccr = servo_angle_to_ccr(angle);
    /* PA8 → TIMA0，单通道 ccIndex=[0]；生成头文件中 GPIO_PWM_0_C0_IDX 即 DL_TIMER_CC_0_INDEX */
    DL_TimerA_setCaptureCompareValue(PWM_0_INST, ccr, DL_TIMER_CC_0_INDEX);
}

int main(void)
{
    SYSCFG_DL_init();

    /* 把比较值更新方式从 immediate 改为 ZERO_EVT：新脉宽只在计数归零(周期起点)载入，
     * 避免在一拍脉冲中途改写导致的畸形脉冲——那是无规律抽动的一个代码侧诱因。 */
    DL_TimerA_setCaptCompUpdateMethod(PWM_0_INST,
        DL_TIMER_CC_UPDATE_METHOD_ZERO_EVT, DL_TIMER_CC_0_INDEX);

    /* SysConfig 已勾选 Start Timer(timerStartTimer=true)，此处再调一次也安全 */
    DL_TimerA_startCounter(PWM_0_INST);

    float angle = 0.0f;
    servo_set_angle(angle);

    while (1) {
        /* 0° → 180° */
        for (angle = 0.0f; angle <= 180.0f; angle += SERVO_ANGLE_STEP) {
            servo_set_angle(angle);
            servo_delay_cycles(SERVO_STEP_DELAY);
        }
        /* 180° → 0° */
        for (angle = 180.0f; angle >= 0.0f; angle -= SERVO_ANGLE_STEP) {
            servo_set_angle(angle);
            servo_delay_cycles(SERVO_STEP_DELAY);
        }
    }
}
