/*
 * 送药任务状态机。
 *
 * 状态转移：
 *   WAIT_TARGET ──(病房号锁定)──→ WAIT_LOAD
 *   WAIT_LOAD   ──(装载检测)───→ OUTBOUND
 *   OUTBOUND    ──(到位判据)───→ ARRIVED
 *   ARRIVED     ──(卸载检测)───→ RETURN
 *   RETURN      ──(药房到位)───→ FINISHED
 *   FINISHED    ──(复位)───────→ WAIT_TARGET
 *   任一状态    ──(故障条件)───→ TASK_FAULT
 *
 * 装载/卸载检测 → load_detect 模块（Task 3）
 * 路线和到位判据 → route 模块（Task 4）
 * 当前版本仅实现状态框架，具体转移条件在各 Task 完成后接入。
 */
#include "medicine_task.h"
#include "board.h"
#include "k230_link.h"

static Medicine_Task g_task;

void Medicine_Task_Init(void)
{
    g_task.target_ward = 0;
    g_task.state = TASK_WAIT_TARGET;
    g_task.prev_state = TASK_WAIT_TARGET;
    g_task.fault = FAULT_NONE;
    g_task.state_enter_ms = 0;
}

static const char *StateName(Medicine_State s)
{
    switch (s) {
        case TASK_WAIT_TARGET: return "WAIT_TARGET";
        case TASK_WAIT_LOAD:   return "WAIT_LOAD";
        case TASK_OUTBOUND:    return "OUTBOUND";
        case TASK_ARRIVED:     return "ARRIVED";
        case TASK_RETURN:      return "RETURN";
        case TASK_FINISHED:    return "FINISHED";
        case TASK_FAULT:       return "FAULT";
        default:               return "?";
    }
}

static void ChangeState(Medicine_State new_state)
{
    g_task.prev_state = g_task.state;
    g_task.state = new_state;

    switch (new_state) {
        case TASK_WAIT_TARGET:
            g_task.target_ward = 0;
            Flag_Stop = 1;
            K230_Clear_Target();
            break;
        case TASK_WAIT_LOAD:
            Flag_Stop = 1;
            break;
        case TASK_OUTBOUND:
            Flag_Stop = 0;  /* 允许电机运转 */
            break;
        case TASK_ARRIVED:
            Flag_Stop = 1;
            /* TODO: 红灯亮 */
            break;
        case TASK_RETURN:
            Flag_Stop = 0;  /* 允许电机运转 */
            break;
        case TASK_FINISHED:
            Flag_Stop = 1;
            /* TODO: 绿灯亮 */
            break;
        case TASK_FAULT:
            Flag_Stop = 1;  /* 故障停车 */
            break;
    }
}

void Medicine_Task_Run(void)
{
    Medicine_State s = g_task.state;

    /* ---- 故障不可自动恢复 ---- */
    if (s == TASK_FAULT) {
        return;
    }

    /* ---- K230 通信超时检测 ---- */
    if (s != TASK_WAIT_TARGET && !K230_Is_Online()) {
        /* 出发后 K230 断线：不立即停车，
         * 因为行驶中不依赖 K230 识别结果。
         * 仅在 WAIT_TARGET 状态通信超时才报故障。 */
    }

    switch (s) {
        case TASK_WAIT_TARGET:
            if (K230_Is_Online() && K230_Is_TargetLocked()) {
                g_task.target_ward = K230_Get_TargetWard();
                if (g_task.target_ward >= 1 && g_task.target_ward <= 8) {
                    ChangeState(TASK_WAIT_LOAD);
                }
            }
            break;

        case TASK_WAIT_LOAD:
            /* TODO(Task 3): Load_Detect_IsLoaded() → ChangeState(TASK_OUTBOUND) */
            break;

        case TASK_OUTBOUND:
            /* TODO(Task 4): Route_CheckArrived() → ChangeState(TASK_ARRIVED) */
            /* TODO: 失线/超时 → ChangeState(TASK_FAULT) */
            break;

        case TASK_ARRIVED:
            /* TODO(Task 3): Load_Detect_IsUnloaded() → ChangeState(TASK_RETURN) */
            break;

        case TASK_RETURN:
            /* TODO(Task 4): Route_CheckHome() → ChangeState(TASK_FINISHED) */
            /* TODO: 失线/超时 → ChangeState(TASK_FAULT) */
            break;

        case TASK_FINISHED:
            /* 等待按键复位 → ChangeState(TASK_WAIT_TARGET) */
            break;

        default:
            break;
    }
}

Medicine_State Medicine_Get_State(void)
{
    return g_task.state;
}

uint8_t Medicine_Task_AllowMove(void)
{
    return (g_task.state == TASK_OUTBOUND ||
            g_task.state == TASK_RETURN) ? 1 : 0;
}

void Medicine_Task_SetFault(Fault_Reason reason)
{
    g_task.fault = reason;
    ChangeState(TASK_FAULT);
}

void Medicine_Task_Reset(void)
{
    ChangeState(TASK_WAIT_TARGET);
}
