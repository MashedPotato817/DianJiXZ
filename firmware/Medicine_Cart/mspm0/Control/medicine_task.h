#ifndef __MEDICINE_TASK_H
#define __MEDICINE_TASK_H
#include "board.h"

/* ---- 状态机 ---- */
typedef enum {
    TASK_WAIT_TARGET = 0,   /* 等待 K230 识别到稳定病房号 */
    TASK_WAIT_LOAD,         /* 等待药品装载 */
    TASK_OUTBOUND,          /* 去程：药房→病房 */
    TASK_ARRIVED,           /* 到房停车，亮红灯，等待卸载 */
    TASK_RETURN,            /* 返程：病房→药房 */
    TASK_FINISHED,          /* 回到药房，亮绿灯 */
    TASK_FAULT              /* 故障停车（失线/通信超时/路径超时） */
} Medicine_State;

/* ---- 故障原因 ---- */
typedef enum {
    FAULT_NONE = 0,
    FAULT_LINE_LOST,        /* 巡线丢失 */
    FAULT_K230_TIMEOUT,     /* K230 通信超时 */
    FAULT_PATH_TIMEOUT,     /* 路径执行超时 */
    FAULT_INVALID_WARD      /* 无效病房号 */
} Fault_Reason;

/* ---- 送药任务上下文 ---- */
typedef struct {
    uint8_t target_ward;        /* 锁定的病房号 1-8 */
    Medicine_State state;       /* 当前状态 */
    Medicine_State prev_state;  /* 前一状态（用于故障恢复判断） */
    Fault_Reason fault;         /* 故障原因 */
    uint32_t state_enter_ms;    /* 进入当前状态的时刻 */
} Medicine_Task;

/* ---- 初始化 ---- */
void Medicine_Task_Init(void);

/* ---- 主循环调用：推进状态机 ---- */
void Medicine_Task_Run(void);

/* ---- 获取当前状态（供 ISR 查询） ---- */
Medicine_State Medicine_Get_State(void);

/* ---- 状态机是否允许电机运转 ---- */
uint8_t Medicine_Task_AllowMove(void);

/* ---- 设置故障状态 ---- */
void Medicine_Task_SetFault(Fault_Reason reason);

/* ---- 复位（FINISHED → WAIT_TARGET） ---- */
void Medicine_Task_Reset(void);

#endif
