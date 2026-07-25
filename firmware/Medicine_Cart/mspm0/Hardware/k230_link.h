#ifndef __K230_LINK_H
#define __K230_LINK_H
#include "ti_msp_dl_config.h"

/* ---- 链路参数 ---- */
#define K230_HELLO_PERIOD_MS          500U
#define K230_DATA_PERIOD_MS           1000U
#define K230_LINK_TIMEOUT_MS          3000U
#define K230_LINE_BUF_SIZE            32U
#define K230_TARGET_CONFIRM_COUNT     3U      /* 连续相同病房号确认次数 */

/* ---- 识别结果 ---- */
#define K230_WARD_NONE                0       /* 无有效识别 */
#define K230_WARD_MIN                 1
#define K230_WARD_MAX                 8

typedef struct {
    uint8_t ward;           /* 病房号 1-8，0=无有效结果 */
    uint8_t confidence;     /* 置信度 0-100 */
    uint8_t confirmed;      /* 锁定标志：连续 K230_TARGET_CONFIRM_COUNT 次一致 */
    uint32_t timestamp_ms;  /* 锁定时刻 */
} K230_Result;

/* ---- 链路状态 ---- */
typedef enum {
    K230_LINK_DOWN = 0,
    K230_LINK_HANDSHAKE,
    K230_LINK_ONLINE
} K230_LinkState;

/* ---- 初始化：清空缓冲、启用 UART1 轮询 ---- */
void K230_Link_Init(void);

/* ---- 主循环调用：处理接收、握手、超时 ---- */
void K230_Link_Process(void);

/* ---- 获取最新稳定病房号（已锁定的） ---- */
uint8_t K230_Get_TargetWard(void);

/* ---- 目标是否已锁定 ---- */
uint8_t K230_Is_TargetLocked(void);

/* ---- 清除已锁定目标（复位/断链时调用） ---- */
void K230_Clear_Target(void);

/* ---- 链路是否在线 ---- */
uint8_t K230_Is_Online(void);

/* ---- 获取链路状态枚举 ---- */
K230_LinkState K230_Get_State(void);

/* ---- 获取最新识别结果（含未锁定的） ---- */
void K230_Get_Result(K230_Result *result);

#endif
