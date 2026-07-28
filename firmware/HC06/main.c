/**
 * @file    main.c
 * @brief   HC-06 Bluetooth Module — 双向通讯测试 v1.0
 *
 * Platform:  MSPM0G3507 (WHEELTEC C07A)
 * UART_1:    PB6(TX) / PB7(RX) @ 9600 bps 8N1
 * Timer:     TIMG0 5ms 时基（由 SYSCFG_DL_init 配置）
 *
 * 功能：
 *   - 环形缓冲接收，不会丢字节
 *   - 命令行解析（PING / LED / ECHO / AT / VER / HELP）
 *   - Echo 模式（默认开启，可用 ECHO OFF 关闭）
 *   - AT 指令透传（直接发给 HC06，响应原样回显）
 *   - 心跳 + LED 翻转（每 3 秒）
 *
 * 手机蓝牙调试助手：
 *   1. 手机蓝牙设置中配对 HC-06（PIN 1234）
 *   2. 打开蓝牙调试助手 APP，连接 HC-06
 *   3. 发送命令测试（见 HELP 命令输出）
 *
 * 接线：
 *   HC-06 VCC → 3.3V（或 5V，看模块版本）
 *   HC-06 GND → GND
 *   HC-06 TXD → PB7（MSPM0 UART1 RX）
 *   HC-06 RXD → PB6（MSPM0 UART1 TX）
 */

#include "ti_msp_dl_config.h"
#include <stdio.h>

/* ============================================================
 * Ring Buffer — 单生产者（ISR）/ 单消费者（main）
 * ============================================================ */
#define RB_SIZE 256

static volatile uint8_t  rb_buf[RB_SIZE];
static volatile uint16_t rb_head;   /* ISR 写入位置 */
static          uint16_t rb_tail;   /* main 读取位置 */

/* ------ Ring Buffer 操作（inline for speed）------ */
static inline uint8_t rb_empty(void)
{
    return (rb_head == rb_tail);
}

static inline uint8_t rb_full(void)
{
    return (((rb_head + 1U) & (RB_SIZE - 1U)) == rb_tail);
}

static inline void rb_put(uint8_t byte)
{
    if (!rb_full()) {
        rb_buf[rb_head] = byte;
        rb_head = (rb_head + 1U) & (RB_SIZE - 1U);
    }
}

static inline uint8_t rb_get(uint8_t *byte)
{
    if (rb_empty()) {
        return 0;
    }
    *byte = rb_buf[rb_tail];
    rb_tail = (rb_tail + 1U) & (RB_SIZE - 1U);
    return 1;
}

/* ============================================================
 * 时基 — TIMG0 ISR 每 5ms 递增
 * ============================================================ */
static volatile uint32_t g_msTick;

void TIMER_0_INST_IRQHandler(void)
{
    switch (DL_TimerG_getPendingInterrupt(TIMER_0_INST)) {
        case DL_TIMER_IIDX_ZERO:
            DL_TimerG_clearInterruptStatus(TIMER_0_INST,
                                           DL_TIMERG_INTERRUPT_ZERO_EVENT);
            g_msTick += 5;
            break;
        default:
            break;
    }
}

/* ============================================================
 * printf redirect → UART_1
 * ============================================================ */
int fputc(int ch, FILE *stream)
{
    while (DL_UART_isBusy(UART_1_INST) == true);
    DL_UART_Main_transmitDataBlocking(UART_1_INST, ch);
    return ch;
}

/* ============================================================
 * 底层 UART 发送（非中断，main 循环中调用）
 * ============================================================ */
static void uart_tx_byte(uint8_t byte)
{
    while (DL_UART_isBusy(UART_1_INST) == true);
    DL_UART_Main_transmitData(UART_1_INST, byte);
}

static void uart_tx_str(const char *s)
{
    while (*s != '\0') {
        uart_tx_byte((uint8_t)*s++);
    }
}

static void uart_tx_line(const char *s)
{
    uart_tx_str(s);
    uart_tx_str("\r\n");
}

/* ============================================================
 * 大小写不敏感字符串比较
 * ============================================================ */
static uint8_t str_icmp(const char *a, const char *b)
{
    while (*a && *b) {
        uint8_t ca = (uint8_t)*a;
        uint8_t cb = (uint8_t)*b;
        if (ca >= 'A' && ca <= 'Z') ca += 32;
        if (cb >= 'A' && cb <= 'Z') cb += 32;
        if (ca != cb) return 0;
        a++; b++;
    }
    return (*a == *b);
}

/* ============================================================
 * 命令处理
 * ============================================================ */

/* Forward 引用，存储是否 echo 和 LED 亮灭 */
static uint8_t gEchoEnabled = 1;

static void cmd_echo(const char *rest)
{
    if (rest[0] == '\0') {
        uart_tx_line(gEchoEnabled ? "ECHO: ON" : "ECHO: OFF");
    } else if (str_icmp(rest, "ON")) {
        gEchoEnabled = 1;
        uart_tx_line("ECHO: ON");
    } else if (str_icmp(rest, "OFF")) {
        gEchoEnabled = 0;
        uart_tx_line("ECHO: OFF");
    } else {
        uart_tx_line("ECHO: ON or OFF?");
    }
}

static uint8_t gLedOn;

static void cmd_led(const char *rest)
{
    if (rest[0] == '\0') {
        uart_tx_line(gLedOn ? "LED: ON" : "LED: OFF");
    } else if (str_icmp(rest, "ON")) {
        gLedOn = 1;
        DL_GPIO_setPins(LED_PORT, LED_led_PIN);
        uart_tx_line("LED: ON");
    } else if (str_icmp(rest, "OFF")) {
        gLedOn = 0;
        DL_GPIO_clearPins(LED_PORT, LED_led_PIN);
        uart_tx_line("LED: OFF");
    } else {
        uart_tx_line("LED: ON or OFF?");
    }
}

/* AT 透传：把整行（含 "AT" 前缀）原样发到 HC06 TX */
static void cmd_at(const char *line)
{
    if (line[2] == '\0') {
        /* 只收到 "AT"，给帮助 */
        uart_tx_line("AT: send AT command, e.g. AT+VERSION");
        return;
    }
    /* 直接把 "AT..." 发给 HC06（HC06 要求 \r\n 结尾） */
    uart_tx_str(line);
    uart_tx_str("\r\n");
}

static void process_command(char *line)
{
    /* 去除行首空白 */
    while (*line == ' ' || *line == '\t') line++;
    if (*line == '\0') return;

    /* 去除行尾 \r 和空白（\n 已由解析器过滤） */
    {
        char *p = line;
        while (*p != '\0' && *p != '\r') p++;
        while (p > line && (p[-1] == ' ' || p[-1] == '\t')) p--;
        *p = '\0';
    }

    if (line[0] == '\0') return;

    /*
     * AT 前缀优先匹配 —— AT 指令没有空格（如 AT+VERSION）
     * 所以不能用 空格分割命令/参数 的方式。
     */
    {
        uint8_t c0 = line[0], c1 = line[1];
        if (c0 >= 'A' && c0 <= 'Z') c0 += 32;
        if (c1 >= 'A' && c1 <= 'Z') c1 += 32;
        if (c0 == 'a' && c1 == 't') {
            cmd_at(line);
            return;
        }
    }

    /* 分割命令和参数（空格分隔） */
    char *arg = line;
    while (*arg != '\0' && *arg != ' ') arg++;
    if (*arg == ' ') {
        *arg++ = '\0';
        while (*arg == ' ') arg++;
    }

    /* ---- 命令分发 ---- */
    if (str_icmp(line, "PING")) {
        static char buf[32];
        snprintf(buf, sizeof(buf), "PONG %lu", (unsigned long)g_msTick);
        uart_tx_line(buf);
    } else if (str_icmp(line, "LED")) {
        cmd_led(arg);
    } else if (str_icmp(line, "ECHO")) {
        cmd_echo(arg);
    } else if (str_icmp(line, "VER")) {
        uart_tx_line("HC06 Test v1.0  MSPM0G3507");
    } else if (str_icmp(line, "HELP") || str_icmp(line, "?")) {
        uart_tx_line("=== HC06 Test Commands ===");
        uart_tx_line("  PING         - Connection test");
        uart_tx_line("  LED ON|OFF   - Control LED (PB9)");
        uart_tx_line("  ECHO ON|OFF  - Toggle echo mode");
        uart_tx_line("  AT<cmd>      - HC06 AT passthrough");
        uart_tx_line("  VER          - Firmware version");
        uart_tx_line("  HELP         - This list");
    } else if (gEchoEnabled) {
        /* 非命令文本 → Echo 回显 */
        uart_tx_line(line);
    }
}

/* ============================================================
 * UART_1 ISR — 只收不发，写入环形缓冲
 * ============================================================ */
void UART_1_INST_IRQHandler(void)
{
    switch (DL_UART_Main_getPendingInterrupt(UART_1_INST)) {
        case DL_UART_MAIN_IIDX_RX:
            rb_put(DL_UART_Main_receiveData(UART_1_INST));
            break;
        default:
            break;
    }
}

/* ============================================================
 * Main
 * ============================================================ */
int main(void)
{
    /* ---- 板级初始化（时钟、GPIO、UART、Timer、DMA 等）---- */
    SYSCFG_DL_init();

    /* ---- 配置 UART_1：关 DMA，开 RX 中断 ---- */
    DL_UART_Main_disableDMAReceiveEvent(UART_1_INST,
                                         DL_UART_DMA_INTERRUPT_RX);
    DL_UART_Main_disableInterrupt(UART_1_INST,
                                   DL_UART_MAIN_INTERRUPT_DMA_DONE_RX);
    DL_UART_Main_enableInterrupt(UART_1_INST,
                                  DL_UART_MAIN_INTERRUPT_RX);

    /* ---- 启动 TIMG0（SYSCFG_DL_init 已配置 5ms 周期）---- */
    DL_TimerG_startCounter(TIMER_0_INST);

    /* ---- 使能中断 ---- */
    NVIC_ClearPendingIRQ(TIMER_0_INST_INT_IRQN);
    NVIC_EnableIRQ(TIMER_0_INST_INT_IRQN);
    NVIC_ClearPendingIRQ(UART_1_INST_INT_IRQN);
    NVIC_EnableIRQ(UART_1_INST_INT_IRQN);

    /* ---- 启动 Banner ---- */
    uart_tx_line("");
    uart_tx_line("====================================");
    uart_tx_line("  HC-06 Bluetooth Test v1.0");
    uart_tx_line("  UART1 @ 9600 bps 8N1");
    uart_tx_line("  TX:PB6  RX:PB7");
    uart_tx_line("====================================");
    uart_tx_line(" Type HELP for commands.");
    uart_tx_line("");

    /* ---- 状态变量 ---- */
    uint32_t lastHeartbeat = 0;
    uint32_t heartbeatNum  = 0;
    char     lineBuf[128];
    uint8_t  lineIdx = 0;

    /* ---- 主循环 ---- */
    while (1) {
        /* --- 从环形缓冲取字节，拼成行 --- */
        uint8_t byte;
        while (rb_get(&byte)) {
            if (byte == '\r' || byte == '\n') {
                if (lineIdx > 0) {
                    lineBuf[lineIdx] = '\0';
                    process_command(lineBuf);
                    lineIdx = 0;
                }
            } else if (lineIdx < sizeof(lineBuf) - 1) {
                lineBuf[lineIdx++] = (char)byte;
            }
            /* 行太长就丢弃（不 crash） */
        }

        /* --- 心跳 + LED 翻转 每 3000ms --- */
        if ((uint32_t)(g_msTick - lastHeartbeat) >= 3000U) {
            lastHeartbeat = g_msTick;
            heartbeatNum++;
            DL_GPIO_togglePins(LED_PORT, LED_led_PIN);
            printf("[%lu] HC06 Alive\r\n", (unsigned long)heartbeatNum);
        }
    }
}
