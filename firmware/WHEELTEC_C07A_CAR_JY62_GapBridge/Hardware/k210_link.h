#ifndef _K210_LINK_H_
#define _K210_LINK_H_

#include "ti_msp_dl_config.h"

#define K210_HANDSHAKE_REQUEST  "$CAR,HELLO#\r\n"
#define K210_HANDSHAKE_RESPONSE "$K210,OK#"

typedef struct {
    uint8_t valid;
    uint16_t x;
    uint16_t y;
    uint16_t area;
    uint32_t sequence;
    uint32_t timestamp_ms;
} K210_VisionFrame;

void K210_Link_Init(void);
void K210_Link_Tick10ms(void);
void K210_Link_Process(void);
void K210_Link_InputByteFromISR(uint8_t data);

uint8_t K210_Link_IsOnline(void);
uint8_t K210_Link_IsHandshakeOk(void);
uint8_t K210_Link_GetLatestVision(K210_VisionFrame *frame);
uint32_t K210_Link_RxBytes(void);
uint32_t K210_Link_RxFrames(void);
uint32_t K210_Link_VisionFrames(void);
uint32_t K210_Link_ChecksumErrors(void);

#endif
