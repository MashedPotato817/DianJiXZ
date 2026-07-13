#ifndef _JY62_H_
#define _JY62_H_

#include <stdint.h>

typedef struct {
    float ax_g;
    float ay_g;
    float az_g;
    float wx_dps;
    float wy_dps;
    float wz_dps;
    float roll_deg;
    float pitch_deg;
    float yaw_deg;
    float gyro_roll_int_deg;
    float gyro_pitch_int_deg;
    float gyro_yaw_int_deg;
    uint32_t rx_bytes;
    uint32_t valid_frames;
    uint32_t checksum_errors;
    uint8_t online;
} JY62_Data;

void JY62_Init(void);
void JY62_Tick10ms(void);
void JY62_Process(void);
void JY62_InputByteFromISR(uint8_t data);
uint8_t JY62_IsOnline(void);
uint8_t JY62_GetData(JY62_Data *data);
int16_t JY62_GetYawDegX10(void);
uint32_t JY62_GetRxBytes(void);
uint32_t JY62_GetValidFrames(void);
uint32_t JY62_GetChecksumErrors(void);
uint32_t JY62_GetCurrentBaud(void);

#endif
