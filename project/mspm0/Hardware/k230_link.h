#ifndef H_K230_LINK
#define H_K230_LINK

#include <stdint.h>

/* K230 输出的小球位置，坐标系和串口帧格式由 K230 程序确认后固化。 */
typedef struct {
    float x_mm;
    float y_mm;
    uint8_t valid;
    uint32_t timestamp_ms;
} K230_BallPosition;

void K230_Link_Init(void);
void K230_Link_UpdatePosition(float x_mm, float y_mm, uint8_t valid,
                              uint32_t timestamp_ms);
void K230_Link_GetPosition(K230_BallPosition *position);

#endif
