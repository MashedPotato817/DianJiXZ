#ifndef H_CALIB_STORE
#define H_CALIB_STORE

#include <stdint.h>

/*
 * 自动标定结果持久化：把扫掠标定得到的摆杆平衡角存入 MSPM0 内部 Flash，
 * 上电自动读取作为 trim 初值，避免换硬件后每次都要重新标定。
 *
 * 存储位置为 Flash 末尾倒数第二个扇区（0x0001F800，1KB），程序当前约
 * 26KB，占用到约 0x7000，该地址远在程序区之外。若程序未来增长到接近
 * 0x0001F000 需重新评估。数据格式：magic(4B) + 平衡角 float(4B)，
 * 8 字节，符合 64-bit flash word 对齐要求。
 */
#define CAL_STORE_ADDR        (0x0001F800UL)
#define CAL_STORE_MAGIC       (0xB11BA1CEUL)

/* 返回 1 且 *balance_deg 有效；Flash 无有效数据（首次/损坏）返回 0。 */
uint8_t CalibStore_Load(float *balance_deg);
/* 写入 Flash；成功返回 1。写 Flash 期间会短暂关闭全局中断。 */
uint8_t CalibStore_Save(float balance_deg);

#endif
