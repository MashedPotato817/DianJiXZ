#ifndef H_CALIB_STORE
#define H_CALIB_STORE

#include <stdint.h>

/*
 * 自动标定结果持久化：把扫掠标定得到的摆杆平衡角存入 MSPM0 内部 Flash，
 * 上电自动读取作为 trim 初值，避免换硬件后每次都要重新标定。
 *
 * 存储位置：MSPM0G3507 128KB 主 Flash 的最后 1KB 扇区 0x0001FC00。
 * Keil scatter 文件把应用镜像限制到 0x00000000~0x0001FBFF，链接阶段即可
 * 防止程序覆盖标定扇区。此前固定 0x00007000 已落入当前程序镜像，不能擦写。
 * 数据格式：magic(4B) + 平衡角 float(4B)，按 64 位对齐一次写入。
 */
#define CAL_STORE_ADDR        (0x0001FC00UL)
#define CAL_STORE_SECTOR_SIZE (0x00000400UL)
#define CAL_STORE_FLASH_SIZE  (0x00020000UL)
#define CAL_STORE_MAGIC       (0xB11BA1CEUL)

/* 返回 1 且 *balance_deg 有效；Flash 无有效数据（首次/损坏）返回 0。 */
uint8_t CalibStore_Load(float *balance_deg);
/* 写入 Flash；成功返回 1。写 Flash 期间会短暂关闭全局中断。 */
uint8_t CalibStore_Save(float balance_deg);

#endif
