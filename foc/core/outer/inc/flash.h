#ifndef __FLASH_H__
#define __FLASH_H__

#include "hpm_common.h"

/* 配置缓冲区: 必须 >= sizeof(tConfig), 当前 512 点齿槽表约 3244B */
#define FLASH_CFG_BUF_SIZE 4096U

/* 双扇区写入: 擦写中掉电不丢全部配置.
 * 使用最后两个扇区, 每次交替写; 读时取 generation 较大且载荷 CRC 有效的那份.
 * 每份前面加一个 12 字节头: {magic(4B), generation(4B), payload crc32(4B)} */
#define FLASH_DUAL_SECTOR_MAGIC 0x43465244 /* "CRFD" */

typedef struct {
	uint32_t magic;
	uint32_t generation; /* 每次写递增, 大者新 */
	uint32_t crc32;      /* 载荷 CRC32: 识别"头有效但载荷撕裂"的半写扇区 */
} flash_cfg_header_t;

/* 板载 XPI NOR 配置区读写, 基于 ROM API */
uint8_t flash_init(void);
/* 写入配置数据 (含双扇区轮转逻辑) */
uint8_t flash_write_config(const void *data, uint32_t size);
/* 读取最新有效配置 (失败返回非 0) */
uint8_t flash_read_config(void *data, uint32_t size);

#endif /* __FLASH_H__ */
