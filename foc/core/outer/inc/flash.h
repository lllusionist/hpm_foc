#ifndef __FLASH_H__
#define __FLASH_H__

#include "hpm_common.h"

/* 板载 XPI NOR 配置区 (最后一个扇区) 读写, 基于 ROM API */
uint8_t flash_init(void);
uint8_t flash_write_config(const void *data, uint32_t size);
uint8_t flash_read_config(void *data, uint32_t size);

#endif /* __FLASH_H__ */
