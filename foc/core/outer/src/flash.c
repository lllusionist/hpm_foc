#include "flash.h"
#include "hpm_romapi.h"
#include "hpm_interrupt.h"
#include <string.h>
#include <stdio.h>

/* HPM5E00EVK 板级 XPI NOR 参数 (见 SDK boards/hpm5e00evk/board.h) */
#define FLASH_XPI_BASE HPM_XPI0
#define FLASH_XPI_OPT_HDR (0xfcf90002U)
#define FLASH_XPI_OPT_OPT0 (0x00000006U)
#define FLASH_XPI_OPT_OPT1 (0x00001000U)

static xpi_nor_config_t s_nor_config;
static bool s_inited;
static uint32_t s_cfg_addr; /* 配置区地址 (最后一个扇区) */
static uint32_t s_sector_size;
static uint32_t s_page_size;
static uint32_t s_buf[FLASH_CFG_BUF_SIZE / 4U];

uint8_t flash_init(void)
{
	xpi_nor_config_option_t option;
	uint32_t flash_size;

	option.header.U = FLASH_XPI_OPT_HDR;
	option.option0.U = FLASH_XPI_OPT_OPT0;
	option.option1.U = FLASH_XPI_OPT_OPT1;

	if (rom_xpi_nor_auto_config(FLASH_XPI_BASE, &s_nor_config, &option) !=
	    status_success) {
		printf("flash: auto config fail\n");
		return 1;
	}

	rom_xpi_nor_get_property(FLASH_XPI_BASE, &s_nor_config,
				 xpi_nor_property_total_size, &flash_size);
	rom_xpi_nor_get_property(FLASH_XPI_BASE, &s_nor_config,
				 xpi_nor_property_sector_size, &s_sector_size);
	rom_xpi_nor_get_property(FLASH_XPI_BASE, &s_nor_config,
				 xpi_nor_property_page_size, &s_page_size);

	s_cfg_addr = flash_size - s_sector_size;
	s_inited = true;

	printf("flash: %uKB, sector %uB, page %uB, cfg@0x%08x\n",
	       flash_size / 1024U, s_sector_size, s_page_size, s_cfg_addr);
	return 0;
}

/* 擦除配置扇区并写入 (大小按页对齐, 不足补 0xFF) */
uint8_t flash_write_config(const void *data, uint32_t size)
{
	uint32_t level;
	uint32_t prog_size;
	hpm_stat_t st;

	if (!s_inited || (size > sizeof(s_buf)) || (size > s_sector_size))
		return 1;

	prog_size = (size + s_page_size - 1U) / s_page_size * s_page_size;
	if (prog_size > sizeof(s_buf))
		prog_size = sizeof(s_buf);

	memset(s_buf, 0xFF, prog_size);
	memcpy(s_buf, data, size);

	/* 擦写期间不能取指/访问 Flash, 关全局中断 */
	level = disable_global_irq(CSR_MSTATUS_MIE_MASK);

	st = rom_xpi_nor_erase_sector(FLASH_XPI_BASE, xpi_xfer_channel_auto,
				      &s_nor_config, s_cfg_addr);
	if (st == status_success) {
		st = rom_xpi_nor_program(FLASH_XPI_BASE, xpi_xfer_channel_auto,
					 &s_nor_config, s_buf, s_cfg_addr,
					 prog_size);
	}

	restore_global_irq(level);

	return (st == status_success) ? 0 : 1;
}

uint8_t flash_read_config(void *data, uint32_t size)
{
	hpm_stat_t st;

	if (!s_inited || (size > s_sector_size) ||
	    (((uint32_t)data & 0x3U) != 0U)) {
		return 1;
	}
	memset(data, 0, size);

	st =
	    rom_xpi_nor_read(FLASH_XPI_BASE, xpi_xfer_channel_auto,
			     &s_nor_config, (uint32_t *)data, s_cfg_addr, size);
	return (st == status_success) ? 0 : 1;
}
