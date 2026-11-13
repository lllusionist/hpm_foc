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
static uint32_t s_sector_size;
static uint32_t s_page_size;
/* 两个扇区的起始地址: 倒数第 2 和倒数第 1 */
static uint32_t s_cfg_addr_a;
static uint32_t s_cfg_addr_b;
/* 当前应该写哪个扇区 (0=A, 1=B); flash_read_config 更新它 */
static uint8_t s_active_sector;

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

	/* A/B 两个扇区: 倒数第 2 和倒数第 1 */
	s_cfg_addr_a = flash_size - 2U * s_sector_size;
	s_cfg_addr_b = flash_size - 1U * s_sector_size;
	s_active_sector = 0;
	s_inited = true;

	printf("flash: %uKB, sector %uB, page %uB, cfgA@0x%08x cfgB@0x%08x\n",
	       flash_size / 1024U, s_sector_size, s_page_size,
	       s_cfg_addr_a, s_cfg_addr_b);
	return 0;
}

/* 在扇区内查找有效配置头: magic 匹配且 generation > prev_gen */
static bool check_sector(uint32_t base_addr, uint32_t *out_gen,
			  void *out_data, uint32_t size)
{
	flash_cfg_header_t hdr;
	hpm_stat_t st;

	st = rom_xpi_nor_read(FLASH_XPI_BASE, xpi_xfer_channel_auto,
			      &s_nor_config, (uint32_t *)&hdr, base_addr,
			      sizeof(hdr));
	if (st != status_success || hdr.magic != FLASH_DUAL_SECTOR_MAGIC)
		return false;

	st = rom_xpi_nor_read(FLASH_XPI_BASE, xpi_xfer_channel_auto,
			      &s_nor_config, (uint32_t *)out_data,
			      base_addr + sizeof(hdr), size);
	if (st != status_success)
		return false;

	*out_gen = hdr.generation;
	return true;
}

/* 向指定扇区写入: 先擦后写, 全程关中断 */
static uint8_t write_sector(uint32_t base_addr, uint32_t generation,
			    const void *data, uint32_t size)
{
	uint32_t level;
	hpm_stat_t st;
	static uint32_t buf[FLASH_CFG_BUF_SIZE / 4U + 2U]; /* +2 for header */
	uint32_t total = size + sizeof(flash_cfg_header_t);

	/* 按页对齐 */
	uint32_t prog_size = (total + s_page_size - 1U) / s_page_size *
			     s_page_size;
	if (prog_size > sizeof(buf))
		prog_size = sizeof(buf);

	/* 组装: header + data */
	flash_cfg_header_t hdr = { .magic = FLASH_DUAL_SECTOR_MAGIC,
				   .generation = generation };
	memcpy(buf, &hdr, sizeof(hdr));
	memcpy((uint8_t *)buf + sizeof(hdr), data, size);
	/* 补 0xFF */
	memset((uint8_t *)buf + total, 0xFF, prog_size - total);

	level = disable_global_irq(CSR_MSTATUS_MIE_MASK);

	st = rom_xpi_nor_erase_sector(FLASH_XPI_BASE, xpi_xfer_channel_auto,
				      &s_nor_config, base_addr);
	if (st == status_success) {
		st = rom_xpi_nor_program(FLASH_XPI_BASE, xpi_xfer_channel_auto,
					 &s_nor_config, buf, base_addr,
					 prog_size);
	}

	restore_global_irq(level);
	return (st == status_success) ? 0 : 1;
}

uint8_t flash_write_config(const void *data, uint32_t size)
{
	if (!s_inited || (size + sizeof(flash_cfg_header_t) > s_sector_size))
		return 1;

	/* 交替写: 上次写 A, 这次写 B, 反之亦然.
	 * generation 每次递增; 读时取大者 */
	uint32_t target_addr;
	uint32_t gen = 0;
	uint32_t old_gen;

	/* 读另一份的 generation 作为递增基数 */
	uint8_t other = s_active_sector ^ 1U;
	uint32_t other_addr = other ? s_cfg_addr_b : s_cfg_addr_a;
	flash_cfg_header_t other_hdr;
	hpm_stat_t st = rom_xpi_nor_read(FLASH_XPI_BASE, xpi_xfer_channel_auto,
					 &s_nor_config, (uint32_t *)&other_hdr,
					 other_addr, sizeof(other_hdr));
	if (st == status_success && other_hdr.magic == FLASH_DUAL_SECTOR_MAGIC)
		gen = other_hdr.generation + 1U;
	else
		gen = 1U;

	/* 写到对面的扇区 */
	target_addr = other ? s_cfg_addr_b : s_cfg_addr_a;
	uint8_t ret = write_sector(target_addr, gen, data, size);
	if (ret == 0) {
		s_active_sector = other;
		(void)old_gen; /* unused */
	}
	return ret;
}

uint8_t flash_read_config(void *data, uint32_t size)
{
	if (!s_inited || (size + sizeof(flash_cfg_header_t) > s_sector_size) ||
	    (((uint32_t)data & 0x3U) != 0U))
		return 1;

	/* 两个扇区都读, 取 generation 大且 CRC 有效的那份 */
	uint32_t gen_a = 0, gen_b = 0;
	uint8_t buf_a[FLASH_CFG_BUF_SIZE];
	uint8_t buf_b[FLASH_CFG_BUF_SIZE];
	bool ok_a, ok_b;

	ok_a = check_sector(s_cfg_addr_a, &gen_a, buf_a, size);
	ok_b = check_sector(s_cfg_addr_b, &gen_b, buf_b, size);

	if (ok_a && ok_b) {
		if (gen_a >= gen_b) {
			memcpy(data, buf_a, size);
			s_active_sector = 1; /* 下次写 B */
		} else {
			memcpy(data, buf_b, size);
			s_active_sector = 0; /* 下次写 A */
		}
		return 0;
	} else if (ok_a) {
		memcpy(data, buf_a, size);
		s_active_sector = 1;
		return 0;
	} else if (ok_b) {
		memcpy(data, buf_b, size);
		s_active_sector = 0;
		return 0;
	}

	return 1; /* 两份都无效 */
}
