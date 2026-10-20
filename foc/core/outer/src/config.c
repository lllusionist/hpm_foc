#include <stddef.h>
#include <string.h>
#include "config.h"
#include "util.h"
#include "flash.h"

tConfig Config;
#define debug 0

/*
 * debug=1: 关掉斜坡/前馈, 便于阶跃标定外环。
 * 斜坡极大 => 设定值一拍到位; 前馈必须清零, 否则阶跃拍被放大成尖峰。
 */
static void config_debug_overrides(void)
{
	Config.velocity_ramp_rate = 1.0e6f; /* rps/s, 近似阶跃 */
	Config.current_ramp_rate = 1.0e6f;  /* A/s,   近似阶跃 */
	Config.current_ff_gain = 0.0f;	    /* 关加速度前馈, 避免阶跃尖峰 */
	Config.position_filter_bw = 200.0f; /* 位置输入滤波加快 (位置模式用) */
}

/* CRC 覆盖除末尾 crc32 字段以外的全部内容 */
uint32_t CONFIG_calculate_crc(const tConfig *cfg)
{
	return util_crc32((const uint8_t *)cfg, offsetof(tConfig, crc32));
}

uint8_t CONFIG_save_to_flash(void)
{
	Config.crc32 = CONFIG_calculate_crc(&Config);
	return flash_write_config(&Config, sizeof(tConfig));
}

uint8_t CONFIG_load_from_flash(void)
{
	tConfig tmp;

	if (flash_read_config(&tmp, sizeof(tConfig)) != 0)
		return 1;
	if (tmp.magic != CONFIG_MAGIC)
		return 2;
	if (tmp.crc32 != CONFIG_calculate_crc(&tmp))
		return 3;

	Config = tmp;
	if (debug)
		config_debug_overrides();
	return 0;
}

void config_init(void)
{
	memset(&Config, 0, sizeof(tConfig));

	Config.magic = CONFIG_MAGIC;
	Config.calib_valid = false;

	Config.invert_motor_dir = 0;
	Config.pole_pairs = 0;
	Config.phase_resistance = 5.0f; /* 默认值, 校准后覆盖 */
	Config.phase_inductance = 0.00164f;
	Config.velocity_limit = 30.0f;

	/* 当前电机: 线-线 15.2Ω/6.56mH (相 7.6Ω/3.28mH), 额定 0.59A。
	 * calib_current 决定拖动电压, 取额定的 85%; 过小会起转失败。
	 * calib_voltage 需 >= R×calib_current。 */
	Config.current_limit = 1.5f;  /* 额定的 2.5 倍 */
	Config.calib_current = 0.50f; /* 额定的 85%, 保证拖动转矩 */
	Config.calib_voltage = 5.0f;  /* R×I = 8.47×0.5 ≈ 4.24, 留裕量 */

	/* 航模 5210 (R=0.03Ω, L=9µH, 额定 44A) */
	/* Config.current_limit   = 8.0f; */
	/* Config.calib_current   = 5.0f; */
	/* Config.calib_voltage   = 1.0f; */

	/* 云台 4310 (R=5.0Ω, L=1.5mH, 额定 0.9A) */
	/* Config.current_limit   = 3.0f; */
	/* Config.calib_current   = 0.3f; */
	/* Config.calib_voltage   = 3.6f; */

	Config.default_op_mode = CONTROL_MODE_POSITION_FILTER;
	Config.pos_p_gain = 5.0f;
	/* 裸云台电机: 原 8.0/78.3 会与 100Hz 编码器 PLL 自激 */
	Config.vel_p_gain = 0.03f;
	Config.vel_i_gain = 1.5f;
	Config.current_ff_gain = 0.001f;
	/* L/R: 本电机 432µs -> 极点 368Hz, 取 300 留裕量 */
	Config.current_ctrl_bw = 300;
	Config.anticogging_enable = 1;
	Config.target_velcity_window = 0.5f;
	Config.target_position_window = 0.01f;
	Config.current_ramp_rate = 25.0f;
	Config.velocity_ramp_rate = 50.0f;
	Config.position_filter_bw = 10;
	Config.profile_velocity = 50;
	Config.profile_accel = 50;
	Config.profile_decel = 50;

	Config.protect_under_voltage = 9.0f;
	Config.protect_over_voltage = 15.0f;
	Config.protect_over_current = 2.0f; /* 额定的 3.4 倍 */
	Config.protect_i_bus_max = 1.5f;

	Config.node_id = 1;

	Config.cogging_map_crc = 0;
	Config.cogging_map_valid = 0;

	Config.current_gain_magic = 0;
	Config.current_kp = 0.0f;
	Config.current_ki = 0.0f;

	Config.crc32 = CONFIG_calculate_crc(&Config);

	if (debug)
		config_debug_overrides();
}