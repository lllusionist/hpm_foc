#include <stddef.h>
#include <string.h>
#include "config.h"
#include "util.h"
#include "flash.h"

tConfig Config;
#define debug 0

/* 配置必须能放进 Flash 缓冲区, 否则 save 会静默失败 */
_Static_assert(sizeof(tConfig) <= FLASH_CFG_BUF_SIZE,
	       "tConfig exceeds flash cfg buffer");

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

/* I/F 强拖缺省参数: 云台/低速电机通用初值, 上板后按实际负载改 */
static void config_if_defaults(void)
{
	Config.if_cfg_magic = CONFIG_MAGIC;
	Config.if_current = 0.45f;    /* 额定的 ~75%, 压得住静摩擦 */
	Config.if_accel_rps2 = 5.0f;  /* 0 -> 2rps 约 0.4s */
	Config.if_switch_rps = 2.0f;  /* 反电势明显、编码器环平滑的转速 */
	Config.if_hold_ms = 200.0f;   /* 稳态保持后再交接, 留给环路收敛 */
	Config.if_blend_ms = 100.0f;  /* 交接相位收敛 */
	Config.if_stall_deg = 60.0f;  /* 失步角 (电角度), 留 30° 余量 */
}

/* 机械参数缺省值 (J 和摩擦电流都由标定/运行自动学出来) */
static void config_mech_defaults(void)
{
	Config.mech_cfg_magic = CONFIG_MAGIC;
	Config.inertia = 0.0f;		/* 未知, 标定后写入 */
	Config.friction_current = 0.0f; /* 未知, 稳速运行后自动记录 */
}

/* CRC 覆盖除末尾 crc32 字段以外的全部内容 */
uint32_t CONFIG_calculate_crc(const tConfig *cfg)
{
	return util_crc32((const uint8_t *)cfg, offsetof(tConfig, crc32));
}

uint8_t CONFIG_save_to_flash(void)
{
	Config.crc32 = CONFIG_calculate_crc(&Config);
	uint8_t ret = flash_write_config(&Config, sizeof(tConfig));

	if (ret)
		printf("config: flash save failed (%u)\n", ret);
	return ret;
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
	/* 旧 Flash 记录里 crc32 之后的字段是垃圾值, 用 magic 判定后套默认 */
	if (Config.if_cfg_magic != CONFIG_MAGIC)
		config_if_defaults();
	if (Config.mech_cfg_magic != CONFIG_MAGIC)
		config_mech_defaults();
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
	/* 4310 云台电机官方值: 相间 R=13.84Ω / L=6.45mH -> 相值 6.92Ω / 3.225mH
	 * (本工程的 R/L 都是"相"量; 标定会覆盖这里) */
	Config.phase_resistance = 6.92f;
	Config.phase_inductance = 0.003225f;
	Config.velocity_limit = 30.0f;

	/* 当前电机: 相 6.9Ω/3.2mH, 额定 0.9A。
	 * calib_current 决定拖动电压, 取额定的 55%; 过小会起转失败。
	 * calib_voltage 24V 母线取 10V: R 步进和 L 注入幅值都由它派生,
	 * 5V 时 L 的交流幅值只剩 0.25V, 测出来只有正确值的一半 */
	Config.current_limit = 1.5f;   /* 额定的 1.7 倍 */
	Config.calib_current = 0.50f;  /* 额定的 55%, 保证拖动转矩 */
	Config.calib_voltage = 10.0f;  /* R×I = 6.92×0.5 ≈ 3.5, 留 AC 注入余量 */

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
	/* PLL 300Hz 下整定: 原 8.0/78.3 会自激 */
	Config.vel_p_gain = 0.17f;
	Config.vel_i_gain = 5.3f;
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

	/* 24V 母线 (2026-11 改用 24V 电源; 12V 时过压 15V/欠压 9V) */
	Config.protect_under_voltage = 18.0f;
	Config.protect_over_voltage = 28.0f;
	Config.protect_over_current = 2.0f; /* 额定的 3.4 倍 */
	Config.protect_i_bus_max = 1.5f;

	Config.node_id = 1;

	Config.cogging_map_crc = 0;
	Config.cogging_map_valid = 0;

	Config.current_gain_magic = 0;
	Config.current_kp = 0.0f;
	Config.current_ki = 0.0f;

	config_if_defaults();
	config_mech_defaults();

	Config.crc32 = CONFIG_calculate_crc(&Config);

	if (debug)
		config_debug_overrides();
}