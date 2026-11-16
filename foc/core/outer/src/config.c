#include <stddef.h>
#include <string.h>
#include "config.h"
#include "util.h"
#include "flash.h"

tConfig Config;
#define debug 0

_Static_assert(sizeof(tConfig) <= FLASH_CFG_BUF_SIZE,
	       "tConfig exceeds flash cfg buffer");

static void config_debug_overrides(void)
{
	Config.velocity_ramp_rate = 1.0e6f; // rps/s
	Config.current_ramp_rate = 1.0e6f;  // A/s
	Config.current_ff_gain = 0.0f;	    // 前馈不清零会放大阶跃尖峰
	Config.position_filter_bw = 200.0f;
}

void CONFIG_apply_motor_profile(int32_t profile)
{
	if (profile == MOTOR_PROFILE_5210) {
		// 5210: KV300, 44A(本板限 ±16.5A)
		Config.phase_resistance = 0.030f;
		Config.phase_inductance = 9.0e-6f;
		// 极对数/磁链待标定
		Config.pole_pairs = 0;
		Config.flux_linkage = 0.0f;
		Config.current_limit = 8.0f;
		Config.velocity_limit = 40.0f;
		Config.protect_over_current = 12.0f;
		Config.protect_i_bus_max = 12.0f;
		Config.protect_under_voltage = 18.0f;
		Config.protect_over_voltage = 28.0f;
		// 死区 48mV, 需 2 级以上差分
		Config.calib_current = 3.0f;
		Config.calib_voltage = 1.0f;
		// 5210 的电气极点 R/(2πL) ≈ 0.042/(2π·10µH) = 670Hz, 1000Hz 交叉
		// 已经越过极点; 实测 0x27 强托时电流环在 ~2.5kHz 自激 (Id±0.85/
		// Iq 3±1.4A, 幅值只受死区非线性限制, 峰值撞 12A 过流).
		// 注意 L 标定值会偏大 (注入电压被死区吃掉), Kp = L_meas·ωc,
		// 实际交叉频率 = ωc·L_meas/L_real, 偏大 2~3 倍; 所以这里取 300Hz,
		// 落到实际 ~800Hz. 等 L 标定修准后再往上调.
		Config.current_ctrl_bw = 300.0f;
		Config.current_ramp_rate = 200.0f;
		Config.velocity_ramp_rate = 50.0f;
		// Kt 为 4310 的 1/8.6, 增益按 J≈1.5e-4
		Config.vel_p_gain = 3.31f;
		Config.vel_i_gain = 104.1f;
		Config.calib_drag_v_min = 0.55f; // 只用于打印
		// 同速校验时不被齿槽卡停
		Config.calib_flux_iq_gain = 1.5f;
		// 4.5A 起转, 0.15s 窗防冲过 12rps
		// 实测 Pn=14 (dir_pp), 4.5A 起转会 2 倍过冲到 9A 撞软限流;
		// J 只打印不参与 calib_valid, 降流把加速段做温和一点 (1.8A)
		Config.calib_j_iq_gain = 0.60f;
		Config.calib_flux_rtest_ratio = 0.7f;
		// 0.1rps 会粘滑
		Config.cog_sweep_rps = 0.30f;
		Config.motor_profile = MOTOR_PROFILE_5210;
		// 速度积分器 / 摩擦前馈随电机档位配置
		Config.vel_integrator_limit = 4.0f; // 5210 限流 8A, 积分器半幅
		Config.friction_move_a = 0.08f;
		Config.friction_break_a = 0.12f;
	} else if (profile == MOTOR_PROFILE_2312) {
		// 2312: 12N14P, KV960(Kt 为 5210 的 1/3)
		Config.phase_resistance = 0.144f;
		// 相间 57µH 折半(mH 则改 28.5e-3f)
		Config.phase_inductance = 28.5e-6f;
		// 极对数已知, 磁链待标定
		Config.pole_pairs = 7;
		Config.flux_linkage = 0.0f;
		Config.current_limit = 8.0f;
		Config.velocity_limit = 50.0f;
		Config.protect_over_current = 12.0f;
		Config.protect_i_bus_max = 8.0f;
		Config.protect_under_voltage = 18.0f;
		Config.protect_over_voltage = 28.0f;
		// 4A 对应 34mN·m, 仅拖动段有此铜耗
		Config.calib_current = 4.0f;
		// 每级 0.15V -> ΔI≈1A, 差分消掉死区
		Config.calib_voltage = 1.5f;
		// 极点 804Hz, 取 600
		Config.current_ctrl_bw = 600.0f;
		Config.current_ramp_rate = 200.0f;
		Config.velocity_ramp_rate = 50.0f;
		// 按 J≈1e-5 折算
		Config.vel_p_gain = 0.5f;
		Config.vel_i_gain = 16.0f;
		Config.calib_drag_v_min = 0.20f; // 只用于打印
		Config.calib_flux_iq_gain = 1.0f;
		// 0.15s 窗内到 12rps
		Config.calib_j_iq_gain = 0.15f;
		Config.calib_flux_rtest_ratio = 0.7f;
		// 84 格/圈, 21Hz
		Config.cog_sweep_rps = 0.25f;
		Config.motor_profile = MOTOR_PROFILE_2312;
		Config.vel_integrator_limit = 3.0f;
		Config.friction_move_a = 0.05f;
		Config.friction_break_a = 0.08f;
	} else {
		// 4310 云台, 0.9A
		Config.phase_resistance = 8.29f;
		Config.phase_inductance = 0.0034f;
		Config.pole_pairs = 14;
		Config.flux_linkage = 0.013024f;
		Config.current_limit = 1.5f;
		Config.velocity_limit = 30.0f;
		Config.protect_over_current = 2.0f;
		Config.protect_i_bus_max = 1.5f;
		Config.protect_under_voltage = 18.0f;
		Config.protect_over_voltage = 28.0f;
		Config.calib_current = 0.5f;
		Config.calib_voltage = 10.0f;
		Config.current_ctrl_bw = 300.0f;
		Config.current_ramp_rate = 25.0f;
		Config.velocity_ramp_rate = 50.0f;
		Config.vel_p_gain = 0.17f;
		Config.vel_i_gain = 5.3f;
		Config.calib_drag_v_min = 0.20f; // 只用于打印
		Config.calib_flux_iq_gain = 0.8f;
		Config.calib_j_iq_gain = 0.15f;
		Config.calib_flux_rtest_ratio = 0.5f;
		Config.cog_sweep_rps = 0.10f;
		Config.motor_profile = MOTOR_PROFILE_4310;
		Config.vel_integrator_limit = 0.5f; // 4310 限流 1.5A
		Config.friction_move_a = 0.030f;
		Config.friction_break_a = 0.050f;
	}

	// 切换电机配置档后，原电机的标定数据失效。
	Config.calib_valid = 0;
	Config.cogging_map_valid = 0;
	Config.anticogging_enable = 0;
}

// CRC 覆盖 crc32 字段之前的数据，不包含 crc32 本身。
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
	/* magic 和 version 双重检查: magic 变更确保 v1 记录失效 */
	if (tmp.magic != CONFIG_MAGIC)
		return 2;
	if (tmp.version != CONFIG_VERSION)
		return 4; /* 布局版本不符, 回退默认值 */
	/* CRC 现在覆盖全部字段 (crc32 本身除外) */
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
	Config.version = CONFIG_VERSION;

	/* 通用参数 (与电机型号无关) */
	Config.default_op_mode = CONTROL_MODE_POSITION_FILTER;
	Config.pos_p_gain = 5.0f;
	Config.current_ff_gain = 0.001f;
	Config.anticogging_enable = 1;
	Config.target_velcity_window = 0.5f;
	Config.target_position_window = 0.01f;
	Config.position_filter_bw = 10.0f;
	Config.profile_velocity = 50.0f;
	Config.profile_accel = 50.0f;
	Config.profile_decel = 50.0f;
	Config.protect_under_voltage = 18.0f;
	Config.protect_over_voltage = 28.0f;
	Config.node_id = 1;
	Config.cogging_map_crc = 0;
	Config.cogging_map_valid = 0;
	Config.current_kp = 0.0f;
	Config.current_ki = 0.0f;

	/* 电机相关参数由档位表统一填充, 避免 config_init 和
	 * CONFIG_apply_motor_profile 重复维护两份 4310 参数 */
	CONFIG_apply_motor_profile(MOTOR_PROFILE_4310);

	/* apply 会把 calib_valid 清零, 这里保持 false */
	Config.calib_valid = false;

	Config.crc32 = CONFIG_calculate_crc(&Config);

	if (debug)
		config_debug_overrides();
}
