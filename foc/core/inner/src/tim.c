#include "tim.h"
#include "foc.h"

static uint32_t g_pwm_clk_hz; /* PWM计数时钟(=AHB) */
static uint32_t g_dead_ticks; /* 死区(计数周期数) */

uint32_t reload; /* 载波周期 */

/* 纳秒换算为 PWM 计数，结果向上取整。 */
static uint32_t pwm_ns_to_ticks(uint32_t ns)
{
	uint64_t ticks =
	    ((uint64_t)ns * (uint64_t)g_pwm_clk_hz + 999999999ULL) /
	    1000000000ULL;

	if (ticks < 1U)
		ticks = 1U;
	if (ticks > 0xFFFFFFU)
		ticks = 0xFFFFFFU;
	return (uint32_t)ticks;
}

/* 死区计数限制在硬件支持范围内。 */
static uint32_t pwm_dead_ns_to_ticks(uint32_t ns)
{
	uint32_t ticks = pwm_ns_to_ticks(ns);

	if (ticks < 2U)
		ticks = 2U;
	if (ticks > 0xFFFFU)
		ticks = 0xFFFFU;
	return ticks;
}

/* 死区约束下的最大占空比。 */
float pwm_get_duty_max(void)
{
	if ((reload + 1U) <= (4U * g_dead_ticks))
		return 0.0f; /* 死区过大,无可用范围 */
	return 1.0f -
	       (2.0f * (float)g_dead_ticks + 4.0f) / (float)(reload + 1U);
}

/* 最小脉宽对应的占空比下限。 */
float pwm_get_duty_min(void)
{
	return (float)pwm_ns_to_ticks(PWM_MIN_ON_NS) / (float)(reload + 1U);
}

/* 配置 PWM 故障输出及恢复方式。 */
static void pwm_fault_config(void)
{
	for (uint8_t ch = 0; ch < 6; ch++) {
		pwmv2_set_fault_mode(
		    PWM, (pwm_channel_t)ch,
		    pwm_fault_output_0); /* 故障输出0(下桥低有效改output_1) */
		pwmv2_set_fault_recovery_time(
		    PWM, (pwm_channel_t)ch,
		    pwm_fault_recovery_on_fault_clear); /* 软件清故障后恢复 */
	}

	/* pwmv2_enable_debug_mode(PWM); // 调试暂停时自动进入故障状态 */
}

void pwm_init(void)
{
	/* 计算 PWM 周期和死区。 */
	g_pwm_clk_hz = clock_get_frequency(clock_pwm0);
	g_dead_ticks = pwm_dead_ns_to_ticks(PWM_DEAD_TIME_NS);
	reload = g_pwm_clk_hz / PWM_FREQ_HZ - 1; /* 周期=(reload+1)个时钟 */

	if ((g_pwm_clk_hz == 0U) || (reload <= (4U * g_dead_ticks))) {
		/* 参数不合理 */
		printf("pwm_init: clock/dead-time config "
		       "invalid!\n");
	}

	/* PWM引脚复用 */
	HPM_IOC->PAD[IOC_PAD_PD00].FUNC_CTL = IOC_PD00_FUNC_CTL_PWM0_P_0;
	HPM_IOC->PAD[IOC_PAD_PD01].FUNC_CTL = IOC_PD01_FUNC_CTL_PWM0_P_1;
	HPM_IOC->PAD[IOC_PAD_PD02].FUNC_CTL = IOC_PD02_FUNC_CTL_PWM0_P_2;
	HPM_IOC->PAD[IOC_PAD_PD03].FUNC_CTL = IOC_PD03_FUNC_CTL_PWM0_P_3;
	HPM_IOC->PAD[IOC_PAD_PD04].FUNC_CTL = IOC_PD04_FUNC_CTL_PWM0_P_4;
	HPM_IOC->PAD[IOC_PAD_PD05].FUNC_CTL = IOC_PD05_FUNC_CTL_PWM0_P_5;

	pwmv2_deinit(PWM);
	pwmv2_enable_shadow_lock_feature(PWM);
	pwmv2_shadow_register_unlock(PWM);

	/* 影子寄存器 */
	pwmv2_set_shadow_val(PWM, PWMV2_SHADOW_INDEX(0), reload, 0,
			     false); /* 周期(三相共用) */
	pwmv2_set_shadow_val(PWM, PWMV2_SHADOW_INDEX(1), reload, 0,
			     false); /* U相开通点 */
	pwmv2_set_shadow_val(PWM, PWMV2_SHADOW_INDEX(2), reload, 0,
			     false); /* U相关断点 */
	pwmv2_set_shadow_val(PWM, PWMV2_SHADOW_INDEX(3), reload, 0,
			     false); /* V相开通点 */
	pwmv2_set_shadow_val(PWM, PWMV2_SHADOW_INDEX(4), reload, 0,
			     false); /* V相关断点 */
	pwmv2_set_shadow_val(PWM, PWMV2_SHADOW_INDEX(5), reload, 0,
			     false); /* W相开通点 */
	pwmv2_set_shadow_val(PWM, PWMV2_SHADOW_INDEX(6), reload, 0,
			     false); /* W相关断点 */
	/* 初值a=b=周期,上管全关下管全开 */

	/* ADC触发 */
	pwmv2_set_shadow_val(PWM, PWMV2_SHADOW_INDEX(7), 1, 0, false);
	pwmv2_select_cmp_source(
	    PWM, PWMV2_CMP_INDEX(20), cmp_value_from_shadow_val,
	    PWMV2_SHADOW_INDEX(7)); /* 比较器0~15计数器绑死 */
	pwmv2_cmp_select_counter(PWM, PWMV2_CMP_INDEX(20), pwm_counter_0);
	pwmv2_set_trigout_cmp_index(PWM, 0, PWMV2_CMP_INDEX(20));
	pwmv2_cmp_update_trig_time(PWM, PWMV2_CMP_INDEX(20),
				   pwm_shadow_register_update_on_reload);

	/* 计数器周期来源 */
	pwmv2_counter_select_data_offset_from_shadow_value(
	    PWM, pwm_counter_0, PWMV2_SHADOW_INDEX(0)); /* 三相共用 */
	pwmv2_counter_select_data_offset_from_shadow_value(
	    PWM, pwm_counter_1, PWMV2_SHADOW_INDEX(0));
	pwmv2_counter_select_data_offset_from_shadow_value(
	    PWM, pwm_counter_2, PWMV2_SHADOW_INDEX(0));

	/* 比较器绑定:通道N用比较点N*2和N*2+1 */
	pwmv2_select_cmp_source(PWM, PWMV2_CMP_INDEX(0),
				cmp_value_from_shadow_val,
				PWMV2_SHADOW_INDEX(1)); /* U相a */
	pwmv2_select_cmp_source(PWM, PWMV2_CMP_INDEX(1),
				cmp_value_from_shadow_val,
				PWMV2_SHADOW_INDEX(2)); /* U相b */
	pwmv2_select_cmp_source(PWM, PWMV2_CMP_INDEX(4),
				cmp_value_from_shadow_val,
				PWMV2_SHADOW_INDEX(3)); /* V相a */
	pwmv2_select_cmp_source(PWM, PWMV2_CMP_INDEX(5),
				cmp_value_from_shadow_val,
				PWMV2_SHADOW_INDEX(4)); /* V相b */
	pwmv2_select_cmp_source(PWM, PWMV2_CMP_INDEX(8),
				cmp_value_from_shadow_val,
				PWMV2_SHADOW_INDEX(5)); /* W相a */
	pwmv2_select_cmp_source(PWM, PWMV2_CMP_INDEX(9),
				cmp_value_from_shadow_val,
				PWMV2_SHADOW_INDEX(6)); /* W相b */

	/* 关burst(否则只发1个周期) */
	pwmv2_counter_burst_disable(PWM, pwm_counter_0);
	pwmv2_counter_burst_disable(PWM, pwm_counter_1);
	pwmv2_counter_burst_disable(PWM, pwm_counter_2);

	/* 周期值在重载点更新 */
	pwmv2_set_reload_update_time(PWM, pwm_counter_0,
				     pwm_reload_update_on_reload);
	pwmv2_set_reload_update_time(PWM, pwm_counter_1,
				     pwm_reload_update_on_reload);
	pwmv2_set_reload_update_time(PWM, pwm_counter_2,
				     pwm_reload_update_on_reload);

	/* 比较值在重载点统一生效 */
	pwmv2_cmp_update_trig_time(PWM, PWMV2_CMP_INDEX(0),
				   pwm_shadow_register_update_on_reload);
	pwmv2_cmp_update_trig_time(PWM, PWMV2_CMP_INDEX(1),
				   pwm_shadow_register_update_on_reload);
	pwmv2_cmp_update_trig_time(PWM, PWMV2_CMP_INDEX(4),
				   pwm_shadow_register_update_on_reload);
	pwmv2_cmp_update_trig_time(PWM, PWMV2_CMP_INDEX(5),
				   pwm_shadow_register_update_on_reload);
	pwmv2_cmp_update_trig_time(PWM, PWMV2_CMP_INDEX(8),
				   pwm_shadow_register_update_on_reload);
	pwmv2_cmp_update_trig_time(PWM, PWMV2_CMP_INDEX(9),
				   pwm_shadow_register_update_on_reload);

	/* 开互补模式 */
	pwmv2_enable_pair_mode(PWM, pwm_channel_0); /* U上 */
	pwmv2_enable_pair_mode(PWM, pwm_channel_1); /* U下 */
	pwmv2_enable_pair_mode(PWM, pwm_channel_2); /* V上 */
	pwmv2_enable_pair_mode(PWM, pwm_channel_3); /* V下 */
	pwmv2_enable_pair_mode(PWM, pwm_channel_4); /* W上 */
	pwmv2_enable_pair_mode(PWM, pwm_channel_5); /* W下 */

	/* 死区 */
	pwmv2_set_dead_area(PWM, pwm_channel_0, g_dead_ticks);
	pwmv2_set_dead_area(PWM, pwm_channel_1, g_dead_ticks);
	pwmv2_set_dead_area(PWM, pwm_channel_2, g_dead_ticks);
	pwmv2_set_dead_area(PWM, pwm_channel_3, g_dead_ticks);
	pwmv2_set_dead_area(PWM, pwm_channel_4, g_dead_ticks);
	pwmv2_set_dead_area(PWM, pwm_channel_5, g_dead_ticks);

	/* 每个通道使用两个比较点。 */
	pwmv2_disable_four_cmp(PWM, pwm_channel_0);
	pwmv2_disable_four_cmp(PWM, pwm_channel_1);
	pwmv2_disable_four_cmp(PWM, pwm_channel_2);
	pwmv2_disable_four_cmp(PWM, pwm_channel_3);
	pwmv2_disable_four_cmp(PWM, pwm_channel_4);
	pwmv2_disable_four_cmp(PWM, pwm_channel_5);

	/* 配置 PWM 故障输出及恢复方式。 */
	pwm_fault_config();

	/* 使能输出 */
	pwmv2_channel_enable_output(PWM, pwm_channel_0);
	pwmv2_channel_enable_output(PWM, pwm_channel_1);
	pwmv2_channel_enable_output(PWM, pwm_channel_2);
	pwmv2_channel_enable_output(PWM, pwm_channel_3);
	pwmv2_channel_enable_output(PWM, pwm_channel_4);
	pwmv2_channel_enable_output(PWM, pwm_channel_5);

	/* 锁定影子寄存器 */
	pwmv2_shadow_register_lock(PWM);

	/* 三相同步启动 */
	pwmv2_reset_multi_counter_sync(PWM, 0x07);  /* 复位CNT0/1/2 */
	pwmv2_enable_multi_counter_sync(PWM, 0x07); /* 同时使能 */
	pwmv2_start_pwm_output_sync(PWM, 0x07);	    /* 同时启动 */
}

/* 占空比限制在 0.96，为低边电流采样保留至少 4% 周期窗口。 */
#define PWM_DUTY_MAX_LIMIT (0.96f)

void pwm_set_duty(float du, float dv, float dw)
{
	if (du > PWM_DUTY_MAX_LIMIT)
		du = PWM_DUTY_MAX_LIMIT;
	if (dv > PWM_DUTY_MAX_LIMIT)
		dv = PWM_DUTY_MAX_LIMIT;
	if (dw > PWM_DUTY_MAX_LIMIT)
		dw = PWM_DUTY_MAX_LIMIT;

	uint32_t half = reload >> 1;
	/* 每相导通宽度的一半 */
	uint32_t hu = (uint32_t)(du * (float)reload) >> 1;
	uint32_t hv = (uint32_t)(dv * (float)reload) >> 1;
	uint32_t hw = (uint32_t)(dw * (float)reload) >> 1;

	pwmv2_shadow_register_unlock(PWM);
	pwmv2_set_shadow_val(PWM, PWMV2_SHADOW_INDEX(1), half - hu, 0,
			     false); /* U相开通点 */
	pwmv2_set_shadow_val(PWM, PWMV2_SHADOW_INDEX(2), half + hu, 0,
			     false); /* U相关断点 */
	pwmv2_set_shadow_val(PWM, PWMV2_SHADOW_INDEX(3), half - hv, 0,
			     false); /* V相开通点 */
	pwmv2_set_shadow_val(PWM, PWMV2_SHADOW_INDEX(4), half + hv, 0,
			     false); /* V相关断点 */
	pwmv2_set_shadow_val(PWM, PWMV2_SHADOW_INDEX(5), half - hw, 0,
			     false); /* W相开通点 */
	pwmv2_set_shadow_val(PWM, PWMV2_SHADOW_INDEX(6), half + hw, 0,
			     false); /* W相关断点 */
	pwmv2_shadow_register_lock(PWM);
}
