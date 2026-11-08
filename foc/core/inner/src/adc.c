#include "adc.h"
#include "foc.h"

#define ADC_IRQ IRQn_ADC0
#define ADC_TRIG_CH ADC16_CONFIG_TRG0A
#define ADC_TRIG_LEN (3U)

#define ADC_CH_IA (0U)	 /* PF27 */
#define ADC_CH_IB (1U)	 /* PF26 */
#define ADC_CH_IC (2U)	 /* PF21 */
#define ADC_CH_IBUS (3U) /* PF20 */

/* 采样保持 20→10: 三路串行转换越短, 越不容易撞上开关沿(Ic 最危险) */
#define ADC_SAMPLE_CYCLE (10U)
#define ADC_PMT_ERROR_MASK \
	(ADC16_INT_STS_AHB_ERR_MASK | ADC16_INT_STS_DMA_FIFO_FULL_MASK | \
	 ADC16_INT_STS_TRIG_SW_CFLCT_MASK | ADC16_INT_STS_TRIG_HW_CFLCT_MASK)

static const uint8_t adc_ch_tab[ADC_TRIG_LEN] = {ADC_CH_IA, ADC_CH_IB,
						 ADC_CH_IC};

static adc_callback_t adc_callback;
static volatile uint32_t current_snapshot_seq;
static volatile uint16_t current_snapshot[ADC_TRIG_LEN];

ATTR_PLACE_AT_NONCACHEABLE_BSS_WITH_ALIGNMENT(ADC_SOC_DMA_ADDR_ALIGNMENT)
static volatile adc16_pmt_dma_data_t
    adc_dma_buf[ADC_SOC_PMT_MAX_DMA_BUFF_LEN_IN_4BYTES];
static volatile uint32_t bad_current_frames;

bool adc_current_snapshot(uint16_t *ia, uint16_t *ib, uint16_t *ic)
{
	for (;;) {
		uint32_t before = current_snapshot_seq;
		if (before & 1U)
			continue;
		__asm volatile("fence r, rw" ::: "memory");
		*ia = current_snapshot[0];
		*ib = current_snapshot[1];
		*ic = current_snapshot[2];
		__asm volatile("fence r, rw" ::: "memory");
		uint32_t after = current_snapshot_seq;
		if (before == after && !(after & 1U))
			return FOC.adc_current_frames != 0U;
	}
}

void adc_init(void)
{
	adc16_config_t cfg;
	adc16_channel_config_t ch_cfg;
	adc16_pmt_config_t pmt_cfg;
	trgm_output_t trgm_cfg;

	clock_add_to_group(clock_adc0, 0);

	/* ADC 时钟源选 ahb0(200MHz) */
	clock_set_adc_source(clock_adc0, clk_adc_src_ahb0);

	adc16_get_default_config(&cfg);

	cfg.res = adc16_res_16_bits;		    /* 分辨率 16 位 */
	cfg.conv_mode = adc16_conv_mode_preemption; /* 工作模式: 抢占(PMT) */
	cfg.adc_clk_div = adc16_clock_divider_4; /* ADC 时钟 = 源/4 = 50MHz */
	cfg.sel_sync_ahb = true; /* ADC 时钟来自 AHB(与上面 ahb0 对应) */
	cfg.adc_ahb_en = true;	 /* 打开 ADC 内置 DMA */

	adc16_init(HPM_ADC0, &cfg);

	adc16_get_channel_default_config(&ch_cfg);

	ch_cfg.sample_cycle = ADC_SAMPLE_CYCLE;
	for (uint8_t i = 0; i < ADC_TRIG_LEN; i++) {
		ch_cfg.ch = adc_ch_tab[i];
		adc16_init_channel(HPM_ADC0, &ch_cfg);
	}

	pmt_cfg.trig_ch = ADC_TRIG_CH;	 /* 用哪个触发源(TRG0A) */
	pmt_cfg.trig_len = ADC_TRIG_LEN; /* 队列长度 */
	for (uint8_t i = 0; i < ADC_TRIG_LEN; i++) {
		pmt_cfg.adc_ch[i] = adc_ch_tab[i]; /* 第 i 个位置转哪个通道 */
		pmt_cfg.inten[i] = false;	   /* 该通道转完是否产生中断 */
	}

	pmt_cfg.inten[ADC_TRIG_LEN - 1U] = true;

	adc16_set_pmt_config(HPM_ADC0, &pmt_cfg);
	adc16_init_pmt_dma(HPM_ADC0, (uint32_t)adc_dma_buf);
	adc16_enable_pmt_queue(HPM_ADC0, ADC_TRIG_CH); /* 使能 TRG0A 这个队列 */

	/* 开中断 */
	adc16_enable_interrupts(HPM_ADC0, adc16_event_trig_complete);
	intc_m_enable_irq_with_priority(ADC_IRQ, 1);

	HPM_IOC->PAD[IOC_PAD_PF27].FUNC_CTL = IOC_PAD_FUNC_CTL_ANALOG_MASK;
	HPM_IOC->PAD[IOC_PAD_PF26].FUNC_CTL = IOC_PAD_FUNC_CTL_ANALOG_MASK;
	HPM_IOC->PAD[IOC_PAD_PF21].FUNC_CTL = IOC_PAD_FUNC_CTL_ANALOG_MASK;
	HPM_IOC->PAD[IOC_PAD_PF20].FUNC_CTL = IOC_PAD_FUNC_CTL_ANALOG_MASK;

	trgm_cfg.invert = false;		   /* 不反相 */
	trgm_cfg.type = trgm_output_same_as_input; /* 直通 */
	trgm_cfg.input = HPM_TRGM0_INPUT_SRC_PWM0_TRGO_0;
	trgm_output_config(HPM_TRGM0, TRGM_TRGOCFG_ADCX_PTRGI0A, &trgm_cfg);
}

/*======================== 三相电压 / 母线电压 (ADC1) ========================*/
/* VA PF16/IN8, VB PF18/IN11, VC PF19/IN13, Vbus PF22/IN14 */
#define ADC1_CH_VA (8U)
#define ADC1_CH_VB (11U)
#define ADC1_CH_VC (13U)
#define ADC1_CH_VBUS (14U)
#define ADC1_TRIG_CH ADC16_CONFIG_TRG0B
#define ADC1_QUEUE_LEN (4U)
#define ADC1_DMA_OFFSET (ADC1_TRIG_CH * 4U)
#define ADC1_IBUS_TRIG_CH ADC16_CONFIG_TRG0C
#define ADC1_IBUS_DMA_OFFSET (ADC1_IBUS_TRIG_CH * 4U)

ATTR_PLACE_AT_NONCACHEABLE_BSS_WITH_ALIGNMENT(ADC_SOC_DMA_ADDR_ALIGNMENT)
static volatile adc16_pmt_dma_data_t
    adc1_dma_buf[ADC_SOC_PMT_MAX_DMA_BUFF_LEN_IN_4BYTES];
static volatile uint32_t bad_voltage_frames;

static const uint8_t adc1_ch_tab[ADC1_QUEUE_LEN] = {ADC1_CH_VA, ADC1_CH_VB,
						    ADC1_CH_VC, ADC1_CH_VBUS};
static const uint8_t adc1_ibus_ch_tab[1] = {ADC_CH_IBUS};

void adc1_init(void)
{
	adc16_config_t cfg;
	adc16_channel_config_t ch_cfg;
	adc16_pmt_config_t pmt_cfg;

	clock_add_to_group(clock_adc1, 0);
	clock_set_adc_source(clock_adc1, clk_adc_src_ahb0);

	adc16_get_default_config(&cfg);
	cfg.res = adc16_res_16_bits;
	cfg.conv_mode = adc16_conv_mode_preemption;
	cfg.adc_clk_div = adc16_clock_divider_4; /* ADC 时钟 = 源/4 = 50MHz */
	cfg.sel_sync_ahb = true;
	cfg.adc_ahb_en = true; /* 打开 ADC 内置 DMA */
	adc16_init(HPM_ADC1, &cfg);

	adc16_get_channel_default_config(&ch_cfg);
	ch_cfg.sample_cycle = ADC_SAMPLE_CYCLE;
	for (uint8_t i = 0; i < ADC1_QUEUE_LEN; i++) {
		ch_cfg.ch = adc1_ch_tab[i];
		adc16_init_channel(HPM_ADC1, &ch_cfg);
	}
	ch_cfg.ch = ADC_CH_IBUS;
	adc16_init_channel(HPM_ADC1, &ch_cfg);

	pmt_cfg.trig_ch = ADC1_TRIG_CH;
	pmt_cfg.trig_len = ADC1_QUEUE_LEN;
	for (uint8_t i = 0; i < ADC1_QUEUE_LEN; i++) {
		pmt_cfg.adc_ch[i] = adc1_ch_tab[i];
		pmt_cfg.inten[i] = (i == ADC1_QUEUE_LEN - 1U);
	}
	adc16_set_pmt_config(HPM_ADC1, &pmt_cfg);
	pmt_cfg.trig_ch = ADC1_IBUS_TRIG_CH;
	pmt_cfg.trig_len = 1U;
	pmt_cfg.adc_ch[0] = adc1_ibus_ch_tab[0];
	pmt_cfg.inten[0] = false;
	adc16_set_pmt_config(HPM_ADC1, &pmt_cfg);
	adc16_init_pmt_dma(HPM_ADC1, (uint32_t)adc1_dma_buf);
	adc16_enable_pmt_queue(HPM_ADC1, ADC1_TRIG_CH);
	adc16_enable_pmt_queue(HPM_ADC1, ADC1_IBUS_TRIG_CH);

	/* 电压输入设为模拟脚 */
	HPM_IOC->PAD[IOC_PAD_PF16].FUNC_CTL = IOC_PAD_FUNC_CTL_ANALOG_MASK;
	HPM_IOC->PAD[IOC_PAD_PF18].FUNC_CTL = IOC_PAD_FUNC_CTL_ANALOG_MASK;
	HPM_IOC->PAD[IOC_PAD_PF19].FUNC_CTL = IOC_PAD_FUNC_CTL_ANALOG_MASK;
	HPM_IOC->PAD[IOC_PAD_PF22].FUNC_CTL = IOC_PAD_FUNC_CTL_ANALOG_MASK;
}

bool adc1_read_ibus(uint16_t *ibus)
{
	volatile adc16_pmt_dma_data_t *sample =
	    &adc1_dma_buf[ADC1_IBUS_DMA_OFFSET];
	const uint32_t error_mask = ADC16_INT_STS_TRIG_SW_CFLCT_MASK |
				    ADC16_INT_STS_AHB_ERR_MASK |
				    ADC16_INT_STS_DMA_FIFO_FULL_MASK;

	adc16_clear_status_flags(HPM_ADC1, error_mask);
	/* 该队列未开 INTEN, TRG_CMPT_FLAG 不置位: 清通道字段等 DMA 发布 */
	sample->adc_ch = 0x1FU;
	adc16_trigger_pmt_by_sw(HPM_ADC1, ADC1_IBUS_TRIG_CH);
	for (uint32_t i = 0; i < 4000U; i++) {
		uint32_t st = adc16_get_status_flags(HPM_ADC1);
		if (st & error_mask) {
			adc16_clear_status_flags(HPM_ADC1, st & error_mask);
			return false;
		}
		if (sample->adc_ch == ADC_CH_IBUS &&
		    sample->trig_ch == ADC1_IBUS_TRIG_CH &&
		    sample->seq_num == 0U) {
			*ibus = sample->result;
			return true;
		}
	}
	return false;
}

/* 软触发 ADC1 抢占队列, 只接受完整帧 */
bool adc1_read_all(uint16_t *va, uint16_t *vb, uint16_t *vc, uint16_t *vbus)
{
	for (uint8_t i = 0; i < ADC1_QUEUE_LEN; i++)
		adc1_dma_buf[ADC1_DMA_OFFSET + i].adc_ch = 0x1FU;
	HPM_ADC1->TRG_CMPT_FLAG = (1U << ADC1_TRIG_CH);
	adc16_clear_status_flags(HPM_ADC1, adc16_get_status_flags(HPM_ADC1));

	adc16_trigger_pmt_by_sw(HPM_ADC1, ADC1_TRIG_CH);

	for (uint32_t i = 0; i < 4000U; i++) {
		uint32_t st = adc16_get_status_flags(HPM_ADC1);
		if (st & ADC_PMT_ERROR_MASK) {
			adc16_clear_status_flags(HPM_ADC1, st);
			break;
		}
		if (HPM_ADC1->TRG_CMPT_FLAG & (1U << ADC1_TRIG_CH)) {
			bool valid = true;
			for (uint8_t ch = 0; ch < ADC1_QUEUE_LEN; ch++) {
				volatile adc16_pmt_dma_data_t *sample =
				    &adc1_dma_buf[ADC1_DMA_OFFSET + ch];
				valid = valid &&
					sample->trig_ch == ADC1_TRIG_CH &&
					sample->seq_num == ch &&
					sample->adc_ch == adc1_ch_tab[ch];
			}
			if (valid) {
				*va = adc1_dma_buf[ADC1_DMA_OFFSET].result;
				*vb = adc1_dma_buf[ADC1_DMA_OFFSET + 1U].result;
				*vc = adc1_dma_buf[ADC1_DMA_OFFSET + 2U].result;
				*vbus =
				    adc1_dma_buf[ADC1_DMA_OFFSET + 3U].result;
				HPM_ADC1->TRG_CMPT_FLAG = (1U << ADC1_TRIG_CH);
				adc16_clear_status_flags(
				    HPM_ADC1, adc16_get_status_flags(HPM_ADC1));
				return true;
			}
			break;
		}
	}

	bad_voltage_frames++;
	return false;
}

void adc_callback_set(adc_callback_t sink)
{
	adc_callback = sink;
}

SDK_DECLARE_EXT_ISR_M(ADC_IRQ, isr_adc)
void isr_adc(void)
{
	uint32_t st = adc16_get_status_flags(HPM_ADC0);
	uint32_t complete = HPM_ADC0->TRG_CMPT_FLAG;
	adc16_clear_status_flags(HPM_ADC0, st);
	if (complete & (1U << ADC_TRIG_CH))
		HPM_ADC0->TRG_CMPT_FLAG = (1U << ADC_TRIG_CH);

	if (ADC16_INT_STS_TRIG_CMPT_GET(st)) {
		bool valid = !(st & ADC_PMT_ERROR_MASK) &&
			     (complete & (1U << ADC_TRIG_CH));
		for (uint8_t i = 0; i < ADC_TRIG_LEN; i++) {
			valid = valid &&
				adc_dma_buf[i].trig_ch == ADC_TRIG_CH &&
				adc_dma_buf[i].seq_num == i &&
				adc_dma_buf[i].adc_ch == adc_ch_tab[i];
		}
		if (!valid) {
			bad_current_frames++;
			FOC.adc_current_missed++;
			return;
		}
		FOC.Ia = adc_dma_buf[0].result;
		FOC.Ib = adc_dma_buf[1].result;
		FOC.Ic = adc_dma_buf[2].result;

		/* Ic 最易采到开关沿: 坏样本用 -(Ia+Ib) 重建, isum 留作诊断 */
		FOC.isum =
		    ((FOC.Ia - FOC.Ia_midpoint) + (FOC.Ib - FOC.Ib_midpoint) +
		     (FOC.Ic - FOC.Ic_midpoint)) *
		    FOC_CURRENT_SCALE;
		if (FOC.adc_ready &&
		    ((FOC.isum > 0.1f) || (FOC.isum < -0.1f))) {
			float ic_fix = FOC.Ic_midpoint -
				       (FOC.Ia - FOC.Ia_midpoint) -
				       (FOC.Ib - FOC.Ib_midpoint);
			if (ic_fix < 0.0f)
				ic_fix = 0.0f;
			else if (ic_fix > 65535.0f)
				ic_fix = 65535.0f;
			FOC.Ic = ic_fix;
		}

		current_snapshot_seq++;
		__asm volatile("fence rw, w" ::: "memory");
		current_snapshot[0] = (uint16_t)FOC.Ia;
		current_snapshot[1] = (uint16_t)FOC.Ib;
		current_snapshot[2] = (uint16_t)FOC.Ic;
		__asm volatile("fence rw, w" ::: "memory");
		current_snapshot_seq++;
		FOC.adc_current_frames++;
		FOC.adc_current_missed = 0U;

		if (adc_callback)
			adc_callback();
	}
}
