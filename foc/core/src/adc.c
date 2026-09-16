#include "adc.h"

#define ADC_IRQ             IRQn_ADC0
#define ADC_TRIG_CH         ADC16_CONFIG_TRG0A
#define ADC_TRIG_LEN        (3U)

#define ADC_CH_IA           (0U)
#define ADC_CH_IB           (1U)
#define ADC_CH_IC           (2U)

#define ADC_SAMPLE_CYCLE    (20U)

static const uint8_t adc_ch_tab[ADC_TRIG_LEN] = {ADC_CH_IA, ADC_CH_IB, ADC_CH_IC};
volatile foc_data_t FOC;

volatile uint32_t g_adc_cnt;            // 进中断的次数, 只用来验证, 验证完可删

void adc_init(void)
{
    adc16_config_t cfg;
    adc16_channel_config_t ch_cfg;
    adc16_pmt_config_t pmt_cfg;
    trgm_output_t trgm_cfg;

    clock_add_to_group(clock_adc0, 0);

    // ADC 时钟源选 ahb0(200MHz)
    clock_set_adc_source(clock_adc0, clk_adc_src_ahb0);

    adc16_get_default_config(&cfg);

    cfg.res          = adc16_res_16_bits;           // 分辨率 16 位
    cfg.conv_mode    = adc16_conv_mode_preemption;  // 工作模式: 抢占(PMT)
    cfg.adc_clk_div  = adc16_clock_divider_4;       // ADC 时钟 = 源/4 = 50MHz
    cfg.sel_sync_ahb = true;                        // ADC 时钟来自 AHB(与上面 ahb0 对应)
    cfg.adc_ahb_en   = false;                       // DMA使能

    adc16_init(HPM_ADC0, &cfg);

    adc16_get_channel_default_config(&ch_cfg);

    ch_cfg.sample_cycle = ADC_SAMPLE_CYCLE;
    for (uint8_t i = 0; i < ADC_TRIG_LEN; i++) {
        ch_cfg.ch = adc_ch_tab[i];
        adc16_init_channel(HPM_ADC0, &ch_cfg);
    }

    pmt_cfg.trig_ch  = ADC_TRIG_CH;     // 用哪个触发源(TRG0A)
    pmt_cfg.trig_len = ADC_TRIG_LEN;    // 队列长度
    for (uint8_t i = 0; i < ADC_TRIG_LEN; i++) {
        pmt_cfg.adc_ch[i] = adc_ch_tab[i];  // 第 i 个位置转哪个通道
        pmt_cfg.inten[i]  = false;          // 该通道转完是否产生中断
    }

    // 转换完产生中断
    pmt_cfg.inten[ADC_TRIG_LEN - 1U] = true;

    adc16_set_pmt_config(HPM_ADC0, &pmt_cfg);
    adc16_enable_pmt_queue(HPM_ADC0, ADC_TRIG_CH);  // 使能 TRG0A 这个队列

    // 开中断
    adc16_enable_interrupts(HPM_ADC0, adc16_event_trig_complete);
    intc_m_enable_irq_with_priority(ADC_IRQ, 1);

    /* ---- ⑥ TRGM 连线: PWM → ADC ----
       PWM 和 ADC 之间不是硬连线, 要经 TRGM(互联管理器)这个可编程开关矩阵:
         input  = 从哪来(PWM0 的 TRGO_0)
         output = 到哪去(ADC 的 PTRGI0A, 即 TRG0A)
       配好后: PWM 比较点一到 → TRGO_0 变高 → TRG0A 收到脉冲 → ADC 开始转换。*/
    trgm_cfg.invert = false;                        // 不反相
    trgm_cfg.type   = trgm_output_same_as_input;    // 直通(不整形)
    trgm_cfg.input  = HPM_TRGM0_INPUT_SRC_PWM0_TRGO_0;
    trgm_output_config(HPM_TRGM0, TRGM_TRGOCFG_ADCX_PTRGI0A, &trgm_cfg);
}

SDK_DECLARE_EXT_ISR_M(ADC_IRQ, isr_adc)
void isr_adc(void)
{
    uint32_t st = adc16_get_status_flags(HPM_ADC0);
    adc16_clear_status_flags(HPM_ADC0, st);     // 清标志

    if (ADC16_INT_STS_TRIG_CMPT_GET(st)) {
        uint16_t ia, ib, ic;

        adc16_get_prd_result(HPM_ADC0, ADC_CH_IA, &ia);
        adc16_get_prd_result(HPM_ADC0, ADC_CH_IB, &ib);
        adc16_get_prd_result(HPM_ADC0, ADC_CH_IC, &ic);

        FOC.Ia = ia;
        FOC.Ib = ib;
        FOC.Ic = ic;
        g_adc_cnt++;
    }
}
