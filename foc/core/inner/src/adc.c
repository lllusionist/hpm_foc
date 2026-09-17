#include "adc.h"

#define ADC_IRQ             IRQn_ADC0
#define ADC_TRIG_CH         ADC16_CONFIG_TRG0A
#define ADC_TRIG_LEN        (3U)

#define ADC_CH_IA           (0U)
#define ADC_CH_IB           (1U)
#define ADC_CH_IC           (2U)

#define ADC_SAMPLE_CYCLE    (20U)

static const uint8_t adc_ch_tab[ADC_TRIG_LEN] = {ADC_CH_IA, ADC_CH_IB, ADC_CH_IC};

ATTR_PLACE_AT_FAST_RAM_WITH_ALIGNMENT(ADC_SOC_DMA_ADDR_ALIGNMENT)
volatile adc16_pmt_dma_data_t adc_dma_buf[ADC_TRIG_LEN];

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
    cfg.adc_ahb_en   = true;                        // 打开 ADC 内置 DMA

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
    adc16_init_pmt_dma(HPM_ADC0, (uint32_t)adc_dma_buf);
    adc16_enable_pmt_queue(HPM_ADC0, ADC_TRIG_CH);  // 使能 TRG0A 这个队列

    // 开中断
    adc16_enable_interrupts(HPM_ADC0, adc16_event_trig_complete);
    intc_m_enable_irq_with_priority(ADC_IRQ, 1);

    trgm_cfg.invert = false;                        // 不反相
    trgm_cfg.type   = trgm_output_same_as_input;    // 直通
    trgm_cfg.input  = HPM_TRGM0_INPUT_SRC_PWM0_TRGO_0;
    trgm_output_config(HPM_TRGM0, TRGM_TRGOCFG_ADCX_PTRGI0A, &trgm_cfg);
}

SDK_DECLARE_EXT_ISR_M(ADC_IRQ, isr_adc)
void isr_adc(void)
{
    uint32_t st = adc16_get_status_flags(HPM_ADC0);
    adc16_clear_status_flags(HPM_ADC0, st);     // 清标志

    if (ADC16_INT_STS_TRIG_CMPT_GET(st)) {
        FOC.Ia = adc_dma_buf[0].result;
        FOC.Ib = adc_dma_buf[1].result;
        FOC.Ic = adc_dma_buf[2].result;
    }
}
