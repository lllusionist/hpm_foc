#include "sys.h"

static void sys_init(void)
{
    uint32_t cpu0_freq = clock_get_frequency(clock_cpu0);
    if (cpu0_freq == PLLCTL_SOC_PLL_REFCLK_FREQ) {
        /* Configure the External OSC ramp-up time: ~9ms */
        pllctlv2_xtal_set_rampup_time(HPM_PLLCTLV2, 32ul * 1000ul * 9u);

        /* select clock setting preset1 */
        sysctl_clock_set_preset(HPM_SYSCTL, 2);
    }
    /* Add Clocks to group 0 */
    clock_add_to_group(clock_cpu0, 0);
    clock_add_to_group(clock_mchtmr0, 0);
    clock_add_to_group(clock_ahb0, 0);
    clock_add_to_group(clock_axif, 0);
    clock_add_to_group(clock_axis, 0);
    clock_add_to_group(clock_axic, 0);
    clock_add_to_group(clock_rom0, 0);
    clock_add_to_group(clock_xpi0, 0);
    clock_add_to_group(clock_lmm0, 0);
    clock_add_to_group(clock_ram0, 0);
    clock_add_to_group(clock_hdma, 0);
    clock_add_to_group(clock_xdma, 0);
    clock_add_to_group(clock_gpio, 0);
    clock_add_to_group(clock_ptpc, 0);
    /* Motor Related */
    clock_add_to_group(clock_qei0, 0);
    clock_add_to_group(clock_plb0, 0);
    clock_add_to_group(clock_qei1, 0);
    clock_add_to_group(clock_qeo0, 0);
    clock_add_to_group(clock_qeo1, 0);
    clock_add_to_group(clock_pwm0, 0);
    clock_add_to_group(clock_pwm1, 0);
    clock_add_to_group(clock_emds, 0);
    /* Connect Group0 to CPU0 */
    clock_connect_group_to_cpu(0, 0);

    /* Bump up DCDC voltage to 1275mv */
    pcfg_dcdc_set_voltage(HPM_PCFG, 1275);
    pcfg_dcdc_switch_to_dcm_mode(HPM_PCFG);

    /* Configure PLL0 Post Divider */
    pllctlv2_set_postdiv(HPM_PLLCTLV2, pllctlv2_pll0, pllctlv2_clk0, pllctlv2_div_1p0);    /* PLL0CLK0: 480MHz */
    pllctlv2_set_postdiv(HPM_PLLCTLV2, pllctlv2_pll0, pllctlv2_clk1, pllctlv2_div_1p2);    /* PLL0CLK1: 400MHz */
    /* Configure PLL0 Frequency to 480MHz */
    pllctlv2_init_pll_with_freq(HPM_PLLCTLV2, pllctlv2_pll0, SYS_CPU_FREQ);
    /* CPU clock use clk_src_pll0_clk0 */
    clock_set_source_divider(clock_cpu0, clk_src_pll0_clk0, 1);
    clock_update_core_clock();

    /* Configure mchtmr to 24MHz */
    clock_set_source_divider(clock_mchtmr0, clk_src_osc24m, 1);
}

static void sys_pmp_init(void)
{
    uint32_t start_addr;
    uint32_t end_addr;
    uint32_t length;
    pmp_entry_t pmp_entry[16] = {0};
    uint8_t index = 0;

    /* 全范围 RWX */
    pmp_entry[index].pmp_addr = 0xFFFFFFFF;
    pmp_entry[index].pmp_cfg.val = PMP_CFG(READ_EN, WRITE_EN, EXECUTE_EN, ADDR_MATCH_NAPOT, REG_UNLOCK);
    index++;

    /* Init noncachable memory */
    extern uint32_t __noncacheable_start__[];
    extern uint32_t __noncacheable_end__[];
    const uint32_t axi_sram_start = 0x01200000;  /* AXI SRAM start */
    const uint32_t axi_sram_end = 0x01240000;    /* AXI SRAM end */
    start_addr = (uint32_t) __noncacheable_start__;
    end_addr = (uint32_t) __noncacheable_end__;
    if ((start_addr >= axi_sram_start) && (end_addr <= axi_sram_end)) {
        length = end_addr - start_addr;
        if (length > 0) {
            /* Ensure the address and the length are power of 2 aligned */
            assert((length & (length - 1U)) == 0U);
            assert((start_addr & (length - 1U)) == 0U);
            pmp_entry[index].pmp_addr = PMP_NAPOT_ADDR(start_addr, length);
            pmp_entry[index].pmp_cfg.val = PMP_CFG(READ_EN, WRITE_EN, EXECUTE_EN, ADDR_MATCH_NAPOT, REG_UNLOCK);
            pmp_entry[index].pma_addr = PMA_NAPOT_ADDR(start_addr, length);
            pmp_entry[index].pma_cfg.val = PMA_CFG(ADDR_MATCH_NAPOT, MEM_TYPE_MEM_NON_CACHE_BUF, AMO_EN);
            index++;
        }
    }

    pmp_config(&pmp_entry[0], index);
}

static void sys_print_clock_freq(void)
{
    printf("==============================\n");
    printf("cpu0:\t\t %dHz\n", clock_get_frequency(clock_cpu0));
    printf("ahb:\t\t %luHz\n", clock_get_frequency(clock_ahb0));
    printf("axif:\t\t %dHz\n", clock_get_frequency(clock_axif));
    printf("axis:\t\t %dHz\n", clock_get_frequency(clock_axis));
    printf("axic:\t\t %dHz\n", clock_get_frequency(clock_axic));
    printf("xpi0:\t\t %dHz\n", clock_get_frequency(clock_xpi0));
    printf("mchtmr0:\t %dHz\n", clock_get_frequency(clock_mchtmr0));
    printf("==============================\n");
}



static void sys_console_init(void)
{
    console_config_t cfg;

    /* 先配引脚再开时钟：反过来的话，配引脚那一刻 RX 上的电平跳变会被当成起始位收进来 */
    HPM_IOC->PAD[SYS_CONSOLE_TX_PAD].FUNC_CTL = SYS_CONSOLE_TX_FUNC;
    HPM_IOC->PAD[SYS_CONSOLE_RX_PAD].FUNC_CTL = SYS_CONSOLE_RX_FUNC;

    clock_add_to_group(SYS_CONSOLE_UART_CLK, 0);

    cfg.type           = CONSOLE_TYPE_UART;
    cfg.base           = (uint32_t)SYS_CONSOLE_UART;
    cfg.src_freq_in_hz = clock_get_frequency(SYS_CONSOLE_UART_CLK);
    cfg.baudrate       = SYS_CONSOLE_BAUDRATE;

    if (console_init(&cfg) != status_success) {
        while (1) {
        }
    }

    /* printf 由 SDK 的 console 组件接管：
     *   SES 的 RTL 会调 __SEGGER_RTL_X_file_write()，那个函数在 hpm_debug_console.c 里
     *   被重定向到串口，而且会自动把 \n 补成 \r\n
     * 两个前提：
     *   1. 编译宏里不能有 CONFIG_NDEBUG_CONSOLE=1（否则组件编译成空壳）
     *   2. SEGGER_RTT_Syscalls_SES.c 必须排除编译（它会把 printf 抢到 RTT 上）
     */

    sys_print_clock_freq();
}

void sys_board_init(void)
{
    sys_pmp_init();
    sys_init();
    sys_console_init();
}

void sys_delay_us(uint32_t us)
{
    clock_cpu_delay_us(us);
}

void sys_delay_ms(uint32_t ms)
{
    clock_cpu_delay_ms(ms);
}
