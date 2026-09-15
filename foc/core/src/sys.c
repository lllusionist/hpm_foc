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

static void print_banner(void)
{
    const uint8_t banner[] = { "\n\
    ----------------------------------------------------------------------\n\
    $$\\   $$\\ $$$$$$$\\  $$\\      $$\\ $$\\\n\
    $$ |  $$ |$$  __$$\\ $$$\\    $$$ |\\__|\n\
    $$ |  $$ |$$ |  $$ |$$$$\\  $$$$ |$$\\  $$$$$$$\\  $$$$$$\\   $$$$$$\\\n\
    $$$$$$$$ |$$$$$$$  |$$\\$$\\$$ $$ |$$ |$$  _____|$$  __$$\\ $$  __$$\\\n\
    $$  __$$ |$$  ____/ $$ \\$$$  $$ |$$ |$$ /      $$ |  \\__|$$ /  $$ |\n\
    $$ |  $$ |$$ |      $$ |\\$  /$$ |$$ |$$ |      $$ |      $$ |  $$ |\n\
    $$ |  $$ |$$ |      $$ | \\_/ $$ |$$ |\\$$$$$$$\\ $$ |      \\$$$$$$  |\n\
    \\__|  \\__|\\__|      \\__|     \\__|\\__| \\_______|\\__|       \\______/\n\
    ----------------------------------------------------------------------\n" };
    printf("%s", banner);
}

static void sys_print_clock_freq(void)
{
    printf("==============================\n");
    printf(" %s clock summary\n", BOARD_NAME);
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

    /* uart needs to configure pin function before enabling clock, otherwise the level change of
     * uart rx pin when configuring pin function will cause a wrong data to be received.
     * And a uart rx dma request will be generated by default uart fifo dma trigger level.
     */
    init_uart_pins((UART_Type *) BOARD_CONSOLE_UART_BASE);

    clock_add_to_group(BOARD_CONSOLE_UART_CLK_NAME, 0);

    cfg.type = BOARD_CONSOLE_TYPE;
    cfg.base = (uint32_t) BOARD_CONSOLE_UART_BASE;
    cfg.src_freq_in_hz = clock_get_frequency(BOARD_CONSOLE_UART_CLK_NAME);
    cfg.baudrate = BOARD_CONSOLE_UART_BAUDRATE;

    if (status_success != console_init(&cfg)) {
        /* failed to initialize debug console */
        while (1) {
        }
    }
    print_banner();
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
