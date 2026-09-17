#ifndef __SYS_H__
#define __SYS_H__

#include "hpm_common.h"
#include "hpm_pmp_drv.h"
#include "hpm_clock_drv.h"
#include "hpm_sysctl_drv.h"
#include "hpm_pllctlv2_drv.h"
#include "hpm_pcfg_drv.h"
#include "hpm_soc_feature.h"
#include "hpm_debug_console.h"
#include "hpm_uart_drv.h"
#include "hpm_iomux.h"
#include <stdio.h>

#define SYS_CPU_FREQ            (480000000UL)
#define SYS_CONSOLE_UART        HPM_UART0
#define SYS_CONSOLE_UART_CLK    clock_uart0
#define SYS_CONSOLE_BAUDRATE    (115200UL)
#define SYS_CONSOLE_TX_PAD      IOC_PAD_PA00
#define SYS_CONSOLE_TX_FUNC     IOC_PA00_FUNC_CTL_UART0_TXD
#define SYS_CONSOLE_RX_PAD      IOC_PAD_PA01
#define SYS_CONSOLE_RX_FUNC     IOC_PA01_FUNC_CTL_UART0_RXD

void sys_board_init(void);
void sys_delay_us(uint32_t us);
void sys_delay_ms(uint32_t ms);

#endif /* __SYS_H__ */
