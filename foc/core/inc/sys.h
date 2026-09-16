#ifndef __SYS_H__
#define __SYS_H__

#include "hpm_common.h"
#include "hpm_pmp_drv.h"
#include "hpm_clock_drv.h"
#include "hpm_sysctl_drv.h"
#include "hpm_pllctlv2_drv.h"
#include "hpm_pcfg_drv.h"
#include "hpm_soc_feature.h"
#include "SEGGER_RTT.h"
#include <stdio.h>

#define SYS_CPU_FREQ        (480000000UL)

void sys_board_init(void);
void sys_delay_us(uint32_t us);
void sys_delay_ms(uint32_t ms);

#endif /* __SYS_H__ */
