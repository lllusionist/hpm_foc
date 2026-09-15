#ifndef __SYS_H__
#define __SYS_H__

#include "board.h"                  /* BOARD_CONSOLE_* 等板级宏 */
#include "hpm_common.h"
#include "hpm_debug_console.h"      /* console_config_t / console_init */
#include "hpm_pmp_drv.h"            /* pmp_entry_t / pmp_config */
#include "hpm_clock_drv.h"
#include "hpm_sysctl_drv.h"         /* sysctl_clock_set_preset */
#include "hpm_pllctlv2_drv.h"       /* pllctlv2_* */
#include "hpm_pcfg_drv.h"           /* pcfg_dcdc_* */
#include "hpm_soc_feature.h"        /* PLLCTL_SOC_PLL_REFCLK_FREQ */

#define SYS_CPU_FREQ        (480000000UL)

void sys_pmp_init(void);
void sys_console_init(void);
void sys_init(void);

#endif /* __SYS_H__ */
