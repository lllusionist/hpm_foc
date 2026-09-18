#ifndef __FOC_TASK_H__
#define __FOC_TASK_H__

#include "foc.h"

typedef enum led_mode {
	ST_BOOT,
	ST_IDLE,  // 空闲
	ST_RUN,	  // 运行
	ST_CAL,	  // 校准
	ST_FAULT, // 故障
} foc_stage_t;

#endif /* __FOC_TASK_H__ */
