#ifndef __FOC_TASK_H__
#define __FOC_TASK_H__

#include "foc.h"
#include "foc_cal.h"

typedef enum eFSMState {
    BOOT_UP     = 0,
    IDLE        = 1,
    RUN         = 2,
    CALIBRATION = 3,
    ANTICOGGING = 4,
} tFSMState;

typedef struct sFSM
{
    tFSMState state;
    tFSMState state_next;
    uint8_t   state_next_ready;
} tFSM;

void foc_request_stage(tFSMState state);
tFSMState foc_get_stage(void);
void foc_smf_init(void);
void foc_app_start(void);

#endif /* __FOC_TASK_H__ */
