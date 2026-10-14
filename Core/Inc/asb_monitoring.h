#ifndef ASB_MONITORING_H
#define ASB_MONITORING_H

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    uint16_t errors;   /* consecutive bad samples */
    uint16_t limit;    /* how many in a row before we call it a fail */
    bool getter;
} Check_t;

/* true = OK, false = fail. a good sample resets the counter */
static inline bool Check(Check_t *c, bool ok)
{
    if (ok)                   { c->errors = 0; return true; }
    if (c->errors < c->limit) { c->errors++; }
    return (c->errors < c->limit);
}

void Monitor_Init(void);        /* clears all the counters */
bool Monitor_Run(void);         /* call every 10ms while armed, true = all ok */

/* results for CAN status / datalogger */
bool Monitor_InterlocksOk(void);
bool Monitor_TankOk(void);
bool Monitor_BrakeOk(void);
bool Monitor_ApuOk(void);
bool Monitor_ServiceBrakeOk(void);
bool Monitor_WatchdogOk(void);
#endif
