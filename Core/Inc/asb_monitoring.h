#ifndef ASB_MONITORING_H
#define ASB_MONITORING_H

#include <stdbool.h>
#include <stdint.h>

/* =========================================================
 * ASB Monitor - asb_monitoring.h
 * Aristurtle Formula Student | 2026
 *
 * Continuous monitoring of the ASB (interlocks, tank and brake
 * pressures, APU / RES alive, watchdog, service brake).
 * main.c calls Monitor_Run() every 10ms and if it returns false
 * it engages the EBS and opens the SDC.
 * ========================================================= */

/* One debounced check. A single bad sample is not enough to stop the
   car, the check only fails after 'limit' bad samples in a row. */
typedef struct {
    uint16_t errors;   /* consecutive bad samples */
    uint16_t limit;    /* how many in a row before we call it a fail */
    bool getter;       /* last result of Check(), read by the Monitor_*Ok() getters */
} Check_t;

/* true = OK, false = fail. a good sample resets the counter */
static inline bool Check(Check_t *c, bool ok)
{
    if (ok)                   { c->errors = 0; return true; }
    if (c->errors < c->limit) { c->errors++; }
    return (c->errors < c->limit);
}

void Monitor_Init(void);        /* clears all the counters */
bool Monitor_Run(void);         /* call every 10ms, true = all ok */

/* results for CAN status / datalogger (not used anywhere yet) */
bool Monitor_InterlocksOk(void);
bool Monitor_TankOk(void);
bool Monitor_BrakeOk(void);
bool Monitor_ApuOk(void);
bool Monitor_ServiceBrakeOk(void);
bool Monitor_WatchdogOk(void);
#endif
