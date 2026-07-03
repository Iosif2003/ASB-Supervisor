#ifndef ASB_MONITORING_H
#define ASB_MONITORING_H

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    uint16_t errors;   /* συνεχόμενα κακά δείγματα */
    uint16_t limit;    /* πόσα στη σειρά → fail */
    bool getter;
} Check_t;

/* true = OK, false = fail. Καλό δείγμα → reset. */
static inline bool Check(Check_t *c, bool ok)
{
    if (ok)                   { c->errors = 0; return true; }
    if (c->errors < c->limit) { c->errors++; }
    return (c->errors < c->limit);
}

void Monitor_Init(void);        /* μηδενίζει όλους τους counters */
bool Monitor_Run(void);         /* κάθε 10ms όταν armed· true = όλα ok */

/* Αποτελέσματα για CAN status / datalogger */
bool Monitor_InterlocksOk(void);
bool Monitor_TankOk(void);
bool Monitor_BrakeOk(void);
bool Monitor_ApuOk(void);
bool Monitor_ServiceBrakeOk(void);
bool Monitor_WatchdogOk(void);
#endif
