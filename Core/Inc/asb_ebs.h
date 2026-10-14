#ifndef ASB_EBS_H
#define ASB_EBS_H

#include <stdbool.h>

/* =========================================================
 * ASB EBS Control - asb_ebs.h
 * Aristurtle Formula Student | 2026
 *
 * EBS = 2 independent pneumatic circuits (system 1 and system 2),
 * each with its own tank and solenoid valve feeding one brake cylinder.
 * Two circuits so that one leak or one dead valve still leaves a
 * working brake (redundancy).
 *
 * The valves are fail-safe:
 *   pin LOW  -> valve not powered -> air goes to the cylinder -> BRAKING
 *   pin HIGH -> valve powered     -> cylinder vented          -> released
 * So the MCU has to actively hold the pins high to drive. A reset, a
 * crash or a lost supply brakes the car.
 * ========================================================= */

 /* EBS Initialization - both circuits braking */
void EBS_Init(void);

/* EBS States */
bool EBS_IsActivated(void);     /* true = at least one circuit braking */
bool EBS_System1_State(void);   /* true = circuit 1 braking */
bool EBS_System2_State(void);   /* true = circuit 2 braking */

/* our own EBS state, sent on the datalogger frame */
typedef enum {
    EBS_UNAVAILABLE  = 1,   /* tank pressure out of range */
    EBS_ARMED        = 2,   /* tanks ok, valves released, SDC closed - ready to brake */
    EBS_TRIGGERED    = 3    /* braking, or the SDC (TSMS) is open */
} EBS_State_t;

EBS_State_t EBS_State(void);

/* DV System Status states - FS Driverless Specification 2026 v1.1, Table 2 (0x502).
 * Sent to the APU as EBS_Status / R_EBS_Status so it can forward them into
 * DV_System_Status. The two enums are numbered differently on purpose:
 * the spec puts engaged=2 / initial_checkup_passed=3 for the redundancy. */
typedef enum {
    DV_EBS_DEACTIVATED            = 1,
    DV_EBS_INITIAL_CHECKUP_PASSED = 2,
    DV_EBS_ACTIVATED              = 3
} DV_EbsStatus_t;

typedef enum {
    DV_R_EBS_DEACTIVATED            = 1,
    DV_R_EBS_ENGAGED                = 2,
    DV_R_EBS_INITIAL_CHECKUP_PASSED = 3
} DV_REbsStatus_t;

DV_EbsStatus_t  EBS_DvStatus(void);
DV_REbsStatus_t EBS_DvRedundancyStatus(void);

/* EBS Control - Activate = brake, Release = let go */
void EBS_Activate(void);           /* both circuits brake */
void EBS_Activate_System1(void);
void EBS_Activate_System2(void);
void EBS_Release_All(void);        /* both circuits released */
void EBS_Release_System1(void);
void EBS_Release_System2(void);

#endif /* ASB_EBS_H */
