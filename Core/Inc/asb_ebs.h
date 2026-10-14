#ifndef ASB_EBS_H
#define ASB_EBS_H

#include <stdbool.h>

/* =========================================================
 * ASB EBS Control - asb_ebs.h
 * Aristurtle Formula Student | 2026
 * ========================================================= */

 /* EBS Initialization */
void EBS_Init(void);

/* EBS States */
bool EBS_IsActivated(void);
bool EBS_System1_State(void);
bool EBS_System2_State(void);

typedef enum {
    EBS_UNAVAILABLE  = 1,
    EBS_ARMED        = 2,
    EBS_TRIGGERED    = 3
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

/* EBS Control */
void EBS_Activate(void);
void EBS_Activate_System1(void);
void EBS_Activate_System2(void);
void EBS_Release_All(void);
void EBS_Release_System1(void);
void EBS_Release_System2(void);

#endif /* ASB_EBS_H */