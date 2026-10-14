#ifndef ASB_CAN_H
#define ASB_CAN_H
#include "main.h"
#include <stdbool.h>

/* =========================================================
 * ASB CAN - asb_can.h
 * Aristurtle Formula Student | 2026
 *
 * Everything the ASB sends or receives on CAN1 (1 Mbit/s) goes
 * through this module. The other modules never touch the CAN
 * peripheral, they only call the getters / senders below.
 * Frame layouts come from the DBC through the cantools generated
 * can_mcu.c / can_mcu.h. Don't edit those two by hand, regenerate
 * them from the DBC.
 * ========================================================= */

/* Initialization - RX filters, start CAN1, enable the RX interrupt */
void CAN_App_Init(void);

/* TX Tasks - called from the main loop on the timer ticks */
void CAN_SendAsbStatus(void);           /* ASB (0x023) every 20ms - our state for the APU */
void CAN_SendDatalogger(void);          /* ASB_DATALOGGER (0x511) every 100ms */
void CAN_SendHydraulicPressures(void);  /* ASB_HYDRAULIC_PRESSURES (0x512) every 50ms */
void CAN_SendTankPressures(void);       /* ASB_TANK_PRESSURES (0x513) every 50ms */

/* RX Processing (called from HAL_CAN_RxFifo0MsgPendingCallback) */
void CAN_ProcessRxMessage(void);

/* RX Getters - APU (DV_SYSTEM_STATUS 0x502, APU_SET_FINISHED 0x02D) */
uint8_t CAN_GetASState(void);        /* 1 off, 2 ready, 3 driving, 4 emergency, 5 finish */
uint8_t CAN_GetASMission(void);      /* AMI: 1-6 driverless missions, 7 = manual */
bool    CAN_GetASSetFinished(void);  /* APU says the mission is done */

/* RX Getters - VCU (VCU_BOOLS 0x320) */
uint8_t CAN_GetVCUMode(void);        /* 0 = driverless, 1 = manual (see SYS_GetSelectedMission) */
int     CAN_GetServoCommand(void);   /* placeholder, always 0 - no such signal in the DBC */

/* RX Getters - brake pressures (front: SMU_BRAKE 0x065, rear: VCU_BRAKE 0x078), in bar */
float CAN_GetBrakePressureFront(void);
float CAN_GetBrakePressureRear(void);

/* RX Getters - RES (RES_STATUS 0x191) */
uint8_t CAN_GetRESStop(void);            /* raw bit: 0 = emergency active, 1 = OK */
uint8_t CAN_GetRESToggle(void);          /* K2 latching switch (Go-signal) */
uint8_t CAN_GetRESButton(void);          /* K3 button (Go-signal) */
uint8_t CAN_GetRESSignalStrength(void);  /* radio quality 0-100 % */
bool    CAN_GetRESEmergency(void);       /* true = RES E-Stop asserted */
uint8_t CAN_GetRESErrorRegister(void);   /* CANopen EMCY 0x091 byte 2, 0 = healthy, latched */

/* RX Getters - RES Init (NMT sent by APU, monitored by ASB) */
uint8_t CAN_GetRESInitRequestedState(void);
uint8_t CAN_GetRESInitAddressedNode(void);

/* Communication Alive Checks - true = the frame came in recently enough */
bool CAN_IsAPUAlive(void);            /* DV_SYSTEM_STATUS within 300ms */
bool CAN_IsBrakePressureAlive(void);  /* front AND rear pressure within 100ms */
bool CAN_IsBrakeOutOfRange(void);     /* out of range flag sent by the SMU */
bool CAN_IsRESAlive(void);            /* RES_STATUS with good signal within 90ms */

#endif /* ASB_CAN_H */
