#ifndef ASB_SERVICE_BRAKE_H
#define ASB_SERVICE_BRAKE_H

#include <stdbool.h>

/* =========================================================
 * ASB Service Brake — asb_service_brake.h
 * Aristurtle Formula Student | 2026
 * ========================================================= */

/* Service Brake States */
typedef enum {
    SERVICE_BRAKE_DISENGAGED = 1,  // DAC = 0    — no pressure
    SERVICE_BRAKE_ENGAGED    = 2,  // DAC = 4095 — full pressure
    SERVICE_BRAKE_PARK       = 3   // DAC = 2000 — hold, vehicle stationary
} ServiceBrakeState_t;

/* Initialization */
void ServiceBrake_Init(void);

/* Service Brake Control */
void ServiceBrake_Engage(void);
void ServiceBrake_Disengage(void);
void ServiceBrake_Park(void);

/* Service Brake State */
ServiceBrakeState_t ServiceBrake_State(void);

#endif /* ASB_SERVICE_BRAKE_H */