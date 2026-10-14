#ifndef ASB_SERVICE_BRAKE_H
#define ASB_SERVICE_BRAKE_H

#include <stdbool.h>

/* =========================================================
 * ASB Service Brake - asb_service_brake.h
 * Aristurtle Formula Student | 2026
 *
 * Service brake = the proportional valve on EBS circuit 1, driven by
 * the DAC (PA4). It sits in parallel with the solenoid valve of
 * circuit 1 and a shuttle valve passes whichever of the two has the
 * higher pressure to the cylinder. So it can brake the car even with
 * the EBS released, and the amount depends on the DAC value.
 * DAC values are in asb_config.h (ASB_SB_DAC_*).
 * ========================================================= */

/* Service Brake States */
typedef enum {
    SERVICE_BRAKE_DISENGAGED = 1,  // DAC = 0    - no pressure
    SERVICE_BRAKE_ENGAGED    = 2,  // DAC = 4095 - full pressure
    SERVICE_BRAKE_PARK       = 3,  // DAC = ASB_SB_DAC_PARK - hold, vehicle stationary
    SERVICE_BRAKE_AVAILABLE  = 4   // ready to brake while driving, DAC not touched
} ServiceBrakeState_t;

/* Initialization - disengaged */
void ServiceBrake_Init(void);

/* Service Brake Control */
void ServiceBrake_Engage(void);
void ServiceBrake_Disengage(void);
void ServiceBrake_Park(void);
void ServiceBrake_Available(void);

/* Service Brake State */
ServiceBrakeState_t ServiceBrake_State(void);

#endif /* ASB_SERVICE_BRAKE_H */
