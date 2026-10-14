#ifndef ASB_SENSORS_H
#define ASB_SENSORS_H

#include <stdbool.h>
#include <stdint.h>

/* =========================================================
 * ASB Sensors - asb_sensors.h
 * Aristurtle Formula Student | 2026
 *
 * Pressures the ASB decides on, all in bar:
 *  - tank 1 / tank 2: the two EBS air tanks, analog sensors on the ADC
 *  - brake: hydraulic brake pressure, average of front and rear that
 *    come over CAN (asb_can.c)
 * Values are refreshed every 10ms by Sensors_Update() (ADC DMA callback).
 * ========================================================= */

/* Initialization */
void Sensors_Init(void);

/* Pressure Readings */
float Sensors_GetTankPressure1(void);
float Sensors_GetTankPressure2(void);
float Sensors_GetBrakePressure(void);

/* Pressure Checks - limits are in asb_config.h */
bool Sensors_TankPressureValid(void);   /* both tanks in range */
bool Sensors_TankPressure1Valid(void);
bool Sensors_TankPressure2Valid(void);
bool Sensors_BrakePressureEngaged(void);      /* > ASB_BRAKE_ENGAGED_MIN_BAR */
bool Sensors_BrakePressureReleased(void);     /* < ASB_BRAKE_RELEASED_MAX_BAR */
bool Sensors_BrakePressureForOneValve(void);  /* inside the one circuit window */

/* Update Sensor Values */
void Sensors_Update(uint16_t *adc_buf);

#endif /* ASB_SENSORS_H */
