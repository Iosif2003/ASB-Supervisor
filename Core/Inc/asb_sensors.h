#ifndef ASB_SENSORS_H
#define ASB_SENSORS_H

#include <stdbool.h>
#include <stdint.h>

/* =========================================================
 * ASB Sensors - asb_sensors.h
 * Aristurtle Formula Student | 2026
 * ========================================================= */

/* Initialization */
void Sensors_Init(void);

/* Pressure Readings */
float Sensors_GetTankPressure1(void);
float Sensors_GetTankPressure2(void);
float Sensors_GetBrakePressure(void);

/* Pressure Checks */
bool Sensors_TankPressureValid(void);   /* both tanks in range */
bool Sensors_TankPressure1Valid(void);
bool Sensors_TankPressure2Valid(void);
bool Sensors_BrakePressureEngaged(void);
bool Sensors_BrakePressureReleased(void);
bool Sensors_BrakePressureForOneValve(void);

/* Update Sensor Values */
void Sensors_Update(uint16_t *adc_buf);

#endif /* ASB_SENSORS_H */