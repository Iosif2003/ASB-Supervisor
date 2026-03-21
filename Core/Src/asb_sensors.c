/* =========================================================
 * ASB Sensors — Implementation
 * Aristurtle Formula Student | 2026
 * ========================================================= */

#include "asb_sensors.h"
#include "asb_config.h"
#include "main.h"
#include <stdint.h>
#include <stdbool.h>

/* External handles from main.c */
extern uint16_t adc_buffer[ADC_BUFFER_SIZE];
/* Private Variables */
static float tank_pressure1 = 0.0f;
static float tank_pressure2 = 0.0f;
static float brake_pressure = 0.0f;

/* States */
float Sensors_GetTankPressure1(void)  { return tank_pressure1;  }
float Sensors_GetTankPressure2(void)  { return tank_pressure2; }
float Sensors_GetBrakePressure(void)  { return brake_pressure; }

/* Sensors Initialization */
void Sensors_Init(void)
{
    tank_pressure1 = 0.0f;
    tank_pressure2 = 0.0f;
    brake_pressure = 0.0f;
}

/* Sensors Update (called from HAL_ADC_ConvCpltCallback in main.c) */
void Sensors_Update(uint16_t *adc_buf)
{
    float voltage1 = ((float)adc_buf[0] / 4095.0f) * 3.3f;
    tank_pressure1 = (2.5f * (voltage1 * 1.5f)) - 2.5f;

    float voltage2 = ((float)adc_buf[1] / 4095.0f) * 3.3f;
    tank_pressure2 = (2.5f * (voltage2 * 1.5f)) - 2.5f;
}

/* Pressure Checks */

bool Sensors_TankPressureValid(void)
{
    return (tank_pressure1 > ASB_TANK_PRESSURE_MIN_BAR &&
            tank_pressure1 < ASB_TANK_PRESSURE_MAX_BAR &&
            tank_pressure2 > ASB_TANK_PRESSURE_MIN_BAR &&
            tank_pressure2 < ASB_TANK_PRESSURE_MAX_BAR);
}

bool Sensors_BrakePressureEngaged(void)
{
    return (brake_pressure > ASB_BRAKE_ENGAGED_MIN_BAR);
}

bool Sensors_BrakePressureReleased(void)
{
    return (brake_pressure < ASB_BRAKE_RELEASED_MAX_BAR);
}

