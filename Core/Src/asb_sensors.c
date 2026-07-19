/* =========================================================
 * ASB Sensors - Implementation
 * Aristurtle Formula Student | 2026
 * ========================================================= */

#include "asb_sensors.h"
#include "asb_config.h"
#include "asb_can.h"
#include "main.h"
#include <stdint.h>
#include <stdbool.h>

/* External handles from main.c */
extern uint16_t adc_buffer[ADC_BUFFER_SIZE];
/* Private Variables */
static float tank_pressure1 = 0.0f;
static float tank_pressure2 = 0.0f;
static float brake_pressure = 0.0f;   /* average front and rear brake pressure from CAN - watchable in Live Expressions */

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
    /* M3041-000005-500PG: 0.5-4.5 V ratiometric output, 0-500 psi (0-34.47 bar).
     * The ADC pin sees the sensor output through a 1.5:1 divider, so scale back
     * up to the sensor voltage, then: P[bar] = (Vout - 0.5 V) * (34.47 bar / 4 V) */
    float voltage1 = ((float)adc_buf[0] / 4095.0f) * 3.3f * 1.5f;
    tank_pressure1 = (voltage1 - 0.5f) * (34.4738f / 4.0f);

    float voltage2 = ((float)adc_buf[1] / 4095.0f) * 3.3f * 1.5f;
    tank_pressure2 = (voltage2 - 0.5f) * (34.4738f / 4.0f);

    brake_pressure = (CAN_GetBrakePressureFront() + CAN_GetBrakePressureRear()) / 2.0f;  /* average front and rear brake pressure */
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

