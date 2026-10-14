/* =========================================================
 * ASB Sensors - Implementation
 * Aristurtle Formula Student | 2026
 *
 * Converts the raw ADC counts of the two tank sensors to bar and builds
 * the brake pressure from the two CAN values. Everything else in the
 * code reads pressures only through the getters / checks here.
 * ========================================================= */

#include "asb_sensors.h"
#include "asb_config.h"
#include "asb_can.h"
#include "main.h"
#include <stdint.h>
#include <stdbool.h>

/* External handles from main.c */
extern uint16_t adc_buffer[ADC_BUFFER_SIZE];   /* filled by DMA, [0] = PC5 tank 1, [1] = PB0 tank 2 */
/* Private Variables (bar) */
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

/* Sensors Update (called from HAL_ADC_ConvCpltCallback in main.c)
   TIM2 triggers one ADC scan every 10ms, so this runs every 10ms in
   interrupt context. */
void Sensors_Update(uint16_t *adc_buf)
{
    /* M3041-000005-500PG (MSP300): 1-5 V output, supply 10-30 V, 0-500 psi
     * (0-34.47 bar). NOT ratiometric - zero pressure sits at 1.0 V (datasheet
     * ordering: M30"4"x = output code 4 = 1-5 V; the trailing 5 is the NPT port).
     * The ADC pin sees the sensor output through a 1.5:1 divider, so scale back
     * up to the sensor voltage, then: P[bar] = (Vout - 1.0 V) * (34.47 bar / 4 V) */
    /* counts -> pin voltage (12 bit, 3.3 V ref) -> sensor voltage (x1.5) -> bar */
    float voltage1 = ((float)adc_buf[0] / 4095.0f) * 3.3f * 1.5f;
    tank_pressure1 = (voltage1 - 1.0f) * (34.4738f / 4.0f);

    float voltage2 = ((float)adc_buf[1] / 4095.0f) * 3.3f * 1.5f;
    tank_pressure2 = (voltage2 - 1.0f) * (34.4738f / 4.0f);
    /* a disconnected sensor reads ~0 V -> about -8.6 bar -> fails the range check */

    brake_pressure = (CAN_GetBrakePressureFront() + CAN_GetBrakePressureRear()) / 2.0f;  /* average front and rear brake pressure */
}

/* Pressure Checks - all limits are exclusive (> min, < max) */

bool Sensors_TankPressure1Valid(void)
{
    return tank_pressure1 > ASB_TANK_PRESSURE_MIN_BAR &&
            tank_pressure1 < ASB_TANK_PRESSURE_MAX_BAR;

}

bool Sensors_TankPressure2Valid(void)
{
    return tank_pressure2 > ASB_TANK_PRESSURE_MIN_BAR &&
            tank_pressure2 < ASB_TANK_PRESSURE_MAX_BAR;
}

bool Sensors_TankPressureValid(void)
{
    return Sensors_TankPressure1Valid() && Sensors_TankPressure2Valid();
}

bool Sensors_BrakePressureEngaged(void)
{
    return (brake_pressure > ASB_BRAKE_ENGAGED_MIN_BAR);
}

bool Sensors_BrakePressureReleased(void)
{
    return (brake_pressure < ASB_BRAKE_RELEASED_MAX_BAR);
}
/* one circuit braking, the other released (used by the initial check) */
bool Sensors_BrakePressureForOneValve(void)
{
    return (brake_pressure > ASB_BRAKE_FOR_ONE_VALVE_MIN_BAR && brake_pressure < ASB_BRAKE_FOR_ONE_VALVE_MAX_BAR);
}
