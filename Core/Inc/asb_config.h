#ifndef ASB_CONFIG_H
#define ASB_CONFIG_H

/* =========================================================
 * ASB Supervisor — System Configuration
 * Aristurtle Formula Student | 2026
 * ========================================================= */

 /* --- Pressure Limits (bar) --- */
#define ASB_TANK_PRESSURE_MIN_BAR        3.5f
#define ASB_TANK_PRESSURE_MAX_BAR        10.0f
#define ASB_BRAKE_ENGAGED_MIN_BAR        11.0f
#define ASB_BRAKE_RELEASED_MAX_BAR       3.0f
#define ASB_BRAKE_ACTIVE_MIN_BAR         5.0f

/* --- Initial Check Timings (ms) --- */
#define ASB_IC_VALVE_SETTLE_MS           700U
#define ASB_IC_APU_READY_TIMEOUT_MS      5000U

/* --- ADC --- */
#define ASB_ADC_BUFFER_SIZE  2U

#endif /* ASB_CONFIG_H */