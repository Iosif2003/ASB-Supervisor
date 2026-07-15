#ifndef ASB_CONFIG_H
#define ASB_CONFIG_H

/* =========================================================
 * ASB Supervisor — System Configuration
 * Aristurtle Formula Student | 2026
 * ========================================================= */

 /* --- Pressure Limits (bar) --- */
#define ASB_TANK_PRESSURE_MIN_BAR        5.5f
#define ASB_TANK_PRESSURE_MAX_BAR        7.5f
#define ASB_BRAKE_MAX_BAR                350.0f  /* Cont.Mon Sheet 8: BP2 valid 0..350 */
#define ASB_BRAKE_ENGAGED_MIN_BAR        5.0f
#define ASB_BRAKE_RELEASED_MAX_BAR       3.0f
#define ASB_BRAKE_ACTIVE_MIN_BAR         5.0f    /* Cont.Mon Sheet 5: service brake fail if <5 */

/* --- Initial Check Timings (ms) --- */
#define ASB_IC_VALVE_SETTLE_MS           700U    /* Initial Checkup: wait 200ms */
#define ASB_IC_APU_READY_TIMEOUT_MS      5000U

/* --- ADC --- */
#define ASB_ADC_BUFFER_SIZE  2U

/* --- Service Brake DAC Values --- */
#define ASB_SB_DAC_DISENGAGED  0U
#define ASB_SB_DAC_PARK        4095U //TODO: find the right signal
#define ASB_SB_DAC_ENGAGED     4095U

/* --- Monitoring debounce (μονάδα = 10ms ticks) --- */
#define ASB_MON_INTERLOCK_LIMIT      10U   /* 100ms */
#define ASB_MON_TANK_LIMIT           10U
#define ASB_MON_BRAKE_LIMIT          10U
#define ASB_MON_APU_LIMIT            5U
#define ASB_MON_SERVICE_BRAKE_LIMIT  100U  /* 1s: χρόνος actuation */
#define ASB_MON_WATCHDOG_LIMIT       5U
#define ASB_MON_MANUAL_LIMIT         200U  /* 2s @10ms (παλιά 20×100ms) */
#define RES_OK                       10U    /* RES signal strength threshold for alive */

#endif /* ASB_CONFIG_H */
