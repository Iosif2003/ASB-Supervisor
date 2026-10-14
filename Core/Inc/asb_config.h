#ifndef ASB_CONFIG_H
#define ASB_CONFIG_H

/* =========================================================
 * ASB Supervisor - System Configuration
 * Aristurtle Formula Student | 2026
 *
 * All the tunable numbers of the ASB in one place: pressure limits,
 * initial check timings, service brake DAC values and the debounce
 * limits of the continuous monitoring. Change things here, not as
 * magic numbers inside the modules.
 * "Sheet N" = sheet of our ASF / continuous monitoring document.
 * ========================================================= */

 /* --- Pressure Limits (bar) --- */
/* tank = the two pneumatic EBS tanks, read by the ADC (asb_sensors.c)
   brake = hydraulic brake line pressure, received over CAN (front/rear average) */
#define ASB_TANK_PRESSURE_MIN_BAR        5.5f    /* Cont.Mon Sheet 6: tanks valid 5.5..8.5 bar */
#define ASB_TANK_PRESSURE_MAX_BAR        8.5f    /* (working pressure 8 bar)                   */
#define ASB_BRAKE_MAX_BAR                350.0f  /* Cont.Mon Sheet 7: BP2 valid 0..350 */
#define ASB_BRAKE_ACTIVE_MIN_BAR         5.0f    /* Cont.Mon Sheet 5: SB actuated -> >=5 bar */
#define ASB_BRAKE_ENGAGED_MIN_BAR        40.0f   /* above this = both circuits braking */
#define ASB_BRAKE_RELEASED_MAX_BAR       3.0f    /* below this = brakes released */
/* window we expect when only ONE circuit brakes and the other one is
   released - used by the initial check to test each valve on its own */
#define ASB_BRAKE_FOR_ONE_VALVE_MIN_BAR  15.0f
#define ASB_BRAKE_FOR_ONE_VALVE_MAX_BAR  60.0f

/* --- Initial Check Timings (ms) --- */
#define ASB_IC_VALVE_SETTLE_MS           700U    /* Initial Checkup: wait 700ms */
#define ASB_IC_APU_READY_TIMEOUT_MS      5000U   /* max wait for the APU to go AS_Ready after the check */
/* --- ADC --- */
#define ASB_ADC_BUFFER_SIZE  2U   /* [0] = tank 1, [1] = tank 2 */

/* --- Service Brake DAC Values --- */
/* 12 bit DAC on PA4 -> proportional valve. 0 = no pressure, 4095 = full (3.3V) */
#define ASB_SB_DAC_DISENGAGED  0U
#define ASB_SB_DAC_PARK        4095U //TODO: find the right signal
#define ASB_SB_DAC_ENGAGED     4095U

/* --- Monitoring debounce (unit = 10ms ticks) --- */
/* number of bad samples IN A ROW before a check counts as failed,
   see Check() in asb_monitoring.h */
#define ASB_MON_INTERLOCK_LIMIT      10U   /* 100ms */
#define ASB_MON_TANK_LIMIT           10U   /* 100ms */
#define ASB_MON_BRAKE_LIMIT          10U   /* 100ms */
#define ASB_MON_APU_LIMIT            30U   /* 300ms, the RES check uses it too */
#define ASB_MON_SERVICE_BRAKE_LIMIT  100U  /* 1s, actuation time */
#define ASB_MON_WATCHDOG_LIMIT       5U    /* 50ms */
#define ASB_MON_MANUAL_LIMIT         200U  /* 2s @10ms (old code had 20x100ms) */
#define RES_OK                       9U    /* RES frame only counts as alive if signal strength > this */

#endif /* ASB_CONFIG_H */
