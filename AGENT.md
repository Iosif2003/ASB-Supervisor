# ASB Supervisor — Claude Code Project Agenda
**Aristurtle Formula Student | Iosif | STM32F446RE | 2026**

> This document is the master reference for Claude Code sessions on the ASB Supervisor project.
> Load it at the start of every session. It contains architecture, current state, known issues, and task backlog.

---

## 1. Repository

```
Repo:     https://github.com/Iosif2003/ASB-Supervisor (PUBLIC)
Branch:   main
Toolchain: STM32CubeMX + STM32CubeIDE / CMake
MCU:      STM32F446RE (Cortex-M4, 160MHz)
```

### File Structure (relevant files)

```
Core/
├── Src/
│   ├── main.c              ← ALL logic lives here (1683 lines — monolithic)
│   ├── can_mcu.c           ← Auto-generated from .dbc (11522 lines)
│   └── stm32f4xx_it.c      ← IRQ handlers (minimal, delegates to HAL)
├── Inc/
│   ├── main.h              ← Pin defines, externs, function prototypes
│   ├── can_mcu.h           ← Auto-generated CAN structs/IDs (17895 lines)
│   └── stm32f4xx_it.h
```

> **Note:** `main.c` lines 1001–1683 have NOT been fully reviewed in past sessions.
> These contain: `Init_All()`, `Manual_Initial_Check()`, `Manual_Monitoring()`,
> `Continuous_Monitoring()`, and all HAL callbacks.
> **Fetch these first** if working on monitoring, callbacks, or initialization logic.

---

## 2. Hardware Quick Reference

### Clock Tree

```
HSI (16MHz) → PLL (×20 / 2) → SYSCLK = 160MHz
APB1 = 40MHz  (TIM2,3,4,5,6,7 / CAN1)
APB2 = 80MHz  (ADC1)
```

### GPIO Pin Map

| Signal | Pin | Dir | Description |
|---|---|---|---|
| Interlock_Steering | PA1 | IN | Steering interlock feedback |
| ASRelay_State | PA2 | OUT OD | SDC AS-Relay control |
| PrAnagSignal (DAC) | PA4 | ANALOG OUT | Proportional valve signal |
| Interlock_Valve1 | PA15 | IN | Valve 1 interlock |
| TankPrServBrake (ADC) | PC4 | ANALOG IN | EBS tank pressure sensor |
| PrSenServBrake (ADC) | PC5 | ANALOG IN | Brake pressure sensor |
| UserLed | PC6 | OUT PP | Debug LED |
| WatchdogPWM | PC9 | OUT PWM | Watchdog pulse (TIM3 CH4) |
| Interlock_valve2 | PC10 | IN | Valve 2 interlock |
| Interlock_PV | PC11 | IN | Proportional valve interlock |
| ASRelay_In | PB12 | IN | AS relay input feedback |
| ASRelay_Out | PB13 | IN | AS relay output feedback |
| TSMS_Out_NOT | PB14 | IN | TSMS inverted |
| ASMS_Out | PB15 | IN | ASMS switch |
| Valve2_GND_ST | PB5 | OUT PP | EBS Valve 2 control |
| Valve1_GND_ST | PB6 | OUT PP | EBS Valve 1 control |

### Timer Configuration

| Timer | PSC | ARR | Period | Function |
|---|---|---|---|---|
| TIM2 | 49999 | 15 | ~200Hz | ADC trigger (TRGO → DMA) |
| TIM3 | 79 | 39999 | 500Hz | Watchdog PWM (CH4, Pulse=30) |
| TIM4 | 49999 | 15 | ~200Hz | Monitoring interval |
| TIM5 | 39999 | 39 | ~50ms | Monitoring task trigger |
| TIM6 | 64934 | 6159 | ~1s | DAC / CAN periodic task |
| TIM7 | 7999 | 999 | 10ms | CAN TX base (50ms via counter) |

### ADC / DAC

```
ADC1:  CH14 (PC4), 12-bit, triggered by TIM2 TRGO, DMA circular, buffer = 1 sample
DAC:   CH1  (PA4), no trigger, output buffer ENABLED, amplified externally to 0–10V
       External op-amp: TLV9351, non-inverting, gain ~3–4×
```

### CAN Bus

```
Peripheral: CAN1 (bxCAN)
Baud:       500 kbps — Prescaler=4, BS1=7TQ, BS2=2TQ, SJW=2TQ @ 40MHz APB1
AutoBusOff: ENABLE
AutoRetransmission: ENABLE
DB tool:    cantools v40.2.1 (generates can_mcu.c/h from .dbc)
```

---

## 3. Software Architecture

### Enumerations

```c
typedef enum { Servo_Disengaged = 1, Servo_Engaged = 2, Servo_Available = 3 } ServoEnum;
typedef enum { EBS_Unavailable  = 1, EBS_Armed     = 2, EBS_Triggered   = 3 } EBSEnum;
```

### Mission Modes (Selected_Mission() return)

```
0 = No_Mission   → invalid / neither path
1 = Autonomous   → VCU mode==0 AND AS_Mission in [1..6]
2 = Manual       → VCU mode==1 OR AS_Mission==7
```

### AS State Machine (from APU via CAN 0x00A)

| Value | State | ASB Behavior |
|---|---|---|
| 0 | AS_ManualDriving | Manual mode path |
| 1 | AS_Off | Reset monitoring, open SDC, run `As_Initial_Check()` if ASMS=1 |
| 2 | AS_Ready | Initial check done, waiting for mission |
| 3 | AS_Driving | Full monitoring. If AS_Set_Finished → open valves + SDC |
| 4 | AS_Finished | Mission done, EBS activated, SDC opened |
| 5 | AS_Emergency | Close both valves, open SDC, stop monitoring |

### EBS Valve Logic — CRITICAL FAIL-SAFE

```c
// EBS ACTIVATE (brakes ON) — de-energize valves
HAL_GPIO_WritePin(GPIOB, Valve1_GND_ST_Pin, GPIO_PIN_RESET);
HAL_GPIO_WritePin(GPIOB, Valve2_GND_ST_Pin, GPIO_PIN_RESET);

// EBS RELEASE (brakes OFF) — energize valves
HAL_GPIO_WritePin(GPIOB, Valve1_GND_ST_Pin, GPIO_PIN_SET);
HAL_GPIO_WritePin(GPIOB, Valve2_GND_ST_Pin, GPIO_PIN_SET);

// SDC OPEN
HAL_GPIO_WritePin(GPIOA, ASRelay_State_Pin, GPIO_PIN_RESET);

// SDC CLOSE
HAL_GPIO_WritePin(GPIOA, ASRelay_State_Pin, GPIO_PIN_SET);
```

> **Default/reset state = brakes ENGAGED (fail-safe per FS rules)**

### Watchdog — TIM3 OPM Mechanism

```c
// Enable OPM (autonomous mode — firmware hang = no pulse = WD fires EBS)
HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_4);
htim3.Instance->CR1 |= TIM_CR1_OPM;
__HAL_TIM_SET_COUNTER(&htim3, 0);
HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);

// Disable OPM (manual mode — continuous PWM always present)
HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_4);
htim3.Instance->CR1 &= ~TIM_CR1_OPM;
__HAL_TIM_SET_COUNTER(&htim3, 0);
HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
```

### Initial Check Sequence (`As_Initial_Check()`)

Tracked via `Initial_Check_Step` (volatile int), reported over CAN.

| Step | Action |
|---|---|
| 0 | Close RES relays and SDC before AS relay |
| 1 | Wait for RES to close — hold EBS activated (valves LOW) |
| 2 | Re-enable watchdog (switch TIM3 to OPM mode) |
| 3 | Check tank pressure (> 3.5 bar) and brake pressure |
| 4 | Close SDC (ASRelay_State = HIGH) |
| 5 | EBS arming — energize valves, verify interlock signals |
| 6 | De-energize EBS electric valve (verify EBS can activate) |
| 7 | Check service brake (servo) status |
| 8 | Check APU CAN heartbeat |

On success: `Initial_Checked = true` → APU proceeds to AS_Ready.

---

## 4. CAN Message Map

### Transmitted by ASB

| Frame | ID | Period | Key Signals |
|---|---|---|---|
| ASB | 0x023 | 50ms | ASMS_State, TSMS_Out, Initial_Checked, EBS_Status, Service_Brake_Status, Initial_Check_Step, Monitor_Tank/Brake/Servo/APU, EBS_Tank_Pressure |
| ASB_DATALOGGER | 0x511 | 100ms | EBS_Pneumatic_Pressure, Brake_Pressure_Front/Rear, EBSstate_*, Servicebrakestate_*, Watchdog_OK, ValveInterlock_OK, ServoInterlock_OK, AS_State, ASMS_Out |

### Received by ASB

| Frame | ID | Period | Key Signals |
|---|---|---|---|
| APU_STATE_MISSION | 0x00A | 125ms | AS_State (1–5), AS_Mission (0–7), AS_Set_Finished |
| VCU_BOOLS | 0x320 | 100ms | Mode (0=Auto, 1=Manual) |
| VCU_SERVO_CONTROL | 0x324 | 10ms | Servo_control command |
| DASH_BRAKE | 0x065 | 5ms | BRAKE_Pressure_Front/Rear |
| RES_STATUS | 0x191 | 30ms | Stop button state |

### CAN TX Architecture

All outgoing CAN messages go through:
```c
static HAL_StatusTypeDef CAN_SendStdMessage(uint32_t std_id, uint32_t dlc,
                                             const uint8_t *data,
                                             uint32_t *mailbox);
```
- 5ms timeout guard for mailbox availability
- Sets `Can_Error = true` on failure
- Calls `Error_Handler()` on critical failure

---

## 5. Key Global Variables

| Variable | Type | Description |
|---|---|---|
| EBS_Status | EBSEnum | Current EBS state |
| Servo_Status | ServoEnum | Service brake state |
| Tank_Pressure | float | EBS pneumatic tank pressure (bar) |
| Brake_Pressure | float | Brake pressure (bar) |
| brake_pressure_front/rear | float | Front/rear brake pressure |
| Initial_Checked | bool | Autonomous initial check done |
| Manual_Initial_Checked | bool | Manual initial check done |
| Monitoring | volatile bool | Monitoring loop active |
| Monitor_All | volatile bool | Full monitoring active |
| ASRelay_State | bool | SDC relay output state |
| Valve1_GND / Valve2_GND | bool | EBS valve control state |
| ASMS_Out / TSMS_Out_NOT | bool | ASMS/TSMS GPIO inputs |
| ASRelay_In / ASRelay_Out | bool | AS relay feedback |
| Interlock_Valve1/2/PV/Steering | bool | Interlock GPIO inputs |
| adc_buffer[1] | uint16_t | DMA ADC buffer (raw 12-bit) |
| Tank_Pressure_min | #define 3.5 | Min valid tank pressure (bar) |
| Tank_Pressure_max | #define 10 | Max valid tank pressure (bar) |
| Debug_State | volatile int | Debug code (readable via debugger) |
| Initial_Check_Step | volatile int | Current initial check step (0–8) |
| APU_Transition | volatile int | APU state transition debug |
| Can_Error | bool | CAN error flag |
| Watchdog_Check | bool | Watchdog monitor result |
| Brake_Pressure_Check | bool | Brake pressure monitor result |
| Tank_Pressure_Check | bool | Tank pressure monitor result |
| Interlock_Valve1/2_Check | bool | Valve interlock monitor results |
| APU_Communication_Check | bool | APU CAN timeout monitor |
| Service_Brake_Check | bool | Servo interlock monitor |

---

## 6. Monitoring System

### Continuous Monitoring (Autonomous)

Each check has:
- A `bool` result flag
- An error counter (e.g., `Brake_Pressure_Check_errors`)
- A timing variable (e.g., `Brake_Pressure_Check_ms`)

Monitored parameters:
- `Watchdog_Check` — external WD circuit responding
- `Tank_Pressure_Check` — pressure in [3.5, 10] bar
- `Brake_Pressure_Check` — above minimum threshold
- `Interlock_Valve1/2_Check` — interlock GPIO signals valid
- `APU_Communication_Check` — CAN messages arriving within timeout
- `Service_Brake_Check` — servo interlock valid

When error counter exceeds threshold → flag goes FALSE → EBS triggered.

### Manual Monitoring (`Manual_Monitoring()`)

Reduced checks:
- `EBS_Status` must stay `EBS_Unavailable`
- `ASMS` must be 0
- SDC state verification

---

## 7. IRQ Handler Summary

| Handler | Function |
|---|---|
| SysTick_Handler | HAL_IncTick() — 1ms tick |
| CAN1_TX_IRQHandler | HAL_CAN_IRQHandler() — TX complete |
| CAN1_RX0_IRQHandler | HAL_CAN_IRQHandler() — RX FIFO0 |
| TIM2_IRQHandler | ADC trigger tick |
| TIM4_IRQHandler | Monitoring interval |
| TIM5_IRQHandler | 50ms monitoring task |
| TIM6_DAC_IRQHandler | DAC update + CAN periodic |
| TIM7_IRQHandler | 10ms CAN TX base |
| DMA2_Stream0_IRQHandler | ADC DMA complete |

### HAL Callbacks (in main.c)

```c
HAL_TIM_PeriodElapsedCallback()      // TIM4, TIM5, TIM6, TIM7 logic
HAL_CAN_RxFifo0MsgPendingCallback()  // Unpacks incoming CAN frames
HAL_ADC_ConvCpltCallback()           // Processes ADC result from DMA
```

---

## 8. Known Issues & Technical Debt

### Architecture

- [ ] **Monolithic main.c (1683 lines)** — All logic in one file. Should be split into:
  - `asb_state.c` / `asb_state.h` — AS state machine
  - `ebs_control.c` / `ebs_control.h` — valve + EBS logic
  - `monitoring.c` / `monitoring.h` — all check functions
  - `can_tasks.c` / `can_tasks.h` — TX/RX task functions
  - `initial_check.c` / `initial_check.h` — `As_Initial_Check()`

### Peripheral / Signal

- [ ] **ADC buffer size = 1** — single sample per DMA trigger, no averaging.
  Susceptible to noise. Consider software averaging (e.g., rolling buffer N=8).

- [ ] **DAC output buffer enabled** — introduces small DC offset. May need
  calibration if proportional valve signal accuracy is critical.

- [ ] **CAN AutoRetransmission ENABLED** — can cause mailbox stalls under high
  bus load. The 5ms timeout in `CAN_SendStdMessage()` partially guards this,
  but monitor for `Can_Error` spikes under load.

- [ ] **TIM4 purpose not confirmed from code** — document what
  `TIM4_IRQHandler` actually triggers in `HAL_TIM_PeriodElapsedCallback()`.

### Code quality

- [ ] **Global ISR-shared variables** — verify all globals shared between
  main loop and ISR are marked `volatile`.

- [ ] **main.c lines 1001–1683 not fully reviewed** — fetch from GitHub before
  touching monitoring, callbacks, or initialization.

---

## 9. Task Backlog

### High Priority (Safety / Competition)

- [ ] Review and document `Continuous_Monitoring()` logic end-to-end
- [ ] Verify all error counter thresholds are appropriate for competition use
- [ ] Confirm `APU_Communication_Check` timeout is within FS rules requirement
- [ ] Test watchdog OPM mode switch (manual → auto → manual) for edge cases
- [ ] Verify DAC → op-amp → valve signal calibration (0–10V range)
- [ ] Document what `Manual_Initial_Check()` verifies and under what conditions it passes

### Medium Priority (Robustness)

- [ ] Add software averaging to ADC readings (tank pressure)
- [ ] Add DAC output offset calibration constant
- [ ] Add `volatile` audit on all ISR-shared globals
- [ ] Verify TIM4 purpose in `PeriodElapsedCallback`
- [ ] Add CAN TX mailbox stall detection / recovery logic

### Low Priority (Maintenance / Refactoring)

- [ ] Split main.c into modules (see Architecture section)
- [ ] Add Doxygen comments to all public functions
- [ ] Create a dedicated `config.h` for all `#define` thresholds
- [ ] Replace magic numbers in monitoring checks with named constants

---

## 10. FS Rules Compliance Checklist

| Requirement | Status | Notes |
|---|---|---|
| EBS activates on RES open | ? | Verify in `HAL_CAN_RxFifo0MsgPendingCallback` |
| EBS activates on APU CAN loss | ? | Verify `APU_Communication_Check` timeout |
| EBS activates on monitoring fault | ? | Verify error counter thresholds |
| Fail-safe default = brakes engaged | ✅ | GPIO_PIN_RESET → valve OFF |
| SDC opens on any fault | ? | Verify in every emergency path |
| Initial check before AS_Ready | ✅ | `As_Initial_Check()` steps 0–8 |
| Interlock verification before arming | ✅ | Step 5 of initial check |
| No blocking loops without timeout | ? | Audit all `while()` loops in main.c |

---

## 11. Claude Code Behavioral Rules

When working on this project:

1. **Fail-safe first** — any undefined state → EBS activate + SDC open.
   Never suggest logic that could leave brakes released on fault.

2. **No `HAL_Delay()` in main loop or ISR** — always use `HAL_GetTick()` with
   timeout guard.

3. **Volatile required** — all globals shared between main loop and ISR must
   be `volatile`. Never remove this.

4. **CAN TX always via `CAN_SendStdMessage()`** — never call
   `HAL_CAN_AddTxMessage()` directly.

5. **Before touching monitoring or callbacks** — fetch
   `main.c` lines 1001–1683 from GitHub first.

6. **Competition-robust ≠ "works in lab"** — account for CAN noise, sensor
   glitches, power-on transients, partial SDC states.

7. **Separate concerns** — hardware signal behavior vs. firmware logic vs.
   CAN timing vs. measurement artifact. Diagnose layer by layer.

8. **Document safety implications** — any change that affects safety behavior
   must be explainable to FS scrutineers.

9. **Error counter pattern** — new monitoring checks must update both the
   `bool` flag AND the error counter. Never use a raw boolean toggle.

10. **State machine completeness** — every new state path must handle the
    default/error case with fail-safe behavior.

---

## 12. Quick Reference Cheat Sheet

```
EBS ACTIVATE   → Valve1_GND_ST = RESET, Valve2_GND_ST = RESET
EBS RELEASE    → Valve1_GND_ST = SET,   Valve2_GND_ST = SET
SDC OPEN       → ASRelay_State = RESET
SDC CLOSE      → ASRelay_State = SET

WATCHDOG OPM ON  → Stop TIM3, CR1|=OPM, counter=0, Start TIM3
WATCHDOG OPM OFF → Stop TIM3, CR1&=~OPM, counter=0, Start TIM3

PRESSURE LIMITS → min=3.5 bar, max=10 bar

CAN TX IDs:
  ASB status     → 0x023
  ASB datalogger → 0x511

CAN RX IDs:
  APU state      → 0x00A (125ms timeout)
  VCU bools      → 0x320
  Servo control  → 0x324
  Brake pressure → 0x065
  RES status     → 0x191

AS_STATE: 0=ManualDriving, 1=Off, 2=Ready, 3=Driving, 4=Finished, 5=Emergency
EBS_STATUS: 1=Unavailable, 2=Armed, 3=Triggered
SERVO_STATUS: 1=Disengaged, 2=Engaged, 3=Available
MISSION: 0=None, 1=Accel, 2=Skidpad, 3=Autocross, 4=Trackdrive,
         5=EBSTest, 6=Inspection, 7=Manual
```

---

*Last updated: 2026-04-14 | Maintainer: Iosif | Aristurtle FS Team*
