# EZkontrol_MCU_Emulator

STM32F103C8T6 (Blue Pill) STM32CubeIDE/CubeMX project that **emulates an
EZkontrol MCU1 motor controller (address 0xEF)** over CAN, for bench
testing the **[EZkontrol_VCU](../EZkontrol_VCU)** project before a real
72V EZkontrol controller is connected.

This is a low-voltage CAN protocol simulator only. There is no real
traction motor, contactor, 72V power circuit, or power stage involved -
all "motor" behavior here is a software ramp model used solely to prove
CAN communication and value tracking end-to-end.

This is the second-generation, standalone repository for the MCU-emulator
role. It supersedes the bench-test code in `controller_emulator`, reusing
that project's **proven, working** CAN configuration and protocol
implementation (confirmed working: CAN physical layer, 500 kbit/s timing,
29-bit extended IDs, handshake, command decode - CAN TEC=0, no timeouts,
no bad DLC) rather than redesigning it from scratch.

No PDF files describing the EZkontrol protocol were present in the build
environment used to create this repository, despite being referenced in
the originating task. Where the task description quotes manufacturer PDF
text that could not be independently verified, that is called out
explicitly below and in code comments (see Section 8, Meter speed field).

---

## 1. Hardware / clock / CAN configuration

Identical to `EZkontrol_VCU` - see that repository's README Section 2 for
the full table. Summary: STM32F103C8T6, HSE 8MHz, PLL x9, SYSCLK 72MHz,
APB1 36MHz, CAN1 Normal mode, PA11=RX/PA12=TX, 500 kbit/s
(Prescaler=4, SJW=1, BS1=15, BS2=2), 29-bit extended frames DLC=8,
only the CAN1 RX FIFO1 interrupt is enabled in NVIC (TX-complete and bus
error state are polled from CAN registers every task tick instead of
wiring additional interrupt lines).

## 2. Addressing / J1939-like ID format

Shared verbatim with `EZkontrol_VCU` via `ezkontrol_protocol.h` - see that
repository's README Section 3 for the full derivation and the node
address table, and for the **documented manufacturer PDF typo** regarding
the Meter address (23 decimal = `0x17`, not `0x11`).

This node represents **MCU1, address 0xEF**.

---

## 3. Handshake

```
MCU1 -> VCU   ID=0x1801D0EF  DLC=8  data = 55 55 55 55 55 55 55 55
VCU  -> MCU1  ID=0x0C01EFD0  DLC=8  data = AA AA AA AA AA AA AA AA
```

At startup this node repeatedly transmits the `0x55` handshake pattern
every 250 ms until it receives the complete 8-byte `0xAA` reply from the
VCU on `0x0C01EFD0`. The handshake reply and normal VCU commands share
that same ID; they are distinguished by checking whether all 8 payload
bytes equal `0xAA`.

```c
emu_handshake_tx_count
emu_handshake_reply_count
emu_handshake_established    // 1 once the AA reply is received
```

Once established: HANDSHAKE LED (PB0) ON, and normal feedback
transmission begins.

---

## 4. Receiving VCU commands (`0x0C01EFD0`, DLC 8)

| Bytes | Field | Resolution | Offset |
|---|---|---|---|
| 0-1 | Target phase current | 0.1 A/bit | -3200 A |
| 2-3 | Target speed | 1 rpm/bit | -32000 rpm |
| 4 bit0 | RUN(1)/HALT(0) | - | - |
| 4 bit1 | Speed(1)/Torque(0) mode | - | - |
| 7 | VCU life counter | - | - |

Decoded values (always unconditionally read/decoded - this node has no
"real controller" to protect, so unlike the VCU it is safe for it to
freely simulate RUN behavior):

```c
emu_command_rx_count
emu_last_command_tick
emu_cmd_run
emu_cmd_speed_mode
emu_cmd_target_phase_current_0p1A
emu_cmd_target_speed_rpm
emu_cmd_life
```

### Command watchdog (250 ms)

If no valid VCU command is received for 250 ms after handshake:

```c
emu_command_timeout_count   // incremented once per timeout transition
```

- `emu_cmd_run` is forced to 0
- target speed/current are forced to 0
- WARNING LED (PB11) ON
- the simulated motor model (below) decays toward 0 exactly as if RUN=0
  had been commanded - a stale RUN is never allowed to persist after a
  communication timeout.

---

## 5. Simulated motor model

Feedback visibly changes in response to commands - it is **not** a fixed
echo. A simple ramp model runs every 20 ms:

- **Speed** (`emu_feedback_speed_rpm`) ramps toward
  `emu_cmd_target_speed_rpm` at `emu_sim_speed_ramp_rate_rpm_per_s`
  (default 2000 rpm/s) when `emu_cmd_run=1`; ramps toward 0 at the same
  rate when `emu_cmd_run=0` (regardless of what the target was).
- **Phase current** (`emu_feedback_phase_current_0p1A`) tracks the full
  commanded target current while the speed error exceeds
  `emu_sim_speed_error_threshold_rpm` (default 50 rpm - i.e. while
  actively accelerating), then ramps down to
  `emu_sim_current_holding_pct`% (default 25%) of the commanded current
  once close to the target speed (approximating a reduced holding
  current at steady speed). Ramps toward 0 when RUN=0.
- **Bus current** is a simple proportional analogy
  (`phase_current / 3`) - not independently modeled.

All rate/threshold constants are `volatile` globals, editable live:

```c
emu_sim_speed_ramp_rate_rpm_per_s         // default 2000
emu_sim_current_ramp_rate_0p1A_per_s      // default 1000
emu_sim_current_holding_pct               // default 25
emu_sim_speed_error_threshold_rpm         // default 50
```

This is intentionally not a physically accurate motor model - it exists
only to make CAN feedback visibly and verifiably respond to commands
(forward ramp, reverse ramp, return to zero) during bench testing.

---

## 6. MCU1 -> VCU feedback (transmitted)

### Message I (`0x1801D0EF`, 50 ms, DLC 8)

Same layout as decoded on the VCU side (see `EZkontrol_VCU` README
Section 7). Voltage/temperature values are **directly editable** test
knobs; speed/current are driven by the simulated motor model above.

```c
emu_feedback_bus_voltage_0p1V     // directly editable, default 720 (72.0V)
emu_feedback_bus_current_0p1A     // derived (phase_current/3)
emu_feedback_phase_current_0p1A   // driven by motor model
emu_feedback_speed_rpm            // driven by motor model
emu_status1_tx_count
```

### Message II (`0x1802D0EF`, 50 ms, DLC 8)

```c
emu_feedback_controller_temp_C    // directly editable, default 25
emu_feedback_motor_temp_C         // directly editable, default 25
emu_controller_running            // mirrors emu_cmd_run
emu_controller_speed_mode
emu_fault_byte4 / emu_fault_byte5 / emu_fault_byte6   // raw, directly editable
emu_simulated_fault               // 0->1: sets byte4 bit0, a generic
                                   // TEST-ONLY flag (see note below)
emu_life_counter
emu_status2_tx_count
```

**Note on `emu_simulated_fault`:** byte4 bit0 is not among the fault bits
that were confirmed against the supplied material during earlier bench
work (confirmed bits are byte4 bit4/5/6 and byte5 bit4 - see
`EZkontrol_VCU` README Section 7). `emu_simulated_fault` uses bit0 purely
as a convenient, clearly-labeled test toggle so you can verify the VCU's
fault-bit decode path end-to-end; it does not claim to represent any
specific documented manufacturer fault condition.

---

## 7. Testable feedback changes

Edit these on the emulator's Live Expressions and confirm the
corresponding `vcu_*` variable updates on the VCU within ~50-100 ms:

| Emulator variable | Example change | VCU variable to check |
|---|---|---|
| `emu_feedback_bus_voltage_0p1V` | 720 -> 700 | `vcu_bus_voltage_0p1V` |
| `emu_feedback_controller_temp_C` | 25 -> 50 | `vcu_controller_temp_C` |
| `emu_feedback_motor_temp_C` | 25 -> 60 | `vcu_motor_temp_C` |
| `emu_simulated_fault` | 0 -> 1 | `vcu_fault_test_bit0` |

All of these are plain `volatile` globals (never optimized away - every
declared variable in this module was individually verified present in
the final linked ELF via `arm-none-eabi-nm`, 62/62 symbols).

---

## 8. Optional MCU1 -> Meter frames

### Message I (`0x180117EF`, 100 ms)

Same voltage/bus-current/phase-current fields as Status I.

#### ⚠️ Meter speed field - implementation choice, not confirmed encoding

The Meter V1.1 PDF (per the task that produced this repository) documents
this field as 0.1 rpm/bit, offset -32000 rpm - which is mathematically
inconsistent for a 16-bit value (see `EZkontrol_VCU` README Section 8 for
the detailed explanation). Since this emulator is the one *encoding* the
frame (not decoding an unknown manufacturer device), it makes an explicit
choice, clearly documented in code
(`send_meter1()` in `ezkontrol_mcu_emulator.c`): it encodes this field
identically to the unambiguous Status I speed field (1 rpm/bit, offset
-32000), purely so the bench test has a self-consistent value. **This is
an implementation choice for testing, not a confirmed manufacturer
encoding** - do not assume a real EZkontrol controller encodes this field
the same way.

```c
emu_meter1_tx_count
```

### Message II (`0x180217EF`, 100 ms)

```c
emu_throttle_percent        // 0-100
emu_gear_raw                // 0=NO,1=R,2=N,3=D1,4=D2,5=D3,6=S,7=P
emu_brake_active
emu_operation_mode_raw      // 0=Stop,1=Drive,2=Cruise,3=EBS,4=Hold
emu_dc_contactor_on
emu_meter_life
emu_meter2_tx_count
```

---

## 9. LED indications

| LED | Pin | Meaning |
|---|---|---|
| HANDSHAKE | PB0 | OFF before handshake, ON after valid AA reply |
| COMMAND HEARTBEAT | PB1 | Toggles on each valid normal VCU command received |
| RUN | PB10 | ON while `emu_cmd_run=1`, OFF for HALT |
| WARNING | PB11 | ON before handshake; ON if command timeout (>250ms); OFF when communication healthy |

GPIO config: Output, Push-Pull, No Pull, Low Speed, initial LOW.
External wiring assumed: GPIO -> resistor -> LED -> GND (active HIGH).

Expected when RUN is active and communication healthy: PB0 ON, PB1
toggling, PB10 ON, PB11 OFF. When HALT: PB10 OFF. When commands stop for
>250ms: PB11 ON and the simulated controller state goes to HALT
(regardless of the last commanded RUN value).

---

## 10. Diagnostics (Live Expressions)

```c
// Initialization results (0 == HAL_OK)
emu_init_filter_result, emu_init_start_result, emu_init_notify_result

// CAN registers (captured every main-loop tick)
emu_can_esr, emu_can_tsr, emu_can_rf0r, emu_can_rf1r
emu_can_tec, emu_can_rec, emu_can_lec
emu_can_bus_off, emu_can_error_passive, emu_can_error_warning
emu_can_hal_error, emu_can_hal_state

// TX detail
emu_tx_attempt_count, emu_tx_hal_ok_count, emu_tx_hal_busy_count,
emu_tx_hal_error_count, emu_tx_complete_count

// RX detail
emu_rx_frame_count, emu_bad_dlc_count, emu_unexpected_id_count

// Alive
emu_main_loop_alive
```

Same polling-based approach as the VCU (see that README Section 10) - no
additional NVIC lines beyond the proven `CAN1_RX1_IRQn`.

---

## 11. Wiring

See `EZkontrol_VCU` README Section 11 (identical - CAN_H/CAN_L/GND between
transceivers, 120-ohm termination at each physical end, ~60 ohm CAN_H-to-
CAN_L with both ends terminated and power off).

---

## 12. Build

```
STM32CubeIDE -> File -> Open Projects from File System -> select this folder
Project -> Build Project
```

Confirmed during repository creation: clean build with `arm-none-eabi-gcc`
(13.2.1), **0 errors, 0 warnings** (`-Wall -Wextra`), at both `-O0` and
`-O2 --gc-sections`. Every `volatile` global declared in
`ezkontrol_mcu_emulator.c` was individually verified present in the final
linked ELF via `arm-none-eabi-nm` (62/62 symbols).

---

## 13. Test procedure

See `EZkontrol_VCU` README Section 13 for the full end-to-end procedure
and test table (handshake, HALT, forward/reverse ramp, temperature
injection, fault injection, disconnect/reconnect) - it exercises both
repositories together and is written from the combined perspective.

---

## 14. Project structure

```
Core/Inc/
    main.h                        CubeMX-generated
    stm32f1xx_hal_conf.h          CubeMX-generated
    stm32f1xx_it.h                CubeMX-generated
    ezkontrol_protocol.h          shared addressing/ID/encoding (this + VCU repo)
    ezkontrol_mcu_emulator.h      public API: EZK_MCU_Emulator_Init() / _Task()
Core/Src/
    main.c                        CubeMX-generated + USER CODE hooks only
    stm32f1xx_it.c                CubeMX-generated (CAN1_RX1_IRQHandler)
    stm32f1xx_hal_msp.c           CubeMX-generated (PA11/PA12, PB0/1/10/11, NVIC)
    ezkontrol_mcu_emulator.c      all emulator protocol + motor-model logic
Drivers/                          STM32 HAL + CMSIS (CubeMX-generated, unmodified)
EZkontrol_MCU_Emulator.ioc        reopen in CubeMX to regenerate
STM32F103C8TX_FLASH.ld            linker script (unmodified)
```

There is exactly **one** implementation of
`HAL_CAN_RxFifo1MsgPendingCallback()` in this project (in
`ezkontrol_mcu_emulator.c`). `main.c` contains only CubeMX-generated code
plus two `USER CODE` calls: `EZK_MCU_Emulator_Init()` after
`MX_CAN_Init()`, and `EZK_MCU_Emulator_Task()` inside `while(1)`.
