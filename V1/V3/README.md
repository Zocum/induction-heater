# Induction Heater Controller — V3

**STM32 Blue Pill (STM32F103C8T6) — phase-locked resonance tracking for a
full-bridge ZVS induction heater.**

V3 is a ground-up restructure of the single-sketch V1 firmware into a modular
PlatformIO project, built around one change of principle: **the PLL runs in
`loop()`, so `loop()` is a safety-critical path and is now supervised in
hardware.** Everything else follows from that.

> ⚠️ This firmware drives a mains-fed full bridge into a resonant tank. Read
> [Safety](#safety) before powering anything. The `TARGET_PHASE_DEG` and
> `DT_FLOOR_NS` constants **must** be retuned for your hardware — the shipped
> values are for one specific build.

---

## What's new since V1

| | V1 (`uC_logic.ino`) | V3 |
|---|---|---|
| Structure | one 1058-line `.ino` | 6 modules, `src/` + `include/`, central `config.h` |
| Build | Arduino IDE | PlatformIO (`bluepill_f103c8_128k`) |
| Loop freeze | **bridge keeps switching, unsupervised** | TIM4 dead-man → graceful detune → MOE cut → IWDG reset |
| Watchdog | none | IWDG, ~800 ms, with safe-boot arming inhibit |
| Gate drive | TIM1 MOE only | + external GDT input clamps (`GDT_EN`/`GDT_NEN`) |
| Digital inputs | raw `digitalRead()` | `stable_read()` EMI glitch filter on all four |
| I2C failure | blocks `loop()` | health check, frame skip, bounded bus recovery |
| Restart | immediate | `RESTART_LOCKOUT_MS` cooldown for bleeder discharge |

If you are running V1, see [MIGRATING](#migrating-from-v1) — the pin map grew
and V3 needs two new hardware provisions.

---

## Layout

```
platformio.ini      board, framework, U8g2 dependency
include/config.h    ALL tunables — pins, frequencies, gains, safety timings
src/main.cpp        module wiring + UI (buttons, switches, pot, serial)
src/bridge.cpp      TIM1 PWM/dead-time/MOE, TIM2 capture, relay, GDT clamps
src/pll.cpp         acquisition, phase tracker, backoff ramps
src/safety.cpp      TIM4 supervisor + IWDG
src/oled.cpp        SH1106 driver with I2C bus guard
src/ds18b20.cpp     bit-banged 1-Wire (no library)
HARDWARE.md         I2C/EMI hardening checklist + bench validation procedure
```

Tune the machine in `config.h`. You should rarely need to touch anything else.

---

## Hardware

### Pin map

| Pin | Function |
|-----|----------|
| PA8 | PWM out → gate driver (TIM1_CH1) |
| PB13 | Complementary PWM out (TIM1_CH1N) |
| PA0 | Tank current zero-crossing from LM339 (TIM2_CH1 capture) |
| PA1 | DS18B20 (4.7 kΩ pull-up to 3.3 V) |
| PA2 | Run-timer setpoint pot wiper (analog) |
| PB0 | Relay driver (follows PWM state) |
| PB1 | PWM enable switch (active HIGH) |
| PB12 | Lock switch (active HIGH) — *also TIM1_BKIN, see note* |
| PB14 / PB15 | Frequency UP / DOWN buttons |
| PB6 / PB7 | I2C1 SCL / SDA → SH1106 OLED |
| **PB8** | **GDT_EN — gate-drive clamp release (new in V3)** |
| **PB9** | **GDT_NEN — inverted clamp enable (new in V3)** |

> **PB12 conflict:** PB12 is TIM1_BKIN. If you ever wire a hardware overcurrent
> trip into the break input, move the lock switch to PB5/PB8/PB9 first.

### Required external provisions

1. **10 kΩ pull-downs on both MIC4421/4422 driver inputs.** During an IWDG
   reset the MCU pins float for a few ms. OSSI drives the outputs low whenever
   the timer is alive, but the reset window itself needs the resistors.
2. **GDT clamp transistors** on `GDT_EN`/`GDT_NEN`, with a 10 k pull-down on EN
   and 10 k pull-up on /EN so the safe (clamped) state holds through reset
   before firmware takes over.

`HARDWARE.md` covers the I2C/EMI hardening — pull-up stiffening, series
resistors, cable twisting — in priority order.

---

## How it works

### Frequency and dead time

TIM1 generates a complementary pair on PA8/PB13 at `f = 72 MHz / (ARR+1)`,
50 % duty, with **hardware dead time** from the BDTR DTG field. Both GDT
primaries are driven differentially; the second is wired reversed, so one pair
protects both half-bridges.

Dead time is *scheduled*, not fixed: it floors at `DT_FLOOR_NS` (800 ns — the
shoot-through minimum, never crossed) at resonance and ramps toward
`DT_MAX_NS` as you operate above resonance, at `DT_SLOPE_NS_PER_KHZ`. The
phase target is compensated for the live dead time and for the fixed
`SIG_DELAY_NS` signal-path delay, so **one** `TARGET_PHASE_DEG` holds the same
actual tank phase across coil swaps and load pull.

### Phase measurement

Zero software latency. TIM1's update event drives TRGO → TIM2 ITR0 in reset
mode, so `TIM2->CNT` clears on every PWM rising edge; TIM2_CH1 latches `CCR1`
the instant PA0 goes high:

```
phase (deg) = (TIM2->CCR1 / (TIM1->ARR + 1)) × 360
```

`CC1IF` is polled at ~200 Hz. Edges landing within `1/NOISE_DIV` of the reload
or compare event are discarded as noise.

### Control

Proportional-only. Frequency is the time-integral of phase, so a single gain
on the phase error already drives steady-state error to zero — no I or D term.
`LOCK_K` sets aggressiveness, `MAX_STEP` caps the per-update jump, and
`LOCK_DEADBAND` silences idle dither.

Acquisition always **starts `ACQ_OFFSET` above** the resonance estimate and
walks down, clearing the unstable near-resonance band. Overshoot the lock point
and the measured angle wraps, and the loop runs away to `MAX_FREQ`.

Every graceful exit — soft stop, timer expiry, manual unlock — uses a
non-blocking ramp at `FREQ_RAMP_HZ_PER_MS` to detune *first*, then cuts the
gates. This preserves ZVS on the way out and avoids ringing the input filter.

---

## Safety

Three layers, in the order they act:

**Layer 1 — TIM4 dead-man, ~200 ms.** `loop()` feeds a heartbeat. If it goes
stale for `SAFETY_STALL_MS` while PWM is enabled, a 1 kHz ISR takes over and
performs the *same* graceful detune as a normal soft stop — ramping up
`SAFETY_RAMP_HZ_TICK` per ms to `PWM_SOFTSTOP_OFFSET` above the stall
frequency, settling, then cutting MOE and latching `SAFETY_FAULT`.

**Layer 2 — IWDG, ~800 ms.** LSI-clocked, survives anything short of power
loss. Fed only from the main loop, with one deliberate exception: the Layer-1
ISR keeps feeding it *while ramping* so a reset can't fire mid-detune. Once MOE
is cut, feeding stops.

**Layer 3 — I2C bus guard.** `oled.cpp` checks both lines are idle-high before
every transaction, skips the frame if not, and attempts bounded bus recovery
(9 clock pulses + manual STOP + re-init). This is what stops a wedged display
from invoking layers 1–2 in the first place.

**After a watchdog reset** the firmware boots into a safe state and *refuses to
arm PWM* until the enable switch has been cycled through OFF. Row 7 shows
`WDT rst-sw off`.

A latched `SAFETY_FAULT` clears only by cycling the PWM switch low.

---

## Building

Needs [PlatformIO](https://platformio.org/). The U8g2 dependency is fetched
automatically.

```bash
pio run              # build
pio run -t upload    # flash via ST-Link
pio device monitor   # 115200 baud
```

Board is `bluepill_f103c8_128k` — the C8 is built as the 128 k CB part because
the firmware overflows the nominal C8 flash size. Serial is USART1
(PA9 TX / PA10 RX); uncomment the CDC block in `platformio.ini` for USB serial.

---

## Operating

Two switches gate the bridge, and both matter:

- **PWM enable (PB1)** — master. PWM does *not* run at power-up. Bring up the
  bus and charge the DC link first, then flip this. Turning it off performs a
  soft stop (detune, then gate cut).
- **Lock (PB12)** — engages closed-loop tracking and arms the run timer. With
  PWM on and lock off, the bridge runs open-loop at the manual frequency.

The **pot (PA2)** sets run time, 0–5 min, followed live. Below
`TIMER_OFF_FRAC` the timer is disabled and lock runs indefinitely. On expiry
the drive parks `TIMER_BACKOFF` above resonance with PWM still on.

### Display

```
Induction Heater
Relay: ON            follows PWM state
L: 43.25kHz          L = locked, F = free/manual
T:  68C              DS18B20, "--" if absent
P: 23.1 E: 0.3       live phase + error (error shown when locked)
Time 03:42           uptime
Run -02:15           timer: Run/Set/Timer done/Timer off
** LOCKED **         safety state dominates this row
```

### Serial commands (115200)

| Cmd | Action |
|---|---|
| `f+` / `f-` | Nudge frequency one `FREQ_STEP` |
| `s` | Status: freq, lock, pwm, gdt, pot, phase target, dead time, safety state, `i2crec`/`i2cskip` |
| `v` | Toggle verbose phase debug (100 ms) |
| `x` | **Deliberately freeze the loop** to validate the safety chain. Refused while PWM is enabled. |

---

## Calibration

Do this on the bench with the **tank unpowered** and the gate drive scoped.

1. **Dead time.** Scope gate-off to collector current (tail included) at your
   actual Rg and Vge(off), hot. Set `DT_FLOOR_NS` to that plus margin. The
   shipped 800 ns is for an FF200R12KS4 at 125 °C, 200 A, ±15 V, Rg = 4.7 Ω.
2. **Phase target.** Enable debug (`v`), sweep manually with the UP/DOWN
   buttons to find resonance, note the phase, and set `TARGET_PHASE_DEG`.
   Set `DT_TUNE_NS` to the dead time you tuned at.
3. **Pot ends.** Read raw counts at both knob extremes via `s` (`pot=`), then
   set `POT_RAW_LO`/`POT_RAW_HI` slightly *inside* those values.
4. **Validate the safety chain** — full procedure in `HARDWARE.md`.

---

## Migrating from V1

1. Add the two hardware provisions above (driver pull-downs, GDT clamps). The
   watchdog is not safe to run without them.
2. Carry your tuned constants across into `config.h`: `TARGET_PHASE_DEG`
   (V1 `TARGET_PHASE`), `DT_FLOOR_NS`, `DT_TUNE_NS`, `SIG_DELAY_NS`,
   `F_TUNE_HZ`, `POT_RAW_LO`/`HI`, `START_FREQUENCY`.
3. Note `ACQ_OFFSET` changed from 12000 to 6000 Hz, and `START_FREQUENCY` from
   46000 to 43000 — check both against your coil.
4. Flash, then run the bench validation in `HARDWARE.md` *before* connecting
   the power stage.

---

## Safety notes

Induction heaters operate at high voltages and currents. The LC tank develops
dangerous voltages even from low supply rails, and the DC link stays charged
after power-down.

- Always verify gate drive on a scope before connecting the power stage.
- The firmware has **no over-temperature shutdown**. The DS18B20 is
  instrumentation only — add hardware thermal protection.
- The supervisor protects against *firmware* stalls. It is not a substitute for
  a hardware overcurrent trip into TIM1_BKIN.
- Never operate without adequate knowledge of high-frequency power electronics.
