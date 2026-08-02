# Induction Heater Controller
**STM32 Blue Pill (STM32F103C8T6) — Phase-Locked Resonance Tracking**

> ### ⚠️ Use V3 instead — see `V3/README.md`
>
> **V3 is the recommended version for any new build.** This document describes
> the older single-sketch firmware in `V1/`, kept for reference and for
> existing hardware.
>
> The reason to move is safety, not features. In this version the entire PLL
> runs in `loop()` while TIM1 generates the gate drive in hardware — so if
> `loop()` ever blocks, **the bridge keeps switching unsupervised**: no
> resonance tracking, the run timer never expires, and the PWM enable and lock
> switches stop being polled, so the front panel goes dead while the tank is
> still being driven.
>
> V3 closes that hole with a TIM4 dead-man supervisor that gracefully detunes
> and cuts the gate drive within ~200 ms, an IWDG watchdog behind it, and
> safe-boot arming inhibit after a reset. It also adds external gate-drive
> clamps and an EMI glitch filter on all digital inputs.
>
> V3 needs two hardware provisions this version does not (10 kΩ pull-downs on
> the gate driver inputs, and the GDT clamp transistors on PB8/PB9). The
> migration steps are in `V3/README.md`.
>
> If you stay on this version, at minimum keep the run timer short and never
> leave the heater unattended.

A firmware controller for a ZVS-style induction heater that automatically tracks the resonant frequency of the LC tank circuit using phase-locked loop (PLL) control. Runs on the STM32F103C8T6 ("Blue Pill") board using the Arduino STM32 framework.

---

## Features

- **Automatic resonance tracking** via a proportional phase-locked loop
- **Complementary gate drive** — TIM1 CH1/CH1N pair with hardware dead time (BDTR DTG)
- **Scheduled dead time** — floors at the shoot-through minimum at resonance, ramps up as you operate above it
- **Hardware input capture** — zero-software-latency phase measurement using TIM2 slaved to TIM1
- **Phase-target compensation** — dead time and signal-path delay are converted to degrees at the live frequency, so one `TARGET_PHASE` holds across coil swaps
- **Noise rejection** — ignores zero-crossing edges near PWM reload and compare events
- **Graceful exits** — soft stop, timer expiry and manual unlock all detune off resonance first, then cut the gates
- **PWM enable switch** — gate drive stays off at power-up until deliberately armed
- **Run timer on a pot** — 0-5 min, followed live, adjustable mid-run
- **Relay output** tied to the PWM state
- **DS18B20 temperature sensing** — raw 1-Wire bit-bang, no library dependency
- **SH1106 128x64 OLED display** — live frequency, phase, temperature, error, uptime, timer, and lock status
- **I2C bus guard** — health check, frame skip and bus recovery so a wedged display cannot stall the control loop
- **Physical controls** — lock switch, frequency UP/DOWN buttons with hold-to-repeat acceleration
- **Flash-optimised** — direct TIM1/TIM2 register access, no HardwareTimer abstraction

---

## Hardware

### Microcontroller
- STM32F103C8T6 ("Blue Pill"), 72 MHz

### Pin Map

| Pin | Function |
|-----|----------|
| PA8 | PWM output -> gate driver (TIM1 CH1) |
| PB13 | Complementary PWM output -> gate driver (TIM1 CH1N) |
| PA0 | Tank current zero-crossing via LM339 (TIM2 CH1 input capture) |
| PA1 | DS18B20 temperature sensor (1-Wire) |
| PA2 | Run-timer setpoint pot wiper (analog) |
| PB0 | Relay driver (follows PWM state) |
| PB1 | PWM enable switch (active HIGH) |
| PB6 | I2C1 SCL -> OLED |
| PB7 | I2C1 SDA -> OLED |
| PB12 | Lock switch (active HIGH) |
| PB14 | Frequency UP button (active HIGH) |
| PB15 | Frequency DOWN button (active HIGH) |

> **PB12 conflict:** PB12 is also TIM1_BKIN. If you wire a hardware overcurrent
> trip into the break input, move the lock switch to a free pin (PB5/PB8/PB9) first.

### Additional Components
- **LM339 comparator** on CT output for clean zero-crossing detection
- **4.7 kOhm pull-up** resistor on PA1 to 3.3 V (DS18B20)
- **SH1106 128x64 OLED** on I2C address (default)
- Current transformer on tank circuit, burden resistor to GND
- **Two GDTs**, primaries driven differentially from the CH1/CH1N pair. The second is wired reversed so leg B runs 180 degrees anti-phase off the same pair and inherits the same dead band — one pair protects both half-bridges.
- **Relay module** on PB0. Set `RELAY_ACTIVE_LOW` to match: 1 for a typical opto-isolated board (LOW = energized), 0 for a low-side transistor driver.
- **Potentiometer** to 3.3 V / GND, wiper to PA2

---

## Dependencies

| Library | Purpose |
|---------|---------|
| `Wire` | I2C for OLED |
| `U8x8lib` (`U8g2`) | OLED driver (SH1106, HW I2C) |

No OneWire or DallasTemperature library required — DS18B20 is bit-banged manually.

---

## How It Works

### Frequency Generation
TIM1 is configured in PWM mode directly via registers, running at 72 MHz with no prescaler. The output frequency is:

```
f = 72,000,000 / (ARR + 1)
```

CCR1 is always set to half of ARR+1, giving a 50% duty cycle. Frequency is updated live by writing new ARR/CCR1 values under a brief interrupt lock.

Output is a **complementary pair** — CH1 (PA8) and CH1N (PB13) — with dead time inserted in hardware by the BDTR DTG field. During the dead-time window both pins go LOW, the GDT primary sits at 0 V, and that leg's gates are off.

### Dead Time
Dead time is scheduled rather than fixed. It floors at `DT_FLOOR_NS` (800 ns) at resonance and under lock, and ramps toward `DT_MAX_NS` (1500 ns) at `DT_SLOPE_NS_PER_KHZ` (100 ns/kHz) as the drive moves above resonance, slew-limited to `DT_SLEW_NS` (40 ns) per scheduler tick. The floor is the **shoot-through minimum and is never crossed**.

The default 800 ns floor is derived for an FF200R12KS4 at Tvj = 125 C, 200 A, +/-15 V, Rg = 4.7 ohm: td(off) + tf - td(on) = 480 ns delay race, plus tail current and GDT/driver skew and margin. **Verify on your own hardware** — scope gate-off until collector current (tail included) reaches zero, hot, at your actual Rg.

Set `DT_SCHEDULE_ENABLE` to 0 for a fixed `DT_FLOOR_NS` with no schedule.

### Phase Measurement (Hardware Input Capture)
Phase is measured entirely in hardware with zero software latency:

- **TIM1** generates the PWM on PA8 and emits an Update trigger (TRGO) on each reload (every PWM rising edge).
- **TIM2** runs at 72 MHz, slaved to TIM1 via ITR0 in Reset Mode — `TIM2->CNT` is reset to 0 at every PWM rising edge.
- **TIM2 CH1** (PA0) is configured as input capture, rising edge. `TIM2->CCR1` is latched in hardware the instant PA0 goes high.

The captured value directly encodes the phase offset:

```
phase (degrees) = (TIM2->CCR1 / (TIM1->ARR + 1)) x 360
```

Software/ISR latency is effectively zero. The residual delays are the LM339 comparator propagation delay plus CT conditioning (together `SIG_DELAY_NS`, default 1560 ns) and the gate dead time, which offsets the real CH1 edge from the TRGO reference the phase is measured against. The CC1IF flag is polled in the main loop at ~200 Hz rather than using interrupts.

### Phase Target Compensation
Both residual delays are **fixed-time**, so their contribution in degrees grows with frequency. The firmware converts them at the live frequency and references them to the point at which `TARGET_PHASE` was tuned:

```
phase_target_eff = TARGET_PHASE
                 + dtPhaseDeg(dead_time_ns + SIG_DELAY_NS, cur_freq)
                 - dtPhaseDeg(DT_TUNE_NS   + SIG_DELAY_NS, F_TUNE_HZ)
```

This is what lets a single `TARGET_PHASE` hold the same *actual* tank phase across coil swaps and load pull, and keeps the lock point from drifting as the dead-time schedule moves. `DT_TUNE_NS` (800 ns) and `F_TUNE_HZ` (40500 Hz) must describe the conditions you tuned `TARGET_PHASE` under.

### PLL Control
The control loop runs at ~200 Hz. It is **proportional-only** — frequency is the time-integral of phase, so a single gain on the phase error already drives the steady-state error to zero, with no I or D term needed.

| Constant | Default | Role |
|----------|---------|------|
| `LOCK_K` | 5.0 | Hz of correction per degree of phase error |
| `MAX_STEP` | 50.0 | Max frequency change per update (Hz) — stops a stray sample yanking the drive |
| `LOCK_DEADBAND` | 2.0 | Degrees; inside this the frequency is held, killing idle dither |
| `FREQ_UPD_INT` | 20 | ms between corrections (captures arrive ~4x faster) |
| `PH_N` | 15 | Depth of the moving-average phase filter |

The phase-vs-frequency slope is steepest near resonance, so the closer `TARGET_PHASE` is tuned toward Fres, the gentler `LOCK_K` has to be — too hot and it limit-cycles around target.

**Acquisition** always begins `ACQ_OFFSET` (12 kHz) **above** the resonance estimate and walks down, so the loop always approaches from the stable, inductive side. Seeding from wherever the knob is parked is what let earlier versions run away. `locked_freq` is only trusted once the loop is genuinely parked inside the deadband.

A filtered phase jump greater than `PHASE_JUMP_TH` (120 degrees) means the drive fell through resonance to the capacitive side. This forces an unlock and a jump back above resonance, reported as `!JUMP` on serial.

### Noise Rejection
Zero-crossing edges arriving within `1/NOISE_DIV` (1/20 = 5%) of either the PWM reload point or the compare event are discarded. If more than `REJECT_SWEEP_TH` (15) consecutive edges are rejected, the frequency is swept upward at 15 Hz/tick to escape the dead zone.

### I2C Bus Guard
Every OLED write is a blocking transfer, and the PLL, the run timer and the switch polling all live in the same `loop()`. A display wedged by an EMI burst would therefore stall the control loop while TIM1 kept the bridge switching. Before each frame the firmware checks both I2C lines are idle-high; if not it skips the frame and, at most every 500 ms, recovers the bus (9 SCL pulses, manual STOP, controller re-init). The `s` command reports `i2crec` / `i2cskip` so a marginal bus is visible before it becomes a failure.

The bus runs at 100 kHz rather than the 400 kHz default for noise margin. See `HARDWARE.md` in the V3 project for the hardware-side hardening checklist.

---

## Frequency Range

| Parameter | Constant | Value |
|-----------|----------|-------|
| Minimum frequency | `MIN_FREQ` | 1 kHz |
| Maximum frequency | `MAX_FREQ` | 100 kHz |
| Step size | `FREQ_STEP` | 100 Hz |
| Default start | `start_frequency` | 46 kHz |
| Unlocked offset | `UNLOCK_OFFSET` | +4000 Hz above start |
| Acquisition offset | `ACQ_OFFSET` | +12000 Hz above resonance estimate |
| Unlock / timer backoff | `UNLOCK_BACKOFF` / `TIMER_BACKOFF` | 7000 Hz |
| Soft-stop backoff | `PWM_SOFTSTOP_OFFSET` | 7000 Hz |
| Backoff ramp rate | `FREQ_RAMP_HZ_PER_MS` | 7.0 Hz/ms (~7 kHz in 1 s) |

---

## Controls

Two switches gate the bridge, and both matter:

- **PWM enable (PB1)** — master. PWM does *not* run at power-up; the gate drivers are held at 0 V until this is flipped HIGH. This lets you power the bridge and charge the DC link first, then start switching. Turning it off while locked performs a soft stop: ramp `PWM_SOFTSTOP_OFFSET` up off resonance over ~1 s, *then* cut the gates. Not locked, the gates cut immediately.
- **Lock (PB12)** — engages closed-loop tracking and arms the run timer. With PWM on and lock off, the bridge free-runs at the current manual frequency.

The **pot (PA2)** sets the run time, 0-5 min (`TIMER_MAX_S`, with a `TIMER_MIN_S` = 10 s floor). It is followed live, so the duration can be changed mid-run — turn it up for more time, or down toward zero to end the run now. Below `TIMER_OFF_FRAC` (2%) the timer is disabled and lock runs indefinitely. On expiry the drive parks `TIMER_BACKOFF` above resonance and unlocks, but **PWM stays on and the relay stays energized**; re-lock to run again. Reported as `!TIMER` on serial.

Calibrate the pot ends with `POT_RAW_LO` / `POT_RAW_HI`: read the raw counts at each knob extreme from the `s` command (`pot=`), then set these slightly *inside* the measured values so 0 and full scale are easy to reach.

---

## Display Layout (OLED)

```
Induction Heater
Relay: ON         <- follows PWM state
L: 46.25kHz       <- L = locked, F = free/manual
T:  68C           <- DS18B20 temperature, "--" if absent
P: 23.1 E: 0.3    <- live phase + error (error shown when locked)
Time 03:42        <- uptime
Run -02:15        <- timer: "Run -mm:ss" / "Set mm:ss" / "Timer done" / "Timer off"
** LOCKED **      <- or "Tracking..." / "PWM ON" / "PWM OFF"
```

Row 4 shows the live phase whenever a valid capture exists, regardless of lock state, so you can watch phase change while sweeping manually; the `E:` error field appears only when locked. Row 7 shows `PWM OFF` whenever the enable switch is down, which takes precedence over the lock state.

---

## Calibration

Do this on the bench with the **tank unpowered** and the gate drive scoped. Dead time first — the phase target depends on it.

**1. Dead time.** Scope gate-off until collector current (tail current included) reaches zero, hot, at your actual Rg and Vge(off). Set `DT_FLOOR_NS` to that plus margin, and set `DT_TUNE_NS` to the dead time you are about to tune the phase target at. At startup the schedule is printed to serial:

```
DT floor=800ns max=1500ns tune=800ns DTGfloor=0x3A
```

**2. Phase target.** `TARGET_PHASE` is the measured phase at true resonance — mostly the LM339 propagation delay and CT conditioning, plus the dead-time offset.

1. Enable debug output (`v` via serial).
2. Manually sweep frequency using the UP/DOWN buttons until resonance is found.
3. Note the phase reading at true resonance.
4. Set `TARGET_PHASE` to that value.
5. Set `F_TUNE_HZ` to the frequency you tuned at, and `SIG_DELAY_NS` to your measured signal-path delay. **Measure this one** — the in-code comment describes a 250 ns two-coil fit (42 deg @ 77.5 kHz, 22 deg @ 24.5 kHz) but the constant actually ships at 1560 ns, so the comment and the value disagree.
6. Recompile and flash.

At startup, the target phase is printed to serial:
```
Target phase=-122.0deg
```

Retune after **any** change to the dead time. The `s` command reports both the raw target (`tgt=`) and the compensated one in use (`eff=`).

**3. Pot ends.** Read raw counts at both knob extremes from `s` (`pot=`), then set `POT_RAW_LO` / `POT_RAW_HI` just inside those values.

---

## Serial Commands

| Command | Action |
|---------|--------|
| `f+` | Increase start frequency by one step |
| `f-` | Decrease start frequency by one step |
| `s` | Status line: frequency, lock, `pwm=`, `pot=`, `set=`, `tgt=`, `eff=`, `dt=`, `i2crec=`, `i2cskip=` |
| `v` | Toggle verbose debug output (phase data every 100 ms) |

The firmware also emits unsolicited `!JUMP` (resonance crossing detected) and `!TIMER` (run timer expired).

---

## Building & Flashing

This project targets the **Arduino STM32** (STM32duino) framework.

1. Install [STM32duino](https://github.com/stm32duino/Arduino_Core_STM32) board package in Arduino IDE.
2. Install **U8g2** library (for `U8x8lib.h`).
3. Select board: **Generic STM32F1 series -> BluePill F103C8**.
4. Upload method: STLink or serial bootloader.
5. Compile and flash.

Builds to roughly 35 KB of flash and 1.8 KB of RAM, comfortably inside the C8's nominal 64 KB.

---

## Safety Notes

> Induction heaters operate at high voltages and currents. The LC tank can develop dangerous voltages even from low supply rails, and the DC link stays charged after power-down. Ensure proper isolation, enclosure, and never operate without adequate knowledge of high-frequency power electronics.

- **This version has no loop supervision.** The PLL runs in `loop()` while TIM1 generates gate drive in hardware, so any blocking fault in `loop()` leaves the bridge switching with the front panel unresponsive. The I2C guard closes the one known cause; it does not make the loop supervised. V3 adds a TIM4 dead-man and an IWDG watchdog — see `V3/README.md`.
- `DT_FLOOR_NS` is a **shoot-through floor**. Verify it against your own devices before applying bus voltage; the default is derived for one specific IGBT.
- The firmware does not implement over-temperature shutdown by itself — the DS18B20 is instrumentation only. Add hardware protection.
- Always verify gate driver signals, including the dead band, with an oscilloscope before connecting the full power stage.
- The PWM enable switch is the master arm and the lock switch enables closed-loop control; ensure the heater is in a safe state before flipping either.
- Add 10 kOhm pull-downs on the gate driver inputs so they read a hard OFF while the MCU pins float during reset.
