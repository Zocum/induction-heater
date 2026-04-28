# Induction Heater Controller
**STM32 Blue Pill (STM32F103C8T6) — Phase-Locked Resonance Tracking**

A firmware controller for a ZVS-style induction heater that automatically tracks the resonant frequency of the LC tank circuit using phase-locked loop (PLL) control. Runs on the STM32F103C8T6 ("Blue Pill") board using the Arduino STM32 framework.

---

## Features

- **Automatic resonance tracking** via PID-based phase-locked loop
- **Hardware input capture** — zero-software-latency phase measurement using TIM2 slaved to TIM1
- **Noise rejection** — ignores zero-crossing edges near PWM reload and compare events
- **DS18B20 temperature sensing** — raw 1-Wire bit-bang, no library dependency
- **SH1106 128x64 OLED display** — live frequency, phase, temperature, error, uptime, and lock status
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
| PA0 | Tank current zero-crossing via LM339 (TIM2 CH1 input capture) |
| PA1 | DS18B20 temperature sensor (1-Wire) |
| PB6 | I2C1 SCL -> OLED |
| PB7 | I2C1 SDA -> OLED |
| PB12 | Lock switch (active HIGH) |
| PB14 | Frequency UP button (active HIGH) |
| PB15 | Frequency DOWN button (active HIGH) |

### Additional Components
- **LM339 comparator** on CT output for clean zero-crossing detection
- **4.7 kOhm pull-up** resistor on PA1 to 3.3 V (DS18B20)
- **SH1106 128x64 OLED** on I2C address (default)
- Current transformer on tank circuit, burden resistor to GND

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

### Phase Measurement (Hardware Input Capture)
Phase is measured entirely in hardware with zero software latency:

- **TIM1** generates the PWM on PA8 and emits an Update trigger (TRGO) on each reload (every PWM rising edge).
- **TIM2** runs at 72 MHz, slaved to TIM1 via ITR0 in Reset Mode — `TIM2->CNT` is reset to 0 at every PWM rising edge.
- **TIM2 CH1** (PA0) is configured as input capture, rising edge. `TIM2->CCR1` is latched in hardware the instant PA0 goes high.

The captured value directly encodes the phase offset:

```
phase (degrees) = (TIM2->CCR1 / (TIM1->ARR + 1)) x 360
```

The only residual delay is the LM339 comparator propagation delay (~1.3-2 us), which is accounted for by the fixed `TARGET_PHASE` constant (default 22 degrees). The CC1IF flag is polled in the main loop at ~200 Hz rather than using interrupts.

### PLL / PID Control
The control loop runs at ~200 Hz. It has two operating modes:

**Tracking mode** (error > `LOCK_ENTER_TH` = 6 degrees):
- Full PID with gain scheduling — higher gain when far from resonance
- Maximum frequency correction clamped to prevent instability

**Locked mode** (error < `LOCK_ENTER_TH` for `DB_STABLE` = 15 consecutive cycles):
- Proportional-only soft correction (coefficient 0.08)
- Exits back to tracking if error exceeds `LOCK_EXIT_TH` = 12 degrees

A large sudden phase jump (> 120 degrees) is treated as a resonance crossing and forces an unlock + frequency reset.

### Noise Rejection
Zero-crossing edges that arrive within `1/NOISE_DIV` (5%) of either the PWM reload point or the compare event are discarded. If more than `REJECT_SWEEP_TH` = 15 consecutive edges are rejected, the frequency is swept upward by 15 Hz/tick to escape the dead zone.

---

## Frequency Range

| Parameter | Value |
|-----------|-------|
| Minimum frequency | 20 kHz |
| Maximum frequency | 150 kHz |
| Step size | 100 Hz |
| Default start | 50 kHz |
| Unlocked offset | +3400 Hz above start |

---

## Display Layout (OLED)

```
Induction Heater

L: 52.4kHz        <- L = locked, F = unlocked
T:  68C           <- DS18B20 temperature
P: 23.1 E: 0.3   <- live phase + error (locked)
Time 03:42
Rej: 0            <- rejected edge count (locked only)
** LOCKED **      <- or "Tracking..." / "Unlocked"
```

When unlocked, row 4 shows only the live phase reading (`P: 23.1`) without the error field.

---

## Calibration

The `TARGET_PHASE` constant represents the expected phase offset at resonance, which is primarily the LM339 comparator propagation delay. To calibrate:

1. Enable debug output (`v` via serial).
2. Manually sweep frequency using the UP/DOWN buttons until resonance is found.
3. Note the phase reading at true resonance.
4. Set `TARGET_PHASE` to that value.
5. Recompile and flash.

At startup, the target phase is printed to serial:
```
Target phase=22.0deg
```

---

## Serial Commands

| Command | Action |
|---------|--------|
| `f+` | Increase start frequency by one step |
| `f-` | Decrease start frequency by one step |
| `s` | Print current frequency, lock status, and target phase |
| `v` | Toggle verbose debug output (phase data every 100 ms) |

---

## Building & Flashing

This project targets the **Arduino STM32** (STM32duino) framework.

1. Install [STM32duino](https://github.com/stm32duino/Arduino_Core_STM32) board package in Arduino IDE.
2. Install **U8g2** library (for `U8x8lib.h`).
3. Select board: **Generic STM32F1 series -> BluePill F103C8**.
4. Upload method: STLink or serial bootloader.
5. Compile and flash.

---

## Safety Notes

> Induction heaters operate at high voltages and currents. The LC tank can develop dangerous voltages even from low supply rails. Ensure proper isolation, enclosure, and never operate without adequate knowledge of high-frequency power electronics.

- The firmware does not implement over-temperature shutdown by itself — add hardware protection.
- Always verify gate driver signals with an oscilloscope before connecting the full power stage.
- The lock switch enables closed-loop control; ensure the heater is in a safe state before enabling it.
