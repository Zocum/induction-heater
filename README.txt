# Induction Heater Controller
**STM32 Blue Pill (STM32F103C8T6) — Phase-Locked Resonance Tracking**

A firmware controller for a ZVS-style induction heater that automatically tracks the resonant frequency of the LC tank circuit using phase-locked loop (PLL) control. Runs on the STM32F103C8T6 ("Blue Pill") board using the Arduino STM32 framework.

---

## Features

- **Automatic resonance tracking** via PID-based phase-locked loop
- **CT phase compensation** — corrects for current transformer phase offset across frequency using a calibrated time constant (τ)
- **Noise rejection** — ignores zero-crossing edges near PWM reload and compare events
- **DS18B20 temperature sensing** — raw 1-Wire bit-bang, no library dependency
- **SH1106 128×64 OLED display** — live frequency, temperature, error, uptime, and lock status
- **Physical controls** — lock switch, frequency UP/DOWN buttons with hold-to-repeat acceleration
- **Flash-optimised** — direct TIM1 register access, no HardwareTimer abstraction

---

## Hardware

### Microcontroller
- STM32F103C8T6 ("Blue Pill"), 72 MHz

### Pin Map

| Pin | Function |
|-----|----------|
| PA8 | PWM output → gate driver (TIM1 CH1) |
| PA0 | Tank current zero-crossing (interrupt) |
| PA1 | DS18B20 temperature sensor (1-Wire) |
| PB6 | I2C1 SCL → OLED |
| PB7 | I2C1 SDA → OLED |
| PB12 | Lock switch (active HIGH) |
| PB14 | Frequency UP button (active HIGH) |
| PB15 | Frequency DOWN button (active HIGH) |

### Additional Components
- **4.7 kΩ pull-up** resistor on PA1 to 3.3 V (DS18B20)
- **SH1106 128×64 OLED** on I2C address (default)
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

### Phase Measurement
The tank current zero-crossing triggers an interrupt on PA0 (RISING edge). Inside the ISR, `TIM1->CNT` is sampled. Since TIM1 also drives the PWM, the counter value directly encodes the phase offset between the voltage drive and the tank current:

```
phase (degrees) = (CNT / (ARR + 1)) × 360°
```

### CT Phase Compensation
A current transformer introduces a frequency-dependent phase lag:

```
φ_CT(f) = arctan(2π·f·τ)
```

where `τ = L_CT / R_burden`. The time constant is derived from a single calibration point (`CAL_FREQ`, `CAL_PHASE`) at startup. The target phase is automatically recalculated at every PID tick as the operating frequency changes.

**To calibrate:** set `CAL_FREQ` to a frequency where resonance locking works reliably, and `CAL_PHASE` to the phase reading observed at true resonance at that frequency.

### PLL / PID Control
The control loop runs at ~200 Hz. It has two operating modes:

**Tracking mode** (error > `LOCK_ENTER_TH` = 6°):
- Full PID with gain scheduling — higher gain when far from resonance
- Maximum frequency correction clamped to prevent instability

**Locked mode** (error < `LOCK_ENTER_TH` for `DB_STABLE` = 15 consecutive cycles):
- Proportional-only soft correction (coefficient 0.08)
- Exits back to tracking if error exceeds `LOCK_EXIT_TH` = 12°

A large sudden phase jump (> 120°) is treated as a resonance crossing and forces an unlock + frequency reset.

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
                
L: 52.4kHz        ← L = locked, F = unlocked
T:  68C           ← DS18B20 temperature
E: +1.3deg        ← phase error (locked only)
Time 03:42
Rej: 0            ← rejected edge count
** LOCKED **      ← or "Tracking..." / "Unlocked"
```

---

## Calibration

1. Set `CAL_FREQ` to a frequency where the heater reliably phase-locks.
2. Enable debug output (`v` via serial) and observe the reported phase at resonance.
3. Set `CAL_PHASE` to that value.
4. Recompile and flash.

The computed τ is printed to serial at startup:
```
CT tau=487.3us
```

---

## Building & Flashing

This project targets the **Arduino STM32** (STM32duino) framework.

1. Install [STM32duino](https://github.com/stm32duino/Arduino_Core_STM32) board package in Arduino IDE.
2. Install **U8g2** library (for `U8x8lib.h`).
3. Select board: **Generic STM32F1 series → BluePill F103C8**.
4. Upload method: STLink or serial bootloader.
5. Compile and flash.

---

## Safety Notes

> ⚠️ Induction heaters operate at high voltages and currents. The LC tank can develop dangerous voltages even from low supply rails. Ensure proper isolation, enclosure, and never operate without adequate knowledge of high-frequency power electronics.

- The firmware does not implement over-temperature shutdown by itself — add hardware protection.
- Always verify gate driver signals with an oscilloscope before connecting the full power stage.
- The lock switch enables closed-loop control; ensure the heater is in a safe state before enabling it.