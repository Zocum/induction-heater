# OLED / I2C hardware hardening checklist

Goal: cut the rate of corrupted I2C transactions so the firmware's recovery
layer (oled.cpp) almost never has to act. Ordered by payoff.

## At the STM32 end

1. **Stiffen the pull-ups.** Replace or parallel the module's typical 10k down
   to **2.2k (or 1.5k) to 3.3 V**, placed at the STM32 side of the cable. Noise
   margin on an open-drain bus is set by how hard the line is held high. At
   100 kHz (firmware now forces this clock) 1.5k is comfortably in spec.
2. **33-100 ohm series resistor** in BOTH SDA and SCL, right at the MCU pins.
   Damps ringing and limits transient current injected into PB6/PB7.

## At the OLED end

3. **47-100 pF to GND** on each line at the display module. With the series R
   this forms an RC that rounds off coupled spikes. Do NOT go larger - I2C
   edges must stay clean (400 pF total bus budget).
4. **Ferrite bead in series with the OLED VCC**, plus **100 nF + 4.7 uF**
   directly at the module. Many "I2C glitches" are really supply dips on the
   display coinciding with an EMI event.

## The cable

5. **Keep it under ~10 cm** if at all possible.
6. **Twist SDA with GND, and SCL with VCC** (two twisted pairs). Never twist
   SDA with SCL - that maximizes crosstalk and manufactures phantom
   start/stop conditions.
7. If the run must be longer: **shielded cable**, shield bonded to STM32
   ground with a short fat connection (no long pigtail). Route it away from
   the tank leads and GDT wiring; cross power runs at 90 degrees.

## Structural options (if it still misbehaves)

8. **Switch the SH1106 module to 4-wire SPI** if it exposes the mode strap
   (many do via a jumper resistor). Push-pull lines, no bidirectional wire,
   no bus-hang state: a corrupted byte becomes one garbage frame instead of a
   wedge. This is the real cure; the firmware's oled.cpp would then shrink to
   a plain driver.
9. Alternatively an **I2C buffer (P82B96 / PCA9600)** at the MCU end for
   strong drive across the cable.

# Watchdog-related hardware (required for the new firmware)

10. **10k pull-downs on the MIC4421/4422 inputs** (both drivers). During an
    IWDG reset the MCU pins float for a few ms; the pull-downs guarantee the
    gate drivers read a hard OFF instead of noise. With OSSI the outputs are
    driven low whenever the timer is alive, but the reset window itself needs
    the resistors.

# Bench validation procedure (tank UNPOWERED, gate drive scoped)

1. Flash, open serial monitor.
2. Enable PWM at low bus voltage / no bus.
3. Send `x` (deliberate loop freeze). Expected sequence on the scope/serial:
   - ~200 ms: supervisor takeover, frequency ramps up ~7 kHz over ~700 ms
   - MOE cut (gate drive dies cleanly at the detuned frequency)
   - ~0.8 s later: IWDG reset, banner shows "!WDT RESET - safe boot"
   - Display row 7 shows "WDT rst - sw off"; PWM refuses to arm until the
     enable switch has been cycled through OFF.
4. Unplug SDA mid-run: display freezes but buttons/frequency control stay
   live; `s` shows i2cskip incrementing. Replug: i2crec increments once and
   the display comes back within ~500 ms. No freeze, no fault.
