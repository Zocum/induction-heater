#ifndef IH_CONFIG_H
#define IH_CONFIG_H

#include <Arduino.h>

// ===================================================================
// Induction Heater Controller - central configuration
// Target: STM32F103CB "Blue Pill" (build as 128k part)
// ===================================================================

// ===== PINS =====
#define PWM_PIN      PA8    // TIM1_CH1
#define CH1N_PIN     PB13   // TIM1_CH1N
#define TANK_PIN     PA0    // TIM2_CH1 input capture (LM339 zero-cross)
#define LOCK_PIN     PB12   // lock switch (active HIGH). NOTE: also TIM1_BKIN.
#define BTN_UP       PB14
#define BTN_DOWN     PB15
#define DS_PIN       PA1    // DS18B20
#define RELAY_PIN    PB0
#define PWM_EN_PIN   PB1    // PWM enable switch, active HIGH
#define POT_PIN      PA2    // run-timer pot wiper
#define OLED_SCL_PIN PB6    // I2C1 SCL (twist together with GND to reduce EMI)
#define OLED_SDA_PIN PB7    // I2C1 SDA (twist together with VCC to reduce EMI)

// GDT input clamps: external transistors that force both MIC4421/4422 inputs
// low (driver outputs 0 V, GDT primaries parked, blocking caps discharging)
// whenever GDT_EN is LOW / GDT_NEN is HIGH. That is the safe state: default
// at boot and after every PWM shutdown. GDT_NEN is always driven as the
// complement of GDT_EN. External pulls (10k down on EN, 10k up on /EN) hold
// the safe state while the MCU pins float during reset; firmware only has to
// init the pins to the safe state and sequence the transitions.
// PB8/PB9: TIM4_CH3/CH4 unused (safety supervisor uses only the update IRQ),
// I2C1 is NOT remapped (runs on PB6/PB7), CAN unused. 5V-tolerant.
#define GDT_EN_PIN   PB8    // HIGH = clamps released, PWM passes to drivers
#define GDT_NEN_PIN  PB9    // inverted enable, complement of GDT_EN

// ===== RELAY POLARITY =====
#define RELAY_ACTIVE_LOW 1

// ===== FREQUENCY =====
#define TCLK_HZ         72000000UL
#define FREQ_STEP       100UL
#define MIN_FREQ        1000UL
#define MAX_FREQ        100000UL
#define START_FREQUENCY 43000UL
#define UNLOCK_OFFSET   4000UL
#define ACQ_OFFSET      6000UL   // acquisition seed: resonance est. + this, walk DOWN
#define UNLOCK_BACKOFF  7000UL
#define TIMER_BACKOFF   7000UL
#define RESTART_LOCKOUT_MS  3000u  // min off-time before re-arm (~6x bleeder tau)

// ===== SOFT STOP / RAMP =====
#define PWM_SOFTSTOP_OFFSET  7000UL   // Hz above resonance before cutting drive
#define FREQ_RAMP_HZ_PER_MS  7.0f     // ~7 kHz in 1 s

// ===== DEAD TIME (FF200R12KS4 schedule) =====
#define DT_FLOOR_NS          800u     // shoot-through floor - NEVER crossed
#define DT_MAX_NS            1500u
#define DT_SLOPE_NS_PER_KHZ  100.0f
#define DT_SLEW_NS           40u
#define DT_SCHEDULE_ENABLE   1
#define DT_TUNE_NS           800u

// ===== PHASE =====
#define TARGET_PHASE_DEG  (-122.0f)   // retune per hardware
#define SIG_DELAY_NS      1560u
#define F_TUNE_HZ         40500u
#define PHASE_JUMP_TH     120.0f
#define PH_N              15          // phase history depth

// ===== CONTROL LOOP =====
#define LOCK_K            5.0f        // Hz per degree
#define MAX_STEP          50.0f       // Hz per update
#define LOCK_DEADBAND     2.0f        // deg
#define FREQ_UPD_INT_MS   20UL
#define REJECT_SWEEP_TH   15
#define NOISE_DIV         20UL

// ===== RUN TIMER =====
#define TIMER_MAX_S       300UL
#define TIMER_MIN_S       10UL
#define TIMER_OFF_FRAC    0.02f
#define POT_RAW_LO        30
#define POT_RAW_HI        4100

// ===== UI =====
#define DEBOUNCE_MS       200UL
#define DISPLAY_PERIOD_MS 250UL

// ===== SAFETY (new) =====
// Main-loop heartbeat supervisor (runs in TIM4 ISR at 1 kHz):
#define SAFETY_STALL_MS      200u   // loop silent this long + PWM on -> takeover
#define SAFETY_RAMP_HZ_TICK  10u    // Hz per 1 ms tick during ISR detune (~700 ms)
#define SAFETY_SETTLE_MS     5u     // dwell at detuned freq before MOE cut
// Independent watchdog (LSI ~40 kHz):
#define IWDG_TIMEOUT_MS      800u   // hardware reset backstop
// I2C guard:
#define I2C_RECOVER_MIN_MS   500u   // min spacing between bus recovery attempts
#define I2C_CLOCK_HZ         100000u

#endif
