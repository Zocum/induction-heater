#ifndef IH_BRIDGE_H
#define IH_BRIDGE_H

#include "config.h"

// ===================================================================
// bridge: everything that touches TIM1 (PWM + dead time + MOE),
// TIM2 (input capture) and the relay. Register-level, no HAL.
//
// TIM1: CH1 (PA8) / CH1N (PB13) complementary pair, hardware dead time
//       via BDTR DTG. Update event -> TRGO -> TIM2 ITR0.
// TIM2: 72 MHz, reset-mode slave of TIM1, CH1 (PA0) rising-edge capture.
//       CCR1 = ticks from PWM rising edge to tank current edge.
// ===================================================================

extern volatile bool     pwm_enabled;
extern uint32_t          cur_freq;
extern volatile uint32_t dead_time_ns;
extern float             phase_target_eff;   // TARGET_PHASE + DT/sig-delay comp

void bridge_init(uint32_t freq_hz);   // TIM1 + GPIO, MOE off
void capture_init(void);              // TIM2 input capture

// Set bridge frequency. Clamped to [MIN_FREQ..MAX_FREQ]. Safe to call from
// the main loop AND from the safety supervisor ISR (PRIMASK save/restore
// critical section - does NOT blindly re-enable interrupts like the old
// noInterrupts()/interrupts() pair did).
void bridge_set_freq(uint32_t f);

void bridge_pwm_enable(void);   // MOE on  + relay on
void bridge_pwm_disable(void);  // MOE off + relay off (outputs driven LOW, OSSI)

// GDT input clamps (see config.h). Both pins always driven complementary;
// both transitions pass through the safe EN=LOW/NEN=LOW intermediate:
// enable releases the clamp LAST, disable engages it FIRST. ISR-safe
// (called from the TIM4 supervisor via bridge_pwm_disable).
void gdt_enable(void);          // release clamps: NEN low, then EN high
void gdt_disable(void);         // engage clamps:  EN low first, then NEN high
bool gdt_is_enabled(void);      // for status/telemetry

// Dead-time schedule + effective phase target refresh.
// f_res = current resonance reference (locked_freq if valid, else base).
void bridge_update_dt_schedule(uint32_t f_res);

bool     bridge_restart_ready(void);        // false while lockout running
uint32_t bridge_lockout_remaining_ms(void); // for display/serial

#endif
