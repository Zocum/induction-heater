#ifndef IH_PLL_H
#define IH_PLL_H

#include "config.h"

// ===================================================================
// pll: ZVS phase-locked-loop control. Acquisition, proportional phase
// tracker, resonance-crossing bailout, non-blocking backoff ramps.
// All of this runs in the MAIN LOOP (TIM2 is polled) - which is exactly
// why the safety supervisor in safety.cpp exists.
// ===================================================================

extern volatile bool phase_locked;
extern uint32_t      locked_freq;
extern uint32_t      start_frequency;

// Live telemetry for display / serial
extern float cur_phase;      // last valid measured phase (deg)
extern bool  phase_valid;
extern float pll_disp_err;   // last control error (deg)
extern bool  pll_in_db;      // parked inside deadband => "LOCKED"

// Backoff ramp state (exposed so main can cancel on switch events)
extern uint32_t freq_ramp_target;          // 0 = idle
extern bool     pwm_disable_after_ramp;

void pll_reset_state(void);

// Lock engagement: seed ACQ_OFFSET above resonance estimate, walk down.
void pll_engage(void);

// Manual unlock (lock switch off): backoff ramp, timer cancelled by caller.
void pll_release_manual(void);

// Soft stop (PWM switch off while locked): ramp up, THEN cut gates.
void pll_softstop(void);

// Run-timer expiry: park TIMER_BACKOFF above resonance, PWM stays on.
void pll_timer_expired(void);

void pll_start_ramp(uint32_t to_hz);
void pll_cancel_ramp(void);
void pll_ramp_task(void);                  // call every loop pass

// Manual frequency nudge while unlocked (also syncs start_frequency).
void pll_manual_step(bool up);

// 200 Hz control tick: dead-time schedule + capture processing + tracker.
void pll_control_task(bool lock_switch);

#endif
