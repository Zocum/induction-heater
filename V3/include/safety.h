#ifndef IH_SAFETY_H
#define IH_SAFETY_H

#include "config.h"

// ===================================================================
// safety: the three-layer protection discussed for the I2C freeze issue.
//
// Layer 1 - ISR dead-man ("detune on freeze"):
//   The whole PLL runs in loop(); if the loop blocks (stuck I2C, anything),
//   the bridge would keep switching at full power unsupervised. A TIM4
//   interrupt at 1 kHz watches a heartbeat that only the main loop feeds.
//   If the heartbeat goes stale for SAFETY_STALL_MS while PWM is enabled,
//   the ISR takes over: it ramps the frequency UP by SAFETY_RAMP_HZ_TICK
//   per ms to (freq at stall + PWM_SOFTSTOP_OFFSET) - same graceful detune
//   as the normal soft stop, ZVS preserved, no LC ring on the 50mH/420uF
//   input filter - then cuts MOE and latches a FAULT.
//
// Layer 2 - IWDG hardware watchdog:
//   Clocked from LSI, survives anything short of power loss. Kicked ONLY
//   from the main loop... with one deliberate exception: while the Layer-1
//   takeover is actively ramping, the ISR keeps the dog fed so the reset
//   cannot fire mid-detune. The moment MOE is cut, feeding stops; if the
//   loop is truly dead the MCU resets ~IWDG_TIMEOUT_MS later into safe boot.
//
// Layer 3 lives in oled.cpp (I2C timeouts + bus recovery) and makes
// layers 1-2 rarely needed.
//
// Boot: if the previous reset was an IWDG reset, safe boot is flagged -
// PWM arming is inhibited until the PWM enable switch is seen LOW once.
// ===================================================================

typedef enum { SAFETY_OK = 0, SAFETY_TAKEOVER, SAFETY_FAULT } safety_state_t;

void safety_init(void);                 // reset-cause check, TIM4, IWDG start
void safety_heartbeat(void);            // call once per loop() pass
void safety_iwdg_kick(void);            // call once per loop() pass
safety_state_t safety_state(void);
bool safety_wdt_boot(void);             // true if we booted from an IWDG reset
void safety_clear_fault(void);          // only honored with PWM switch safe/low

#endif
