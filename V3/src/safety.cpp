#include "safety.h"
#include "bridge.h"

static HardwareTimer sup_timer(TIM4);

static volatile uint32_t       sup_tick   = 0;   // 1 kHz, owned by ISR
static volatile uint32_t       hb_tick    = 0;   // last loop heartbeat, in ticks
static volatile safety_state_t state      = SAFETY_OK;
static volatile uint32_t       takeover_target = 0;
static volatile uint32_t       settle_cnt = 0;
static bool                    wdt_boot   = false;

// ---- IWDG (register-level; LSI ~40 kHz) ----
static void iwdg_start(void) {
  // /64 prescaler -> 625 Hz. reload = timeout_ms * 0.625
  uint32_t rlr = (IWDG_TIMEOUT_MS * 625u) / 1000u;
  if (rlr > 0xFFF) rlr = 0xFFF;
  IWDG->KR  = 0x5555;      // unlock PR/RLR
  IWDG->PR  = 0x04;        // /64
  IWDG->RLR = rlr;
  IWDG->KR  = 0xAAAA;      // load
  IWDG->KR  = 0xCCCC;      // start - cannot be stopped except by reset
}

static inline void iwdg_feed(void) { IWDG->KR = 0xAAAA; }

// ---- 1 kHz supervisor ISR ----
static void supervisor_tick(void) {
  sup_tick++;

  switch (state) {

    case SAFETY_OK:
      if (pwm_enabled && (sup_tick - hb_tick) > SAFETY_STALL_MS) {
        // Main loop is dead while the bridge is switching. Take over.
        takeover_target = cur_freq + PWM_SOFTSTOP_OFFSET;
        if (takeover_target > MAX_FREQ) takeover_target = MAX_FREQ;
        settle_cnt = 0;
        state = SAFETY_TAKEOVER;
      }
      break;

    case SAFETY_TAKEOVER: {
      // Keep the hardware watchdog from resetting mid-detune (the one
      // sanctioned ISR feed - bounded to <1s by construction).
      iwdg_feed();
      if (cur_freq < takeover_target) {
        uint32_t nf = cur_freq + SAFETY_RAMP_HZ_TICK;
        if (nf > takeover_target) nf = takeover_target;
        bridge_set_freq(nf);
      } else if (++settle_cnt >= SAFETY_SETTLE_MS) {
        bridge_pwm_disable();          // MOE off, relay off
        state = SAFETY_FAULT;          // feeding stops here
        // If the loop never recovers, IWDG resets the MCU ~IWDG_TIMEOUT_MS
        // from now and we come back up in safe boot. If the loop DOES
        // recover, it resumes feeding, sees SAFETY_FAULT, and requires the
        // PWM switch to be cycled before re-arming.
      }
      break;
    }

    case SAFETY_FAULT:
    default:
      break;
  }
}

void safety_init(void) {
  // Reset-cause: did the watchdog put us here?
  if (RCC->CSR & RCC_CSR_IWDGRSTF) wdt_boot = true;
  RCC->CSR |= RCC_CSR_RMVF;          // clear reset flags for next time

  sup_timer.setOverflow(1000, HERTZ_FORMAT);
  sup_timer.attachInterrupt(supervisor_tick);
  sup_timer.resume();

  iwdg_start();
}

void safety_heartbeat(void) { hb_tick = sup_tick; }

void safety_iwdg_kick(void) {
  // Loop-side feed. During TAKEOVER the ISR feeds; after FAULT the loop
  // may feed again (keeps a recovered system alive with the fault latched).
  if (state != SAFETY_TAKEOVER) iwdg_feed();
}

safety_state_t safety_state(void) { return state; }
bool safety_wdt_boot(void)        { return wdt_boot; }

void safety_clear_fault(void) {
  if (state == SAFETY_FAULT && !pwm_enabled) {
    settle_cnt = 0;
    hb_tick = sup_tick;
    state = SAFETY_OK;
  }
}
