#include "bridge.h"

volatile bool     pwm_enabled      = false;
uint32_t          cur_freq         = 0;
volatile uint32_t dead_time_ns     = DT_FLOOR_NS;
float             phase_target_eff = TARGET_PHASE_DEG;

static uint8_t cur_dtg_byte = 0xFF;

// ---- relay ----
#if RELAY_ACTIVE_LOW
  #define RELAY_ON()  digitalWrite(RELAY_PIN, LOW)
  #define RELAY_OFF() digitalWrite(RELAY_PIN, HIGH)
#else
  #define RELAY_ON()  digitalWrite(RELAY_PIN, HIGH)
  #define RELAY_OFF() digitalWrite(RELAY_PIN, LOW)
#endif

// ---- GDT input clamps ----
// Single-byte flag, written with IRQs in whatever state the caller has them
// (gdt_disable runs inside the TIM4 supervisor ISR via bridge_pwm_disable).
static volatile bool gdt_en_state = false;

void gdt_enable(void) {
  digitalWrite(GDT_NEN_PIN, LOW);    // intermediate EN=LOW/NEN=LOW: clamp still engaged
  digitalWrite(GDT_EN_PIN,  HIGH);   // release LAST - PWM now passes to the drivers
  gdt_en_state = true;
}

void gdt_disable(void) {
  digitalWrite(GDT_EN_PIN,  LOW);    // engage clamp FIRST
  digitalWrite(GDT_NEN_PIN, HIGH);   // then the complement
  gdt_en_state = false;
}

bool gdt_is_enabled(void) { return gdt_en_state; }

// ---- DTG encoding (RM0008 14.4.18, tDTS = 13.89 ns @ CKD=00) ----
static uint8_t computeDTG(uint32_t ns) {
  uint32_t t = (ns * 72u + 500u) / 1000u;
  if (t <= 127u) return (uint8_t)t;
  if (t <= 254u) { uint32_t v = t >> 1; if (v < 64u) v = 64u; if (v > 127u) v = 127u;
                   return (uint8_t)(0x80u | (v - 64u)); }
  if (t <= 504u) { uint32_t v = t >> 3; if (v < 32u) v = 32u; if (v > 63u) v = 63u;
                   return (uint8_t)(0xC0u | (v - 32u)); }
  uint32_t v = t >> 4; if (v < 32u) v = 32u; if (v > 63u) v = 63u;
  return (uint8_t)(0xE0u | (v - 32u));
}

static inline float dtPhaseDeg(uint32_t dt_ns, uint32_t f_hz) {
  return (float)dt_ns * 1e-9f * (float)f_hz * 360.0f;
}

static inline void applyDeadTime(uint32_t ns) {
  uint8_t dtg = computeDTG(ns);
  if (dtg != cur_dtg_byte) {
    cur_dtg_byte = dtg;
    TIM1->BDTR = (TIM1->BDTR & ~0xFFUL) | dtg;
  }
  dead_time_ns = ns;
}

void bridge_init(uint32_t freq_hz) {
  RCC->APB2ENR |= RCC_APB2ENR_TIM1EN | RCC_APB2ENR_IOPAEN | RCC_APB2ENR_IOPBEN;

  // PA8 = TIM1_CH1, PB13 = TIM1_CH1N : AF push-pull 50 MHz
  GPIOA->CRH = (GPIOA->CRH & ~(0xFUL << 0))  | (0xBUL << 0);
  GPIOB->CRH = (GPIOB->CRH & ~(0xFUL << 20)) | (0xBUL << 20);

  TIM1->CR1 = 0;
  TIM1->CR2 = (0x2 << 4);              // MMS=010: Update -> TRGO (TIM2 ITR0)
  TIM1->PSC = 0;

  uint32_t arr = (TCLK_HZ / freq_hz) - 1;
  TIM1->ARR  = arr;
  TIM1->CCR1 = (arr + 1) / 2;

  TIM1->CCMR1 = (0x6 << 4) | TIM_CCMR1_OC1PE;          // PWM1, preload
  TIM1->CCER  = TIM_CCER_CC1E | TIM_CCER_CC1NE;        // CH1 + CH1N

  // OSSI=1: with MOE=0 outputs are actively driven to idle (LOW), not Hi-Z.
  // MOE stays OFF until bridge_pwm_enable().
  cur_dtg_byte = computeDTG(DT_FLOOR_NS);
  dead_time_ns = DT_FLOOR_NS;
  TIM1->BDTR = TIM_BDTR_OSSI | cur_dtg_byte;

  TIM1->DIER = 0;
  TIM1->CR1  = TIM_CR1_ARPE | TIM_CR1_CEN;

  cur_freq = freq_hz;
}

void capture_init(void) {
  RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
  // PA0 already configured as INPUT in setup().
  TIM2->CR1   = 0;
  TIM2->PSC   = 0;
  TIM2->ARR   = 0xFFFF;
  TIM2->CNT   = 0;
  TIM2->CCMR1 = TIM_CCMR1_CC1S_0;   // IC1 <- TI1 (PA0), no filter/prescaler
  TIM2->CCER  = TIM_CCER_CC1E;      // rising edge
  TIM2->SMCR  = 0x04;               // SMS=100 reset mode, TS=000 (ITR0=TIM1)
  TIM2->DIER  = 0;                  // polled
  TIM2->SR    = 0;
  TIM2->CR1   = TIM_CR1_CEN;
}

void bridge_set_freq(uint32_t f) {
  if (f < MIN_FREQ) f = MIN_FREQ;
  if (f > MAX_FREQ) f = MAX_FREQ;
  if (f == cur_freq) return;
  cur_freq = f;
  uint32_t arr = (TCLK_HZ / f) - 1;

  // ISR-safe critical section: restore prior interrupt state instead of
  // unconditionally re-enabling (this is also called from TIM4's supervisor).
  uint32_t prim = __get_PRIMASK();
  __disable_irq();
  TIM1->ARR  = arr;
  TIM1->CCR1 = (arr + 1) / 2;
  if (!prim) __enable_irq();
}

static volatile uint32_t last_disable_ms = 0;
static volatile bool     ever_disabled   = false;

bool bridge_restart_ready(void) {
  if (!ever_disabled) return true;   // first arm after boot: no lockout
  return (millis() - last_disable_ms) >= RESTART_LOCKOUT_MS;
}

uint32_t bridge_lockout_remaining_ms(void) {
  if (bridge_restart_ready()) return 0;
  return RESTART_LOCKOUT_MS - (millis() - last_disable_ms);
}

void bridge_pwm_enable(void) {
  if (pwm_enabled) { RELAY_ON(); return; }   // already live: don't touch CNT

  // Flux-centered first pulse: steady state swings flux +/-V*T/4 around
  // zero; a full-width first pulse from zero flux would peak at DOUBLE
  // that. Waking the timer mid-way through the high phase makes the
  // first pulse HALF width, landing flux exactly on the steady-state
  // trajectory. Bleeder across the blocking cap guarantees the cap
  // starts at ~0 V = its 50%-duty equilibrium.
  // Release the GDT clamps BEFORE the flux-centered boarding - never between
  // the CNT preload and MOE. gdt_enable() is two digitalWrites (~a couple us);
  // the TIM1 counter free-runs at 72 MHz, so sitting it in the CNT->MOE window
  // lets CNT drift ~100-300 counts past the CCR1/2 boarding point and the first
  // pulse comes out well under half width. Out here, CNT->MOE is back to a
  // single register write and the half-pulse is exact again.
  gdt_enable();
  delayMicroseconds(5);
  TIM1->EGR  = TIM_EGR_UG;        // latch preloaded ARR/CCR; CNT -> 0
  TIM1->CNT  = TIM1->CCR1 / 2;    // board the cycle mid-pulse (unchanged)
  TIM1->BDTR |= TIM_BDTR_MOE;     // first live edge; nothing between CNT and here
  pwm_enabled = true;
  RELAY_ON();
}

void bridge_pwm_disable(void) {
  TIM1->BDTR &= ~TIM_BDTR_MOE;
  gdt_disable();                  // park the driver inputs right behind MOE
  pwm_enabled = false;
  RELAY_OFF();
  last_disable_ms = millis();
  ever_disabled   = true;
}

void bridge_update_dt_schedule(uint32_t f_res) {
  uint32_t target_dt = DT_FLOOR_NS;
#if DT_SCHEDULE_ENABLE
  int32_t f_above = (int32_t)cur_freq - (int32_t)f_res;
  if (f_above < 0) f_above = 0;
  target_dt = DT_FLOOR_NS + (uint32_t)(DT_SLOPE_NS_PER_KHZ * (f_above / 1000.0f) + 0.5f);
  if (target_dt > DT_MAX_NS)   target_dt = DT_MAX_NS;
  if (target_dt < DT_FLOOR_NS) target_dt = DT_FLOOR_NS;
#endif
  uint32_t dt = dead_time_ns;
  if      (target_dt > dt) dt += (target_dt - dt > DT_SLEW_NS) ? DT_SLEW_NS : (target_dt - dt);
  else if (target_dt < dt) dt -= (dt - target_dt > DT_SLEW_NS) ? DT_SLEW_NS : (dt - target_dt);
  applyDeadTime(dt);

  // Hold the ACTUAL tank phase as DT and frequency move (fixed-time delays
  // converted to degrees at the live frequency, referenced to the tune point).
  phase_target_eff = TARGET_PHASE_DEG
                   + dtPhaseDeg(dead_time_ns + SIG_DELAY_NS, cur_freq)
                   - dtPhaseDeg(DT_TUNE_NS   + SIG_DELAY_NS, F_TUNE_HZ);
}
