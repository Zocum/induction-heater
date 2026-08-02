#include "pll.h"
#include "bridge.h"

volatile bool phase_locked   = false;
uint32_t      locked_freq    = 0;
uint32_t      start_frequency = START_FREQUENCY;

float cur_phase    = 0.0f;
bool  phase_valid  = false;
float pll_disp_err = 0.0f;
bool  pll_in_db    = false;

uint32_t freq_ramp_target       = 0;
bool     pwm_disable_after_ramp = false;

static unsigned long freq_ramp_last_ms = 0;
static float         freq_acc          = 0.0f;

static float ph_hist[PH_N] = {0};
static int   ph_idx = 0;
static bool  phase_hist_init = false;
static float last_valid_phase = 0.0f;

static volatile uint16_t rejected_cnt = 0;
static unsigned long last_upd = 0, last_fupd = 0;

// debug taps (read by main's serial debug)
volatile uint16_t dbg_cnt_val  = 0;
volatile bool     dbg_rejected = false;
volatile uint16_t dbg_rej_cnt  = 0;

static float wrap180(float a) {
  while (a >= 180.0f) a -= 360.0f;
  while (a < -180.0f) a += 360.0f;
  return a;
}

static float filterPhase(float p) {
  ph_hist[ph_idx] = p;
  ph_idx = (ph_idx + 1) % PH_N;
  float s = 0;
  for (int i = 0; i < PH_N; i++) s += ph_hist[i];
  return s / (float)PH_N;
}

void pll_reset_state(void) {
  phase_locked = false;
  pll_in_db = false;
  rejected_cnt = 0;
  freq_acc = (float)cur_freq;
}

void pll_start_ramp(uint32_t to_hz) {
  if (to_hz > MAX_FREQ) to_hz = MAX_FREQ;
  freq_ramp_target  = to_hz;
  freq_ramp_last_ms = millis();
}

void pll_cancel_ramp(void) {
  freq_ramp_target = 0;
  pwm_disable_after_ramp = false;
}

void pll_ramp_task(void) {
  if (!freq_ramp_target) return;
  unsigned long now = millis();
  unsigned long dms = now - freq_ramp_last_ms;
  if (!dms) return;
  freq_ramp_last_ms = now;
  uint32_t stepHz = (uint32_t)(FREQ_RAMP_HZ_PER_MS * dms + 0.5f);
  bool done = false;
  if (cur_freq < freq_ramp_target) {
    uint32_t nf = cur_freq + stepHz;
    if (nf >= freq_ramp_target) { nf = freq_ramp_target; done = true; }
    bridge_set_freq(nf);
    freq_acc = (float)cur_freq;
  } else {
    done = true;   // already at/above target
  }
  if (done) {
    freq_ramp_target = 0;
    if (pwm_disable_after_ramp) { pwm_disable_after_ramp = false; bridge_pwm_disable(); }
  }
}

void pll_engage(void) {
  pll_cancel_ramp();
  phase_locked = true;
  ph_idx = 0; phase_hist_init = false;
  uint32_t fa = (locked_freq > 0 ? locked_freq : start_frequency) + ACQ_OFFSET;
  if (fa > MAX_FREQ) fa = MAX_FREQ;
  bridge_set_freq(fa);
  pll_in_db = false;
  freq_acc = (float)fa;
  last_valid_phase = phase_target_eff;
  rejected_cnt = 0;
}

void pll_release_manual(void) {
  phase_locked = false; pll_in_db = false; rejected_cnt = 0;
  uint32_t sf = (locked_freq > 0) ? locked_freq + UNLOCK_BACKOFF
                                  : start_frequency + UNLOCK_OFFSET;
  locked_freq = 0;
  pll_start_ramp(sf);
  freq_acc = (float)sf;
}

void pll_softstop(void) {
  pll_cancel_ramp();
  if (phase_locked) {
    phase_locked = false; pll_in_db = false; rejected_cnt = 0;
    pll_start_ramp(cur_freq + PWM_SOFTSTOP_OFFSET);
    pwm_disable_after_ramp = true;   // gates cut when the ramp lands
  } else {
    bridge_pwm_disable();            // not locked -> cut now
  }
}

void pll_timer_expired(void) {
  pll_start_ramp(cur_freq + TIMER_BACKOFF);
  phase_locked = false; pll_in_db = false; rejected_cnt = 0;
}

void pll_manual_step(bool up) {
  if (up) {
    start_frequency += FREQ_STEP;
    if (start_frequency > MAX_FREQ) start_frequency = MAX_FREQ;
  } else {
    start_frequency = (start_frequency > MIN_FREQ + FREQ_STEP)
                      ? start_frequency - FREQ_STEP : MIN_FREQ;
  }
  if (!phase_locked) {
    // Nudge from where we ARE (both directions - the old sketch still had the
    // pre-fix base on BTN_DOWN), keep the base in sync for the next lock.
    pll_cancel_ramp();
    uint32_t nf = up ? cur_freq + FREQ_STEP
                     : (cur_freq > MIN_FREQ + FREQ_STEP ? cur_freq - FREQ_STEP : MIN_FREQ);
    bridge_set_freq(nf);
    freq_acc = (float)cur_freq;
    start_frequency = cur_freq;
  }
}

void pll_control_task(bool lock_switch) {
  if (millis() - last_upd <= 5) return;
  bridge_update_dt_schedule(locked_freq > 0 ? locked_freq : start_frequency);
  last_upd = millis();

  if (!(TIM2->SR & TIM_SR_CC1IF)) return;

  // Reading CCR1 clears CC1IF. CC1OF (missed captures) is informational.
  uint16_t cnt_val = (uint16_t)TIM2->CCR1;
  TIM2->SR &= ~TIM_SR_CC1OF;

  uint32_t arr_plus1 = TIM1->ARR + 1;
  uint32_t ccr1      = TIM1->CCR1;

  // ---- noise rejection: ignore edges hugging the reload or the compare ----
  uint32_t margin = arr_plus1 / NOISE_DIV;
  bool near_reload  = (cnt_val < margin) || (cnt_val > arr_plus1 - margin);
  bool near_compare = (cnt_val > ccr1 - margin) && (cnt_val < ccr1 + margin);
  bool valid = !(near_reload || near_compare);

  dbg_cnt_val  = cnt_val;
  dbg_rejected = !valid;

  float meas_deg = 0.0f;
  if (valid) {
    meas_deg = ((float)cnt_val / (float)arr_plus1) * 360.0f;
    while (meas_deg > phase_target_eff + 180.0f) meas_deg -= 360.0f;
    while (meas_deg < phase_target_eff - 180.0f) meas_deg += 360.0f;
    cur_phase = meas_deg;      // published regardless of lock state
    phase_valid = true;
  }

  if (!(phase_locked && lock_switch && pwm_enabled)) { dbg_rej_cnt = rejected_cnt; return; }

  if (!valid) {
    rejected_cnt++;
    if (rejected_cnt > REJECT_SWEEP_TH) {
      // No usable edges: creep upward (safe, inductive side).
      freq_acc += 15.0f;
      int32_t nf = (int32_t)(freq_acc + 0.5f);
      if (nf > (int32_t)MAX_FREQ) { nf = MAX_FREQ; freq_acc = (float)MAX_FREQ; }
      if ((millis() - last_fupd) >= FREQ_UPD_INT_MS) {
        bridge_set_freq((uint32_t)nf);
        last_fupd = millis();
      }
    }
    dbg_rej_cnt = rejected_cnt;
    return;
  }

  rejected_cnt = 0;
  dbg_rej_cnt = 0;

  if (!phase_hist_init) {
    for (int i = 0; i < PH_N; i++) ph_hist[i] = meas_deg;
    ph_idx = 0; phase_hist_init = true;
    last_valid_phase = meas_deg;
  }

  float fp = filterPhase(meas_deg);

  // Resonance crossing: >120 deg jump in filtered phase means we fell through
  // to the capacitive side -> re-acquire from above (the safe, ZVS side).
  if (fabs(wrap180(fp - last_valid_phase)) > PHASE_JUMP_TH) {
    Serial.println(F("!JUMP"));
    pll_engage();              // re-seed above resonance and hunt again
    return;
  }
  last_valid_phase = fp;

  // ---- proportional tracker ----
  float err = wrap180(fp - phase_target_eff);
  pll_disp_err = err;
  pll_in_db = (fabs(err) < LOCK_DEADBAND);

  // Only trust locked_freq once genuinely parked in the deadband.
  if (pll_in_db) locked_freq = cur_freq;

  // One correction per FREQ_UPD_INT, not per capture (gain sanity).
  if (!pll_in_db && (millis() - last_fupd) >= FREQ_UPD_INT_MS) {
    float step = -LOCK_K * err;
    if (step >  MAX_STEP) step =  MAX_STEP;
    if (step < -MAX_STEP) step = -MAX_STEP;
    freq_acc += step;
    if (freq_acc < (float)MIN_FREQ) freq_acc = (float)MIN_FREQ;
    if (freq_acc > (float)MAX_FREQ) freq_acc = (float)MAX_FREQ;
    uint32_t nf = (uint32_t)(freq_acc + 0.5f);
    if (nf != cur_freq) bridge_set_freq(nf);
    last_fupd = millis();
  }
}
