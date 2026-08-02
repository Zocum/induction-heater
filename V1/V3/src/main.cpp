#include "config.h"
#include "bridge.h"
#include "pll.h"
#include "safety.h"
#include "oled.h"
#include "ds18b20.h"

// ===================================================================
// main: wiring between modules + all UI (buttons, switches, pot, serial).
//
// Loop structure (order matters):
//   serial -> safety feed -> temp -> ramp -> BUTTONS/SWITCHES -> timer
//   -> PLL control -> debug -> DISPLAY LAST.
// Inputs are polled before the display so the slowest, most EMI-fragile
// peripheral can never add latency to the safety-relevant controls.
//
// ALL four digital inputs (BTN_UP/BTN_DOWN/LOCK/PWM_EN) go through the
// stable_read() EMI glitch filter: a state change is accepted only after
// the raw pin has held the new level for STABLE_MS continuously. This is
// what stops switch-node dV/dt bursts above ~40 V bus from "pressing"
// buttons through the weak internal pull-downs.
// ===================================================================

// ---- temperature ----
static int16_t currentTempX10 = 0;
static unsigned long lastTempRead = 0;
static bool tempSensorFound = false;
static bool tempConvStarted = false;

// ---- run timer ----
static uint32_t timer_setpoint_s = 0;
static uint32_t pot_set_s = 0;
static bool timer_running = false;
static bool timer_fired = false;
static unsigned long timer_start_ms = 0;

// ---- UI state ----
static unsigned long prog_start = 0;
static unsigned long last_up = 0, last_down = 0;
static unsigned long up_press_start = 0, down_press_start = 0;
static bool up_was_pressed = false, down_was_pressed = false;
static bool last_lock = false, last_pwm = false;
static bool wdt_arm_inhibit = false;    // set on WDT boot until switch seen LOW
static bool pwm_pending = false;

// ---- debug ----
static bool debug_phase = true;
static unsigned long last_debug = 0;
extern volatile uint16_t dbg_cnt_val;
extern volatile bool     dbg_rejected;
extern volatile uint16_t dbg_rej_cnt;

// ===================================================================
// EMI glitch filter: a state change is accepted only after the raw pin
// has held the new level continuously for STABLE_MS. Real presses are
// >50 ms; switching-edge EMI bursts are ns-us. Latency cost: 30 ms,
// imperceptible on a human control.
#define STABLE_MS 30
typedef struct { bool stable; bool raw; unsigned long t; } debounced_t;

static bool stable_read(debounced_t *d, uint32_t pin) {
  bool r = digitalRead(pin);
  if (r != d->raw) { d->raw = r; d->t = millis(); }
  else if (r != d->stable && (millis() - d->t) >= STABLE_MS) d->stable = r;
  return d->stable;
}

static debounced_t db_up, db_dn, db_lock, db_pwm;

// ===================================================================

static uint32_t readTimerPot(void) {
  uint32_t acc = 0;
  for (int i = 0; i < 16; i++) acc += analogRead(POT_PIN);
  int raw = (int)(acc / 16);
  float f = (float)(raw - POT_RAW_LO) / (float)(POT_RAW_HI - POT_RAW_LO);
  if (f < 0.0f) f = 0.0f;
  if (f > 1.0f) f = 1.0f;
  if (f < TIMER_OFF_FRAC) return 0;
  uint32_t s = (uint32_t)(f * (float)TIMER_MAX_S + 0.5f);
  if (s < TIMER_MIN_S) s = TIMER_MIN_S;
  if (s > TIMER_MAX_S) s = TIMER_MAX_S;
  return s;
}

static void armTimer(void) {
  timer_setpoint_s = readTimerPot();
  timer_fired = false;
  if (timer_setpoint_s > 0) { timer_running = true; timer_start_ms = millis(); }
  else timer_running = false;
}

static void readTemp(void) {
  if (millis() - lastTempRead < 1500) return;
  lastTempRead = millis();
  if (!tempSensorFound) return;
  if (tempConvStarted) {
    int16_t t10 = (int16_t)((ds_read_temp_raw() * 10) / 16);
    if (t10 > -500 && t10 < 1500) currentTempX10 = t10;
  }
  ds_start_conv();
  tempConvStarted = true;
}

static void handleSerial(void) {
  static char cb[12];
  static uint8_t cp = 0;

  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (cp == 0) continue;
      cb[cp] = '\0'; cp = 0;

      if (cb[0] == 'f' || cb[0] == 'F') {
        pll_manual_step(cb[1] == '+');
      }
      else if (cb[0] == 's') {
        Serial.print(cur_freq); Serial.print(F(" "));
        Serial.print(phase_locked ? '1' : '0');
        Serial.print(F(" pwm="));  Serial.print(pwm_enabled ? '1' : '0');
        Serial.print(F(" gdt="));  Serial.print(gdt_is_enabled() ? '1' : '0');
        Serial.print(F(" pot="));  Serial.print(analogRead(POT_PIN));
        Serial.print(F(" set="));  Serial.print(pot_set_s);
        Serial.print(F(" tgt="));  Serial.print(TARGET_PHASE_DEG, 1);
        Serial.print(F(" eff="));  Serial.print(phase_target_eff, 1);
        Serial.print(F(" dt="));   Serial.print(dead_time_ns);
        Serial.print(F(" saf="));  Serial.print((int)safety_state());
        Serial.print(F(" i2crec=")); Serial.print(oled_recover_count());
        Serial.print(F(" i2cskip=")); Serial.println(oled_skip_count());
      }
      else if (cb[0] == 'v') {
        debug_phase = !debug_phase;
        Serial.print(F("Debug:")); Serial.println(debug_phase ? F("ON") : F("OFF"));
      }
      else if (cb[0] == 'x') {
        // DELIBERATE FREEZE - validates the safety chain (ISR detune ~200ms,
        // MOE cut, IWDG reset ~0.8s, safe boot). Refused while the bridge is
        // switching: serial RX is an EMI-exposed line and this command must
        // never be triggerable by noise during a real run. Test with the
        // drive disabled (supervisor arms on pwm_enabled anyway, so validate
        // by enabling PWM with the TANK UNPOWERED, then sending 'x' - the
        // refusal below only blocks the case where the bridge is live).
        if (pwm_enabled) {
          Serial.println(F("!FREEZE refused: PWM is enabled"));
        } else {
          Serial.println(F("!FREEZE test - blocking loop now"));
          Serial.flush();
          for (;;) { /* heartbeat starves; supervisor takes over */ }
        }
      }
    } else if (cp < sizeof(cb) - 1) cb[cp++] = c;
  }
}

// ===================================================================

void setup() {
  Serial.begin(115200);
  delay(300);

  Serial.print(F("Target phase=")); Serial.print(TARGET_PHASE_DEG, 1); Serial.println(F("deg"));
  Serial.print(F("DT floor=")); Serial.print(DT_FLOOR_NS);
  Serial.print(F("ns max="));   Serial.print(DT_MAX_NS);
  Serial.print(F("ns tune="));  Serial.println(DT_TUNE_NS);

  prog_start = millis();

  oled_init();
  oled_splash("Init...");

  tempSensorFound = ds_reset();
  if (tempSensorFound) {
    ds_start_conv();
    delay(750);
    currentTempX10 = (int16_t)((ds_read_temp_raw() * 10) / 16);
    tempConvStarted = true;
  }
  delay(200);

  pinMode(LOCK_PIN,   INPUT_PULLDOWN);
  pinMode(TANK_PIN,   INPUT);
  pinMode(BTN_UP,     INPUT_PULLDOWN);
  pinMode(BTN_DOWN,   INPUT_PULLDOWN);
  pinMode(PWM_EN_PIN, INPUT_PULLDOWN);

  analogReadResolution(12);
  pinMode(POT_PIN, INPUT_ANALOG);

  // Relay off before the pin becomes an output (no active-low glitch).
  #if RELAY_ACTIVE_LOW
    digitalWrite(RELAY_PIN, HIGH);
  #else
    digitalWrite(RELAY_PIN, LOW);
  #endif
  pinMode(RELAY_PIN, OUTPUT);
  #if RELAY_ACTIVE_LOW
    digitalWrite(RELAY_PIN, HIGH);
  #else
    digitalWrite(RELAY_PIN, LOW);
  #endif

  // GDT clamps: safe state (EN=LOW clamped, NEN=HIGH) driven BEFORE the pins
  // become outputs and before any TIM1 config below - same no-glitch pattern
  // as the relay. Takes over seamlessly from the external pulls that held
  // this exact state through reset.
  digitalWrite(GDT_EN_PIN,  LOW);
  digitalWrite(GDT_NEN_PIN, HIGH);
  pinMode(GDT_EN_PIN,  OUTPUT);
  pinMode(GDT_NEN_PIN, OUTPUT);
  digitalWrite(GDT_EN_PIN,  LOW);
  digitalWrite(GDT_NEN_PIN, HIGH);

  // Seed the glitch filters from the real pin states so a switch already in
  // a position at boot doesn't produce a phantom edge 30 ms in. wdt_arm_inhibit
  // still guards the PWM switch after a watchdog reset.
  db_up.raw   = db_up.stable   = digitalRead(BTN_UP);     db_up.t   = millis();
  db_dn.raw   = db_dn.stable   = digitalRead(BTN_DOWN);   db_dn.t   = millis();
  db_lock.raw = db_lock.stable = digitalRead(LOCK_PIN);   db_lock.t = millis();
  db_pwm.raw  = db_pwm.stable  = digitalRead(PWM_EN_PIN); db_pwm.t  = millis();
  last_lock = db_lock.stable;
  last_pwm  = false;   // an already-HIGH PWM switch at boot must still produce
                       // one deliberate edge to arm (matches old behavior)

  bridge_init(start_frequency + UNLOCK_OFFSET);
  capture_init();
  pll_reset_state();
  bridge_update_dt_schedule(start_frequency);

  // Safety LAST, after everything is in a defined state. If the previous
  // reset was an IWDG reset, inhibit arming until the switch is seen LOW.
  safety_init();
  if (safety_wdt_boot()) {
    wdt_arm_inhibit = true;
    Serial.println(F("!WDT RESET - safe boot, cycle PWM switch to arm"));
  }

  oled_clear();
}

// ===================================================================

void loop() {
  handleSerial();

  // Feed both protection layers once per healthy pass.
  safety_heartbeat();
  safety_iwdg_kick();

  readTemp();
  pll_ramp_task();

  // --- ALL digital inputs through the EMI glitch filter ---
  bool lsw = stable_read(&db_lock, LOCK_PIN);
  bool psw = stable_read(&db_pwm,  PWM_EN_PIN);

  // WDT-boot arming inhibit clears once the switch has been seen safe/LOW.
  if (wdt_arm_inhibit && !psw) wdt_arm_inhibit = false;

  // Latched supervisor fault: bookkeeping (ISR already cut MOE + relay),
  // clears only via switch LOW.
  if (safety_state() == SAFETY_FAULT) {
    timer_running = false;
    if (!psw) { safety_clear_fault(); last_pwm = false; }
  }

  // --- BTN_UP / BTN_DOWN (polled BEFORE any display work) ---
  bool up_now = stable_read(&db_up, BTN_UP);
  if (up_now && !up_was_pressed) { up_press_start = millis(); up_was_pressed = true; }
  if (!up_now) up_was_pressed = false;
  if (up_now) {
    unsigned long hold = millis() - up_press_start;
    unsigned long rep = (hold > 6000) ? 40 : (hold > 3000) ? 70 : DEBOUNCE_MS;
    if (millis() - last_up > rep) { last_up = millis(); pll_manual_step(true); }
  }

  bool dn_now = stable_read(&db_dn, BTN_DOWN);
  if (dn_now && !down_was_pressed) { down_press_start = millis(); down_was_pressed = true; }
  if (!dn_now) down_was_pressed = false;
  if (dn_now) {
    unsigned long hold = millis() - down_press_start;
    unsigned long rep = (hold > 6000) ? 40 : (hold > 3000) ? 70 : DEBOUNCE_MS;
    if (millis() - last_down > rep) { last_down = millis(); pll_manual_step(false); }
  }

  // --- Lock switch ---
  if (lsw != last_lock) {
    last_lock = lsw;
    if (lsw) {
      pll_engage();
      if (pwm_enabled && !timer_running) armTimer();
    } else {
      timer_running = false; timer_fired = false;
      pll_release_manual();
    }
  }

  // --- PWM enable switch ---
  if (psw != last_pwm) {
    last_pwm = psw;
    if (psw) {
      if (safety_state() != SAFETY_OK || wdt_arm_inhibit) {
        // refuse; display shows why
      } else if (!bridge_restart_ready()) {
        pwm_pending = true;          // arm when cooldown expires
      } else {
        pll_cancel_ramp();
        if (lsw) { pll_engage(); bridge_pwm_enable(); armTimer(); }
        else     { phase_locked = false; bridge_pwm_enable(); }
      }
    } else {
      pwm_pending = false;
      timer_running = false; timer_fired = false;
      pll_softstop();
    }
  }

  // --- Run timer: follow the pot live ---
  static unsigned long last_pot_ms = 0;
  if (millis() - last_pot_ms > 150) {
    last_pot_ms = millis();
    pot_set_s = readTimerPot();
    if (timer_running) timer_setpoint_s = pot_set_s;
  }

  if (timer_running && pwm_enabled &&
      (millis() - timer_start_ms >= timer_setpoint_s * 1000UL)) {
    if (phase_locked) {
      pll_timer_expired();
      last_lock = lsw;               // suppress auto re-lock while switch held
    }
    timer_running = false;
    timer_fired = true;
    Serial.println(F("!TIMER"));
  }

  // --- PLL control (200 Hz internally) ---
  pll_control_task(lsw);

  if (pwm_pending && psw && bridge_restart_ready() &&
      safety_state() == SAFETY_OK && !wdt_arm_inhibit) {
    pwm_pending = false;
    pll_cancel_ramp();
    if (lsw) { pll_engage(); bridge_pwm_enable(); armTimer(); }
    else     { phase_locked = false; bridge_pwm_enable(); }
  }

  // --- Debug ---
  if (debug_phase && phase_locked && (millis() - last_debug > 100)) {
    last_debug = millis();
    uint32_t arr1 = TIM1->ARR + 1;
    float deg = ((float)dbg_cnt_val / (float)arr1) * 360.0f;
    Serial.print(cur_freq);   Serial.print(' ');
    Serial.print(dbg_cnt_val); Serial.print(' ');
    Serial.print(deg, 1);
    Serial.print(dbg_rejected ? F(" nope ") : F("  ok "));
    Serial.print(dbg_rej_cnt);
    Serial.print(F(" tgt=")); Serial.println(phase_target_eff, 1);
  }

  // --- Display LAST (guarded, throttled, skippable) ---
  disp_data_t d;
  d.err = pll_disp_err;       d.in_db = pll_in_db;
  d.timer_running = timer_running; d.timer_fired = timer_fired;
  d.timer_setpoint_s = timer_setpoint_s; d.timer_start_ms = timer_start_ms;
  d.pot_set_s = pot_set_s;
  d.temp_x10 = currentTempX10; d.temp_ok = tempSensorFound;
  d.prog_start_ms = prog_start;
  oled_task(&d);

  delay(1);
}