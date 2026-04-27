// ===================================================================
// Induction Heater Controller - STM32 Blue Pill (STM32F103C8T6)
// ===================================================================
// Phase measurement: HARDWARE INPUT CAPTURE.
//   - TIM1 generates PWM on PA8 and emits an Update trigger (TRGO) on
//     each reload (i.e. on every PWM rising edge).
//   - TIM2 runs at 72 MHz, slaved to TIM1 via ITR0 in Reset Mode so
//     TIM2->CNT is reset to 0 at every PWM rising edge.
//   - TIM2_CH1 (= PA0) is configured as input capture, rising edge.
//     TIM2->CCR1 is latched in hardware the instant PA0 goes high.
//   - Therefore CCR1 = time in 72 MHz ticks from PWM rising edge to
//     tank-current rising edge at the pin, with zero software latency.
//
// TARGET_PHASE now represents only physical delay in the signal path
// (mostly LM339 propagation delay), not ISR latency.
//
// WIRING:
//   PA0  -> Tank current zero-crossing (from LM339 after CT) [TIM2_CH1]
//   PA8  -> PWM output                                       [TIM1_CH1]
//   PB12 -> Lock switch (active HIGH)
//   PB14 -> Frequency UP button
//   PB15 -> Frequency DOWN button
//   PA1  -> DS18B20 (4.7k pull-up to 3.3V)
//   PB6(SCL)/PB7(SDA) -> I2C1 OLED
// ====================================================================

#include <Wire.h>
#include <U8x8lib.h>

// ===== PINS =====
#define PWM_PIN      PA8
#define TANK_PIN     PA0
#define LOCK_PIN     PB12
#define BTN_UP       PB14
#define BTN_DOWN     PB15
#define DS_PIN       PA1

// ===== OLED =====
U8X8_SH1106_128X64_NONAME_HW_I2C u8x8(U8X8_PIN_NONE);

// ===== TEMPERATURE =====
int16_t currentTempX10 = 0;
unsigned long lastTempRead = 0;
bool tempSensorFound = false;
bool tempConvStarted = false;

// ===== FREQUENCY =====
const uint32_t TCLK = 72000000;
uint32_t start_frequency = 50000;
const uint32_t FREQ_STEP = 100;
const uint32_t MIN_FREQ = 20000, MAX_FREQ = 150000;
const uint32_t UNLOCK_OFFSET = 3400;

// ===== PHASE TARGET =====
// With hardware input capture, the ONLY residual delay between the
// tank-current edge at the pin and the captured CCR1 value is:
//   - LM339 propagation delay (~1.3-2 us typical @ 52 kHz -> ~25-38 deg)
//   - CT/signal conditioning phase shift (usually small)
// Software/ISR latency is effectively zero. Retune this after flashing.
const float TARGET_PHASE = 22.0f;

const float PHASE_JUMP_TH = 120.0f;
float last_valid_phase = TARGET_PHASE;

// ===== CONTROL =====
const float LOCK_ENTER_TH = 6.0f, LOCK_EXIT_TH = 12.0f;
const float Kp = 0.25f, Ki = 0.02f, Kd = 0.08f;
const int32_t MFC_L = 50, MFC_M = 10, MFC_S = 2;
const unsigned long FREQ_UPD_INT = 20;

// ===== PHASE MEASUREMENT =====
bool phase_hist_init = false;

// ===== NOISE REJECTION =====
volatile uint16_t rejected_cnt = 0;
const uint16_t REJECT_SWEEP_TH = 15;
const uint32_t NOISE_DIV = 20;

// ===== GLOBALS =====
volatile bool phase_locked = false;
uint32_t cur_freq = 0, locked_freq = 0;
unsigned long prog_start = 0;
unsigned long last_up = 0, last_down = 0, last_fupd = 0;
const unsigned long DEBOUNCE = 200;
unsigned long up_press_start = 0, down_press_start = 0;
bool up_was_pressed = false, down_was_pressed = false;

const int PH_N = 15;
float ph_hist[PH_N] = {0};
int ph_idx = 0;
float last_err = 0, int_err = 0, freq_acc = 0;
const int DB_STABLE = 15;

// ===== DEBUG =====
bool debug_phase = true;
unsigned long last_debug = 0;
volatile uint16_t dbg_cnt_val = 0;
volatile bool dbg_rejected = false;

// ===== LIVE PHASE DISPLAY =====
// Updated on every valid capture, regardless of lock state, so the user
// can watch phase change while sweeping frequency manually.
float cur_phase = 0.0f;
bool  phase_valid = false;

// ===== HELPERS =====
static float wrap180(float a) {
  while (a >= 180.0f) a -= 360.0f;
  while (a < -180.0f) a += 360.0f;
  return a;
}

float filterPhase(float p) {
  ph_hist[ph_idx] = p;
  ph_idx = (ph_idx + 1) % PH_N;
  float s = 0;
  for (int i = 0; i < PH_N; i++) s += ph_hist[i];
  return s / (float)PH_N;
}

// ===== RAW DS18B20 BIT-BANG =====
static inline void ds_low() {
  pinMode(DS_PIN, OUTPUT);
  digitalWrite(DS_PIN, LOW);
}

static inline void ds_release() {
  pinMode(DS_PIN, INPUT_PULLUP);
}

static inline uint8_t ds_rd() {
  return digitalRead(DS_PIN);
}

bool ds_reset() {
  ds_low();
  delayMicroseconds(480);
  ds_release();
  delayMicroseconds(70);
  uint8_t p = !ds_rd();
  delayMicroseconds(410);
  return p;
}

void ds_write_bit(uint8_t b) {
  if (b) {
    ds_low(); delayMicroseconds(6);
    ds_release(); delayMicroseconds(64);
  } else {
    ds_low(); delayMicroseconds(60);
    ds_release(); delayMicroseconds(10);
  }
}

uint8_t ds_read_bit() {
  ds_low(); delayMicroseconds(3);
  ds_release(); delayMicroseconds(10);
  uint8_t b = ds_rd();
  delayMicroseconds(53);
  return b;
}

void ds_write(uint8_t d) {
  for (int i = 0; i < 8; i++) { ds_write_bit(d & 1); d >>= 1; }
}

uint8_t ds_read() {
  uint8_t d = 0;
  for (int i = 0; i < 8; i++) if (ds_read_bit()) d |= (1 << i);
  return d;
}

void ds_start_conv() {
  ds_reset(); ds_write(0xCC); ds_write(0x44);
  tempConvStarted = true;
}

int16_t ds_read_temp() {
  ds_reset(); ds_write(0xCC); ds_write(0xBE);
  uint8_t lo = ds_read(), hi = ds_read();
  return (int16_t)((hi << 8) | lo);
}

// ===== TIM1 DIRECT REGISTER INIT =====
void TIM1_Init(uint32_t freq_hz) {
  RCC->APB2ENR |= RCC_APB2ENR_TIM1EN | RCC_APB2ENR_IOPAEN;

  // PA8 = TIM1_CH1 alternate-function push-pull
  GPIOA->CRH = (GPIOA->CRH & ~(0xFUL << 0)) | (0xBUL << 0);

  TIM1->CR1 = 0;
  TIM1->CR2 = (0x2 << 4);  // MMS = 010: Update event as TRGO -> TIM2 ITR0
  TIM1->PSC = 0;

  uint32_t arr = (TCLK / freq_hz) - 1;
  TIM1->ARR = arr;
  TIM1->CCR1 = (arr + 1) / 2;

  TIM1->CCMR1 = (0x6 << 4) | TIM_CCMR1_OC1PE;

  // Enable CH1 only
  TIM1->CCER = TIM_CCER_CC1E;

  TIM1->BDTR |= TIM_BDTR_MOE;

  TIM1->DIER = 0;
  TIM1->CR1 = TIM_CR1_ARPE | TIM_CR1_CEN;

  cur_freq = freq_hz;
}

// ===== TIM2 INPUT CAPTURE INIT =====
// PA0 = TIM2_CH1 (default pin mapping, no remap needed).
// TIM2 runs at 72 MHz (same as TIM1: APB1=36 MHz -> TIMxCLK = 2x = 72 MHz).
// Slave mode Reset: TIM1's Update TRGO (via ITR0) resets TIM2->CNT to 0
// at every PWM rising edge. CH1 captures CCR1 on rising edge of PA0.
// CCR1 therefore holds the exact delay in 72 MHz ticks from PWM rising
// edge to tank edge at the pin.
void TIM2_CaptureInit(void) {
  RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;

  // PA0 is already configured as INPUT by pinMode() in setup(), which is
  // what TIM2_CH1 needs (the timer reads from the pin IDR).

  TIM2->CR1  = 0;
  TIM2->PSC  = 0;        // 72 MHz tick (matches TIM1)
  TIM2->ARR  = 0xFFFF;   // free-run max; will be reset each PWM period
  TIM2->CNT  = 0;

  // CC1 channel: input capture, IC1 mapped to TI1 (PA0)
  //   CCMR1: CC1S = 01 (IC1 on TI1), no prescaler, no filter
  TIM2->CCMR1 = TIM_CCMR1_CC1S_0;

  // Enable capture, rising edge (CC1P = 0, CC1NP = 0, CC1E = 1)
  TIM2->CCER = TIM_CCER_CC1E;

  // Slave mode control:
  //   SMS = 100 (Reset Mode): rising edge of selected trigger resets CNT
  //   TS  = 000 (ITR0 = TIM1)
  TIM2->SMCR = 0x04;

  TIM2->DIER = 0;        // no interrupts; we poll CC1IF in the main loop
  TIM2->SR   = 0;        // clear any pending flags

  TIM2->CR1 = TIM_CR1_CEN;
}

// ===== FREQUENCY UPDATE =====
void updateFreq(uint32_t f) {
  if (f < MIN_FREQ) f = MIN_FREQ;
  if (f > MAX_FREQ) f = MAX_FREQ;
  if (f == cur_freq) return;
  cur_freq = f;
  uint32_t arr = (TCLK / f) - 1;
  noInterrupts();
  TIM1->ARR = arr;
  TIM1->CCR1 = (arr + 1) / 2;
  interrupts();
}

// ===== DISPLAY =====
void updateDisplay(float err, bool in_db) {
  u8x8.setCursor(0, 0);
  u8x8.print(F("Induction Heater"));

  u8x8.setCursor(0, 2);
  u8x8.print(phase_locked ? F("L:") : F("F:"));
  uint32_t khz = cur_freq / 1000;
  uint32_t frac = cur_freq % 1000;
  if (khz < 100) u8x8.print(' ');
  u8x8.print(khz);
  u8x8.print('.');
  u8x8.print(frac / 100);
  if (phase_locked) u8x8.print((frac / 10) % 10);
  u8x8.print(F("kHz  "));

  u8x8.setCursor(0, 3);
  u8x8.print(F("T:"));
  if (tempSensorFound) {
    int t = currentTempX10 / 10;
    if (t < 100) u8x8.print(' ');
    if (t < 10)  u8x8.print(' ');
    u8x8.print(t);
  } else {
    u8x8.print(F(" --"));
  }
  u8x8.print(F("C           "));

  // Row 4: live phase always; error too when locked
  u8x8.setCursor(0, 4);
  if (phase_valid) {
    u8x8.print(F("P:"));
    float p = cur_phase;
    if (p < 0) { u8x8.print('-'); p = -p; } else u8x8.print(' ');
    int pi = (int)(p * 10.0f + 0.5f);     // tenths of a degree
    if (pi < 1000) u8x8.print(' ');
    if (pi < 100)  u8x8.print(' ');
    u8x8.print(pi / 10);
    u8x8.print('.');
    u8x8.print(pi % 10);
    // 8 chars used: "P: 35.2"  /  "P:-12.3"  /  "P:180.0"

    if (phase_locked) {
      u8x8.print(F(" E:"));
      float e = err;
      if (e < 0) { u8x8.print('-'); e = -e; } else u8x8.print(' ');
      int ei = (int)(e * 10.0f + 0.5f);
      if (ei < 100) u8x8.print(' ');
      u8x8.print(ei / 10);
      u8x8.print('.');
      u8x8.print(ei % 10);
      // 7 more chars: " E:+0.3"  -> 15 total, +1 pad
      u8x8.print(F(" "));
    } else {
      u8x8.print(F("        "));      // pad to clear old error
    }
  } else {
    u8x8.print(F("P: ---          "));
  }

  u8x8.setCursor(0, 5);
  unsigned long el = (millis() - prog_start) / 1000;
  unsigned long m = el / 60, s = el % 60;
  u8x8.print(F("Time "));
  if (m < 10) u8x8.print('0');
  u8x8.print(m);
  u8x8.print(':');
  if (s < 10) u8x8.print('0');
  u8x8.print(s);
  u8x8.print(F("     "));

  // Row 6: show reject count when debugging
  u8x8.setCursor(0, 6);
  if (phase_locked && rejected_cnt > 0) {
    u8x8.print(F("Rej:"));
    u8x8.print(rejected_cnt);
    u8x8.print(F("        "));
  } else {
    u8x8.print(F("                "));
  }

  u8x8.setCursor(0, 7);
  if (phase_locked && in_db)  u8x8.print(F("** LOCKED **    "));
  else if (phase_locked)      u8x8.print(F("Tracking...     "));
  else                        u8x8.print(F("Unlocked        "));
}

// ===== TEMPERATURE =====
void readTemp() {
  if (millis() - lastTempRead >= 1500) {
    lastTempRead = millis();
    if (tempSensorFound) {
      if (tempConvStarted) {
        int16_t raw = ds_read_temp();
        int16_t t10 = (raw * 10) / 16;
        if (t10 > -500 && t10 < 1500) currentTempX10 = t10;
      }
      ds_start_conv();
    }
  }
}

// ===== SERIAL COMMANDS =====
void handleSerial() {
  static char cb[12];
  static uint8_t cp = 0;

  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (cp == 0) continue;
      cb[cp] = '\0'; cp = 0;

      if (cb[0] == 'f' || cb[0] == 'F') {
        if (cb[1] == '+') {
          start_frequency += FREQ_STEP;
          if (start_frequency > MAX_FREQ) start_frequency = MAX_FREQ;
        } else {
          start_frequency -= FREQ_STEP;
          if (start_frequency < MIN_FREQ) start_frequency = MIN_FREQ;
        }
        if (!phase_locked) {
          updateFreq(start_frequency + UNLOCK_OFFSET);
          freq_acc = (float)cur_freq;
        }
      }
      else if (cb[0] == 's') {
        Serial.print(cur_freq); Serial.print(F(" "));
        Serial.print(phase_locked ? '1' : '0');
        Serial.print(F(" tgt=")); Serial.println(TARGET_PHASE, 1);
      }
      // Toggle debug output with 'v' (verbose)
      else if (cb[0] == 'v') {
        debug_phase = !debug_phase;
        Serial.print(F("Debug:")); Serial.println(debug_phase ? F("ON") : F("OFF"));
      }
    } else {
      if (cp < sizeof(cb) - 1) cb[cp++] = c;
    }
  }
}

// ===================================================================
void setup() {
  Serial.begin(115200);
  delay(300);

  Serial.print(F("Target phase=")); Serial.print(TARGET_PHASE, 1); Serial.println(F("deg"));

  prog_start = millis();

  Wire.begin();
  u8x8.begin();
  u8x8.setFont(u8x8_font_chroma48medium8_r);
  u8x8.setCursor(0, 3);
  u8x8.print(F("Init..."));

  tempSensorFound = ds_reset();
  if (tempSensorFound) {
    ds_start_conv();
    delay(750);
    int16_t raw = ds_read_temp();
    currentTempX10 = (raw * 10) / 16;
  }

  delay(200);

  pinMode(LOCK_PIN, INPUT_PULLDOWN);
  pinMode(TANK_PIN, INPUT);
  pinMode(BTN_UP, INPUT_PULLDOWN);
  pinMode(BTN_DOWN, INPUT_PULLDOWN);

  // No attachInterrupt needed: tank edge is captured in hardware by TIM2_CH1,
  // and the lock switch / buttons are polled in loop().
  TIM1_Init(start_frequency + UNLOCK_OFFSET);
  TIM2_CaptureInit();
  u8x8.clear();
}

// ===================================================================
void loop() {
  static unsigned long last_upd = 0, last_disp = 0;
  static bool last_lock = false, in_db = false;
  static uint32_t stb_cnt = 0;
  static float disp_err = 0.0f;

  handleSerial();
  readTemp();

  // --- BTN_UP ---
  bool up_now = digitalRead(BTN_UP);
  if (up_now && !up_was_pressed) {
    up_press_start = millis();
    up_was_pressed = true;
  }
  if (!up_now) up_was_pressed = false;

  if (up_now) {
    unsigned long hold = millis() - up_press_start;
    unsigned long rep = (hold > 6000) ? 40 :
                        (hold > 3000) ? 70 :
                                       DEBOUNCE;
    if (millis() - last_up > rep) {
      last_up = millis();
      start_frequency += FREQ_STEP;
      if (start_frequency > MAX_FREQ) start_frequency = MAX_FREQ;
      if (!phase_locked) { updateFreq(start_frequency + UNLOCK_OFFSET); freq_acc = cur_freq; }
    }
  }

  // --- BTN_DOWN ---
  bool dn_now = digitalRead(BTN_DOWN);
  if (dn_now && !down_was_pressed) {
    down_press_start = millis();
    down_was_pressed = true;
  }
  if (!dn_now) down_was_pressed = false;

  if (dn_now) {
    unsigned long hold = millis() - down_press_start;
    unsigned long rep = (hold > 6000) ? 40 :
                        (hold > 3000) ? 70 :
                                       DEBOUNCE;
    if (millis() - last_down > rep) {
      last_down = millis();
      start_frequency -= FREQ_STEP;
      if (start_frequency < MIN_FREQ) start_frequency = MIN_FREQ;
      if (!phase_locked) { updateFreq(start_frequency + UNLOCK_OFFSET); freq_acc = cur_freq; }
    }
  }

  // Lock switch
  bool lsw = digitalRead(LOCK_PIN);
  if (lsw != last_lock) {
    last_lock = lsw;
    if (lsw) {
      phase_locked = true; ph_idx = 0; phase_hist_init = false;
      in_db = false; stb_cnt = 0; int_err = 0; freq_acc = cur_freq;
      last_valid_phase = TARGET_PHASE;
      rejected_cnt = 0;
    } else {
      phase_locked = false; in_db = false; int_err = 0; stb_cnt = 0;
      rejected_cnt = 0;
      uint32_t sf = (locked_freq > 0) ? locked_freq + 1000 : start_frequency + UNLOCK_OFFSET;
      if (sf > MAX_FREQ) sf = MAX_FREQ;
      updateFreq(sf); freq_acc = sf; last_lock = LOW;
    }
  }

  // Phase lock control - 200Hz
  if (millis() - last_upd > 5) {
    last_upd = millis();

    if (TIM2->SR & TIM_SR_CC1IF) {
      // Reading CCR1 automatically clears CC1IF.
      uint16_t cnt_val = (uint16_t)TIM2->CCR1;
      // If the loop fell behind and missed captures, CC1OF is set; it's
      // informational only (CCR1 always holds the most recent capture).
      TIM2->SR &= ~TIM_SR_CC1OF;

      uint32_t arr_plus1 = TIM1->ARR + 1;
      uint32_t ccr1 = TIM1->CCR1;

      // ---- NOISE REJECTION ----
      uint32_t margin = arr_plus1 / NOISE_DIV;
      bool near_reload  = (cnt_val < margin) || (cnt_val > arr_plus1 - margin);
      bool near_compare = (cnt_val > ccr1 - margin) && (cnt_val < ccr1 + margin);

      dbg_cnt_val = cnt_val;
      dbg_rejected = (near_reload || near_compare);

      bool valid = !(near_reload || near_compare);
      float meas_deg = 0.0f;

      if (valid) {
        meas_deg = ((float)cnt_val / (float)arr_plus1) * 360.0f;
        // Wrap around TARGET_PHASE
        while (meas_deg > TARGET_PHASE + 180.0f) meas_deg -= 360.0f;
        while (meas_deg < TARGET_PHASE - 180.0f) meas_deg += 360.0f;

        // Always publish for display, regardless of lock state.
        cur_phase = meas_deg;
        phase_valid = true;
      }

      // Locked-mode control logic
      if (phase_locked && lsw) {
        if (!valid) {
          rejected_cnt++;

          if (rejected_cnt > REJECT_SWEEP_TH) {
            freq_acc += 15.0f;
            int32_t nf = (int32_t)(freq_acc + 0.5f);
            if (nf > (int32_t)MAX_FREQ) { nf = MAX_FREQ; freq_acc = MAX_FREQ; }
            if ((millis() - last_fupd) >= FREQ_UPD_INT) {
              updateFreq((uint32_t)nf);
              last_fupd = millis();
            }
          }
        } else {
          // ---- VALID MEASUREMENT ----
          bool was_rejecting = (rejected_cnt > 5);
          rejected_cnt = 0;

        if (!phase_hist_init || was_rejecting) {
          for (int i = 0; i < PH_N; i++) ph_hist[i] = meas_deg;
          ph_idx = 0; phase_hist_init = true;
          last_err = wrap180(meas_deg - TARGET_PHASE);
          last_valid_phase = meas_deg;
          int_err = 0;
        }

        float fp = filterPhase(meas_deg);

        // Resonance crossing detection
        if (fabs(wrap180(fp - last_valid_phase)) > PHASE_JUMP_TH) {
          phase_locked = false; in_db = false; int_err = 0; stb_cnt = 0;
          rejected_cnt = 0;
          uint32_t sf = start_frequency + UNLOCK_OFFSET + 5000;
          if (sf > MAX_FREQ) sf = MAX_FREQ;
          updateFreq(sf); freq_acc = sf; last_lock = LOW;
          Serial.println(F("!JUMP"));
          return;
        }

        last_valid_phase = fp;
        float fe = wrap180(fp - TARGET_PHASE);
        disp_err = fe;

        if (in_db) {
          if (fabs(fe) > LOCK_EXIT_TH) { in_db = false; stb_cnt = 0; int_err = 0; }
          else {
            stb_cnt++; last_err = fe;
            float db_correction = -fe * 0.08f;
            if (db_correction > 1.0f) db_correction = 1.0f;
            if (db_correction < -1.0f) db_correction = -1.0f;
            freq_acc += db_correction;

            int32_t nf = (int32_t)(freq_acc + 0.5f);
            if (nf < (int32_t)MIN_FREQ) { nf = MIN_FREQ; freq_acc = MIN_FREQ; }
            if (nf > (int32_t)MAX_FREQ) { nf = MAX_FREQ; freq_acc = MAX_FREQ; }

            if (nf != (int32_t)cur_freq && (millis() - last_fupd) >= FREQ_UPD_INT) {
              updateFreq((uint32_t)nf);
              locked_freq = nf;
              last_fupd = millis();
            }
          }
        } else {
          if (fabs(fe) <= LOCK_ENTER_TH) {
            stb_cnt++;
            if (stb_cnt >= DB_STABLE) in_db = true;
            last_err = fe;
          } else {
            stb_cnt = 0;
            float dv = fe - last_err;
            int_err += fe * 0.01f;
            if (int_err > 100.0f) int_err = 100.0f;
            if (int_err < -100.0f) int_err = -100.0f;
            last_err = fe;

            float ae = fabs(fe);
            float gm = (ae > 30.0f) ? 4.0f : (ae > 15.0f) ? 1.0f : 0.15f;
            float dh = -(Kp * gm * fe + Ki * gm * int_err + Kd * gm * dv);

            int32_t mfc = (ae > 30.0f) ? MFC_L : (ae > 15.0f) ? MFC_M : MFC_S;
            if (dh > mfc) dh = mfc;
            if (dh < -mfc) dh = -mfc;
            freq_acc += dh;

            int32_t nf = (int32_t)(freq_acc + 0.5f);
            if (nf < (int32_t)MIN_FREQ) { nf = MIN_FREQ; freq_acc = MIN_FREQ; }
            if (nf > (int32_t)MAX_FREQ) { nf = MAX_FREQ; freq_acc = MAX_FREQ; }

            if (nf != (int32_t)cur_freq && (millis() - last_fupd) >= FREQ_UPD_INT) {
              updateFreq((uint32_t)nf);
              locked_freq = nf;
              last_fupd = millis();
            }
          }
        }
      }  // end valid measurement (else branch of !valid)
      }  // end if (phase_locked && lsw)
    }  // end if (CC1IF)
  }  // end 200Hz tick

  // Debug output - raw phase data every 100ms when locked
  if (debug_phase && phase_locked && (millis() - last_debug > 100)) {
    last_debug = millis();
    uint32_t arr1 = TIM1->ARR + 1;
    float deg = ((float)dbg_cnt_val / (float)arr1) * 360.0f;
    Serial.print(cur_freq);
    Serial.print(' ');
    Serial.print(dbg_cnt_val);
    Serial.print(' ');
    Serial.print(deg, 1);
    Serial.print(dbg_rejected ? F(" nope ") : F("  ok "));
    Serial.print(rejected_cnt);
    Serial.print(F(" tgt="));
    Serial.println(TARGET_PHASE, 1);
  }

  // Display every 250ms
  if (millis() - last_disp > 250) {
    last_disp = millis();
    updateDisplay(disp_err, in_db);
  }

  delay(1);
}
