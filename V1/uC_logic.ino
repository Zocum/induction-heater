#include <Arduino.h>

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
// GATE DRIVE: COMPLEMENTARY OUTPUTS + HARDWARE DEAD TIME.
//   - CH1 (PA8) and its complement CH1N (PB13) form one dead-timed pair.
//   - The TIM1 dead-time generator (BDTR DTG[7:0]) inserts a non-overlap
//     window at every transition: during it BOTH CH1 and CH1N are LOW.
//   - Each GDT primary is driven differentially by CH1 / CH1N (via its own
//     MIC4421/4422 pair). In the DTG window the primary sees 0 V, so the
//     secondary collapses and that leg's high+low gates are both OFF ->
//     genuine within-leg dead time, no shoot-through.
//   - The second half-bridge uses a SECOND GDT wired with its primary
//     leads REVERSED. That makes leg B run 180 deg anti-phase off the same
//     CH1/CH1N pair and gives it the identical dead band. One timer pair
//     protects both half-bridges; no second channel needed (fixed 50% drive).
//   - Set the dead time with DEAD_TIME_NS below. computeDTG() encodes it
//     into DTG[7:0]. Re-tune TARGET_PHASE after changing it: the dead time
//     shifts the CH1 rising edge relative to the TRGO/TIM2 reset by roughly
//     dead-time/period in degrees (~12 deg per 500 ns at 70 kHz).
//
// RELAY (PB0):
//   The relay simply follows the PWM enable state: PWM on -> relay on,
//   PWM off -> relay off. Energized/de-energized together with the drive.
//
// WIRING:
//   PA0  -> Tank current zero-crossing (from LM339 after CT) [TIM2_CH1]
//   PA8  -> PWM output, leg drive                            [TIM1_CH1]
//   PB13 -> Complementary PWM output, leg drive              [TIM1_CH1N]
//   PB0  -> Relay driver (follows PWM state)
//   PB1  -> PWM enable switch (active HIGH; PWM off until flipped)
//   PA2  -> Run-timer setpoint pot wiper (analog, 0..3.3V = 0..5 min)
//   PB12 -> Lock switch (active HIGH)   [NOTE: PB12 is also TIM1_BKIN; move
//           the lock switch to a free pin (PB5/PB8/PB9) when you add the
//           hardware overcurrent trip into the break input.]
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

// ===== RELAY (tied to PWM state) =====
#define RELAY_PIN    PB0            // relay control pin
// Most opto-isolated relay boards are ACTIVE-LOW (LOW = energized) -> set to 1.
// A low-side transistor/MOSFET driver is ACTIVE-HIGH -> set to 0.
#define RELAY_ACTIVE_LOW 1
#if RELAY_ACTIVE_LOW
  #define RELAY_ON()  digitalWrite(RELAY_PIN, LOW)
  #define RELAY_OFF() digitalWrite(RELAY_PIN, HIGH)
#else
  #define RELAY_ON()  digitalWrite(RELAY_PIN, HIGH)
  #define RELAY_OFF() digitalWrite(RELAY_PIN, LOW)
#endif

// ===== PWM ENABLE SWITCH =====
// PWM does NOT run at power-up. The gate drivers are held at 0 V until this
// switch is flipped HIGH (active-high, same convention as the lock switch).
// Lets you power the bridge / charge the DC-link first, then start switching.
#define PWM_EN_PIN   PB1
const uint32_t PWM_SOFTSTOP_OFFSET = 7000;  // Hz above resonance before cutting drive
const uint16_t PWM_SOFTSTOP_MS     = 5;     // settle at the higher freq, then MOE off
volatile bool pwm_enabled = false;

// ===== DEAD TIME (complementary outputs) =====
// CH1 = PA8, CH1N = PB13 drive each GDT primary differentially. During the
// DTG window both pins go LOW -> primary = 0 V -> that leg's gates off. The
// second GDT primary is wired REVERSED, so leg B runs 180 deg anti-phase off
// the same pair and inherits the same dead band. One pair protects both
// half-bridges.
//   Start at 500 ns; trim against the gate scope shot. It must be long enough
//   that the switch node fully commutates to the opposite rail before the
//   incoming gate rises (true ZVS), but not so long the body diode conducts
//   needlessly. For an IGBT build (e.g. FF200R12KE4) raise it to cover the
//   tail current. computeDTG() reaches ~14 us.
#define CH1N_PIN     PB13
// ---- Operating-point dead-time schedule (FF200R12KS4) ----
// DT floors at resonance/lock and ramps toward DT_MAX_NS as we operate above
// resonance. The floor is the SHOOT-THROUGH minimum and is never crossed.
//
// FF200R12KS4 @ Tvj=125C, 200A, +/-15V, Rg=4.7ohm (datasheet rev 3.4):
//   td(off)=550ns, tf=40ns, td(on)=110ns.
//   delay race  = td(off)+tf-td(on) = 480ns  (zero margin)
//   + tail (~150-250ns) + GDT/driver skew + margin  ->  ~800ns floor.
// VERIFY: scope gate-off until collector current (tail included) hits zero, at
// your actual Rg / Vge(off), hot. Trim DT_FLOOR_NS to that + margin.
#define DT_FLOOR_NS   800u            // shoot-through floor (resonance / lock)
#define DT_MAX_NS     1500u           // ceiling (inductive region / acquisition)
#define DT_SLOPE_NS_PER_KHZ  100.0f   // +ns dead time per kHz above resonance
                                      // (100 -> hits ceiling ~7kHz up, matches backoffs)
#define DT_SLEW_NS    40u             // max DT change per scheduler tick (~1 DTG LSB)
#define DT_SCHEDULE_ENABLE 1      // 0 -> fixed DT_FLOOR_NS, no schedule
// DT at which TARGET_PHASE was tuned. The phase compensation is referenced to
// this, so the actual operating point is held as DT moves. If you retune
// TARGET_PHASE at a different fixed DT, set this to that DT.
#define DT_TUNE_NS    800u

volatile uint32_t dead_time_ns = DT_FLOOR_NS;   // current applied dead time
static uint8_t    cur_dtg_byte = 0xFF;          // cached DTG[7:0]
float phase_target_eff = 0.0f;                  // TARGET_PHASE + DT phase offset

// Fixed-time signal-path delay (LM339 prop + CT) and the frequency at which
// TARGET_PHASE was tuned. With DT_TUNE_NS, these let ONE TARGET_PHASE hold the
// same ACTUAL tank phase across any Fres -- coil swaps, load pull -- because the
// fixed delays are converted to degrees at the live frequency.
// 250 ns is the two-coil fit (42 deg @77.5k, 22 deg @24.5k). MEASURE to confirm.
#define SIG_DELAY_NS  1560u
#define F_TUNE_HZ     40500u


// ===== RUN TIMER (analog setpoint) =====
// Pot wiper on POT_PIN sets the locked run time: 0 V..3.3 V -> 0..5 min, with
// a 10 s floor. Below TIMER_OFF_FRAC the timer is disabled (locks indefinitely).
// When the timed cycle expires, the drive moves TIMER_BACKOFF Hz above
// resonance (PWM stays on).
#define POT_PIN      PA2          // timer setpoint pot wiper (analog)
const uint32_t TIMER_MAX_S    = 300;    // full-scale (3.3 V) = 5 minutes
const uint32_t TIMER_MIN_S    = 10;     // smallest non-zero setting
const float    TIMER_OFF_FRAC = 0.02f;  // pot below this -> timer off (indefinite)
const uint32_t TIMER_BACKOFF  = 7000;   // Hz above resonance when timer expires
const uint32_t UNLOCK_BACKOFF = 7000;
// Pot calibration: raw ADC counts at the knob positions you want as the active
// range. Read these off the 's' command (pot=) at each end, then set them a bit
// INSIDE the measured extremes so 0 and full are easy to reach.
const int POT_RAW_LO = 30;              // reading at the "0 / off" position
const int POT_RAW_HI = 4100;            // reading at the "5 min" position
uint32_t timer_setpoint_s = 0;          // armed duration (captured at cycle start)
uint32_t pot_set_s        = 0;          // live pot reading, for display
bool     timer_running = false;
bool     timer_fired   = false;
unsigned long timer_start_ms = 0;

// ===== OLED =====
// I2C1 pins, needed by name so the bus can be bit-banged during recovery.
#define OLED_SCL_PIN PB6
#define OLED_SDA_PIN PB7

// 100 kHz, not the 400 kHz default: on an open-drain bus the noise margin is
// set by how hard the line is held high, and slower edges survive the switch-
// node dV/dt far better. The display has nothing like enough traffic to care.
const uint32_t I2C_CLOCK_HZ       = 100000;
const uint32_t I2C_RECOVER_MIN_MS = 500;    // min spacing between recovery attempts

U8X8_SH1106_128X64_NONAME_HW_I2C u8x8(U8X8_PIN_NONE);

uint16_t i2c_recover_cnt = 0;               // reported by the 's' command
uint16_t i2c_skip_cnt    = 0;
unsigned long last_i2c_recover_ms = 0;

// ---- I2C bus health & recovery -------------------------------------
// WHY THIS EXISTS: every u8x8.print() below is a blocking HAL transfer. If an
// EMI burst desynchronises the SH1106 it holds SDA low waiting to finish a
// byte that will never come, and each print stalls until the HAL times out --
// or forever, if the peripheral latches BUSY. loop() is where the PLL, the run
// timer AND the PWM/lock switch polling live, but TIM1 is hardware and keeps
// switching regardless. A wedged display would therefore leave the bridge
// driving the tank open-loop with a dead front panel. Never touch the bus
// without checking it first.

// Idle I2C lines float high via the pull-ups. Either line low between
// transactions means a slave is wedged mid-byte.
bool i2c_bus_ok() {
  return digitalRead(OLED_SDA_PIN) && digitalRead(OLED_SCL_PIN);
}

// Clock the stuck slave through its remaining bits, then issue a manual STOP
// and re-init the controller (it may have lost its state).
void i2c_bus_recover() {
  i2c_recover_cnt++;
  Wire.end();

  pinMode(OLED_SDA_PIN, INPUT_PULLUP);
  pinMode(OLED_SCL_PIN, OUTPUT_OPEN_DRAIN);
  digitalWrite(OLED_SCL_PIN, HIGH);
  delayMicroseconds(10);
  for (int i = 0; i < 9 && !digitalRead(OLED_SDA_PIN); i++) {
    digitalWrite(OLED_SCL_PIN, LOW);  delayMicroseconds(10);
    digitalWrite(OLED_SCL_PIN, HIGH); delayMicroseconds(10);
  }

  // Manual STOP: SDA low -> SCL high -> SDA released.
  pinMode(OLED_SDA_PIN, OUTPUT_OPEN_DRAIN);
  digitalWrite(OLED_SDA_PIN, LOW);  delayMicroseconds(10);
  digitalWrite(OLED_SCL_PIN, HIGH); delayMicroseconds(10);
  digitalWrite(OLED_SDA_PIN, HIGH); delayMicroseconds(10);

  Wire.begin();
  Wire.setClock(I2C_CLOCK_HZ);
  u8x8.begin();
  u8x8.setFont(u8x8_font_chroma48medium8_r);
}

// ===== TEMPERATURE =====
int16_t currentTempX10 = 0;
unsigned long lastTempRead = 0;
bool tempSensorFound = false;
bool tempConvStarted = false;

// ===== FREQUENCY =====
const uint32_t TCLK = 72000000;
uint32_t start_frequency = 46000;
const uint32_t FREQ_STEP = 100;
const uint32_t MIN_FREQ = 1000, MAX_FREQ = 100000;
const uint32_t UNLOCK_OFFSET = 4000;
// Acquisition always begins this far ABOVE the resonance estimate and walks
// DOWN. It must clear the unstable near-resonance band (empirically ~6 kHz),
// or the loop overshoots the lock point, the measured angle wraps, and it
// runs away to MAX_FREQ. Seed = (last locked freq, else start_frequency) +
// ACQ_OFFSET. start_frequency is assumed to sit near your resonance estimate.
const uint32_t ACQ_OFFSET = 12000;

// Non-blocking backoff ramp for graceful exits (soft-stop / timer / unlock).
// target != 0 => active. pwm_disable_after_ramp defers the soft-stop gate cut
// until the ramp lands. !JUMP stays instant (it's an escape, not a graceful exit).
uint32_t freq_ramp_target = 0;
unsigned long freq_ramp_last_ms = 0;
bool pwm_disable_after_ramp = false;
const float FREQ_RAMP_HZ_PER_MS = 7.0f;   // 7 kHz in ~1 s

// ===== PHASE TARGET =====
// With hardware input capture, the ONLY residual delay between the
// tank-current edge at the pin and the captured CCR1 value is:
//   - LM339 propagation delay (~1.3-2 us typical @ 52 kHz -> ~25-38 deg)
//   - CT/signal conditioning phase shift (usually small)
//   - the gate dead time, which now offsets the CH1 edge from TRGO
// Software/ISR latency is effectively zero. Retune this after flashing and
// after any change to DEAD_TIME_NS.
const float TARGET_PHASE = -122.0f; // Important! Retune for your hardware. See notes above.

const float PHASE_JUMP_TH = 120.0f;
float last_valid_phase = phase_target_eff;

// ===== CONTROL (proportional-only phase tracker) =====
// Frequency is the time-integral of phase, so a single proportional gain on
// the phase error already drives the steady-state error to zero -- no I or D
// term needed. LOCK_K sets the aggressiveness, MAX_STEP caps the per-update
// jump so a stray sample can't yank the frequency, and LOCK_DEADBAND silences
// idle dither once parked on target. Note: the phase-vs-frequency slope is
// steepest near resonance, so the closer you tune TARGET_PHASE toward Fres,
// the gentler LOCK_K has to be -- too hot and it limit-cycles around target.
const float LOCK_K        = 5.0f;   // Hz per degree of phase error (raise=faster)
const float MAX_STEP      = 50.0f;  // max frequency change per update (Hz)
const float LOCK_DEADBAND = 2.0f;   // deg; within this, hold frequency (0 = off)
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
float last_err = 0, int_err = 0, freq_acc = 0;   // last_err/int_err now vestigial

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

// Convert a dead time in ns to the BDTR DTG[7:0] encoding.
// tDTS = 1/72 MHz = 13.89 ns (CKD = 00). Covers ~14 ns .. 14 us across the
// four encoded ranges of RM0008 sec 14.4.18.
static uint8_t computeDTG(uint32_t ns) {
  uint32_t t = (ns * 72u + 500u) / 1000u;                 // dead time in tDTS ticks
  if (t <= 127u) return (uint8_t)t;                       // DTG7=0 : step 1*tDTS
  if (t <= 254u) { uint32_t v = t >> 1; if (v < 64u) v = 64u; if (v > 127u) v = 127u;
                   return (uint8_t)(0x80u | (v - 64u)); }  // 10x   : step 2*tDTS
  if (t <= 504u) { uint32_t v = t >> 3; if (v < 32u) v = 32u; if (v > 63u) v = 63u;
                   return (uint8_t)(0xC0u | (v - 32u)); }  // 110   : step 8*tDTS
  uint32_t v = t >> 4; if (v < 32u) v = 32u; if (v > 63u) v = 63u;
  return (uint8_t)(0xE0u | (v - 32u));                     // 111   : step 16*tDTS
}

// Degrees of phase a dead time represents at frequency f. The complementary
// DTG delays the real CH1 edge by DT relative to the TRGO/TIM2-reset reference
// the phase is measured from, so DT shifts the measured phase by this much.
static inline float dtPhaseDeg(uint32_t dt_ns, uint32_t f_hz) {
  return (float)dt_ns * 1e-9f * (float)f_hz * 360.0f;
}

// Live-write DTG[7:0] without disturbing MOE/OSSI. BDTR LOCK is never set, so
// this is permitted. In the 800..1500ns band DTG lands in the 1x tDTS (~13.9ns)
// range -> monotonic, ~14ns resolution. Skipped if the byte is unchanged.
static inline void applyDeadTime(uint32_t ns) {
  uint8_t dtg = computeDTG(ns);
  if (dtg != cur_dtg_byte) {
    cur_dtg_byte = dtg;
    TIM1->BDTR = (TIM1->BDTR & ~0xFFUL) | dtg;
  }
  dead_time_ns = ns;
}

// Map operating point -> dead time (slew-limited), then refresh the effective
// phase target so the lock point doesn't drift as DT moves. Resonance ref =
// locked_freq once locked, else start_frequency. Only freq ABOVE resonance
// ramps DT; DT is hard-floored, so it can never drop below the shoot-through min.
void updateDeadTimeSchedule() {
  uint32_t target_dt = DT_FLOOR_NS;
#if DT_SCHEDULE_ENABLE
  uint32_t f_res = (locked_freq > 0) ? locked_freq : start_frequency;
  int32_t  f_above = (int32_t)cur_freq - (int32_t)f_res;
  if (f_above < 0) f_above = 0;
  target_dt = DT_FLOOR_NS + (uint32_t)(DT_SLOPE_NS_PER_KHZ * (f_above / 1000.0f) + 0.5f);
  if (target_dt > DT_MAX_NS)   target_dt = DT_MAX_NS;
  if (target_dt < DT_FLOOR_NS) target_dt = DT_FLOOR_NS;   // safety: never sub-floor
#endif
  uint32_t dt = dead_time_ns;
  if      (target_dt > dt) dt += (target_dt - dt > DT_SLEW_NS) ? DT_SLEW_NS : (target_dt - dt);
  else if (target_dt < dt) dt -= (dt - target_dt > DT_SLEW_NS) ? DT_SLEW_NS : (dt - target_dt);
  applyDeadTime(dt);

  // Hold the ACTUAL tank phase fixed as both DT and frequency move. DT and the
  // signal-path delay are both fixed-time, so fold them together and reference
  // to the tuning point (DT_TUNE_NS + SIG_DELAY_NS at F_TUNE_HZ). One TARGET_PHASE
  // now stays valid at any Fres -- no per-coil retune.
  phase_target_eff = TARGET_PHASE
                   + dtPhaseDeg(dead_time_ns + SIG_DELAY_NS, cur_freq)
                   - dtPhaseDeg(DT_TUNE_NS   + SIG_DELAY_NS, F_TUNE_HZ);
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
  RCC->APB2ENR |= RCC_APB2ENR_TIM1EN | RCC_APB2ENR_IOPAEN | RCC_APB2ENR_IOPBEN;

  // PA8 = TIM1_CH1, PB13 = TIM1_CH1N : both alternate-function push-pull 50 MHz
  GPIOA->CRH = (GPIOA->CRH & ~(0xFUL << 0))  | (0xBUL << 0);   // PA8
  GPIOB->CRH = (GPIOB->CRH & ~(0xFUL << 20)) | (0xBUL << 20);  // PB13

  TIM1->CR1 = 0;
  TIM1->CR2 = (0x2 << 4);  // MMS = 010: Update event as TRGO -> TIM2 ITR0
  TIM1->PSC = 0;

  uint32_t arr = (TCLK / freq_hz) - 1;
  TIM1->ARR = arr;
  TIM1->CCR1 = (arr + 1) / 2;

  TIM1->CCMR1 = (0x6 << 4) | TIM_CCMR1_OC1PE;   // PWM mode 1 on CH1, preload

  // Enable CH1 AND its complement CH1N. CC1P = CC1NP = 0 (both active high);
  // the dead-time generator inserts the non-overlap, so during each DTG window
  // CH1 and CH1N are BOTH low -> GDT primary 0 V -> that leg's gates off.
  TIM1->CCER = TIM_CCER_CC1E | TIM_CCER_CC1NE;

  // OIS1 = OIS1N = 0 (default) -> idle state is LOW on both pins.
  // OSSI = 1 -> when MOE = 0 the outputs are actively driven to that idle (0 V),
  // so the gate drivers see a clean 0 V (not Hi-Z) and the gates stay off.
  // DTG[7:0] sets the dead time. MOE is left OFF here: PWM does not run until
  // pwmEnable().
  // Initialise at the floor; updateDeadTimeSchedule() takes over at runtime.
  cur_dtg_byte = computeDTG(DT_FLOOR_NS);
  dead_time_ns = DT_FLOOR_NS;
  TIM1->BDTR = TIM_BDTR_OSSI | cur_dtg_byte;

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

static inline void startFreqRamp(uint32_t to_hz) {
  if (to_hz > MAX_FREQ) to_hz = MAX_FREQ;
  freq_ramp_target = to_hz;
  freq_ramp_last_ms = millis();
}

// ===== PWM OUTPUT ENABLE =====
// MOE (Main Output Enable) gates the TIM1 outputs. With OSSI=1 set in
// TIM1_Init, clearing MOE drives PA8 and PB13 actively LOW (0 V to the gate
// drivers) rather than leaving them Hi-Z. Setting MOE hands the pins back to
// the dead-timed PWM pair. The relay is tied to the PWM state and switches
// with it.
static inline void pwmEnable()  { TIM1->BDTR |= TIM_BDTR_MOE;  pwm_enabled = true;  RELAY_ON();  }
static inline void pwmDisable() { TIM1->BDTR &= ~TIM_BDTR_MOE; pwm_enabled = false; RELAY_OFF(); }

// ===== RUN TIMER HELPERS =====
// Read the pot and convert to a duration in seconds: 0 (off) or [10..300].
uint32_t readTimerPot() {
  uint32_t acc = 0;
  for (int i = 0; i < 16; i++) acc += analogRead(POT_PIN);
  int raw = (int)(acc / 16);
  // Map the calibrated [LO..HI] window to 0..1, clamping outside it so the
  // dead travel past each end just pins to off / full scale.
  float f = (float)(raw - POT_RAW_LO) / (float)(POT_RAW_HI - POT_RAW_LO);
  if (f < 0.0f) f = 0.0f;
  if (f > 1.0f) f = 1.0f;
  if (f < TIMER_OFF_FRAC) return 0;
  uint32_t s = (uint32_t)(f * (float)TIMER_MAX_S + 0.5f);
  if (s < TIMER_MIN_S) s = TIMER_MIN_S;
  if (s > TIMER_MAX_S) s = TIMER_MAX_S;
  return s;
}

// Capture the pot setting and start the countdown. If the pot is at zero,
// the timer is left disabled (lock runs indefinitely).
void armTimer() {
  timer_setpoint_s = readTimerPot();
  timer_fired = false;
  if (timer_setpoint_s > 0) {
    timer_running = true;
    timer_start_ms = millis();
  } else {
    timer_running = false;
  }
}

// ===== DISPLAY =====
void updateDisplay(float err, bool in_db) {
  u8x8.setCursor(0, 0);
  u8x8.print(F("Induction Heater"));

  // Row 1: relay (follows PWM) status
  u8x8.setCursor(0, 1);
  u8x8.print(pwm_enabled ? F("Relay: ON       ") : F("Relay: OFF      "));

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

  // Row 6: run-timer status only.
  u8x8.setCursor(0, 6);
  if (timer_running) {
    long rem = (long)timer_setpoint_s - (long)((millis() - timer_start_ms) / 1000);
    if (rem < 0) rem = 0;
    u8x8.print(F("Run -"));
    if (rem / 60 < 10) u8x8.print('0');
    u8x8.print(rem / 60);
    u8x8.print(':');
    if (rem % 60 < 10) u8x8.print('0');
    u8x8.print(rem % 60);
    u8x8.print(F("     "));
  } else if (timer_fired) {
    u8x8.print(F("Timer done      "));
  } else if (pot_set_s > 0) {
    u8x8.print(F("Set "));
    if (pot_set_s / 60 < 10) u8x8.print('0');
    u8x8.print(pot_set_s / 60);
    u8x8.print(':');
    if (pot_set_s % 60 < 10) u8x8.print('0');
    u8x8.print(pot_set_s % 60);
    u8x8.print(F("       "));
  } else {
    u8x8.print(F("Timer off       "));
  }

  u8x8.setCursor(0, 7);
  if (!pwm_enabled)                u8x8.print(F("PWM OFF         "));
  else if (phase_locked && in_db)  u8x8.print(F("** LOCKED **    "));
  else if (phase_locked)           u8x8.print(F("Tracking...     "));
  else                             u8x8.print(F("PWM ON          "));
}

// The ONLY place loop() is allowed to reach the display. Checks the bus first
// and drops the frame if it is wedged, so a dead OLED costs one skipped
// refresh instead of the control loop. Recovery is attempted at a bounded
// rate -- the bit-bang sequence itself takes ~200 us, cheap enough to sit in
// loop(), but not something to retry every single pass.
void displayTask(float err, bool in_db) {
  if (!i2c_bus_ok()) {
    i2c_skip_cnt++;
    if (millis() - last_i2c_recover_ms > I2C_RECOVER_MIN_MS) {
      last_i2c_recover_ms = millis();
      i2c_bus_recover();
    }
    return;
  }
  updateDisplay(err, in_db);
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
        Serial.print(F(" pwm=")); Serial.print(pwm_enabled ? '1' : '0');
        Serial.print(F(" pot=")); Serial.print(analogRead(POT_PIN));
        Serial.print(F(" set=")); Serial.print(pot_set_s);
        Serial.print(F(" tgt=")); Serial.print(TARGET_PHASE, 1);
        Serial.print(F(" eff=")); Serial.print(phase_target_eff, 1);
        Serial.print(F(" dt="));  Serial.print(dead_time_ns);
        Serial.print(F(" i2crec=")); Serial.print(i2c_recover_cnt);
        Serial.print(F(" i2cskip=")); Serial.println(i2c_skip_cnt);
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
  Serial.print(F("DT floor=")); Serial.print(DT_FLOOR_NS);
  Serial.print(F("ns max="));   Serial.print(DT_MAX_NS);
  Serial.print(F("ns tune="));  Serial.print(DT_TUNE_NS);
  Serial.print(F("ns DTGfloor=0x")); Serial.println(computeDTG(DT_FLOOR_NS), HEX);

  prog_start = millis();

  // If the display came up wedged (or we got here via a reset mid-transaction),
  // free the bus BEFORE the first blocking transfer, or setup() hangs and the
  // bridge never reaches its defined off-state below.
  pinMode(OLED_SDA_PIN, INPUT_PULLUP);
  pinMode(OLED_SCL_PIN, INPUT_PULLUP);
  delayMicroseconds(50);
  if (!i2c_bus_ok()) i2c_bus_recover();

  Wire.begin();
  Wire.setClock(I2C_CLOCK_HZ);
  u8x8.begin();
  u8x8.setFont(u8x8_font_chroma48medium8_r);
  if (i2c_bus_ok()) {
    u8x8.setCursor(0, 3);
    u8x8.print(F("Init..."));
  }

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
  pinMode(PWM_EN_PIN, INPUT_PULLDOWN);   // PWM stays off until this is flipped HIGH

  analogReadResolution(12);              // 0..4095 for the timer pot
  pinMode(POT_PIN, INPUT_ANALOG);

  // Relay output, off at boot (PWM is also off at boot).
  // Preset the level before pinMode so an active-low board doesn't get a
  // brief power-on glitch while the pin is momentarily LOW.
  RELAY_OFF();
  pinMode(RELAY_PIN, OUTPUT);
  RELAY_OFF();

  // No attachInterrupt needed: tank edge is captured in hardware by TIM2_CH1,
  // and the lock switch / buttons are polled in loop().
  TIM1_Init(start_frequency + UNLOCK_OFFSET);
  TIM2_CaptureInit();
  phase_target_eff = TARGET_PHASE;
  updateDeadTimeSchedule();
  if (i2c_bus_ok()) u8x8.clear();
}

// ===================================================================
void loop() {
  static unsigned long last_upd = 0, last_disp = 0;
  static bool last_lock = false, in_db = false, last_pwm = false;
  static uint32_t stb_cnt = 0;
  static float disp_err = 0.0f;

  handleSerial();
  readTemp();

  // Walk cur_freq toward freq_ramp_target at ~7 Hz/ms, non-blocking. On arrival,
  // if a soft-stop deferred the gate cut, do it now.
  if (freq_ramp_target) {
    unsigned long now = millis();
    unsigned long dms = now - freq_ramp_last_ms;
    if (dms) {
      freq_ramp_last_ms = now;
      uint32_t stepHz = (uint32_t)(FREQ_RAMP_HZ_PER_MS * dms + 0.5f);
      bool done = false;
      if (cur_freq < freq_ramp_target) {
        uint32_t nf = cur_freq + stepHz;
        if (nf >= freq_ramp_target) { nf = freq_ramp_target; done = true; }
        updateFreq(nf); freq_acc = cur_freq;
      } else {
        done = true;                       // already at/above target
      }
      if (done) {
        freq_ramp_target = 0;
        if (pwm_disable_after_ramp) { pwm_disable_after_ramp = false; pwmDisable(); }
      }
    }
  }

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
     // in BTN_UP (and mirror in BTN_DOWN):
if (!phase_locked) {
  freq_ramp_target = 0;                        // stop any backoff ramp
  uint32_t nf = cur_freq + FREQ_STEP;          // nudge from where we ARE
  if (nf > MAX_FREQ) nf = MAX_FREQ;
  updateFreq(nf); freq_acc = cur_freq;
  start_frequency = cur_freq;                  // keep base in sync for next lock
}
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
       freq_ramp_target = 0; pwm_disable_after_ramp = false;   // cancel any backoff ramp
      phase_locked = true; ph_idx = 0; phase_hist_init = false;
      // Jump a fixed margin ABOVE the resonance estimate and walk down, so we
      // always acquire from the stable side -- never seed from wherever the
      // knob happens to be parked (that's what let it run away).
      uint32_t fa = (locked_freq > 0 ? locked_freq : start_frequency) + ACQ_OFFSET;
      if (fa > MAX_FREQ) fa = MAX_FREQ;
      updateFreq(fa);
      in_db = false; stb_cnt = 0; int_err = 0; freq_acc = fa;
      last_valid_phase = phase_target_eff;
      rejected_cnt = 0;
      // Start the run timer on a genuine lock engagement (not on an automatic
      // re-lock after a !JUMP, where timer_running is already true).
      if (pwm_enabled && !timer_running) armTimer();
    } else {
      phase_locked = false; in_db = false; int_err = 0; stb_cnt = 0;
      rejected_cnt = 0;
      timer_running = false; timer_fired = false;   // manual unlock cancels the timer
      uint32_t sf = (locked_freq > 0) ? locked_freq + UNLOCK_BACKOFF : start_frequency + UNLOCK_OFFSET;
      locked_freq = 0;
      startFreqRamp(sf); freq_acc = sf; last_lock = LOW;
    }
  }

  // PWM enable switch
  bool psw = digitalRead(PWM_EN_PIN);
  if (psw != last_pwm) {
    last_pwm = psw;
    if (psw) {
      freq_ramp_target = 0; pwm_disable_after_ramp = false;   // cancel pending soft-stop ramp/disable
      // Starting the drive. If the lock switch is already HIGH, kick off the
      // resonance search now; if it's low, just free-run at the current
      // frequency (you can engage the lock switch later to start hunting).
      if (lsw) {
        phase_locked = true; ph_idx = 0; phase_hist_init = false;
        uint32_t fa = (locked_freq > 0 ? locked_freq : start_frequency) + ACQ_OFFSET;
        if (fa > MAX_FREQ) fa = MAX_FREQ;
        updateFreq(fa);
        in_db = false; stb_cnt = 0; int_err = 0; freq_acc = fa;
        last_valid_phase = phase_target_eff; rejected_cnt = 0;
        pwmEnable();
        armTimer();             // begin the timed run (no-op if pot at zero)
      } else {
        phase_locked = false;   // stay put at the current frequency
        pwmEnable();
      }
    } else {
     // Soft stop. If locked, ramp ~7 kHz up off resonance over ~1 s, THEN cut
      // the gates when the ramp lands. If not locked, just cut now.
      timer_running = false; timer_fired = false;
      freq_ramp_target = 0; pwm_disable_after_ramp = false;   // clear any stale ramp
      if (phase_locked) {
        phase_locked = false; in_db = false; rejected_cnt = 0;
        startFreqRamp(cur_freq + PWM_SOFTSTOP_OFFSET);
        pwm_disable_after_ramp = true;     // gates cut on ramp completion
      } else {
        pwmDisable();   // not locked -> cut gates now
      }  // gate drivers -> 0 V
    }
  }

  // Run timer, live. Track the pot during a run so the duration can be
  // changed on the fly: turn it up for more time, or down toward zero to end
  // the run now. Cheap cadence so the ADC isn't hammered every loop.
  static unsigned long last_pot_ms = 0;
  if (millis() - last_pot_ms > 150) {
    last_pot_ms = millis();
    pot_set_s = readTimerPot();
    if (timer_running) timer_setpoint_s = pot_set_s;   // follow the knob mid-run
  }

  bool stop_run = false;

  // Natural expiry, or pot dialed to zero (setpoint 0 -> elapsed always >= it).
  if (timer_running && pwm_enabled &&
      (millis() - timer_start_ms >= timer_setpoint_s * 1000UL)) stop_run = true;

  // Execute the stop: park TIMER_BACKOFF Hz above resonance, unlock, end timer.
  // PWM stays on and the relay (tied to PWM) stays energized. Re-lock with the
  // lock switch to run again.
  if (stop_run) {
    if (pwm_enabled && phase_locked) {
      startFreqRamp(cur_freq + TIMER_BACKOFF);   // was: updateFreq(f); freq_acc=f;
      phase_locked = false; in_db = false; int_err = 0; stb_cnt = 0;
      rejected_cnt = 0;
      last_lock = lsw;          // suppress auto re-lock while lock switch held
    }
    timer_running = false;
    timer_fired = true;
    Serial.println(F("!TIMER"));
  }

  // Phase lock control - 200Hz
  if (millis() - last_upd > 5) {
    updateDeadTimeSchedule();
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
        // Wrap around phase_target_eff
        while (meas_deg > phase_target_eff + 180.0f) meas_deg -= 360.0f;
        while (meas_deg < phase_target_eff - 180.0f) meas_deg += 360.0f;

        // Always publish for display, regardless of lock state.
        cur_phase = meas_deg;
        phase_valid = true;
      }

      // Locked-mode control logic (only when PWM is actually driving)
      if (phase_locked && lsw && pwm_enabled) {
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
          rejected_cnt = 0;

          if (!phase_hist_init) {
            for (int i = 0; i < PH_N; i++) ph_hist[i] = meas_deg;
            ph_idx = 0; phase_hist_init = true;
            last_valid_phase = meas_deg;
          }

          float fp = filterPhase(meas_deg);

          // Resonance crossing detection: a >120 deg jump in the filtered
          // phase means we fell through resonance to the capacitive side ->
          // bail back above it (the safe, ZVS side).
          if (fabs(wrap180(fp - last_valid_phase)) > PHASE_JUMP_TH) {
            phase_locked = false; in_db = false;
            rejected_cnt = 0;
           uint32_t sf = (locked_freq > 0 ? locked_freq : start_frequency) + ACQ_OFFSET;
            if (sf > MAX_FREQ) sf = MAX_FREQ;
            updateFreq(sf); freq_acc = sf; last_lock = LOW;
            Serial.println(F("!JUMP"));
            return;
          }

          last_valid_phase = fp;

          // ---- PROPORTIONAL TRACKER ----
          // Frequency is the integral of phase, so a single proportional gain
          // already drives the steady-state error to zero. The step is clamped
          // (MAX_STEP) so a stray sample can't yank the frequency, and inside
          // the deadband we hold to kill idle dither.
          float err = wrap180(fp - phase_target_eff);
          disp_err = err;
          in_db = (fabs(err) < LOCK_DEADBAND);   // drives the "LOCKED" display

          // Only trust locked_freq once we're genuinely parked in the deadband.
          // Writing it on every tracking step (the old way) captured the
          // acquisition sweep -- so an unlock backed off relative to a runaway
          // value and ratcheted upward each cycle.
          if (in_db) locked_freq = cur_freq;

          // One proportional correction per update interval -- NOT per capture.
          // Captures arrive ~4x faster than FREQ_UPD_INT, so stepping every
          // capture integrated the same error ~4x before the frequency moved,
          // quadrupling the effective gain and driving the limit cycle.
          if (!in_db && (millis() - last_fupd) >= FREQ_UPD_INT) {
            float step = -LOCK_K * err;
            if (step >  MAX_STEP) step =  MAX_STEP;
            if (step < -MAX_STEP) step = -MAX_STEP;
            freq_acc += step;
            if (freq_acc < (float)MIN_FREQ) freq_acc = (float)MIN_FREQ;
            if (freq_acc > (float)MAX_FREQ) freq_acc = (float)MAX_FREQ;

            uint32_t nf = (uint32_t)(freq_acc + 0.5f);
            if (nf != cur_freq) updateFreq(nf);
            last_fupd = millis();
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

  // Display every 250ms -- via displayTask(), never updateDisplay() directly.
  if (millis() - last_disp > 250) {
    last_disp = millis();
    displayTask(disp_err, in_db);
  }

  delay(1);
}