#include "oled.h"
#include "bridge.h"
#include "pll.h"
#include <Wire.h>
#include <U8x8lib.h>

static U8X8_SH1106_128X64_NONAME_HW_I2C u8x8(U8X8_PIN_NONE);

static uint16_t      recover_cnt = 0;
static uint16_t      skip_cnt    = 0;
static unsigned long last_recover_ms = 0;
static unsigned long last_disp_ms    = 0;

// ---- bus health & recovery ----

static bool bus_ok(void) {
  // Idle I2C lines float high via pull-ups. Either line held low between
  // transactions = a slave is wedged mid-byte.
  return digitalRead(OLED_SDA_PIN) && digitalRead(OLED_SCL_PIN);
}

static void bus_recover(void) {
  recover_cnt++;
  Wire.end();

  // Clock the stuck slave through its remaining bits: up to 9 SCL pulses
  // with SDA released, until SDA floats high again.
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
  u8x8.begin();                       // controller may have lost state
  u8x8.setFont(u8x8_font_chroma48medium8_r);
}

void oled_init(void) {
  Wire.begin();
  Wire.setClock(I2C_CLOCK_HZ);
  u8x8.begin();
  u8x8.setFont(u8x8_font_chroma48medium8_r);
}

void oled_splash(const char *line) {
  if (!bus_ok()) return;
  u8x8.setCursor(0, 3);
  u8x8.print(line);
}

void oled_clear(void) { if (bus_ok()) u8x8.clear(); }

uint16_t oled_recover_count(void) { return recover_cnt; }
uint16_t oled_skip_count(void)    { return skip_cnt; }

// ---- frame ----

static void draw(const disp_data_t *d) {
  u8x8.setCursor(0, 0);
  u8x8.print(F("Induction Heater"));

  u8x8.setCursor(0, 1);
  u8x8.print(pwm_enabled ? F("Relay: ON       ") : F("Relay: OFF      "));

  u8x8.setCursor(0, 2);
  u8x8.print(phase_locked ? F("L:") : F("F:"));
  uint32_t khz = cur_freq / 1000, frac = cur_freq % 1000;
  if (khz < 100) u8x8.print(' ');
  u8x8.print(khz); u8x8.print('.'); u8x8.print(frac / 100);
  if (phase_locked) u8x8.print((frac / 10) % 10);
  u8x8.print(F("kHz  "));

  u8x8.setCursor(0, 3);
  u8x8.print(F("T:"));
  if (d->temp_ok) {
    int t = d->temp_x10 / 10;
    if (t < 100) u8x8.print(' ');
    if (t < 10)  u8x8.print(' ');
    u8x8.print(t);
  } else u8x8.print(F(" --"));
  u8x8.print(F("C           "));

  u8x8.setCursor(0, 4);
  if (phase_valid) {
    u8x8.print(F("P:"));
    float p = cur_phase;
    if (p < 0) { u8x8.print('-'); p = -p; } else u8x8.print(' ');
    int pi = (int)(p * 10.0f + 0.5f);
    if (pi < 1000) u8x8.print(' ');
    if (pi < 100)  u8x8.print(' ');
    u8x8.print(pi / 10); u8x8.print('.'); u8x8.print(pi % 10);
    if (phase_locked) {
      u8x8.print(F(" E:"));
      float e = d->err;
      if (e < 0) { u8x8.print('-'); e = -e; } else u8x8.print(' ');
      int ei = (int)(e * 10.0f + 0.5f);
      if (ei < 100) u8x8.print(' ');
      u8x8.print(ei / 10); u8x8.print('.'); u8x8.print(ei % 10);
      u8x8.print(F(" "));
    } else u8x8.print(F("        "));
  } else u8x8.print(F("P: ---          "));

  u8x8.setCursor(0, 5);
  unsigned long el = (millis() - d->prog_start_ms) / 1000;
  u8x8.print(F("Time "));
  if (el / 60 < 10) u8x8.print('0');
  u8x8.print(el / 60); u8x8.print(':');
  if (el % 60 < 10) u8x8.print('0');
  u8x8.print(el % 60);
  u8x8.print(F("     "));

  u8x8.setCursor(0, 6);
  if (d->timer_running) {
    long rem = (long)d->timer_setpoint_s - (long)((millis() - d->timer_start_ms) / 1000);
    if (rem < 0) rem = 0;
    u8x8.print(F("Run -"));
    if (rem / 60 < 10) u8x8.print('0');
    u8x8.print(rem / 60); u8x8.print(':');
    if (rem % 60 < 10) u8x8.print('0');
    u8x8.print(rem % 60);
    u8x8.print(F("     "));
  } else if (d->timer_fired) {
    u8x8.print(F("Timer done      "));
  } else if (d->pot_set_s > 0) {
    u8x8.print(F("Set "));
    if (d->pot_set_s / 60 < 10) u8x8.print('0');
    u8x8.print(d->pot_set_s / 60); u8x8.print(':');
    if (d->pot_set_s % 60 < 10) u8x8.print('0');
    u8x8.print(d->pot_set_s % 60);
    u8x8.print(F("       "));
  } else u8x8.print(F("Timer off       "));

  // Row 7: safety state dominates everything else.
  u8x8.setCursor(0, 7);
  if (safety_state() == SAFETY_FAULT)      u8x8.print(F("FAULT-cycle sw  "));
  else if (safety_wdt_boot() && !pwm_enabled) u8x8.print(F("WDT rst-sw off  "));
  else if (!pwm_enabled)                   u8x8.print(F("PWM OFF         "));
  else if (phase_locked && d->in_db)       u8x8.print(F("** LOCKED **    "));
  else if (phase_locked)                   u8x8.print(F("Tracking...     "));
  else                                     u8x8.print(F("PWM ON          "));
}

bool oled_task(const disp_data_t *d) {
  if (millis() - last_disp_ms < DISPLAY_PERIOD_MS) return true;   // not due yet

  if (!bus_ok()) {
    // Wedged bus: never block on it. Skip the frame, recover at a bounded rate.
    skip_cnt++;
    if (millis() - last_recover_ms > I2C_RECOVER_MIN_MS) {
      last_recover_ms = millis();
      bus_recover();
    }
    return false;
  }

  last_disp_ms = millis();
  draw(d);
  return true;
}
