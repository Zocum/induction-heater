#ifndef IH_OLED_H
#define IH_OLED_H

#include "config.h"
#include "safety.h"

// ===================================================================
// oled: SH1106 over I2C1 with a defensive wrapper (Layer 3).
//
// - I2C clocked at 100 kHz (edge/timing margin over 400 kHz).
// - Before every frame the bus is health-checked by reading the pin
//   states: SDA or SCL held low outside a transaction = wedged slave.
//   The frame is SKIPPED (loop keeps running, buttons stay live) and a
//   rate-limited recovery runs: Wire off, SCL bit-banged 9 pulses to
//   clock the stuck slave out, manual STOP, Wire re-init, display
//   re-init. Counters exposed for the serial 's' command.
// - The STM32duino core's HAL I2C already carries internal timeouts, so
//   a corrupted transaction returns with an error after ~ms instead of
//   spinning forever; this wrapper adds detection + bus recovery on top.
// ===================================================================

typedef struct {
  float err;            // phase error (deg)
  bool  in_db;          // deadband => LOCKED
  bool  timer_running;
  bool  timer_fired;
  uint32_t timer_setpoint_s;
  unsigned long timer_start_ms;
  uint32_t pot_set_s;
  int16_t  temp_x10;
  bool     temp_ok;
  unsigned long prog_start_ms;
} disp_data_t;

void oled_init(void);
void oled_splash(const char *line);
void oled_clear(void);
bool oled_task(const disp_data_t *d);   // throttled internally; false = skipped
uint16_t oled_recover_count(void);
uint16_t oled_skip_count(void);

#endif
