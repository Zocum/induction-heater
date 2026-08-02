#include "ds18b20.h"

static inline void ds_low(void)     { pinMode(DS_PIN, OUTPUT); digitalWrite(DS_PIN, LOW); }
static inline void ds_release(void) { pinMode(DS_PIN, INPUT_PULLUP); }
static inline uint8_t ds_rd(void)   { return digitalRead(DS_PIN); }

bool ds_reset(void) {
  ds_low();      delayMicroseconds(480);
  ds_release();  delayMicroseconds(70);
  uint8_t p = !ds_rd();
  delayMicroseconds(410);
  return p;
}

static void ds_write_bit(uint8_t b) {
  if (b) { ds_low(); delayMicroseconds(6);  ds_release(); delayMicroseconds(64); }
  else   { ds_low(); delayMicroseconds(60); ds_release(); delayMicroseconds(10); }
}

static uint8_t ds_read_bit(void) {
  ds_low();     delayMicroseconds(3);
  ds_release(); delayMicroseconds(10);
  uint8_t b = ds_rd();
  delayMicroseconds(53);
  return b;
}

static void ds_write(uint8_t d) {
  for (int i = 0; i < 8; i++) { ds_write_bit(d & 1); d >>= 1; }
}

static uint8_t ds_read(void) {
  uint8_t d = 0;
  for (int i = 0; i < 8; i++) if (ds_read_bit()) d |= (1 << i);
  return d;
}

void ds_start_conv(void) { ds_reset(); ds_write(0xCC); ds_write(0x44); }

int16_t ds_read_temp_raw(void) {
  ds_reset(); ds_write(0xCC); ds_write(0xBE);
  uint8_t lo = ds_read(), hi = ds_read();
  return (int16_t)((hi << 8) | lo);
}
