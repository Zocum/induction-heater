#ifndef IH_DS18B20_H
#define IH_DS18B20_H

#include "config.h"

// Raw 1-Wire bit-bang on DS_PIN. Non-blocking usage pattern:
// start a conversion, come back >=750 ms later and read it.

bool    ds_reset(void);        // true = presence pulse seen
void    ds_start_conv(void);
int16_t ds_read_temp_raw(void);  // raw 1/16 C counts

#endif
