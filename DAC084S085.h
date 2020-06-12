
#include "msp.h"
#include <stdint.h>

#ifndef DAC084S085_H_
#define DAC084S085_H_

#define IO_A01   00000000  // DAC 1
#define IO_A02   00000001
#define IO_A03   00000010
#define IO_A04   00000011
#define IO_A05   10000000  // DAC 2
#define IO_A06   10000001
#define IO_A07   10000010
#define IO_A08   10000011

void init_DAC084S085(void);

void output_DAC084S085(uint16_t value, uint8_t addr);

void write_DAC084S085(uint32_t d_word, uint8_t addr);

#endif /* DAC084S085_H_ */
