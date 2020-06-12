
#include "msp.h"
#include <stdint.h>

#ifndef BU257FV_H_
#define BU257FV_H_

#define PS_A01   0x02
#define PS_A02   0x03
#define PS_A03   0x05
#define PS_A04   0x06
#define PS_A05   0x08
#define PS_A06   0x09

void init_BU257FV(void);

void output_BU257FV(uint16_t value, uint8_t addr);

void write_BU257FV(uint32_t d_word, uint8_t addr);

#endif /* BU257FV_H_ */
