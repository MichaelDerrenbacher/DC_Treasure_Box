
#include "msp.h"
#include <stdint.h>


#ifndef DAC7565_H_
#define DAC7565_H_


#define SMU_LOAD  0x10

#define SMU_A01   0x00 //00000000
#define SMU_A02   0x02 //00000010
#define SMU_A03   0x04 //00000100
#define SMU_A04   0x06 //00000110


void init_DAC7565(void);

void output_DAC7565(uint16_t value, uint8_t addr);

void write_DAC7565(uint32_t d_word, uint8_t addr);

#endif /* DAC7565_H_ */
