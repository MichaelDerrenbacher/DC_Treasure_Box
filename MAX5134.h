
#include "msp.h"
#include <stdint.h>

#ifndef MAX5134_H_
#define MAX5134_H_

#define LIN_CONTROL      0x05
#define LIN_BIT          0x0200


#define AO_DAC_1         0x31  // DAC 0
#define AO_DAC_2         0x32  // DAC 1
#define AO_DAC_3         0x34  // DAC 2
#define AO_DAC_0         0x38  // DAC 3

#define ALL_DACS         0x3F

void init_MAX5134(void);

void output_MAX5134(uint16_t value, uint8_t control_byte);

void write_MAX5134(uint32_t d_word, uint8_t control_byte);



#endif /* MAX5134_H_ */
