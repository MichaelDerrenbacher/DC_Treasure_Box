
#include "msp.h"
#include <stdint.h>



#ifndef ADC122S051_H_
#define ADC122S051_H_



#define SMU_CURRENT         0x00  // ADC 1
#define SMU_VOLTAGE         0x01  // ADC 2


void init_ADC122S051(void);

void convert_ADC122S051(uint8_t addr, uint16_t data);

//void read_ADC122S051(uint32_t data);

//void start_ADC122S051(uint8_t input);




#endif /* ADC122S051_H_ */
