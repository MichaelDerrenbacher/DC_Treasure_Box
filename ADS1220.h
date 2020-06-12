
#include "msp.h"
#include <stdint.h>


#ifndef ADS1220_H_
#define ADS1220_H_



/*
#define RESET         00000110  // Reset ADC
#define START         00001000  // Start Conversion
#define RDATA         00010000  // Force read data
#define WREG          01000000  // Write to control reg

#define REG_0         00000000  // Control reg 0
#define REG_1         00000100  // Control reg 1
#define REG_2         00001000  // Control reg 2
#define REG_3         00001100  // Control reg 3

#define NUM_BYTES     00000001  // Send 2 bytes of data
*/

#define RESET         0x06  // Reset ADC
#define START         0x08  // Start Conversion
#define RDATA         0x10  // Force read data
#define WREG          0x40  // Write to control reg

#define REG_0         0x00  // Control reg 0
#define REG_1         0x04  // Control reg 1
#define REG_2         0x08  // Control reg 2
#define REG_3         0x0A  // Control reg 3

#define AIN_1         0x80  //
#define AIN_0         0x90  //
#define AIN_2         0xA0  //
#define AIN_3         0xB0  //

#define NUM_BYTES     0x01  // Send 2 bytes of data


void init_ADS1220(void);

void write_ADS1220(uint8_t data_MSB, uint8_t data_LSB, uint8_t control_byte);

double read_ADS1220(uint32_t data);

double start_ADS1220(uint8_t input);

void prep_ADS1220(uint8_t input);




#endif /* ADS1220_H_ */
