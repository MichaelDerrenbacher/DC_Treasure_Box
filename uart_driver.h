
#include "msp.h"
#include <stdint.h>

#ifndef UART_DRIVER_H_
#define UART_DRIVER_H_

uint8_t check_rx_flag(void);
uint8_t get_data(void);
void EUSCIA0_IRQHandler(void);

#endif /* UART_DRIVER_H_ */
