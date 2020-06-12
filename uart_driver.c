
#include "uart_driver.h"

static uint8_t in_data = 0;
static uint8_t driverFlag = 0;

uint8_t check_rx_flag(void)
{
    return driverFlag;  // check if new data has come in
}

uint8_t get_data(void)
{
    driverFlag = 0;  // data has been got
    return in_data;
}

void EUSCIA0_IRQHandler(void)
{
    EUSCI_A0->IFG &= ~EUSCI_A_IFG_RXIFG;

    in_data = EUSCI_A0->RXBUF;  // save RX data

    driverFlag = 1;             // data is here, come and get it!
}

