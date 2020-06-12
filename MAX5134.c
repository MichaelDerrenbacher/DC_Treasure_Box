
#include "MAX5134.h"

/*
 *  Setup SPI settings
 *
 *  SCLK on rising edge, !SYNC pin goes low during transfer
 *
 *
 *
 *   *  AIO DAC:
 *
 *      P7.5   =  SYNC
 *      P10.1  =  CLK
 *      P10.2  =  TX
 *      P10.3  =  RX    not used
 *

 */
void init_MAX5134(void)
{
    int loop;
    EUSCI_B3->CTLW0 = EUSCI_B_CTLW0_SWRST;                               // clear settings, go into reset mode
    EUSCI_B3->CTLW0 |= (EUSCI_B_CTLW0_MSB | EUSCI_B_CTLW0_MST);        // MSB/master modes, clock normally low
    EUSCI_B3->CTLW0 |= (EUSCI_B_CTLW0_SYNC | EUSCI_B_CTLW0_UCSSEL_2);    // synchronous, use SMCLK

    EUSCI_B3->BRW = 0x01; // 1/1 clock division


    P10->SEL0 |=  (BIT1 | BIT2);     // selecting UCB0 Mode for pins 10.1, 10.2
    P10->SEL1 &= ~(BIT1 | BIT2);
    P10->DIR  |=  (BIT1 | BIT2);     // pins 10.1, 10.2 as output
    P10->OUT  &=  (BIT1 | BIT2);

    P7->DIR |= BIT5;                // P7.5 SYNC
    P7->OUT |= BIT5;                // default high


    EUSCI_B3->CTLW0 &= ~EUSCI_B_CTLW0_SWRST;    // un-reset

    output_MAX5134(LIN_BIT, LIN_CONTROL);
    for(loop = 0; loop < 3000; loop++); // delay 10ms (@ 3MHz)
    output_MAX5134(0x0000, LIN_CONTROL);
}


/*
 *  Writes sent voltage level to DAC
 */
void output_MAX5134(uint16_t value, uint8_t control_byte)
{
    uint8_t high_data_byte;
    uint8_t low_data_byte;

    high_data_byte = (value >> 8) & 0xFF;
    low_data_byte =  (value & 0xFF);

    //low_byte = addr | ((value & 0x0F) << 4);
    //high_byte = (value >> 4) & 0x3F;


    P7->OUT &= ~(BIT5);

    while(!(EUSCI_B3->IFG & EUSCI_B_IFG_TXIFG)); // wait for TX buffer availability
    EUSCI_B3->TXBUF = control_byte;
    while(!(EUSCI_B3->IFG & EUSCI_B_IFG_TXIFG)); // wait for TX buffer availability
    EUSCI_B3->TXBUF = high_data_byte;
    while(!(EUSCI_B3->IFG & EUSCI_B_IFG_TXIFG)); // wait for TX buffer availability
    EUSCI_B3->TXBUF = low_data_byte;
    while(!(EUSCI_B3->IFG & EUSCI_B_IFG_TXIFG)); // wait for TX buffer availability
    while(!(EUSCI_B3->IFG & EUSCI_B_IFG_RXIFG)); // wait for transmit to complete


    P7->OUT |=  (BIT5);

}


/*
 *  Convert and write 32 bit 4 value DWORD to DAC
 */
void write_MAX5134(uint32_t d_word, uint8_t control)
{
    int i;
    int data = 0;
    int sig_value = 100; // MSB is thousands place, (MSB-1) is hundreds place, ...

    for(i = 2; i >= 0; i--)
    {
        data += (0xFF & (d_word >> i*8)) * sig_value;  // scale each number accordingly and add to total
        sig_value /= 10;
    }

    if(data > 255)  // hard cap on output level (2^8)
    {
        data = 255;
    }

    output_MAX5134(data, control); // write value to DAC
}
