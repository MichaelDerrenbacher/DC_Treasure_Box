
#include "DAC084S085.h"

/*
 *  Setup SPI settings
 *
 *  SCLK on rising edge, !SYNC pin goes low during transfer
 *
 *  DIO DAC:
 *
 *      P7.6   =  SYNC1 (D input)
 *      P7.7   =  SYNC2 (D output)
 *      P10.1  =  CLK
 *      P10.2  =  TX
 *      P10.3  =  RX
 *
 *
 */
void init_DAC084S085(void)
{
    EUSCI_B3->CTLW0 = EUSCI_B_CTLW0_SWRST;                               // clear settings, go into reset mode
    EUSCI_B3->CTLW0 |= (EUSCI_B_CTLW0_MSB | EUSCI_B_CTLW0_MST);        // MSB/master modes, clock normally low
    EUSCI_B3->CTLW0 |= (EUSCI_B_CTLW0_SYNC | EUSCI_B_CTLW0_UCSSEL_2);    // synchronous, use SMCLK

    EUSCI_B3->BRW = 0x01; // 1/1 clock division


    P10->SEL0 |=  (BIT1 | BIT2);     // selecting UCB3 Mode for pins 10.1, 10.2
    P10->SEL1 &= ~(BIT1 | BIT2);
    P10->DIR  |=  (BIT1 | BIT2);     // pins 1.5, 1.6 as output
    P10->OUT  &=  (BIT1 | BIT2);

    P7->DIR |= (BIT6 | BIT7);                // P7.6, P7.7 SYNC
    P7->OUT |= (BIT6 | BIT7);                // default high


    EUSCI_B3->CTLW0 &= ~EUSCI_B_CTLW0_SWRST;    // un-reset
}


/*
 *  Writes sent voltage level to DAC
 */
void output_DAC084S085(uint16_t value, uint8_t addr)
{
    uint8_t low_byte;
    uint8_t high_byte;

    //low_byte = addr | ((value & 0x0F) << 4);
    //high_byte = (value >> 4) & 0x3F;

    high_byte = ((addr << 6) & 0xC0) | 0x20 | ((value >> 4) & 0x0F);
    low_byte =  ((value << 4) & 0xF0);


    if(addr & BIT8)
    {
        P7->OUT &= ~BIT6; // SYNC1 low
    }
    else
    {
        P7->OUT &= ~BIT7; // SYNC2 low
    }

    while(!(EUSCI_B3->IFG & EUSCI_B_IFG_TXIFG)); // wait for TX buffer availability
    EUSCI_B3->TXBUF = high_byte;
    while(!(EUSCI_B3->IFG & EUSCI_B_IFG_TXIFG)); // wait for TX buffer availability
    EUSCI_B3->TXBUF = low_byte;
    while(!(EUSCI_B3->IFG & EUSCI_B_IFG_TXIFG)); // wait for TX buffer availability
    while(!(EUSCI_B3->IFG & EUSCI_B_IFG_RXIFG)); // wait for transmit to complete


    P7->OUT |=  (BIT6 | BIT7); // SYNC1/2 high

}


/*
 *  Convert and write 32 bit 4 value DWORD to DAC
 */
void write_DAC084S085(uint32_t d_word, uint8_t addr)
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

    output_DAC084S085(data, addr); // write value to DAC
}
