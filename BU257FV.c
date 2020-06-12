
#include "BU257FV.h"

/*
 *  Setup SPI settings
 *
 *  SCLK on rising edge, pulse at end
 *
 *
 *  PS DAC:
 *
 *      P10.4  =  SYNC
 *      P2.3  =  RST
 *
 *      P3.5  =  CLK
 *      P3.6  =  TX
 *      P3.7  =  RX
 *
 */
void init_BU257FV(void)
{
    EUSCI_B2->CTLW0 = EUSCI_B_CTLW0_SWRST;                              // clear settings, go into reset mode
    EUSCI_B2->CTLW0 |= (EUSCI_B_CTLW0_CKPL | EUSCI_B_CTLW0_MST);        // LSB/master modes, clock normally high
    EUSCI_B2->CTLW0 |= (EUSCI_B_CTLW0_SYNC | EUSCI_B_CTLW0_UCSSEL_2);   // synchronous, use SMCLK

    EUSCI_B2->BRW = 0x01; // 1/1 clock division


    P3->SEL0 |=  (BIT5 | BIT6);     // selecting UCB2 Mode for pins 3.5, 3.6
    P3->SEL1 &= ~(BIT5 | BIT6);
    P3->DIR  |=  (BIT5 | BIT6);     // pins 3.5, 3.7 as output
    P3->OUT  &=  (BIT5 | BIT6);

    P10->DIR |=  BIT4;                // P10.4 CS
    P10->OUT |=  BIT4;                // default high

    P2->DIR |= BIT3;                // P2.3 RST
    P2->OUT |= BIT3;                // default high

    EUSCI_B2->CTLW0 &= ~EUSCI_B_CTLW0_SWRST;    // un-reset
}


/*
 *  Writes sent voltage level to DAC
 */
void output_BU257FV(uint16_t value, uint8_t addr)
{
    uint8_t low_byte;
    uint8_t high_byte;

    low_byte = ((addr << 2) | ((value << 6) & 0xF0));
    high_byte = ((value >> 2) & 0xFF);

    P10->OUT &= ~BIT4; // LD low

    while(!(EUSCI_B2->IFG & EUSCI_B_IFG_TXIFG)); // wait for TX buffer availability
    EUSCI_B2->TXBUF = low_byte;
    while(!(EUSCI_B2->IFG & EUSCI_B_IFG_TXIFG)); // wait for TX buffer availability
    EUSCI_B2->TXBUF = high_byte;
    while(!(EUSCI_B2->IFG & EUSCI_B_IFG_TXIFG)); // wait for TX buffer availability
    while(!(EUSCI_B2->IFG & EUSCI_B_IFG_RXIFG)); // wait for transmit to complete


    // Pulse LD (min 50ns)
    P10->OUT |=  BIT4;
    _delay_cycles(5);
    P10->OUT &= ~BIT4;
}


/*
 *  Convert and write 32 bit 4 value DWORD to DAC
 */
void write_BU257FV(uint32_t d_word, uint8_t addr)
{
    int i;
    int data = 0;
    int sig_value = 1000; // MSB is thousands place, (MSB-1) is hundreds place, ...

    for(i = 3; i >= 0; i--)
    {
        data += (0xFF & (d_word >> i*8)) * sig_value;  // scale each number accordingly and add to total
        sig_value /= 10;
    }

    if(data > 1023)  // hard cap on output level (2^10)
    {
        data = 1023;
    }

    output_BU257FV(data, addr); // write value to DAC
}
