
#include "ADC122S051.h"

/*
 *  Setup SPI settings
 *
 *  SCLK on falling edge, !SYNC pin goes low during transfer
 *
 *
 *  SMU DAC:
 *
 *      P5.7  =  SYNC
 *
 *      P1.5  =  CLK
 *      P1.6  =  TX
 *      P1.7  =  RX
 *
 *
 */



void init_ADC122S051(void)
{
    EUSCI_B0->CTLW0 = EUSCI_B_CTLW0_SWRST;                             // clear settings, go into reset mode
    EUSCI_B0->CTLW0 |= (EUSCI_B_CTLW0_MSB | EUSCI_B_CTLW0_MST);        // MSB/master modes, clock normally low
    EUSCI_B0->CTLW0 |= (EUSCI_B_CTLW0_SYNC | EUSCI_B_CTLW0_UCSSEL_2);  // synchronous, use SMCLK

    EUSCI_B0->BRW = 0x01; // 1/1 clock division


    P10->SEL0 |=  (BIT1 | BIT2 | BIT3);     // selecting UCB3 Mode for pins [10.1 : 10.3]
    P10->SEL1 &= ~(BIT1 | BIT2 | BIT3);
    P10->DIR  |=  (BIT1 | BIT2);            // pins 10.1, 10.2 as output
    P10->OUT  &=  (BIT1 | BIT2);

    P5->DIR |= BIT7;                // P10.0 SYNC
    P5->OUT |= BIT7;                // default high


    EUSCI_B3->CTLW0 &= ~EUSCI_B_CTLW0_SWRST;    // un-reset

}


/*
 *  Writes sent voltage level to DAC
 */
void convert_ADC122S051(uint8_t addr, uint16_t data)
{
    uint8_t control_byte;

    P5->OUT &= ~(BIT7);

    control_byte = (addr << 3) & 0x08;

    // send which input to capture on next track-hold cycle
    while(!(EUSCI_B0->IFG & EUSCI_B_IFG_TXIFG)); // wait for TX buffer availability
    EUSCI_B3->TXBUF = control_byte;
    while(!(EUSCI_B0->IFG & EUSCI_B_IFG_TXIFG)); // wait for TX buffer availability
    EUSCI_B3->TXBUF = 0x00;
    while(!(EUSCI_B0->IFG & EUSCI_B_IFG_TXIFG)); // wait for TX buffer availability
    while(!(EUSCI_B3->IFG & EUSCI_B_IFG_RXIFG)); // wait for transmit to complete


    // read next 16 bits spit out from ADC
    while(!(EUSCI_B0->IFG & EUSCI_B_IFG_TXIFG)); // wait for TX buffer availability
    EUSCI_B3->TXBUF = 0x00;
    while(!(EUSCI_B0->IFG & EUSCI_B_IFG_TXIFG)); // wait for TX buffer availability
    while(!(EUSCI_B0->IFG & EUSCI_B_IFG_RXIFG)); // wait for transmit to complete
    data |= EUSCI_B0->RXBUF;

    data = (data << 8);

    EUSCI_B3->TXBUF = 0x00;
    while(!(EUSCI_B3->IFG & EUSCI_B_IFG_TXIFG)); // wait for TX buffer availability
    while(!(EUSCI_B3->IFG & EUSCI_B_IFG_RXIFG)); // wait for transmit to complete
    data |= EUSCI_B3->RXBUF;
    EUSCI_B3->TXBUF = 0x00;


    data &= 0x00FFFFFF; // 12-bit ADC

    P5->OUT |=  (BIT7);

}


