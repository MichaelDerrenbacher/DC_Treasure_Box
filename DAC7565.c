
#include "DAC7565.h"

/*
 *  Setup SPI settings
 *
 *  SCLK on falling edge, !SYNC pin goes low during transfer
 *
 *
 *  SMU DAC:
 *
 *      P5.0  =  SYNC
 *      P5.1  =  EN
 *      P5.2  =  RST
 *
 *      P1.5  =  CLK
 *      P1.6  =  TX
 *      P1.7  =  RX   not used
 *
 *
 */
void init_DAC7565(void)
{
    EUSCI_B0->CTLW0 = EUSCI_B_CTLW0_SWRST;                               // clear settings, go into reset mode
    EUSCI_B0->CTLW0 |= (EUSCI_B_CTLW0_MSB | EUSCI_B_CTLW0_MST);        // MSB/master modes, clock normally low
    EUSCI_B0->CTLW0 |= (EUSCI_B_CTLW0_SYNC | EUSCI_B_CTLW0_UCSSEL_2);    // synchronous, use SMCLK

    EUSCI_B0->BRW = 0x01; // 1/1 clock division


    P1->SEL0 |=  (BIT5 | BIT6);     // selecting UCB0 Mode for pins 1.5, 1.6
    P1->SEL1 &= ~(BIT5 | BIT6);
    P1->DIR  |=  (BIT5 | BIT6);     // pins 1.5, 1.6 as output
    P1->OUT  &=  (BIT5 | BIT6);

    P5->DIR |= (BIT0 | BIT1 | BIT2);                // P5.0 SYNC, P5.1 EN
    P5->OUT |= (BIT0 | BIT1 | BIT2);                // default high


    EUSCI_B0->CTLW0 &= ~EUSCI_B_CTLW0_SWRST;    // un-reset
}


/*
 *  Writes sent voltage level to DAC
 */
void output_DAC7565(uint16_t value, uint8_t addr)
{
    uint8_t control_byte;
    uint8_t data_low_byte;
    uint8_t data_high_byte;

    //low_byte = addr | ((value & 0x0F) << 4);
    //high_byte = (value >> 4) & 0x3F;


    control_byte = (SMU_LOAD | addr);
    data_high_byte = ((value >> 4) & 0xFF);
    data_low_byte = ((value << 4) & 0xF0);



    P5->OUT &= ~BIT1;   // EN low
    _delay_cycles(10);  // 15ns min delay
    P5->OUT &= ~BIT0;   // SYNC low


    while(!(EUSCI_B0->IFG & EUSCI_B_IFG_TXIFG)); // wait for TX buffer availability
    EUSCI_B0->TXBUF = control_byte;
    while(!(EUSCI_B0->IFG & EUSCI_B_IFG_TXIFG)); // wait for TX buffer availability
    EUSCI_B0->TXBUF = data_high_byte;
    while(!(EUSCI_B0->IFG & EUSCI_B_IFG_TXIFG)); // wait for TX buffer availability
    EUSCI_B0->TXBUF = data_low_byte;
    while(!(EUSCI_B0->IFG & EUSCI_B_IFG_TXIFG)); // wait for TX buffer availability
    while(!(EUSCI_B0->IFG & EUSCI_B_IFG_RXIFG)); // wait for transmit to complete


    P5->OUT |=  (BIT0 | BIT1); // EN/SYNC high

}


/*
 *  Convert and write 32 bit 4 value DWORD to DAC
 */
void write_DAC7565(uint32_t d_word, uint8_t addr)
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

    output_DAC7565(data, addr); // write value to DAC
}

