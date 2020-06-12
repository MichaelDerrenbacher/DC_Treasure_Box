
#include "ADS1220.h"

/*
 *  Setup SPI settings
 *
 *  SCLK on rising edge, !SYNC pin goes low during transfer
 *
 *  AIO ADC:
 *
 *      P10.0  =  SYNC
 *      P10.1  =  CLK
 *      P10.2  =  TX
 *      P10.3  =  RX
 *
 */

uint64_t avg[] = {0, 0, 0, 0, 0, 0, 0, 0};


void init_ADS1220(void)
{
    EUSCI_B3->CTLW0 = EUSCI_B_CTLW0_SWRST;                             // clear settings, go into reset mode
    EUSCI_B3->CTLW0 |= (EUSCI_B_CTLW0_MSB | EUSCI_B_CTLW0_MST);        // MSB/master modes, clock normally low
    EUSCI_B3->CTLW0 |= (EUSCI_B_CTLW0_SYNC | EUSCI_B_CTLW0_UCSSEL_2);  // synchronous, use SMCLK

    EUSCI_B3->BRW = 0x01; // 1/1 clock division


    P10->SEL0 |=  (BIT1 | BIT2 | BIT3);     // selecting UCB3 Mode for pins [10.1 : 10.3]
    P10->SEL1 &= ~(BIT1 | BIT2 | BIT3);
    P10->DIR  |=  (BIT1 | BIT2);            // pins 10.1, 10.2 as output
    P10->OUT  &=  (BIT1 | BIT2);

    P10->DIR |= BIT0;                // P10.0 SYNC
    P10->OUT |= BIT0;                // default high


    EUSCI_B3->CTLW0 &= ~EUSCI_B_CTLW0_SWRST;    // un-reset

    write_ADS1220(0x81, 0x00, (WREG | REG_0 | NUM_BYTES)); // disable PGA :(,

    write_ADS1220(0x50, 0x00, (WREG | REG_2 | NUM_BYTES)); // Use external reference, 50/60Hz rejection

}


/*
 *  Writes sent voltage level to DAC
 */
void write_ADS1220(uint8_t data_MSB, uint8_t data_LSB, uint8_t control_byte)
{
    P10->OUT &= ~(BIT0);

    while(!(EUSCI_B3->IFG & EUSCI_B_IFG_TXIFG)); // wait for TX buffer availability
    EUSCI_B3->TXBUF = control_byte;
    while(!(EUSCI_B3->IFG & EUSCI_B_IFG_TXIFG)); // wait for TX buffer availability
    EUSCI_B3->TXBUF = data_MSB;
    while(!(EUSCI_B3->IFG & EUSCI_B_IFG_TXIFG)); // wait for TX buffer availability
    EUSCI_B3->TXBUF = data_LSB;
    while(!(EUSCI_B3->IFG & EUSCI_B_IFG_TXIFG)); // wait for TX buffer availability
    while(!(EUSCI_B3->IFG & EUSCI_B_IFG_RXIFG)); // wait for transmit to complete


    P10->OUT |=  (BIT0);

}

void prep_ADS1220(uint8_t input)
{
    write_ADS1220(0x81, 0x00, (WREG | REG_0 | NUM_BYTES)); // disable PGA :(,

    write_ADS1220(0x50, 0x00, (WREG | REG_2 | NUM_BYTES)); // Use external reference, 50/60Hz rejection



    if (input == 0x02)
    {
        write_ADS1220((0x01 | AIN_3), 0x00, (WREG | REG_0 | NUM_BYTES));
    }
    else if (input == 0x00)
    {
        write_ADS1220((0x01 | AIN_0), 0x00, (WREG | REG_0 | NUM_BYTES));
    }
    else if (input == 0x01)
    {
        write_ADS1220((0x01 | AIN_1), 0x00, (WREG | REG_0 | NUM_BYTES));
    }
    else
    {
        write_ADS1220((0x01 | AIN_2), 0x00, (WREG | REG_0 | NUM_BYTES));
    }
}

double start_ADS1220(uint8_t input)
{
    int loop;

    if (input == 0x02)
    {
        write_ADS1220((0x01 | AIN_3), 0x00, (WREG | REG_0 | NUM_BYTES));
    }
    else if (input == 0x00)
    {
        write_ADS1220((0x01 | AIN_0), 0x00, (WREG | REG_0 | NUM_BYTES));
    }
    else if (input == 0x01)
    {
        write_ADS1220((0x01 | AIN_1), 0x00, (WREG | REG_0 | NUM_BYTES));
    }
    else
    {
        write_ADS1220((0x01 | AIN_2), 0x00, (WREG | REG_0 | NUM_BYTES));
    }
    for(loop = 0; loop < 4096; loop++);
    //write_ADS1220((0x01 | AIN_3), 0x00, (WREG | REG_0 | NUM_BYTES));
    P10->OUT &= ~(BIT0);


    // Send Start Conversion Signal
    while(!(EUSCI_B3->IFG & EUSCI_B_IFG_TXIFG)); // wait for TX buffer availability
    EUSCI_B3->TXBUF = START;
    while(!(EUSCI_B3->IFG & EUSCI_B_IFG_TXIFG)); // wait for TX buffer availability
    while(!(EUSCI_B3->IFG & EUSCI_B_IFG_RXIFG)); // wait for transmit to complete

    P10->OUT &= ~(BIT0);
    for(loop = 0; loop < 4096; loop++);
    P10->OUT |=  (BIT0);

    // Send read data command
    while(!(EUSCI_B3->IFG & EUSCI_B_IFG_TXIFG)); // wait for TX buffer availability
    EUSCI_B3->TXBUF = RDATA;
    while(!(EUSCI_B3->IFG & EUSCI_B_IFG_TXIFG)); // wait for TX buffer availability
    while(!(EUSCI_B3->IFG & EUSCI_B_IFG_RXIFG)); // wait for transmit to complete

    P10->OUT &= ~(BIT0);
    for(loop = 0; loop < 50000; loop++);
    P10->OUT |=  (BIT0);

    // send 24 sclk edges
    return (read_ADS1220(0x00));
}


double read_ADS1220(uint32_t data)
{
    double voltage;
    static uint8_t i = 0;
    uint8_t j;
    uint64_t send;
    uint64_t total;

    P10->OUT &= ~(BIT0);

    while(!(EUSCI_B3->IFG & EUSCI_B_IFG_TXIFG)); // wait for TX buffer availability
    EUSCI_B3->TXBUF = 0x00;
    while(!(EUSCI_B3->IFG & EUSCI_B_IFG_TXIFG)); // wait for TX buffer availability
    while(!(EUSCI_B3->IFG & EUSCI_B_IFG_RXIFG)); // wait for transmit to complete
    data |= EUSCI_B3->RXBUF;
    data = (data << 8);

    EUSCI_B3->TXBUF = 0x00;
    while(!(EUSCI_B3->IFG & EUSCI_B_IFG_TXIFG)); // wait for TX buffer availability
    while(!(EUSCI_B3->IFG & EUSCI_B_IFG_RXIFG)); // wait for transmit to complete
    data |= EUSCI_B3->RXBUF;
    EUSCI_B3->TXBUF = 0x00;

    data = (data << 8);
    while(!(EUSCI_B3->IFG & EUSCI_B_IFG_TXIFG)); // wait for TX buffer availability
    while(!(EUSCI_B3->IFG & EUSCI_B_IFG_RXIFG)); // wait for transmit to complete
    data |= EUSCI_B3->RXBUF;


    P10->OUT |=  (BIT0);

    //avg[i] = data;
    //if(i++ > 7)
    //{
    //    i = 0;
    //}

    // average
    //total = 0;
    //for(j=0; j<8; j++)
    //{
    //    total += (avg[j] >> 3); // add and divide by 8
    //}

    voltage = ((data / 16777216.0) * 5) * 9 - 10;


    return voltage;

}
