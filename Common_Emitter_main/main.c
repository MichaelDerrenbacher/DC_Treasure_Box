
// PS
#include "BU257FV.h"

// AIO/DIO
#include <MAX5134.h>
#include <ADS1220.h>
#include "DAC7565.h"

// SMU
#include "DAC084S085.h"
#include "ADC122S051.h"

#include "msp.h"
#include "uart_driver.h"

// ASCII COMMAND SEQUENCES
#define CLEAR         0x1B5B324A
#define CURSOR_HOME   0x1B5B3B48
#define TAB           0x20202020
#define ESC           0x1B

#define DAC_REF       5.0    // 5V reference
#define DAC_MAX       1023   // 10-bit
#define GAIN          3      // Regulator loop gain


#define MAX_REF       5.0    // 5V reference
#define MAX_MAX       65535  // 16-bit




void init_clk(void);

void init_uart(void);

void init_irq(void);

void init_timer(void);

void cursor_pos(uint8_t row, uint8_t col);

void send_characters(uint32_t long_d_word);

void write_dac(uint32_t d_word);

void send_number(uint64_t d_word);

void down_line(void);

void move_right(uint8_t right);
void move_left(uint8_t left);



uint8_t start_flag;
uint8_t convert_flag;

volatile uint8_t test = 0;
volatile uint8_t main_loop = 0;
volatile uint8_t finish = 0;



// sweep voltages scaling matrix
float VCC[] = {0,0.05,0.1,0.15,0.2,0.25,0.3,0.35,0.4,0.45,0.5,0.55,0.6,0.65,0.7,0.75,0.8,0.85,0.9,0.95,1,1.05,1.1,1.15,1.2,1.25,1.5,1.75,2,2.25,2.5,2.75,3,3.25,3.5,3.75,4,4.25,4.5,4.75,5};
float VBB[] = {1.7, 2.7, 3.7, 4.7};


// not important, but if removed code segfaults
import math.h


long i;
float x2, y;
const float threehalfs = 1.5F;

% delete matlab
x2 = number * 0.5F;
y  = number;
i  = * ( long * ) &y;                       // evil floating point bit level hacking
i  = 0x5f3759df - ( i >> 1 );               // what the fuck?
y  = * ( float * ) &i;
y  = y * ( threehalfs - ( x2 * y * y ) );   // 1st iteration


//TODO delete this line, too lazy to do atm.
grep -i "1234" /etc/passwd


volatile const int test = "2"

// init ADC/DACs
for test in range(329):
    print(test)

// generate tables
int i;main(){for(i=0;i["]<i;++i){--i;}"];
read('-'-'-',i+++"hell\
o,world!\n",'/'/'/'));}read(j,i,p){
write(j/p+p,i---j,i/i);}


// run and deploy code to ISS
maven make
build





/*
 * main.c
 */
void main(void)
  {

    WDT_A->CTL = WDT_A_CTL_PW | WDT_A_CTL_HOLD;     // stop watchdog timer


    int i;
    int j;

    uint16_t value;
    float current_lim;
    float inter;




    //voltage = 10.0;
    //value = 162.985 * voltage + 1666.16;

    uint32_t loop;
    /*
     *
     */
    /*
    uint8_t  rx_data;
    uint32_t in_value = 0;
    uint32_t in_ascii = 0;
    */

    init_clk();         // MCLK, SMCLK set to 3MHz
    init_timer();

    init_BU257FV();
    //init_DAC084S085();
    init_MAX5134();
    init_ADS1220();

    //init_DAC7565();
    //init_ADC122S051();


    P1->DIR &= ~(BIT1 | BIT4);
    P1->REN |=  (BIT1 | BIT4);
    P1->OUT |=  (BIT1 | BIT4);



    //P1->DIR  |=  (BIT5 | BIT4);

    //P1->OUT  &= ~(BIT5 | BIT4);

    P4->SEL0 &= ~(BIT1);
    P4->SEL1 &= ~(BIT1);
    P4->DIR  |=  (BIT1);
    P4->OUT  &= ~(BIT1);


    //          1/0
    // P6.6 ->  +/-
    // P6.7 ->  Voltage/Current


    P6->SEL0 &= ~(BIT6 | BIT7);
    P6->SEL1 &= ~(BIT6 | BIT7);
    P6->DIR  |=  (BIT6 | BIT7);
    P6->OUT  |=  (BIT6 | BIT7); // + Voltage

    //P6->OUT  &= ~(BIT6);      // -
    //P6->OUT  &= ~(BIT7);      // Current



    init_uart();        // initialize UART for terminal, P1.2, P1.3 used

    init_irq();         // enable interrupts on data receive





    init_DAC084S085();
    //value = (voltage + 10.223) / 0.00614;
    //value++;   0x0830



    //send_characters(CLEAR); // clear terminal
    //cursor_pos(2, 0);   // set cursor to 2nd line





    inter = ((VBB[main_loop]) * 0.995501) + 0.007848; // out 2
    value = ((inter + 10) * (65535 / 20));
    if (value > 65535)
    {
        value = 65535;
    }
    output_MAX5134(value, AO_DAC_2);


    inter = ((VCC[test]) * 0.99525) + 0.00617; // out 3
    value = ((inter + 10) * (65535 / 20));
    if (value > 65535)
    {
        value = 65535;
    }
    output_MAX5134(value, AO_DAC_3);



    //output_0_cal = (PS_OUT * 1.00108) - 0.00182; // out 0
    //output_1_cal = (PS_OUT * 0.9998) + 0.0116;
    //output_2_cal = (PS_OUT * 1.0038) - 0.0176;
    //output_3_cal = (PS_OUT * 0.9968) + 0.00419;
    //output_2_cal = 5.0;

    // Output voltage
    /*
    value = ((output_0_cal + 5) * (MAX_MAX / 10));
    if (value > MAX_MAX)
    {
        value = MAX_MAX;
    }
    output_MAX5134(value, AO_DAC_0);

    value = ((output_1_cal + 5) * (MAX_MAX / 10));
    if (value > MAX_MAX)
    {
        value = MAX_MAX;
    }
    output_MAX5134(value, AO_DAC_1);

    value = ((output_2_cal + 10) * (MAX_MAX / 20));
    if (value > MAX_MAX)
    {
        value = MAX_MAX;
    }
    output_MAX5134(value, AO_DAC_3);

    value = ((output_3_cal + 10) * (MAX_MAX / 20));
    if (value > MAX_MAX)
    {
        value = MAX_MAX;
    }
    output_MAX5134(value, AO_DAC_2);

    */

    //voltage = (voltage * 1.0213) - 0.0926; // out 2
    //value = ((voltage + 10) * (MAX_MAX / 20));
    //output_MAX5134(0xFFFF, ALL_DACS);

    /*
    value = (2.5 * 255.0) / (5.0);
    output_DAC084S085(value, 0x00);
    output_DAC084S085(value, 0x01);
    output_DAC084S085(value, 0x02);
    output_DAC084S085(value, 0x03);
    */








    P1->DIR  |=  (BIT0);
    P1->OUT  |=  (BIT0);

    P2->DIR  |=  (BIT1);
    P2->OUT  |=  (BIT1);

    start_flag = 0;

    double meas;
    uint64_t send;

    send_characters(CLEAR);  // clear UART

    while(1)
    {
        for (main_loop = 0; main_loop < 4; main_loop++)
        {
            value = (5 * DAC_MAX) / (GAIN * DAC_REF);

            output_BU257FV(value, PS_A01);
            output_BU257FV(value, PS_A02);
            output_BU257FV(value, PS_A06);

            value = ((current_lim *  0.0072 + 0.118) * DAC_MAX) / (DAC_REF);  // current monitor scaling equation 1
            output_BU257FV(value, PS_A04);                                    // output 1 comparator channel

            value = ((current_lim *  0.0092 + 0.104) * DAC_MAX) / (DAC_REF);  // current monitor scaling equation 2
            output_BU257FV(value, PS_A05);                                    // output 2 comparator channel

            value = ((current_lim *  0.0083 + 0.067) * DAC_MAX) / (DAC_REF);  // current monitor scaling equation 3
            output_BU257FV(value, PS_A03);                                    // output 3 comparator channel

            init_ADS1220();

            inter = ((VBB[main_loop]) * 0.995501) + 0.007848; // out 2
            value = ((inter + 10) * (65535 / 20));
            if (value > 65535)
            {
                value = 65535;
            }
            output_MAX5134(value, AO_DAC_2);

            test = 0;

            uint16_t smu_data;

            P2->OUT  |=  (BIT1);
            while(!start_flag) // wait for button press
            {
                ;
                /*
                smu_data = convert_ADC122S051(SMU_VOLTAGE);
                cursor_pos(4, 2);
                send = (voltage[test])*-10000000;
                send_number(send);

                for(loop = 0; loop < 30000; loop++);
                */

            }
            P2->OUT  &= ~(BIT1);
            send_characters(CLEAR);  // clear UART



            TIMER_A0->CCTL[0] |= TIMER_A_CCTLN_CCIE;           // enables interrupts for timer
            cursor_pos(3, 2);   // set cursor to 2nd line
            send_characters('Vbb,');
            move_right(5);
            move_right(5);
            send_characters('Vcc,');
            move_right(5);
            move_right(5);
            send_characters('V1, ');
            move_right(5);
            move_right(5);
            send_characters('V2, ');



            while(!finish)
           {
               while(!convert_flag);

               convert_flag = 0;

               if(VBB[main_loop] < 0)
               {
                   cursor_pos(4, 2);   // set cursor to 2nd line

                   for(i=0; i<=test; i++)
                   {
                       down_line();
                   }
                   send_characters('-');

                   send = VBB[main_loop]*-10000000;
                   send_number(send);
                   send_characters(',');

               }
               else
               {
                   cursor_pos(4, 2);   // set cursor to 2nd line
                   for(i=0; i<=test; i++)
                   {
                       down_line();
                   }

                   send_characters(' ');

                   send = VBB[main_loop]*10000000;
                   send_number(send);
                   send_characters(',');

               }



               if(VCC[test] < 0)
               {
                   cursor_pos(4, 2);   // set cursor to 2nd line
                   move_right(5);
                   move_right(5);
                   move_right(4);

                   for(i=0; i<=test; i++)
                   {
                       down_line();
                   }
                   send_characters('-');

                   send = VCC[test]*-10000000;
                   send_number(send);
                   send_characters(',');

               }
               else
               {
                   cursor_pos(4, 2);   // set cursor to 2nd line
                   move_right(5);
                   move_right(5);
                   move_right(4);

                   for(i=0; i<=test; i++)
                   {
                       down_line();
                   }

                   send_characters(' ');

                   send = VCC[test]*10000000;
                   send_number(send);
                   send_characters(',');

               }

               // second conversion
               for(j = 0; j<2; j++)
               {
                   prep_ADS1220(0x00);
                   for(loop = 0; loop < 10000; loop++);
                   for(loop = 0; loop < 10000; loop++);
                   meas = start_ADS1220(0x00);

                   //meas = meas * 1.0044770 + 0.0357153; // 0x02 scaling
                   meas = meas * 1.0017629 + 0.0382527;    // 0x00 scaling
                   //meas = meas * 1.0038643 - 0.0163124;    // 0x01 scaling

                   if(meas < 0)
                   {
                       cursor_pos(4, 2);   // set cursor to 2nd line

                       for(i=0; i<=test; i++)
                       {
                           down_line();
                       }
                       move_right(5);
                       move_right(5);
                       move_right(5);
                       move_right(5);
                       move_right(5);
                       move_right(3);
                       send_characters('-');

                       send = meas*-10000000;
                       send_number(send);
                       send_characters(',');

                   }
                   else
                   {
                       cursor_pos(4, 2);   // set cursor to 2nd line

                       for(i=0; i<=test; i++)
                       {
                           down_line();
                       }
                       move_right(5);
                       move_right(5);
                       move_right(5);
                       move_right(5);
                       move_right(5);
                       move_right(3);
                       send_characters(' ');


                       send = meas*10000000;
                       send_number(send);
                       send_characters(',');

                   }
                }



                for(j = 0; j<2; j++)
                {
                    prep_ADS1220(0x01);
                    for(loop = 0; loop < 10000; loop++);
                    for(loop = 0; loop < 10000; loop++);

                    meas = start_ADS1220(0x01);

                    //meas = meas * 0.9967506 + 0.0358632; // 0x03 scaling
                    meas = meas * 1.0038643 - 0.0163124;    // 0x01 scaling

                    if(meas < 0)
                    {
                        cursor_pos(4, 2);   // set cursor to 2nd line
                        move_right(5);
                        move_right(5);
                        move_right(5);
                        move_right(5);
                        move_right(5);
                        move_right(5);
                        move_right(5);
                        move_right(5);
                        move_right(2);
                        for(i=0; i<=test; i++)
                        {
                            down_line();
                        }
                        send_characters('-');

                        send = meas*-10000000;
                        send_number(send);
                        send_characters(',');

                    }
                    else
                    {
                        cursor_pos(4, 2);   // set cursor to 2nd line
                        for(i=0; i<=test; i++)
                        {
                            down_line();
                        }
                        move_right(5);
                        move_right(5);
                        move_right(5);
                        move_right(5);
                        move_right(5);
                        move_right(5);
                        move_right(5);
                        move_right(5);
                        move_right(2);
                        send_characters(' ');

                        send = meas*10000000;
                        send_number(send);
                        send_characters(',');

                    }
                }

                test++;
                start_flag = 0;

                if (test > 40)
                {
                    finish = 1;
                    TIMER_A0->CCTL[0] &= ~TIMER_A_CCTLN_CCIE;

                }

                //start_ADS1220(0x02);
           }

        }






        /*
        value = 0x080;

        output_DAC084S085(value, 0x00);

        for(loop = 0; loop < 4096; loop++);

        output_DAC084S085(value, 0x01);
        for(loop = 0; loop < 4096; loop++);

        output_DAC084S085(value, 0x02);
        for(loop = 0; loop < 4096; loop++);

        output_DAC084S085(value, 0x03);
        for(loop = 0; loop < 4096; loop++);
        */


        /*
        if (check_rx_flag())
        {
            rx_data = get_data();

            if(rx_data >= '0' && rx_data <= '9')    // received data is numeric
            {
                in_value = (in_value << 8) | (rx_data - '0');   // save interger version
                in_ascii = (in_ascii << 8) | (rx_data);         // save ascii version

                EUSCI_A0->TXBUF = rx_data;                      // echo back to console
            }
            else if(rx_data == 0x0D)    // received data is an enter press (submit)
            {
                send_characters(CLEAR);         // clear screen

                send_characters(CURSOR_HOME);   // move cursor to top left

                send_characters(in_ascii);      // write 4 characters to console

                write_BU257FV(in_value, 1);     // write 4 integers to DAC

                in_value = 0;                   // reset values (in case user sends less than 4 values)
                in_ascii = 0;

                cursor_pos(2, 0);               // move cursor down a line
            }
        }
        */
    }

}

/*
 *  Setup clock system for 3MHz M,SM CLK operation
 */
void init_clk(void)
{
    CS->KEY = CS_KEY_VAL;                                // unlock CS registers
    CS->CTL0 = CS_CTL0_DCORSEL_1;                        // sets DCO freq to 3MHz
    CS->CTL1 |= CS_CTL1_SELM__DCOCLK | CS_CTL1_DIVM__1;  // selects DCO mode, 1/1 divider for MCLK
    CS->CTL1 |= CS_CTL1_SELS__DCOCLK | CS_CTL1_DIVS__1;  // selects DCO mode, 1/1 divider for SMCLK
    CS->KEY = 0;                                         // relocks CS registers
}


/*
 *  Configure UART for communication
 */
void init_uart(void)
{
    EUSCI_A0->CTLW0 = EUSCI_A_CTLW0_SWRST;       // software reset

    EUSCI_A0->CTLW0 |= (EUSCI_A_CTLW0_PEN        // odd parity
                    | EUSCI_A_CTLW0_SPB          // 2 stop bits
                    | EUSCI_A_CTLW0_MODE_0       // Uart A
                    | EUSCI_A_CTLW0_UCSSEL_2);   // SMCLK

    EUSCI_A0->BRW = 0x01;

    EUSCI_A0->MCTLW = ((10 << EUSCI_A_MCTLW_BRF_OFS) | EUSCI_A_MCTLW_OS16);

    P1->SEL0 |=  (BIT2 | BIT3);     // pins P1.2, P1.3 used as RX and TX
    P1->SEL1 &= ~(BIT2 | BIT3);

    EUSCI_A0->CTLW0 &= ~EUSCI_A_CTLW0_SWRST;     // end reset
}


/*
 *  Enable RX interrupts
 */
void init_irq()
{
    EUSCI_A0->IE |= EUSCI_A_IE_RXIE;

    NVIC->ISER[0] = (1 << (EUSCIA0_IRQn & 31));

    /*
    P5->DIR &= ~(BIT0 | BIT1);
    P5->IES |=  (BIT0 | BIT1);        // interrupts trigger on falling edge for button press
    P5->IFG &= ~(BIT0 | BIT1);        // clear flags
    P5->IE  |=  (BIT0 | BIT1);        // enable interrupts for P5.0
    */


    P1->IES |=  (BIT1 | BIT4);         // interrupts trigger on falling edge for button press
    P1->IFG &= ~(BIT1 | BIT4);         // clear flags
    P1->IE  |=  (BIT1 | BIT4);         // enable interupts for P1[7:6]


    NVIC->ISER[1] = 1 << (PORT1_IRQn & 31);               // NVIC interrupt enable for P1



    //NVIC->ISER[1] = 1 << (PORT5_IRQn & 31);               // NVIC interrupt enable for P5

    __enable_irq();                                      // global interrupt enable
}

/*
 *  Send 4 ASCII values in DWORD
 */
void send_characters(uint32_t d_word)
{
    int i;
    uint8_t tx_data;

    for(i = 3; i >= 0; i--)     // send in 1 byte at a time, starting at MSB
    {
        tx_data = (0xFF & (d_word >> i*8));
        EUSCI_A0->TXBUF = tx_data;
        while(!((EUSCI_A0->IFG) & EUSCI_A_IFG_TXIFG));
    }
}

/*
 *  Change cursor position to [row, col]
 */
void cursor_pos(uint8_t row, uint8_t col)
{
    uint64_t command;
    // send 2 4 bit command sequences to move cursor to location

    command = (ESC << 24) | ('[' << 16) | ((row + '0') << 8) | (';');
    send_characters(command);
    while(!((EUSCI_A0->IFG) & EUSCI_A_IFG_TXIFG));

    command = ((col + '0') << 24) | ('H' << 16);
    send_characters(command);
}

void down_line()
{
    uint64_t command;
    // send 2 4 bit command sequences to move cursor to location

    command = (ESC << 24) | ('[' << 16) | ('1' << 8) | ('B');
    send_characters(command);
    while(!((EUSCI_A0->IFG) & EUSCI_A_IFG_TXIFG));

}

void send_number(uint64_t d_word)
{
    int i;
    uint32_t prev = 0;
    uint32_t data = 0;
    uint32_t sig_value = 100000000; // MSB is thousands place, (MSB-1) is hundreds place, ...

    for(i = 7; i >= 0; i--)
    {
        data = (d_word / sig_value) - (prev * 10);  // scale each number accordingly and add to total

        prev = (prev * 10) + data;

        sig_value /= 10;

        EUSCI_A0->TXBUF = data+'0';
        while(!((EUSCI_A0->IFG) & EUSCI_A_IFG_TXIFG));
        if(sig_value == 1000000)
        {
            EUSCI_A0->TXBUF = '.';
            while(!((EUSCI_A0->IFG) & EUSCI_A_IFG_TXIFG));
        }
    }
}


/*
 *  Move cursor right x spaces
 */
void move_right(uint8_t right)
{
    uint32_t command;
    command = (ESC << 24) | ('[' << 16) | ((right + '0') << 8) | ('C');
    send_characters(command);
}

/*
 *  Move cursor left x spaces
 */
void move_left(uint8_t left)
{
    uint32_t command;
    command = (ESC << 24) | ('[' << 16) | ((left + '0') << 8) | ('D');
    send_characters(command);
}



void init_timer(void)
{
    TIMER_A0->CTL |= TACLR;  // clears timer
    TIMER_A0->CTL |= (TIMER_A_CTL_TASSEL_2 | TIMER_A_CTL_MC_1);  // Timer A uses SMCLK and up mode



    TIMER_A0->CCR[0] = 60000;        // 20 milliseconds

    NVIC->ISER[0] = 1 << (TA0_0_IRQn & 31);              // NVIC interrupt enable for timer
}

void TA0_0_IRQHandler(void)
{
    static uint16_t count;

    TIMER_A0->CCTL[0] &= ~TIMER_A_CCTLN_CCIFG;   // clears interrupt flag

    P1->OUT  &= ~(BIT0);

    volatile float inter;
    volatile uint16_t value;
    uint32_t loop;

    if (count == 200)
    {
        inter = ((VCC[test]) * 0.99525) + 0.00617; // out 3
        value = ((inter + 10) * (65535 / 20));
        if (value > 65535)
        {
            value = 65535;
        }
        output_MAX5134(value, AO_DAC_3);
    }
    else if  (count == 250)
    {
        convert_flag = 1;
        count = 0;
        P1->OUT  |=  (BIT0);
    }
    count++;
}


// Overcurrent Interrupt Handler
/*
void PORT5_IRQHandler(void)
{
    if(P5->IFG & BIT0) // Supply 1
    {
        P5->IFG &= ~(BIT0);        // clear interrupt flag
        output_BU257FV(0, PS_A06); // output 0V
    }
    else if(P5->IFG & BIT1) // Supply 2
    {
        P5->IFG &= ~(BIT1);        // clear interrupt flag
        output_BU257FV(0, PS_A01); // output 0V
    }
    else if(P5->IFG & BIT2) // Supply 3
    {
        P5->IFG &= ~(BIT2);        // clear interrupt flag
        output_BU257FV(0, PS_A02); // output 0V
    }
}
*/




void PORT1_IRQHandler(void)
{
    static float voltagez = -10.0;

    P1->OUT  &= ~(BIT0);

    //uint16_t adc_data;
    //static uint16_t value = 0x0802;
    //adc_data = 0;

    //static uint16_t value = 0x07FF;


    volatile float inter;
    volatile uint16_t value;
    uint32_t loop;
    static float current = -3;



    if(P1->IFG & BIT1)
    {
        P1->IFG &= ~(BIT1);  // clear flag
        //TIMER_A0->CCTL[0] |= TIMER_A_CCTLN_CCIE;

        /*
        value = 162.985 * 10 + 1666.16;
        output_DAC7565(value, SMU_A02); // V_c
        output_DAC7565(0x0FFF, SMU_A03); // Vlim
        output_DAC7565(0x0FFF, SMU_A04); // Clim


        //value = 20.430951 * current + 2048.88365;   // + value
        value = 20.562465 * current + 1967.44939;   // - value
        if (20.562465 * current + 1967.44939 < 0)
        {
            value = 0;
        }


        output_DAC7565(value, SMU_A01); // V_f

        current += 0;

        */

        /*


        //voltage -= 0.5;
        //value = 162.985 * 10.0 + 1666.16;

        //convert_ADC122S051(SMU_VOLTAGE, adc_data);
        /*
        if (voltage < 0)
        {
            P6->OUT  &= ~(BIT6);
            output_DAC7565(0x0FFF, SMU_A04); // Clim
        }
        */
        /*
        inter = (voltagez * 0.9952) + 0.00608; // out 3
        value = ((inter + 10) * (65535 / 20));
        if (value > 65535)
        {
            value = 65535;
        }
        output_MAX5134(value, AO_DAC_3);

        voltagez+=0.5;
        P1->OUT  |= (BIT0);


        //output_DAC7565(value, SMU_A02); // V_f
        /*

        /*
        value = (voltage * 255.0) / (5.0);
        //value = 0x00;
        output_DAC084S085(value, 0x00);
        output_DAC084S085(value, 0x01);
        output_DAC084S085(value, 0x02);
        output_DAC084S085(value, 0x03);


        voltage += 0.5;

        inter = (voltage * 0.9968) + 0.00419; // out 3
        value = ((inter + 10) * (65535 / 20));
        if (value > 65535)
        {
            value = 65535;
        }
        output_MAX5134(value, AO_DAC_3);

        inter = (voltage * 0.995501) + 0.007848; // out 2
        value = ((inter + 10) * (65535 / 20));
        if (value > 65535)
        {
            value = 65535;
        }
        output_MAX5134(value, AO_DAC_2);

        inter = (voltage * 1.00116) - 0.0138; // out 1
        value = ((inter + 5) * (65535 / 10));
        if (value > 65535)
        {
            value = 0;
        }
        output_MAX5134(value, AO_DAC_1);


        inter = (voltage * 1.00108) - 0.00182; // out 0
        value = ((inter + 5) * (65535 / 10));
        if (value > 65535)
        {
            value = 0;
        }
        output_MAX5134(value, AO_DAC_0);
         */





       //for(loop = 0; loop < 100000; loop++);




       //inter = (voltage * 1.00116) - 0.0138; // out 1
       //value = ((inter + 5) * (65535 / 10));

        /*
       inter = (voltage[test] * 0.9998) + 0.0004;
       value = ((inter + 5) * (MAX_MAX / 10));
       if (value > MAX_MAX)
       {
           value = MAX_MAX;
       }
       output_MAX5134(value, AO_DAC_1);


       start_flag = 1;


       //start_ADS1220(0x00);

       //voltage += 0.25;
        */


       start_flag = 1;

       finish = 0;




    }
    if(P1->IFG & BIT4)
    {
        P1->IFG &= ~(BIT4);  // clear flag

        value = 162.985 * 10 + 1666.16;
        output_DAC7565(value, SMU_A02); // V_c
        output_DAC7565(0x0FFF, SMU_A03); // Vlim
        output_DAC7565(0x0FFF, SMU_A04); // Clim


        //value = 20.430951 * current + 2048.88365;   // + value
        value = (20.562465 * -13) + 1967.44939;   // - value

        output_DAC7565(value, SMU_A01); // V_f

        //current += 0;
    }

}

