#include "io430.h"

#define ON  1
#define OFF 0
#define DELAY 20000

#define ASCII_CR 0x0D
#define ASCII_LF 0x0A

#define BUTTON P1IN_bit.P3

#define GREEN_LED P1OUT_bit.P0 
#define RED_LED   P1OUT_bit.P6

#define NPOINTS 400

#define Kp
#define Ki
#define Kd
#define setpoint

//--------------------------------------------------------
//  Global Variables
//--------------------------------------------------------

const int Kp;
const int Ki;
const int Kd;
const float setpoint;

unsigned char v[400];

//--------------------------------------------------------
//  Miscellaneous Functions
//--------------------------------------------------------

__no_init volatile union
{
  struct
  {
  float max;
  float min;
  float error;
  float prevError;
  float integrator;
  float differentiator;
  float prevValue;
  float Kp;
  float Ki;
  float Kd;
  float T=5e-6;
  float tau=10;
  float Output;
} PIDvar;
void PID_init(PIDvar);
float PID_Update(PIDvar, float setpoint, float Feedback);

}


void PID_init(PIDvar)
{
  PIDvar->integrator = 0.0f;
  PIDvar->prevError = 0.0f;
  PIDvar->differentiator = 0.0f;
  PIDvar->Output = 0.0f;
}

float PID_Update(float setpoint, float Feedback, PIDvar)
{
  float Feedback = ADC10MEM;
  /*Error Signal*/
  float error = setpoint - Feedback;
  
  /*Proportional Term*/
  float Proportional = PIDvar->Kp*error;
  
  /*Integral Term*/
    PIDvar->integrator = PIDvar->integrator + 0.5f*PIDvar->Ki*PIDvar->T*(error-PIDvar->prevError);
  
  /*Derivative Term*/
    PIDvar->differentiator= (2.0f*PIDvar->Kd*(Output-PIDvar->prevValue) + (2.0f*PIDvar->tau-PIDvar->T)*PIDvar->differentiator)/(2.0f*PIDvar->tau+PIDvar->T);
    
  /*To Prevent Saturation*/
  float minInt;
  float maxInt;
  if (PIDvar->max > Proportional)
  {
    maxInt=PIDvar->max - Proportional;
  }
  else
  {
    maxInt=0.0f;
  }
  
  if (PIDvar->min < Proportional)
  {
    minInt=PIDvar->min-Proportional;
  }
  else
  {
    minInt=0.0f;
  }
  
  /*clamp integrator*/
  if (PIDvar->integrator>maxInt)
  {
    PIDvar->integrator=maxInt;
  }
  else if (PIDvar->integrator<minInt)
  {
    PIDvar->integrator=minInt;
  }
  
  /*Output of PID Controller*/
  PIDvar->Output=Proportional + PIDvar->integrator+PIDvar->differentiator;
  
  /*Set Boundary Limits on Output (0V to 5V)*/
  if (PIDvar->Output > PIDvar->max)
  {
    PIDvar->Output=PIDvar->max;
  }
  else if (PIDvar->Output < PIDvar->min)
  {
    PIDvar->Output=PIDvar->min;
  }
  
  /*Storing Error and Previous Measured Value*/
  PIDvar->prevError=error;
  PIDvar->prevValue=Feedback;
  
  return PIDvar->Output;
  
     //PWM OUTPUT
    WDTCTL = WDTPW + WDTHOLD; //Disable the Watchdog time
    P2DIR |= BIT1; //Set pin 2.1 to the output direction.
    P2SEL |= BIT1; //Select pin 2.1 as our PWM output.
    TA1CCR0 = 1000; //Set the period in the Timer A0 Capture/Compare 0 register to 1000 us.
    TA1CCTL1 = OUTMOD_7;
    TA1CCR1 = Output*1000/255; //The period in microseconds that the power is ON. It's half the time, which translates to a 50% duty cycle.
    TA1CTL = TASSEL_2 + MC_1; //TASSEL_2 selects SMCLK as the clock source, and MC_1 tells it to count up to the value in TA0CCR0.
}

void delay (unsigned long d)
{
  while (d--);
}

#pragma vector = PORT1_VECTOR
__interrupt void PORT1_ISR(void) {
    GREEN_LED = OFF;
    P1IFG_bit.P3 = 0;    // clear the interrupt request flag 
} 

//--------------------------------------------------------
//  UART Module
//--------------------------------------------------------

void Init_UART(void)
{
  // initialize the USCI
  // RXD is on P1.1
  // TXD is on P1.2

  // configure P1.1 and P1.2 for secondary peripheral function
  P1SEL_bit.P1  = 1;
  P1SEL2_bit.P1 = 1;
  P1SEL_bit.P2  = 1;
  P1SEL2_bit.P2 = 1;

  // divide by  104 for 9600b with  1MHz clock
  // divide by 1667 for 9600b with 16MHz clock
  // divide by  139 for 115200b with 16MHz clock
  
  UCA0BR1 = 0;
  UCA0BR0 = 9;
  
  // use x16 clock
  UCA0MCTL_bit.UCOS16 = 1;

  // select UART clock source
  UCA0CTL1_bit.UCSSEL1 = 1;
  UCA0CTL1_bit.UCSSEL0 = 0;

  // release UART RESET
  UCA0CTL1_bit.UCSWRST = 0;
}

unsigned char getc(void)
{
  while (!IFG2_bit.UCA0RXIFG);
  return (UCA0RXBUF);
}

void putc(unsigned char c)
{
  while (!IFG2_bit.UCA0TXIFG);
  UCA0TXBUF = c;
}

void puts(char *s)
{
  while (*s) putc(*s++);
}

void newline(void)
{
  putc(ASCII_CR);
  putc(ASCII_LF);
}

void itoa(unsigned int n)
{
  unsigned int i;
  char s[6] = "    0";
  i = 4;
  while (n)
  {
    s[i--] = (n % 10) + '0';
    n = n / 10;
  }
  puts(s);
}

//--------------------------------------------------------
//  ADC Module
//--------------------------------------------------------

void Init_ADC(void)
{
//initialize 10-bit ADC using input channel 4 on P1.4 
// use Mode 2 - Repeat single channel 
ADC10CTL1 = INCH_4 + CONSEQ_2; 
// use P1.4 (channel 4) 
ADC10AE0 |= BIT4; 
// enable analog input channel 4 
//select sample-hold time, multisample conversion and turn on the ADC 
ADC10CTL0 |= ADC10SHT_0 + MSC + ADC10ON; 
// start ADC 
ADC10CTL0 |= ADC10SC + ENC; 
}

void Sample(int n)
{
  int i;
  for (i = 0; i < n; i++)
    v[i] = ADC10MEM;
}

void Send(int n)
{
  int i;
  for (i = 0; i < n; i++)
    putc(v[i]);
}

//--------------------------------------------------------
//  Initialization
//--------------------------------------------------------

void Init(void)
{
  // Stop watchdog timer to prevent time out reset
  WDTCTL = WDTPW + WDTHOLD;

  DCOCTL  = CALDCO_16MHZ;
  BCSCTL1 = CALBC1_16MHZ;
  
  P1REN = 0x08;    // enable output resistor
  P1OUT = 0x08;    // enable P1.3 pullup resistor
  P1DIR = 0x41;    // setup LEDs as output
  P1IE_bit.P3 = 1; // enable interrupts on P1.3 input
  
  //__enable_interrupt();
}

void main(void)
{
  
  Init();
  Init_UART();
  Init_ADC();
 
  while(1)
  {
    getc();
    GREEN_LED = ON;
    Send(NPOINTS);
    GREEN_LED = OFF;
    Sample(NPOINTS);
  }
  
}