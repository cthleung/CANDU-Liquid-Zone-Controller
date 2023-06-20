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

const int Kp;
const int Ki;
const int Kd;
const float setpoint;

unsigned char v[400];

void PID_init(PIDvar)
{
  PIDvar->integrator = 0.0f;
  PIDvar->prevError = 0.0f;
  PIDvar->differentiator = 0.0f;
  PIDvar->Output = 0.0f;
}

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


float PID_Update(float setpoint, float Feedback, PIDvar)
{
  float Feedback = ADC10MEM;
  /*Err*/
  float error = setpoint - Feedback;
  
  /*P*/
  float Proportional = PIDvar->Kp*error;
  
  /*I*/
    PIDvar->integrator = PIDvar->integrator + 0.5f*PIDvar->Ki*PIDvar->T*(error-PIDvar->prevError);
  
  /*D*/
    PIDvar->differentiator= (2.0f*PIDvar->Kd*(Output-PIDvar->prevValue) + (2.0f*PIDvar->tau-PIDvar->T)*PIDvar->differentiator)/(2.0f*PIDvar->tau+PIDvar->T);
    
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
  

  if (PIDvar->integrator>maxInt)
  {
    PIDvar->integrator=maxInt;
  }
  else if (PIDvar->integrator<minInt)
  {
    PIDvar->integrator=minInt;
  }

  PIDvar->Output=Proportional + PIDvar->integrator+PIDvar->differentiator;
  

  if (PIDvar->Output > PIDvar->max)
  {
    PIDvar->Output=PIDvar->max;
  }
  else if (PIDvar->Output < PIDvar->min)
  {
    PIDvar->Output=PIDvar->min;
  }
  

  PIDvar->prevError=error;
  PIDvar->prevValue=Feedback;
  
  return PIDvar->Output;

    WDTCTL = WDTPW + WDTHOLD; 
    P2DIR |= BIT1; 
    P2SEL |= BIT1; 
    TA1CCR0 = 1000; 
    TA1CCTL1 = OUTMOD_7;
    TA1CCR1 = Output*1000/255; 
    TA1CTL = TASSEL_2 + MC_1; 

void delay (unsigned long d)
{
  while (d--);
}

#pragma vector = PORT1_VECTOR
__interrupt void PORT1_ISR(void) {
    GREEN_LED = OFF;
    P1IFG_bit.P3 = 0;  
} 

//--------------------------------------------------------
//  UART Module
//--------------------------------------------------------

void Init_UART(void)
{


  P1SEL_bit.P1  = 1;
  P1SEL2_bit.P1 = 1;
  P1SEL_bit.P2  = 1;
  P1SEL2_bit.P2 = 1;


  
  UCA0BR1 = 0;
  UCA0BR0 = 9;
  

  UCA0MCTL_bit.UCOS16 = 1;


  UCA0CTL1_bit.UCSSEL1 = 1;
  UCA0CTL1_bit.UCSSEL0 = 0;


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



void Init_ADC(void)
{

ADC10CTL1 = INCH_4 + CONSEQ_2; 
 
ADC10AE0 |= BIT4; 

ADC10CTL0 |= ADC10SHT_0 + MSC + ADC10ON; 

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


void Init(void)
{

  WDTCTL = WDTPW + WDTHOLD;

  DCOCTL  = CALDCO_16MHZ;
  BCSCTL1 = CALBC1_16MHZ;
  
  P1REN = 0x08; 
  P1OUT = 0x08;   
  P1DIR = 0x41;    
  P1IE_bit.P3 = 1; 
  

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