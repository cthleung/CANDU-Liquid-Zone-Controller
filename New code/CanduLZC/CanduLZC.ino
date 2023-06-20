/*  
Control the liquid zones of a CANDU reactor.  Initially, the zones are at 50%.
The total reactivity worth of the zones is 7 mk, the maximum rate is 0.14 mk/s.
The model is a one-delayed-neutron-group point kinetics model.
*/

#define kappa    0.75    // 
#define Lam      2.0     // s
#define p0       50.     // Reference power
#define pinit    50.     // initial power
#define umax     200.    // Max LZC level
#define u0       50.     // Default
#define dt       0.01    // timestep s

#define PWM_PIN  9       // pwm output pin (Reactor power 0-100%)
#define ADC_PIN  A0      // ADC (LZC control) pin
#define CYC_PIN  13      // Beginning and end of cycle. The period on this pin should be DTR seconds
#define NBUF     100     // buffer length

float p[NBUF+1];         // power level
float u[NBUF+1];         // LZC level
float rhoAve;            // Average reactivity from LZC
float pMeas;             // Detected power level
float glitchFactor;
int it;                  // Time step counter
int iZCL;                // Zone control level
int ipMeas;              // Measured power level

void setup()
{
  Serial.begin(9600);          // For testing
  pinMode(PWM_PIN, OUTPUT);    // This is the output
  pinMode(CYC_PIN, OUTPUT);    // This is the timing pin  (should be 100 Hz)
  for(it=0; it<=NBUF ; it++)   // Initialize all variables
    {p[it]=pinit;
     u[it]=u0;
    }     
  randomSeed(analogRead(ADC_PIN));  // set up a random number seed for the glitchFactor
}

void loop()
{
  digitalWrite(CYC_PIN,HIGH);               // Timing pin.
  iZCL = analogRead(ADC_PIN);               // Zone control level from ADC
  // Serial.write(map(iZCL,0,1023,0,127));  // for testing
  u[NBUF] = (float)(iZCL-512)/1023*umax;    // Convert to actual LZC

// Do the time step.  Note the delay.
  rhoAve = (u[NBUF-10]+ 2*u[NBUF-20]+ 3*u[NBUF-30]+4*u[NBUF-40]+8*u[NBUF-50]+4*u[NBUF-60]+2*u[NBUF-70])/24; 
  p[NBUF] = p[NBUF-1]+dt*(rhoAve/Lam - kappa/Lam*(p[NBUF-1]-p0));

// Introduce the occasional glitch  (once per 5 seconds on average)
  glitchFactor = 1.;
  if (random(500) == 1)  glitchFactor = 0.8+0.04*random(10);

// Shift the buffers by one position, apply the glitchFactor and limit the output to 1 - 99% 
  for(it=0; it< NBUF; it++)
    {u[it]=u[it+1];
     p[it]=glitchFactor*p[it+1];
     if (p[it] <= 1.0) p[it]=1.0;
     if (p[it] >= 99.0) p[it]=99.0;
    }

// Check of the timing.  The delay should be such that the cycle pin frequency becomes 100 Hz.
  digitalWrite(CYC_PIN,LOW);  
  delay(7);

// Change the output pin duty cycle to correspond to the reactor power.
  ipMeas = (int)(p[NBUF]/100.*255.);
  analogWrite(PWM_PIN,ipMeas);
//  Serial.write(ipMeas);  // Testing
}
