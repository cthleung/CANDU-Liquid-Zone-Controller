/*  
Control the liquid zones of a CANDU reactor.  Initially, the zones are at 50%.
The total reactivity worth of the zones is 7 mk, the maximum rate is 0.14 mk/s.
The model is a one-delayed-neutron-group point kinetics model.
*/


#define LAMBDA_G 0.001   // Neutron generation time in seconds
                         // Equal to neutron lifetime for a critical reactor.
#define LAMBDA_D 0.3     // Delayed neutron group decay constant in Hz.
#define BETA     0.006   // Delayed neutron fraction.
#define DRHO_MAX 0.00014 // Maximum reactivity insertion rate. 1/s
#define RHO_MAX  0.007   // Reactivity worth of the liquid zone controllers.
#define DT       0.002   // Fine timestep in s
#define DTR      0.1     // Time between readings (adjust delay accordingly) in s
#define NCYC     5       // Number of cycles 
#define NTCYC    (int)(DTR/DT) 
                         // Number of time steps per reading cycle
#define NBUF     NCYC*NTCYC    
                         // Total number of time steps.
#define NT0      25      // Starting buffer element
#define P0       127     // Initial power level (50% of 255) 
                         // Same as neutron population or thermal flux.
#define LZC_INIT 511     // Initial zone control level at 50% of 1023
#define C0       BETA*P0/LAMBDA_D/LAMBDA_G
                         // Initial delayed neutron population

#define PWM_PIN  9       // pwm output pin (Reactor power 0-100%)
#define ADC_PIN  A0      // ADC (LZC control) pin
#define CYC_PIN  13      // Beginning and end of cycle. The period on this pin should be DTR seconds

float p[NBUF];           // power level
float pMeas;             // Detected power level
float Cdel;              // Delayed neutron concentration.
float rho;               // Reactivity
float rhoOld;            // Starting reactivity
int it;                  // Time step counter
int iZCL;                // Zone control level
int ipMeas;              // Measured power level

void setup()
{
  Serial.begin(9600);
  pinMode(PWM_PIN, OUTPUT);
  pinMode(CYC_PIN, OUTPUT);
  for(it=0; it<NBUF ; it++) p[it]=P0;     
  pMeas=P0;   
  Cdel=C0;    
  rho=0.;     
  rhoOld=0; 
  randomSeed(analogRead(ADC_PIN));  // set up a random number seed.
}

void loop()
{
  digitalWrite(CYC_PIN,HIGH);
  // get rho, the inserted reactivity.  
  iZCL = analogRead(ADC_PIN); // Zone control level from ADC
  Serial.write(map(iZCL,0,1023,0,127));  // for testing
  rho = (float)(iZCL-512)/1023.*RHO_MAX;
  
  // Stay within bounds of reactivity and reactivity increase.
  if (rho-rhoOld > DRHO_MAX*DTR) rho=rhoOld+DRHO_MAX*DTR;
  if (rhoOld-rho > DRHO_MAX*DTR) rho=rhoOld-DRHO_MAX*DTR;
  rhoOld = rho; 

  for(it=(NCYC-1)*NTCYC ; it < NCYC*NTCYC ; it++)
  { 
    Cdel = Cdel*(1.-LAMBDA_D*DT)+BETA/LAMBDA_G*DT*p[it-1];  // Recalculate delayed neutrons.
    p[it] = p[it-1]*(1.+DT*(rho-BETA)/LAMBDA_G) + DT*LAMBDA_D*Cdel;  // Recalculate power/neutrons etc.
    pMeas = 0.25*p[it-NTCYC]+0.35*p[it-2*NTCYC]+0.25*p[it-3*NTCYC]+0.15*p[it-4*NTCYC]; // The measurement (delayed)
  }

// Introduce the occasional glitch  (once per 5 seconds on average)
  float glitchFactor = 1.;
  if (random(50) == 1)  glitchFactor = 0.95;

  for (it=NTCYC; it < NCYC*NTCYC; it++) p[it-NTCYC]= glitchFactor*p[it];  // shift all by one reading cycle.

  digitalWrite(CYC_PIN,LOW);  // Check of the time calibration
  delay(92); //wait to end the reading cycle. (the processor is faster than the real time in this model)

  ipMeas = (int)pMeas;
  analogWrite(PWM_PIN,ipMeas);
//  Serial.write(ipMeas);
}
