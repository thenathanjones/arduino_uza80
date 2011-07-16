//TCCR1A Timer/Counter 1 Control Register A
#define COM1A1 7
#define COM1A0 6
#define COM1B1 5
#define COM1B0 4
#define WGM11 1
#define WGM10 0

//TCCR1B Timer/Counter 1 Control Register B
#define ICNC1 7
#define ICES1 6
#define WGM13 4
#define WGM12 3
#define CS12 2
#define CS11 1
#define CS10 0

//TIMSK1 Timer/Counter 1 Interrupt Mask Register
#define ICE1 5
#define OCIE1B 2
#define OCIE1A 1
#define TOIE1 0

// SPEEDO_PULSES_T56(17) * W58_GEAR_SENSOR_SIDE(33) / SPEEDO_PULSES_W58(4) / W58_GEAR_BOX_SIDE(11)
unsigned long T56_TO_W58_FACTOR = 1275L;
// 1 / (2000 / 15625)
unsigned long PRESCALER_FACTOR = 7812L; 

int speedoInterrupt = 0; 

int exhaustInputPin = 4;
int relayOpenPin = 7;
int relayClosePin = 3;
int lockoutOutputPin = 11;

volatile unsigned long lastPulseAt = 0;
volatile unsigned long currentPeriod = 0;
unsigned long currentMicros = 0;
unsigned long OCRegisterValue = 2500L;

int currentExhaustInput = HIGH;
int exhaustPulse = 1;
int exhaustInput1 = HIGH;
int exhaustInput2 = HIGH;
int exhaustInput3 = HIGH;
int exhaustInput4 = HIGH;
int lastExhaustInput = HIGH;

/* States
 * 0 - closed
 * 1 - opening  
 * 2 - open
 * 3 - delay
 * 4 - closing
 */
int exhaustState = 0;

// How long the Varex takes to open/close
int MOVEMENT_DELAY = 1100;

// How long to wait before closing when given the signal.
// Just in case we get a call to open in the meantime
int PRE_CLOSE_DELAY = 2000;

// Timestamps for state transitions
unsigned long lastTime = 0;
unsigned long currentTime = 0;

void closeMuffler()
{
  currentTime = millis();
  
  // turn the relay on to close it
  digitalWrite(relayClosePin, HIGH);

  // keep track of when we started opening it
  lastTime = currentTime;

  // block until it's complete
  while ((currentTime - lastTime) < MOVEMENT_DELAY)
  {
    delay(50);        
    currentTime = millis();
  }

  // stop, we'll assume it's closed now
  digitalWrite(relayClosePin, LOW);
}

ISR(TIMER1_COMPA_vect)
{
  OCRegisterValue = currentPeriod * T56_TO_W58_FACTOR / 1000L * PRESCALER_FACTOR / 1000L / 10;
  // Uncomment to test specific values
  // OCRegisterValue = 58L * T56_TO_W58_FACTOR / 1000L * PRESCALER_FACTOR / 1000L / 10;
  OCR1A = OCRegisterValue;
}

void speedoPulsed()
{
  Serial.println("I got one!");
  currentMicros = micros();
  Serial.println(currentMicros);
  currentPeriod = (currentMicros - lastPulseAt);
  Serial.println(micros());
  lastPulseAt = currentMicros;
}

void manageSpeedo()
{
  
}

int filteredExhaustInput()
{
  int reading = digitalRead(exhaustInputPin);
  switch(exhaustPulse)
  {
    case 1:
    exhaustInput1 = reading;
    exhaustPulse++;
    break;
    case 2:
    exhaustInput2 = reading;
    exhaustPulse++;
    break;
    case 3:
    exhaustInput3 = reading;
    exhaustPulse++;
    break;
    case 4:
    exhaustInput4 = reading;
    exhaustPulse = 1;
    break;
  }
  
  if (exhaustInput1 == LOW && exhaustInput2 == LOW && exhaustInput3 == LOW && exhaustInput4 == LOW)
  {
    return LOW;
  }
  else if (exhaustInput1 == HIGH && exhaustInput2 == HIGH && exhaustInput3 == HIGH && exhaustInput4 == HIGH)
  {
    return HIGH;
  }
  else
  {
    return currentExhaustInput;
  }
}

void manageExhaust()
{
  // get the current request from the Adaptronic or override
  currentExhaustInput = filteredExhaustInput();  
  currentTime = millis(); 
  
  switch(exhaustState)
  {
  case 0:
    if (currentExhaustInput == LOW)
    {
      // turn the relay on to open it
      digitalWrite(relayOpenPin, HIGH);

      // keep track of when we started opening it
      lastTime = currentTime;

      // change to opening state
      exhaustState = 1;
    }
    break;
  case 1:
    if ((currentTime - lastTime) > MOVEMENT_DELAY)
    {
      // turn the relay off
      digitalWrite(relayOpenPin, LOW);

      // change to open state
      exhaustState = 2;
    }
    break;
  case 2:      
    if (currentExhaustInput == HIGH)
    { 
      // keep track of when we started got the initial signal
      lastTime = currentTime;

      // change to delay state
      exhaustState = 3;
    }
    break;
  case 3:
    // go back for being told when to close if still open
    if (currentExhaustInput == LOW)
    {
      // change to open state
      exhaustState = 2;
    }
    else if ((currentTime - lastTime) > PRE_CLOSE_DELAY)
    {
      // turn the relay on to close it
      digitalWrite(relayClosePin, HIGH);

      // keep track of when we started closing it
      lastTime = currentTime;

      // change to closing state
      exhaustState = 4;
    }
    break;
  case 4:
    if ((currentTime - lastTime) > MOVEMENT_DELAY)
    {
      // turn the relay off
      digitalWrite(relayClosePin, LOW);

      // change to close state
      exhaustState = 0;
    }
    break;     
  }   
}

void setupVarexControl()
{
  pinMode(relayOpenPin, OUTPUT);
  pinMode(relayClosePin, OUTPUT);
  pinMode(exhaustInputPin, INPUT);
  digitalWrite(exhaustInputPin, HIGH);
}

void setupOutputCompare()
{
  TCCR1A = B00000000
           //  | _BV(COM1A1)    //Uncomment to set as one.
               | _BV(COM1A0)    //Uncomment to set as one.
           //  | _BV(COM1B1)    //Uncomment to set as one.
           //  | _BV(COM1B0)    //Uncomment to set as one.
               | _BV(WGM11)     //Uncomment to set as one.
               | _BV(WGM10)     //Uncomment to set as one.
               ;
  TCCR1B = B00000000
           //  | _BV(ICNC1)     //Uncomment to set as one.
           //  | _BV(ICES1)     //Uncomment to set as one.
               | _BV(WGM13)     //Uncomment to set as one.     
               | _BV(WGM12)     //Uncomment to set as one.
               | _BV(CS12)      //Uncomment to set as one.
           //  | _BV(CS11)      //Uncomment to set as one.
               | _BV(CS10)      //Uncomment to set as one.
               ;
   
   TIMSK1 = B00000000
           //  | _BV(ICIE1)     //Uncomment to set as one.
           //  | _BV(OCIE1B)    //Uncomment to set as one.
               | _BV(OCIE1A)    //Uncomment to set as one.
           //  | _BV(TOIE1)     //Uncomment to set as one.
               ;

   SREG = B10000000; //Global interrrupt enable
}

void setupSpeedoControl()
{
  DDRB = DDRB | B00000010;
  setupOutputCompare();
  OCR1A = OCRegisterValue;
  
  pinMode(lockoutOutputPin, OUTPUT);
    
  // External Interrupt
  attachInterrupt(speedoInterrupt, speedoPulsed, RISING);
}

void setupSerialComms()
{
  Serial.begin(9600);
  Serial.println(" UZA80 Exhaust-Speedo Controller");
}

void setup()
{     
  setupSerialComms();
  
  setupVarexControl();

  closeMuffler();

  setupSpeedoControl();
}

void loop() 
{  
  manageExhaust();
  
  manageSpeedo();
}
