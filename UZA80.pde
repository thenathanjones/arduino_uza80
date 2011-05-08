
// Exhaust Edge Detector
int exhaustInputPin = 4;
int currentExhaustInput = LOW;
int exhaustPulse = 0;
int exhaustInput1 = LOW;
int exhaustInput2 = LOW;
int exhaustInput3 = LOW;
int exhaustInput4 = LOW;
int lastExhaustInput = LOW;

// T56 Edge Detector
int speedoInputPin = 12; // via LM1815
int currentSpeedoInput = LOW;
int lastSpeedoInput = LOW;
unsigned long lastInputTime = 0;
unsigned long currentInputWidth = 0;
// These keep track of the time between pulses on the speedo sensor
unsigned long inputWidth1 = 0;
unsigned long inputWidth2 = 0;
unsigned long inputWidth3 = 0;
unsigned long inputWidth4 = 0;
// Indicator to work out where in the values for the average to insert this
short speedoPulse = 1;
// This is the rolling average of the above and used for the square wave generation
unsigned long rollingAverage = 0;

// Signal Generator
int currentSpeedoOutput = LOW;
unsigned long lastOutputTime = 0;
unsigned long outputPeriod = 0; 

// Relay Control
int relayOpenPin = 2;
int relayClosePin = 3;

// Speedo outputs
int speedoOutputPin = 10;
int lockoutOutputPin = 11;

// keep track of whether the first cycle has been completed
int firstTimeComplete = 0;

/* States
 * 0 - closed
 * 1 - opening
 * 2 - open
 * 3 - delay
 * 4 - closing
 */
int exhaustState = 0;

// Keep track of time for state transitions
unsigned long lastTime = 0;
unsigned long currentTime = 0;

// How long the Varex takes to open/close
int MOVEMENT_DELAY = 1100L;

// How long to wait before closing when given the signal.
// Just in case we get a call to open in the meantime
int PRE_CLOSE_DELAY = 2000L;

// Number of pulses per rev on the W58 speedo sensor
int PULSES_REV_W58 = 4;

// Number of pulses per rev on the T56 speedo sensor
int PULSES_REV_T56 = 17;

// How long to wait for a new pulse before determining there isn't one coming
int NO_PULSE_WAITTIME = 150L;

void checkFirstPassMuffler()
{
  // make sure it's closed to begin with
  if (firstTimeComplete == 0)
  {
    // turn the relay on to close it
    analogWrite(relayClosePin, 255);

    // keep track of when we started opening it
    lastTime = currentTime;

    // block until it's complete
    while ((currentTime - lastTime) < MOVEMENT_DELAY)
    {
      delay(50);        
      currentTime = millis();
    }

    // stop, we'll assume it's closed now
    analogWrite(relayClosePin, 0);

    // signal we've completed the first cycle
    firstTimeComplete = 1;
  }
}

int filteredExhaustInput()
{
  int reading = digitalRead(exhaustInputPin);
  switch(exhaustPulse)
  {
    case 0:
    exhaustInput1 = reading;
    exhaustPulse++;
    break;
    case 1:
    exhaustInput2 = reading;
    exhaustPulse++;
    break;
    case 2:
    exhaustInput3 = reading;
    exhaustPulse++;
    break;
    case 3:
    exhaustInput4 = reading;
    exhaustPulse = 0;
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

void manageExhaustState()
{
  // get the current request from the Adaptronic or override
  currentExhaustInput = filteredExhaustInput();   

  switch(exhaustState)
  {
  case 0:
    if (currentExhaustInput == LOW)
    {
      analogWrite(13, 0);

      // turn the relay on to open it
      analogWrite(relayOpenPin, 255);

      // keep track of when we started opening it
      lastTime = currentTime;

      // change to opening state
      exhaustState = 1;
      Serial.println("Exhaust Opening");
    }
    break;
  case 1:
    if ((currentTime - lastTime) > MOVEMENT_DELAY)
    {
      // turn the relay off
      analogWrite(relayOpenPin, 0);

      // change to open state
      exhaustState = 2;
      Serial.println("Exhaust Open");
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
      Serial.println("Exhaust Open");
    }
    else if ((currentTime - lastTime) > PRE_CLOSE_DELAY)
    {
      // turn the relay on to close it
      analogWrite(relayClosePin, 255);

      // keep track of when we started closing it
      lastTime = currentTime;

      // change to closing state
      exhaustState = 4;
      Serial.println("Exhaust Closing");
    }
    break;
  case 4:
    if ((currentTime - lastTime) > MOVEMENT_DELAY)
    {
      // turn the relay off
      analogWrite(relayClosePin, 0);

      // change to close state
      exhaustState = 0;
      Serial.println("Exhaust Closed");
    }
    break;     
  }   
}

void manageSpeedo()
{
    // EDGE DETECTOR
    currentSpeedoInput = digitalRead(speedoInputPin);
    if (currentSpeedoInput != lastSpeedoInput)
    {
        currentInputWidth = currentTime - lastInputTime;
        
        switch (speedoPulse)
        {
          case 1:
          inputWidth1 = currentInputWidth;
          break;
          case 2:
          inputWidth2 = currentInputWidth;
          break;
          case 3:
          inputWidth3 = currentInputWidth;
          break;
          case 4:
          inputWidth4 = currentInputWidth;
          break;
        }
        speedoPulse++;
        
        if (speedoPulse == 5)
          speedoPulse = 1;
        
        rollingAverage = (inputWidth1 + inputWidth2 + inputWidth3 + inputWidth4) / 4;
        
        lastSpeedoInput = currentSpeedoInput;
        
        lastInputTime = currentTime;
    }
    else if ((currentTime - lastInputTime) > NO_PULSE_WAITTIME)
    {
      inputWidth1 = inputWidth2 = inputWidth3 = inputWidth4 = 0;
      rollingAverage = 0;
    }
      
    // CONTROL
    outputPeriod = rollingAverage / PULSES_REV_T56 * PULSES_REV_W58; // Removed the extra 1000L as working in millis    
    
    // REVERSE LOCKOUT
    if (outputPeriod == 0)
      digitalWrite(lockoutOutputPin, HIGH);
    else
      digitalWrite(lockoutOutputPin, LOW);      
    
    // SIGNAL GENERATOR   
    if( (currentTime - lastOutputTime) >= outputPeriod  )
    {
      currentSpeedoOutput = currentSpeedoOutput == LOW ? HIGH : LOW;  
              
      digitalWrite(speedoOutputPin, currentSpeedoOutput);
        
      lastOutputTime = currentTime; 
    }
}

void setup()
{   
  // VAREX Controller ------------------------------------------------------  
  // Exhaust Edge Detector
  pinMode(exhaustInputPin, INPUT);
  digitalWrite(exhaustInputPin, HIGH); // this enables a 20k resistor on the input    
  // Relay Control
  pinMode(relayOpenPin, OUTPUT);
  pinMode(relayClosePin, OUTPUT);
  // -----------------------------------------------------------------------

  // T56 -------------------------------------------------------------------
  // Speedo Sensor Input (via LM1815)
  pinMode(speedoInputPin, INPUT);
  digitalWrite(speedoInputPin, LOW); 
  // Reverse Lockout Solenoid
  pinMode(lockoutOutputPin, OUTPUT);
  // Speedo Output
  pinMode(speedoOutputPin, OUTPUT);
  // -----------------------------------------------------------------------
  
  // Serial Comms ----------------------------------------------------------
  Serial.begin(9600);
  Serial.println("UZA80 Exhaust-Speedo Controller : 8th May 2011");
  // -----------------------------------------------------------------------
}

void loop()
{    
  // get the current time for our calculations
  currentTime = millis();
  
  checkFirstPassMuffler();
  
  manageExhaustState();
  
  manageSpeedo();
}

