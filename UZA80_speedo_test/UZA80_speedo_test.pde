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

// Speedo outputs
int speedoOutputPin = 10;
int lockoutOutputPin = 11;

// Keep track of time for state transitions
unsigned long lastTime = 0;
unsigned long currentTime = 0;

// Number of pulses per rev on the W58 speedo sensor
int PULSES_REV_W58 = 4;

// Number of pulses per rev on the T56 speedo sensor
int PULSES_REV_T56 = 17;


void manageSpeedo()
{      
    rollingAverage = 17;
  
    // CONTROL
    outputPeriod = rollingAverage * 1000 / PULSES_REV_T56 * PULSES_REV_W58 / 1000; // Removed the extra 1000L as working in millis    
    
    outputPeriod = 5;
    
    // SIGNAL GENERATOR   
    if( (currentTime - lastOutputTime) > outputPeriod  )
    {      
        currentSpeedoOutput = currentSpeedoOutput == LOW ? HIGH : LOW;
              
        digitalWrite(speedoOutputPin, currentSpeedoOutput);
        
        digitalWrite(13, currentSpeedoOutput);
        
        lastOutputTime = currentTime; 
    }
}

void setup()
{   
  // Speedo Output
  pinMode(speedoOutputPin, OUTPUT);
  // Lockout Solenoid
  pinMode(lockoutOutputPin, OUTPUT);
  
  pinMode(13, OUTPUT);
  
  digitalWrite(lockoutOutputPin, HIGH);
  // -----------------------------------------------------------------------
  
  // Serial Comms ----------------------------------------------------------
  Serial.begin(9600);
  Serial.println("UZA80 Exhaust-Speedo Controller : 28 February 2011");
  // -----------------------------------------------------------------------
}

void loop()
{    
  // get the current time for our calculations
  currentTime = millis();
  
  manageSpeedo();
}

