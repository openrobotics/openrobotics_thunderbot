/*

SEND:

"SETPOINT POSITION IN DEGEREES, E-STOP (1 is killed, 0 is active) \n"

- Position should always be 0-360 degrees, sending values outside of this range will not be corrected
- Robot can physically move from about 32 to 326 degrees.  At about 40 and 310 degrees, limit switches will
  be triggered and activate e-stop. Code continues to run but motor not active.
- To re-enable movement, first, send command with legal position value (i.e. one that is within physical limits),
  then manually turn robot back into range to untrigger limit switch.  Robot will immediately begin to move to
  new setpoint.
- Sending E-STOP of 1 will disable motor.  Sending 0 will re-enable motor and torso will attempt to move to
  latest setpoint.

RECEIVE:

byte1 + byte2 + 255

byte1 = most siginificant byte
byte2 = least significant byte

concatenated, byte1 and byte2 give a 16 bit integer value for current angle in degrees, 0-360

255 is a delimiter

*/



#include <BaseMotorControl.h>

#define STEP_SIZE 3.5 / 0.067


// ------------------------- FORWARD DECLARATIONS ---------------------
void onReceived(String s);
void printDouble( double, unsigned int );
void processTwist( double&, double&, double& );
void logSetup();
void logLoop();

// ---------- ---------- ---------- ---------- ---------- ---------- ---------- ---------- ---------- ---------- ---------- ---------- ---------- //
//                                                             VARIABLE DECLARATIONS                                                              //  
// ---------- ---------- ---------- ---------- ---------- ---------- ---------- ---------- ---------- ---------- ---------- ---------- ---------- //

// -------------------------- //  
//    SERIAL COMMUNICATION    //
// -------------------------- //  

// Variables to handle serial communication
String inputString = "";         // a string to hold incoming data
boolean stringComplete = false;  // whether the string is complete

// intermediate variables for storing results for new position setpoint
double _positionSet;

typedef union {
  float floatingPoint;
  byte binary[4];
} binaryFloat;

// ------------------- //  
//    MOTOR DRIVER     //
// ------------------- //

const int directionPin = 42;
const int stepPin = 44;

//const int velocity = 1000;

const int minVelocity = 4000;           // starting velocity - delay in microseconds
const int maxVelocity = 500;            // top velocity - delay in microseconds
const int accelSteps = 400;              // number of steps taken to get from min to max velocity
const float acceleration = -(maxVelocity - minVelocity) / accelSteps;     // decrease in delay (us) per step
float velocity = minVelocity;
int currentStep = 1;

//  binaryFloat velocity;

const int deadband = 1;

int error = 0;

// -------------- //  
//    ENCODER     //
// -------------- //

const int encoderPin = 3;
const int encoderInt = 1;

double encoderLow = 0;
double encoderHigh = 4095;

unsigned int encoder_duration = 0;
unsigned int encoder_period = 3940;
unsigned int start_time = 0;
unsigned int high_time = 0;
unsigned int end_time = 0;
long encoderPosition = 0;
unsigned int encoder_offset = 265;

boolean killed = false;
unsigned int serial_killed = 0;

// -------------------- //  
//    LIMIT SWITCHES    //
// -------------------- //

boolean leftLimit = false;
boolean rightLimit = false;

int leftLimitPin = 50;      // SET CORRECT PIN
int rightLimitPin = 52;


// ---------- ---------- ---------- ---------- ---------- ---------- ---------- ---------- ---------- ---------- ---------- ---------- ---------- //
//                                                                   VOID SETUP                                                                   //  
// ---------- ---------- ---------- ---------- ---------- ---------- ---------- ---------- ---------- ---------- ---------- ---------- ---------- //

void setup()
{    
  // Initialize serial port
  Serial.begin(9600);
  inputString.reserve(100);
  
  // Set output pins
  pinMode(directionPin, OUTPUT);
  pinMode(stepPin, OUTPUT);
  
  digitalWrite(directionPin, LOW);
  digitalWrite(stepPin, LOW);
  
  // Attach encoder pin ISR (interrupt service routine)
  attachInterrupt(encoderInt, risingEdge, RISING);
  interrupts();
  
  // Setup limit switches
  pinMode(leftLimitPin, INPUT);
  pinMode(rightLimitPin, INPUT);
  
  // Set initial setpoint to current position
  delay(1000);
  getEncoderData();
  _positionSet = encoderPosition;

//velocity.floatingPoint = minVelocity;

  Serial.print("Startup Encoder Value: ");
  Serial.println(encoderPosition);
  Serial.print("Startup setpoint: ");
  Serial.println(_positionSet);
  Serial.println();
  Serial.println("---------------------------------");
  Serial.println();
  
  Serial.write(0xFF);
  //logSetup(); 
}
  
// ---------- ---------- ---------- ---------- ---------- ---------- ---------- ---------- ---------- ---------- ---------- ---------- ---------- //
//                                                                   VOID LOOP                                                                    //  
// ---------- ---------- ---------- ---------- ---------- ---------- ---------- ---------- ---------- ---------- ---------- ---------- ---------- //

void loop()
{ 
  //Serial.println("Alive");
  //if(checkLimits())      // Check limit switches, if neither triggered, returns true)
  //getEncoderData();
  //Serial.println(encoderPosition);
  //checkLimits();
  
  //Serial.println(checkLimits());
  if(checkLimits() && !serial_killed)
  {
    // Calculate current position from latest encoder data
    getEncoderData();
    
    Serial.write(short(velocity) >> 8);
    Serial.write(int(velocity));
    Serial.write(0xFF);
    
    //Serial.print("Encoder: ");
    //Serial.println(encoderPosition);
    //Serial.print("Setpoint: ");
    //Serial.println(_positionSet);
    
    error = encoderPosition - _positionSet;
    
    if(error > deadband)
    {
      digitalWrite(directionPin, LOW);  //need to check direction
      move(error);
      //digitalWrite(stepPin, HIGH);
      //digitalWrite(stepPin, LOW);
      //delayMicroseconds(velocity);
      //Serial.println("Down");
    }
    else if(error < -deadband)
    {
      digitalWrite(directionPin, HIGH);  //need to check direction
      move(-error);
      //digitalWrite(stepPin, HIGH);
      //digitalWrite(stepPin, LOW);
      //delayMicroseconds(velocity);
      //Serial.println("Up");
    }
    else
    {
      delayMicroseconds(velocity);
    }
  }
  
  //delay(500);
  
}

void risingEdge()
{
  //  Timer1.restart();
  start_time = micros();
  attachInterrupt(encoderInt, fallingEdge, FALLING);
}

void fallingEdge()
{
  high_time = micros();
  encoder_duration = high_time - start_time;
  attachInterrupt(encoderInt, risingEdge, RISING);
}


void getEncoderData()
{
  unsigned int up = 0;
  //unsigned int total = 0;

  noInterrupts();
  up = encoder_duration;
  //total = encoder_period;
  interrupts();

  encoderPosition = up * 360.0 / encoder_period + encoder_offset;
  if(encoderPosition > 360)
  {
    encoderPosition = encoderPosition - 360;
  } 
  //Serial.println(up);
  //Serial.println(total);
  //Serial.println(encoderPosition);
  //Serial.println();
  //delay(500);
}

boolean checkLimits()
{
  if(!digitalRead(leftLimitPin))
  {
    return false;
    //killed = true;
  }
  if(!digitalRead(rightLimitPin))
  {
    return false;
    //killed = true;
  }
  
  return true;
}

void move(int error)
{
  int stepsRemaining = (error - deadband) * STEP_SIZE;
  
  if(stepsRemaining > currentStep)
  {
    if(currentStep < accelSteps)
    {
      currentStep++;
      velocity -= acceleration;
    }
  }
  else if(stepsRemaining < currentStep)
  {
    currentStep--;
    velocity += acceleration;
  }
  /*
  Serial.println(STEP_SIZE,6);
  Serial.println(error);
  Serial.println(stepsRemaining);
  Serial.println(currentStep);
  Serial.println(accelSteps);
  Serial.println(acceleration);
  Serial.println(velocity);
  Serial.println();
  */
  //Serial.write((unsigned int)velocity);
  //Serial.println();
  //Serial.write(velocity && 0xFF);
  //Serial.write(velocity.binary,4);
  //Serial.write(0x09);
  //Serial.println("\n");
  
  //Serial.write(0xFF);
  //Serial.write(short(velocity) >> 8);
  //Serial.write(int(velocity));
  //Serial.write(int(encoderPosition));
  ////Serial.println(velocity);
  

  
  digitalWrite(stepPin, HIGH);
  digitalWrite(stepPin, LOW);
  delayMicroseconds(velocity);
  //Serial.println("Step");
}

void onReceived(String s) {
  if (s == "initializing\n") {
    Serial.print(s);
  } else {
    
    int first_comma_index = s.indexOf(',');
    int newline_index = s.indexOf('\n', first_comma_index+1);
    
    // String buffers
    String arg_1 = s.substring( 0, first_comma_index );
    String arg_2 = s.substring( first_comma_index+1, newline_index );
    
    char arg_1_char[arg_1.length()+1];
    char arg_2_char[arg_2.length()+1];
    
    arg_1.toCharArray( arg_1_char, arg_1.length()+1 );
    arg_2.toCharArray( arg_2_char, arg_2.length()+1 );
    
    _positionSet = atof( arg_1_char );
    serial_killed = atoi( arg_2_char );
  }
}

void serialEvent() {
  while (Serial.available()) {
    // get the new byte:
    char inChar = (char)Serial.read(); 
    // add it to the inputString:
    inputString += inChar;
    // if the incoming character is a newline, set a flag
    // so the main loop can do something about it:
    if (inChar == '\n') {
      stringComplete = true;
      
    if (stringComplete) {
      onReceived(inputString);

    // clear the string:
    inputString = "";
      stringComplete = false;
      }  
    } 
  }
}
