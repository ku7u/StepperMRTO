
/*
 This program uses the StepperMRTO library code. 
 The backAndForth routine can be used in a demonstration. It moves the stepper motor back and forth.
 In production mode, the stepper would be commanded by a momentary switch or a remote system command.
 The code has no way of knowing the actual position of the stepper and doesn't need to. It attempts to drive
 the stepper in a known direction, assumes that the move was successful and reports it as such.


  Pin assignments:
   A0 (19)  stepper 1 R/G LED
   A1 (20)  stepper 2 R/G LED
   A2 (21)  stepper 3 R/G LED
   A3 (22)  stepper 4 R/G LED
   A4 (23)  LED mux red
   A5 (24)  LED mux green
   A6 (25)  serial port switch  // TBD likely replace this with pressing switch 1 while reseting
   A7 (26)  A7                  // A6 and A7 are analog input only
   D0  (1)  A+ common           // The two pins D0, D1 are normally used for serial comm. configMode will set them as such
   D1  (2)  B+ common
   D2  (5)  A- stepper 1
   D3  (6)  B- stepper 1
   D4  (7)  A- stepper 2
   D5  (8)  B- stepper 2
   D6  (9)  A- stepper 3
   D7  (10) B- stepper 3
   D8  (11) A- stepper 4
   D9  (12) B- stepper 4
   D10 (13) stepper 1 switch
   D11 (14) stepper 2 switch
   D12 (15) stepper 3 switch
   D13 (16) stepper 4 switch
*/

#include <Arduino.h>
#include "StepperMRTO.h"

const int numSteppers = 4;
const int stepsPerRevolution = 20;  //number of steps per revolution       
bool direction = true;              //when true travels in one direction, when false the other
unsigned long lastStartTime = 0;
unsigned long timeStamp;
bool configMode = false;            // on reset, test pin A6 high to set configuration mode, else normal mode

//switch pins
const int switchPin[4] = {A1, A2, A3, A4};
//const int switchPin[4] = {10, 11, 12, 13};


//LED pins
const int ledRedMuxPin = A4;
const int ledGreenMuxPin = A5;
const int ledPin[] = {A0, A1, A2, A3};


// create an array of steppers with common A+ and B+ connections
StepperMRTO myStepper[] = 
{StepperMRTO(stepsPerRevolution, 4, 5, 6, 7),
 StepperMRTO(stepsPerRevolution, 4, 8, 6, 9),
 StepperMRTO(stepsPerRevolution, 4, 10, 6, 11),
 StepperMRTO(stepsPerRevolution, 4, 12, 6, 13)};

 /* These are the ultimate pins
{StepperMRTO(stepsPerRevolution, 0, 2, 1, 3),
 StepperMRTO(stepsPerRevolution, 0, 4, 1, 5),
 StepperMRTO(stepsPerRevolution, 0, 6, 1, 7),
 StepperMRTO(stepsPerRevolution, 0, 8, 1, 9)};
 */
 
void setup() {
  if (analogRead(A6) > 800) // TBD a better way, hold down switch #1 while reset button pushed
  {
    configMode = true;
    Serial.begin(9600);
    return;
  }
  
  // set the rotational speed using rpm as parameter, defaults to 1000 rpm
//  myStepper[0].setSpeed(1000);

  // configure the direction
  // also can reverse the wires on one coil instead
  // if reversed is set to true the device will pull the throwbar toward itself, true is the default?
  myStepper[0].setReversed(false);
  
  // limit the torque
  // this defines the maximum time in microseconds that current will be allowed to flow in each step
  // the default is 1800, a larger number provides more torque but consumes more current 
  // at 1000 rpm the step length is 3000 ms for a 20 step/revolution motor
// myStepper[0].setTorqueLimit(1000);

  // set the length of the stroke
  // default is 700 steps which should be enough for all turnouts
  // it could be made less through experimentation
 myStepper[0].setStrokeSteps(400);

  // switches
  for (int i=0; i<numSteppers; i++) pinMode(switchPin[i], INPUT_PULLUP);
/*
  // LEDs
  pinMode (ledRedMuxPin, OUTPUT);
  pinMode (ledGreenMuxPin, OUTPUT);
  for (int i=0; i<numSteppers; i++) pinMode(ledPin[i], OUTPUT);
*/
}

void loop()
{
  if (configMode) 
  {
    // TBA show a menu and do some configuration
    return;
  }
  
  checkSwitches();

  // the runSteppers method must be called repeatedly
  runSteppers();
}


void checkSwitches()
{
  for (int i=0; i<numSteppers; i++)
  {
    if ((digitalRead(switchPin[i]) == LOW) && (!myStepper[i].getRunState())) //if corresponding stepper is running just skip it (debounce)
    {
      switch (myStepper[i].getLastCommanded())
      {
        case 0:
          myStepper[i].setReady(1);
          break;
        case 1:
          myStepper[i].setReady(0);
          break;
        default:
          myStepper[i].setReady(1);
          break;
      }
      return;
    }
  }
}


void setLEDs()
{
  static long lastTimeStamp;
  static long lastBlink;
  enum color_t {RED, GREEN, UNK};  
  static color_t color = GREEN;
  bool state;
  static bool blinkState;

  // multiplex the LEDs at a relatively slow rate, 25 ms should be fast enough to prevent flicker
  if (millis() - lastTimeStamp < 25) return; 

  lastTimeStamp = millis();

  if (millis() - lastBlink > 500) //TBA
    { 
      blinkState = !blinkState;
      lastBlink = millis();
    }
    
  if (color == GREEN)
  {
    // get the status of each stepper
    for (int i=0; i<numSteppers; i++)
    {
      //if position[i] = unknown
      //state = blinkState;
      //else
      state = myStepper[i].getLastCommanded() && !myStepper[i].getRunState(); //if running show dark
      digitalWrite(ledPin[i], !state); //TBD may need to be reversed if direction has been reversed
    }
    digitalWrite(ledGreenMuxPin, HIGH);
    digitalWrite(ledRedMuxPin, LOW);
    color = RED;
  }
  else
  {
    for (int i=0; i<numSteppers; i++)
    {
      state = !myStepper[i].getLastCommanded() && !myStepper[i].getRunState(); //if running show dark
      digitalWrite(ledPin[i], state); //TBD may need to be reversed if direction has been reversed
    }
    digitalWrite(ledGreenMuxPin, LOW);
    digitalWrite(ledRedMuxPin, HIGH);
    color = GREEN;
  }
}


void runSteppers()
{
  // this routine must be called repeatedly in the loop
  for (int i=0; i<numSteppers; i++)
  {
    if (myStepper[i].run()) return;  // returns false if not in running state, if it did run don't run any others
  }
  
  // if we got here nothing was running so check for steppers that are ready and set the first found to run
  for (int i=0; i<numSteppers; i++)
  {
    if (myStepper[i].getReadyState())
    {
      // the first one we find that is ready we set to run and then exit
      myStepper[i].run();
      return;
    }
  }
}

void backAndForth(int deviceID)
{
  // this demo loop just toggles back and forth for testing
  // WARNING - call this only once for one device in the loop
  // do not run anything else in the loop

  unsigned long delta;
  bool running;
  
  if ((deviceID < 0) || (deviceID > 3)) return;
  
  if (!myStepper[deviceID].getRunState())
  // done running, so wait for five seconds from last startime then restart
  {
    timeStamp = millis();
    delta = timeStamp - lastStartTime;
    
    if (delta > 5000)
    {
      direction = !direction;
      myStepper[deviceID].setReady(direction);  
      lastStartTime = timeStamp;    
    }
  }
}
