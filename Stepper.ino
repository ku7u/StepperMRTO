
/*
 This program uses the StepperMRTO library code.
 In production mode, the stepper would be commanded by a momentary switch or a remote system command.
 The code has no way of knowing the actual position of the stepper and doesn't need to. It attempts to drive
 the stepper in a known direction, assumes that the move was successful and reports it as such.

 Device can be configured in place by the user using a menu (TBD). A phone or tablet running a serial app can be
 used. Or the device can be removed and attached to a computer. Device must be removed from socket when downloading the entire program. 


  Pin assignments:
  Revised for D13 as output not input. D13 is pulled low by internal LED attached to it.
   A0  stepper 1 switch
   A1  stepper 2 switch
   A2  stepper 3 switch
   A3  stepper 4 switch
   A4  LED mux red - connect to cathode of all red LED, active low, also serial in config mode          
   A5  LED mux green - connect to cathode of all green LED, active low, also serial in config mode
   A6  A6 is analog input only
   A7  A7 is analog input only
   D0  A+ common, serial pin when configuring
   D1  B+ common, serial pin when configuring
   D2  A- stepper 1
   D3  B- stepper 1
   D4  A- stepper 2
   D5  B- stepper 2
   D6  A- stepper 3
   D7  B- stepper 3
   D8  A- stepper 4
   D9  B- stepper 4
   D10 stepper 1 R/G LED - connect to anode of both R and G for device 0, active high
   D11 stepper 2 R/G LED - connect to anode of both R and G for device 1, active high
   D12 stepper 3 R/G LED - connect to anode of both R and G for device 2, active high
   D13 stepper 4 R/G LED - connect to anode of both R and G for device 3, active high
   */

#include <Arduino.h>
#include "StepperMRTO.h"

const int numSteppers = 4;
const int stepsPerRevolution = 20; // number of steps per revolution
bool direction = true;             // when true travels in one direction, when false the other
unsigned long lastStartTime = 0;
unsigned long timeStamp;
bool configMode = false; // on reset, test switch 0 low to set configuration mode, else normal mode

// motor pins
const int APlusPin = 0;
const int BPlusPin = 1;
const int AMinus1Pin = 2;
const int BMinus1Pin = 3;
const int AMinus2Pin = 4;
const int BMinus2Pin = 5;
const int AMinus3Pin = 6;
const int BMinus3Pin = 7;
const int AMinus4Pin = 8;
const int BMinus4Pin = 9;

// switch pins
const int switchPin[4] = {A0, A1, A2, A3};

// LED pins
const int ledRedMuxPin = A4;
const int ledGreenMuxPin = A5;
const int ledPin[4] = {10, 11, 12, 13};

// create an array of steppers with common A+ and B+ connections
StepperMRTO myStepper[] =
    {StepperMRTO(stepsPerRevolution, APlusPin, AMinus1Pin, BPlusPin, BMinus1Pin),
     StepperMRTO(stepsPerRevolution, APlusPin, AMinus2Pin, BPlusPin, BMinus2Pin),
     StepperMRTO(stepsPerRevolution, APlusPin, AMinus3Pin, BPlusPin, BMinus3Pin),
     StepperMRTO(stepsPerRevolution, APlusPin, AMinus4Pin, BPlusPin, BMinus4Pin)};

void setup()
{
  // switches
  for (int i = 0; i < numSteppers; i++)
    pinMode(switchPin[i], INPUT_PULLUP);

  // LEDs
  pinMode(ledRedMuxPin, OUTPUT);
  pinMode(ledGreenMuxPin, OUTPUT);
  digitalWrite(ledRedMuxPin, HIGH);
  digitalWrite(ledGreenMuxPin, HIGH);
  for (int i = 0; i < numSteppers; i++)
  {
    pinMode(ledPin[i], OUTPUT);
    digitalWrite(ledPin[i], LOW);
  }

  // to trigger menu display, hold down switch 0 while resetting the device - reset when done with menu
  if (digitalRead(switchPin[0]) == LOW)
  {
    // do the menu thing TBA
    configMode = true;
    Serial.begin(9600);
    return;
  }

  // set the rotational speed using rpm as parameter, defaults to 1000 rpm
    myStepper[3].setSpeed(500);

  // configure the direction
  // also can reverse the wires on one coil instead
  // if reversed is set to true the device will pull the throwbar toward itself, true is the default?
  myStepper[3].setReversed(true);

  // limit the torque
  // this defines the maximum time in microseconds that current will be allowed to flow in each step
  // the default is 1800, a larger number provides more torque but consumes more current
  // at 1000 rpm the step length is 3000 ms for a 20 step/revolution motor
  myStepper[3].setTorqueLimit(1200);

  // set the length of the stroke
  // default is 700 steps which should be enough for all turnouts
  // it could be made less through experimentation
  myStepper[3].setStrokeSteps(400);
}

void loop()
{
  if (configMode)
  {
    // TBA show a menu and do some configuration
    configMode = false;
    Serial.end();
    return;
  }

  // closed switch will set its device to ready
  checkSwitches();

  // the runSteppers method must be called repeatedly
  runSteppers();

  // display status on LEDs
  setLEDs();
}

void checkSwitches()
{
  for (int i = 0; i < numSteppers; i++)
  {
    // if corresponding stepper is in ready state or is running just skip it (debounces the switch)
    if ((digitalRead(switchPin[i]) == LOW) && (!myStepper[i].getRunState()) && (!myStepper[i].getReadyState()))
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

void setLEDs() // not complete
{
  enum color_t
  {
    RED,
    GREEN,
    UNK
  };
  static color_t color = GREEN;
  static long lastTimeStamp;
  static long lastBlink;
  bool state;
  static bool blinkState;

  // multiplex the LEDs at a relatively slow rate, 25 ms should be fast enough to prevent flicker
  if (millis() - lastTimeStamp < 25)
    return;

  lastTimeStamp = millis();

  if (millis() - lastBlink > 100) // TBA
  {
    blinkState = !blinkState;
    lastBlink = millis();
  }

  if (color == GREEN)
  {
    // get the status of each stepper
    for (int i = 0; i < numSteppers; i++)
    {
      // if position[i] = unknown
      // state = blinkState;
      // else
      state = myStepper[i].getLastCommanded() && !myStepper[i].getRunState(); // if running show dark
      digitalWrite(ledPin[i], !state);                                        // TBD may need to be reversed if direction has been reversed
    }
    digitalWrite(ledGreenMuxPin, HIGH);
    digitalWrite(ledRedMuxPin, LOW);
    color = RED;
  }
  else
  {
    for (int i = 0; i < numSteppers; i++)
    {
      state = !myStepper[i].getLastCommanded() && !myStepper[i].getRunState(); // if running show dark
      digitalWrite(ledPin[i], state);                                          // TBD may need to be reversed if direction has been reversed
    }
    digitalWrite(ledGreenMuxPin, LOW);
    digitalWrite(ledRedMuxPin, HIGH);
    color = GREEN;
  }
}

void runSteppers()
{
  // this routine must be called repeatedly in the loop
  for (int i = 0; i < numSteppers; i++)
  {
    if (myStepper[i].run())
      return; // returns false if not in running state, if it did run don't try to run any others
  }

  // if we got here nothing was running so check for steppers that are ready and set the first found to run
  for (int i = 0; i < numSteppers; i++)
  {
    if (myStepper[i].getReadyState())
    {
      // the first one we find that is ready we set to run and then exit
      myStepper[i].run();
      return;
    }
  }
}
