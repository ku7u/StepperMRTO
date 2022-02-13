
/*
 This program uses the StepperMRTO library code.
 In production mode, the stepper would be commanded by a momentary switch or a remote system command.
 The code has no way of knowing the actual position of the stepper and doesn't need to. It attempts to drive
 the stepper in a known direction, assumes that the move was successful and reports it as such.

 Device can be configured in place by the user using a menu. A phone or tablet running a serial app can be
 used. Or the device can be removed and attached to a computer. Device must be removed from socket when downloading the entire program.


  Pin assignments:
  Revised for D13 as output not input. D13 is pulled low by internal LED attached to it.
   A0  stepper 1 switch
   A1  stepper 2 switch
   A2  stepper 3 switch
   A3  stepper 4 switch
   A4  LED mux red - connect to cathode of all red LED, active low
   A5  LED mux green - connect to cathode of all green LED, active low
   A6  A6 is analog input only
   A7  A7 is analog input only
   D0  A+ common, also used as serial pin when displaying configuration menu
   D1  B+ common, also used as serial pin when displaying configuration menu
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
#include <EEPROM.h>
#include "StepperMRTO.h"

uint16_t const numSteppers = 4;
uint16_t const stepsPerRevolution = 20; // number of steps per revolution
bool direction = true;                  // when true travels in one direction, when false the other
bool configMode = false;                // on reset, test switch 0 low to set configuration mode, else normal mode

// motor pins
uint16_t const APlusPin = 0;
uint16_t const BPlusPin = 1;
uint16_t const AMinus1Pin = 2;
uint16_t const BMinus1Pin = 3;
uint16_t const AMinus2Pin = 4;
uint16_t const BMinus2Pin = 5;
uint16_t const AMinus3Pin = 6;
uint16_t const BMinus3Pin = 7;
uint16_t const AMinus4Pin = 8;
uint16_t const BMinus4Pin = 9;

// switch pins
uint16_t const switchPin[4] = {A0, A1, A2, A3};

// LED pins
uint16_t const redLEDCathodePin = A4;
uint16_t const greenLEDCathodePin = A5;
uint16_t const ledAnodePin[4] = {10, 11, 12, 13};

// create an array of steppers with common A+ and B+ connections
StepperMRTO myStepper[] =
    {StepperMRTO(stepsPerRevolution, APlusPin, AMinus1Pin, BPlusPin, BMinus1Pin),
     StepperMRTO(stepsPerRevolution, APlusPin, AMinus2Pin, BPlusPin, BMinus2Pin),
     StepperMRTO(stepsPerRevolution, APlusPin, AMinus3Pin, BPlusPin, BMinus3Pin),
     StepperMRTO(stepsPerRevolution, APlusPin, AMinus4Pin, BPlusPin, BMinus4Pin)};

uint16_t const NOMINAL_SPEED = 1000;
uint16_t const NOMINAL_STROKE = 500;
uint16_t const NOMINAL_TORQUE_INTERVAL = 1500;

bool notSet[] = {true, true, true, true};

/*****************************************************************************/
void setup()
/*****************************************************************************/
{
  uint16_t initVal;
  uint16_t eepVal;
  uint8_t eepByte;

  // switches
  for (int i = 0; i < numSteppers; i++)
    pinMode(switchPin[i], INPUT_PULLUP);

  // LEDs
  pinMode(redLEDCathodePin, OUTPUT);
  pinMode(greenLEDCathodePin, OUTPUT);
  digitalWrite(redLEDCathodePin, HIGH);
  digitalWrite(greenLEDCathodePin, HIGH);
  for (int i = 0; i < numSteppers; i++)
  {
    pinMode(ledAnodePin[i], OUTPUT);
    digitalWrite(ledAnodePin[i], LOW);
  }

  // to trigger menu display, hold down switch 0 while resetting the device
  if (digitalRead(switchPin[0]) == LOW)
    configMode = true;

  // check for initialization
  EEPROM.get(0x200, initVal);
  if (initVal != 666)
  {
    EEPROM.put(0x200, 666); // put the magic number 666 at a lofty EEPROM address to indicate initialized state of EEPROM
    for (int i = 0; i < numSteppers; i++)
    {
      eepByte = NOMINAL_SPEED / 10;   // nominal value for speed divided by 10 to make bytesize
      EEPROM.write((4 * i), eepByte); // speed

      eepByte = NOMINAL_STROKE / 10;
      EEPROM.write((4 * i) + 1, eepByte); // strokesteps

      eepByte = NOMINAL_TORQUE_INTERVAL / 10;
      EEPROM.write((4 * i) + 2, eepByte); // torque limit

      EEPROM.write((4 * i) + 3, 0); // reversed = false
    }
  }

  for (int i = 0; i < numSteppers; i++)
  {
    // read the stored values for speed, throw, torque and reversed
    // send those values to the stepper objects

    // set the rotational speed using rpm as parameter, defaults to 1000 rpm
    eepVal = 10 * EEPROM.read(4 * i);
    myStepper[i].setSpeed(eepVal);

    // set the length of the stroke
    // default is 700 steps which should be enough for all turnouts
    // it could be made less through experimentation
    eepVal = 10 * EEPROM.read((4 * i) + 1);
    myStepper[i].setStrokeSteps(eepVal);

    // limit the torque
    // this defines the maximum time in microseconds that current will be allowed to flow in each step
    // the default is 1500, a larger number provides more torque but consumes more current
    // at 1000 rpm the step length is 3000 ms for a 20 step/revolution motor
    eepVal = 10 * EEPROM.read((4 * i) + 2);
    myStepper[i].setTorqueLimit(eepVal);

    // configure the direction
    // design assumes device is installed on the closed side of turnout and that turnout is closed
    // the first movement after startup will be to pull the throwbar thus throwing the track switch and lighting red LED
    // setting reversed to true will set motion to the opposite of above as required if machine is located on diverging side
    myStepper[i].setReversed(EEPROM.read((4 * i) + 3));
  }
}

/*****************************************************************************/
void loop()
/*****************************************************************************/
{
  if (configMode)
  {
    Serial.begin(115200);
    configure();
    configMode = false;
    Serial.end();
  }

  // closed switch will set its device to ready
  checkSwitches();

  // the runSteppers method must be called repeatedly
  runSteppers();

  // display status on LEDs
  setLEDs();
}

/*****************************************************************************/
void checkSwitches()
/*****************************************************************************/
{
  for (int i = 0; i < numSteppers; i++)
  {
    // if corresponding stepper is in ready state or is running just skip it (debounces the switch)
    if ((digitalRead(switchPin[i]) == LOW) && (!myStepper[i].getRunState()) && (!myStepper[i].getReadyState()))
    {
      notSet[i] = false; // turn off blinking seen at startup
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

/*****************************************************************************/
void setLEDs()
/*****************************************************************************/
{
  static uint32_t lastBlink;
  static uint8_t currentLED = 0;
  bool state;
  static bool blinkState;

  if (millis() - lastBlink > 100)
  {
    blinkState = !blinkState;
    lastBlink = millis();
  }

  for (int i = 0; i < numSteppers; i++)
    digitalWrite(ledAnodePin[i], LOW); // turn them all off briefly

  if (myStepper[currentLED].getLastCommanded() == 1)
  {
    // display red
    digitalWrite(greenLEDCathodePin, HIGH);
    digitalWrite(redLEDCathodePin, LOW);
  }
  else
  {
    // display green
    digitalWrite(greenLEDCathodePin, LOW);
    digitalWrite(redLEDCathodePin, HIGH);
  }

  if (myStepper[currentLED].getRunState() || myStepper[currentLED].getReadyState() || notSet[currentLED])
    digitalWrite(ledAnodePin[currentLED], blinkState); // blinking while moving or ready to move
  else
    digitalWrite(ledAnodePin[currentLED], HIGH); // common anodes

  if (++currentLED == 4)
    currentLED = 0;
}

/*****************************************************************************/
void runSteppers()
/*****************************************************************************/
{
  // this routine must be called repeatedly in the loop
  for (int i = 0; i < numSteppers; i++)
  {
    if (myStepper[i].getRunState()) // returns false if not in running state
    {
      myStepper[i].run();
      return; // if it did run don't try to run any others
    }
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

/*****************************************************************************/
void showMenu()
/*****************************************************************************/
{
  Serial.println(F(" "));
  Serial.println(F("\nMain menu"));
  Serial.println(F(" Enter: "));
  Serial.println(F(" 'S' - Set speed"));
  Serial.println(F(" 'T' - Set throw"));
  Serial.println(F(" 'F' - Set force limit"));
  Serial.println(F(" 'D' - Set direction"));
  Serial.println(F(" 'P' - Print status"));

  Serial.println(F("\n Enter 'R' to return to run mode"));
}

/*****************************************************************************/
void configure()
/*****************************************************************************/
{
  uint16_t devID;
  uint8_t eepByte;

  while (true)
  {
    showMenu();

    switch (getUpperChar())
    {
    case 'S':
      Serial.println(F("Speed (rpm, valid value 1 - 2000, default = 1000)"));
      Serial.println(F("Enter device (1 - 4) or 0 for all"));
      devID = getNumber(0, 4);
      Serial.print(F("Enter value for "));
      if (devID == 0)
        Serial.println(F("all devices"));
      else
      {
        Serial.print(F("device "));
        Serial.println(devID);
      }
      eepByte = getNumber(1, 2000) / 10;
      if (devID == 0)
        for (int i = 0; i < numSteppers; i++)
        {
          EEPROM.write(4 * i, eepByte);
          myStepper[i].setSpeed(eepByte * 10);
        }
      else
      {
        EEPROM.write(4 * (devID - 1), eepByte);
        myStepper[devID].setSpeed(eepByte * 10);
      }
      break;

    case 'T':
      Serial.println(F("Throw (steps, valid value 200 - 800, default = 600, 1000 steps = 7mm)"));
      Serial.println(F("Enter device (1 - 4) or 0 for all"));
      devID = getNumber(0, 4);
      Serial.print(F("Enter value for "));
      if (devID == 0)
        Serial.println(F("all devices"));
      else
      {
        Serial.print(F("device "));
        Serial.println(devID);
      }
      eepByte = getNumber(200, 800) / 10;
      if (devID == 0)
        for (int i = 0; i < numSteppers; i++)

        {
          EEPROM.write((4 * i) + 1, eepByte);
          myStepper[i].setStrokeSteps(eepByte * 10);
        }
      else
      {
        EEPROM.write(4 * (devID - 1) + 1, eepByte);
        myStepper[devID].setStrokeSteps(eepByte * 10);
      }
      break;

    case 'F':
      Serial.println(F("Force (microseconds, valid value 800 - 2550, default = 1500)"));
      Serial.println(F("Enter device (1 - 4) or 0 for all"));
      devID = getNumber(0, 4);
      Serial.print(F("Enter value for "));
      if (devID == 0)
        Serial.println(F("all devices"));
      else
      {
        Serial.print(F("device "));
        Serial.println(devID);
      }
      eepByte = getNumber(800, 2550) / 10;
      if (devID == 0)
        for (int i = 0; i < numSteppers; i++)
        {
          EEPROM.write((4 * i) + 2, eepByte);
          myStepper[i].setTorqueLimit(eepByte * 10);
        }
      else
      {
        EEPROM.write(4 * (devID - 1) + 2, eepByte);
        myStepper[devID].setTorqueLimit(eepByte * 10);
      }
      break;

    case 'D':
      Serial.println(F("Direction (0 or 1, 1 reverses the normal direction of throw"));
      Serial.println(F("Enter device (1 - 4) or 0 for all"));
      devID = getNumber(0, 4);
      Serial.print(F("Enter value for "));
      if (devID == 0)
        Serial.println(F("all devices"));
      else
      {
        Serial.print(F("device "));
        Serial.println(devID);
      }
      eepByte = getNumber(0, 1);
      if (devID == 0)
        for (int i = 0; i < numSteppers; i++)
        {
          EEPROM.write(4 * i + 3, eepByte);
          myStepper[i].setReversed(eepByte);
        }
      else
      {
        EEPROM.write(4 * (devID - 1) + 3, eepByte);
        myStepper[devID].setReversed(eepByte);
      }
      break;

    case 'P':
      Serial.println(F("Print current configuration")); // TBD
      for (int i = 0; i < numSteppers; i++)
      {
        Serial.print("Device: ");
        Serial.print(i+1);
        Serial.print(" ");
        eepByte = EEPROM.read(4 * i);
        Serial.print(eepByte * 10);
        Serial.print(" ");
        eepByte = EEPROM.read(4 * i + 1);
        Serial.print(eepByte * 10);
        Serial.print(" ");
        eepByte = EEPROM.read(4 * i + 2);
        Serial.print(eepByte * 10);
        Serial.print(" ");
        eepByte = EEPROM.read(4 * i + 3);
        Serial.println(eepByte);
      }
      break;

    case 'R':
      return;

    default:
      break;
    }
  }
}

/*****************************************************************************/
char getUpperChar()
/*****************************************************************************/
{
  // gets one character and converts to upper case, clears input buffer of C/R and newline
  char _myChar;

  while (true)
  {
    if (Serial.available() > 0)
    {
      delay(5);
      _myChar = Serial.read();

      if (_myChar > 96)
        _myChar -= 32;

      flushSerialIn();
      return _myChar;
    }
  }
}

/*****************************************************************************/
void flushSerialIn(void)
/*****************************************************************************/
{
  while (Serial.available() > 0)
  { // clear the buffer
    delay(5);
    Serial.read();
  }
}

/*****************************************************************************/
int getNumber(int min, int max)
/*****************************************************************************/
{
  int _inNumber;

  while (true)
  {
    if (Serial.available() > 0)
    {
      _inNumber = Serial.parseInt();
      // Serial.println(_inNumber);

      // if (_inNumber == 0)
      // {
      //   // Serial.println(F("Exiting"));
      //   flushSerialIn();
      //   return -1;
      // }
      // else
      if ((_inNumber < min) || (_inNumber > max))
      {
        Serial.println(F("Out of range, reenter"));
        flushSerialIn();
      }
      else
      {
        flushSerialIn();
        return _inNumber;
      }
    }
  }
}
