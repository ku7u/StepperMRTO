/*
 * mrTOStepper.cpp - Stepper library for model railroad turnout control
 *
 * Derived from original stepper.cpp and modifications by Tom Igoe et. al.
 * Total rewrite for microstepper         by George Hofmann 1/5/2022
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 *
 *
 * Drives a bipolar motor.
 * The sequence of control signals for 4 control wires is as follows:
 *
 * Step C0 C1 C2 C3
 *    1  1  0  1  0
 *    2  0  1  1  0
 *    3  0  1  0  1
 *    4  1  0  0  1
 *
 *
 */

#include "StepperMRTO.h"

// Constructor, sets the number of steps per revolution and the pin connections to the windings
StepperMRTO::StepperMRTO(int stepsPerRevolution, int motorAPlus, int motorAMinus,
                         int motorBPlus, int motorBMinus)
{
  _isRunning = false;
  _readyToRun = false;
  _currentStep = 0; // which step the motor is on
  _direction = 0;   // motor direction
  _lastCommanded = 0;
  _lastStepStartTime = 0; // time stamp in us of the start of the last step
  // _lastSliceStartTime = 0;
  _stepsPerRevolution = stepsPerRevolution; // number of steps in one revolution
  _strokeSteps = 700;                       // typical default value for length of stroke
  _torqueInterval = 1800;                   // value derived empirically, value in usecs - effect is to limit the torque and heat
  _reversed = false;
  _stepsLeftToGo = 0;

  // Arduino pins for the motor control connection:
  _motorAPlus = motorAPlus;
  _motorAMinus = motorAMinus;
  _motorBPlus = motorBPlus;
  _motorBMinus = motorBMinus;

  // setup the pins on the microcontroller:
  pinMode(_motorAPlus, OUTPUT);
  pinMode(_motorAMinus, INPUT); // for multiplexing, assures no current flow when not running, set to output in run routine
  pinMode(_motorBPlus, OUTPUT);
  pinMode(_motorBMinus, INPUT); // for multiplexing, assures no current flow when not running, set to output in run routine

  // set the default step interval that results in 1000 rpm
  _stepInterval = 60L * 1000L * 1000L / _stepsPerRevolution / 1000;
}

// get the run state
bool StepperMRTO::getRunState()
{
  return _isRunning;
}

// get the ready to run state
bool StepperMRTO::getReadyState()
{
  return _readyToRun;
}

// return the last position command
int StepperMRTO::getLastCommanded()
{
  return _lastCommanded;
}

// set the step interval length in usecs using RPM as parameter to control speed
void StepperMRTO::setSpeed(long rpmSpeed)
{
  _stepInterval = 60L * 1000L * 1000L / _stepsPerRevolution / rpmSpeed;
}

// set the value for torqueInterval if the default value is not optimum
// limiting the interval controls average current and torque
void StepperMRTO::setTorqueLimit(unsigned long stepLimit)
{
  _torqueInterval = stepLimit;
}

// set stroke length to override the default value
void StepperMRTO::setStrokeSteps(int strokeSteps)
{
  _strokeSteps = strokeSteps;
}

// set last known position to establish default direction
void StepperMRTO::setReversed(bool reversed)
{
  _reversed = reversed;
}

// prep for running, subsequent run command will refer to ready state
void StepperMRTO::setReady(bool direction)
{
  _lastCommanded = direction;
  if (_reversed)
    _direction = !direction;
  else
    _direction = direction;

  _readyToRun = true;
}

// called repeatedly by loop code
bool StepperMRTO::run(void)
{
  // uint32_t dutyFactor = 50;
  // uint32_t wholeSlice = 100; //_stepInterval/10;
  // uint32_t onSlice = wholeSlice * dutyFactor / 100;
  // uint32_t offSlice = wholeSlice * (100 - dutyFactor) / 100;
  // static bool _chopper;
  // static bool _on;

  // onSlice = wholeSlice * dutyFactor / 100;
  // offSlice = wholeSlice * (100 - dutyFactor) / 100;

  // return false if not currently running and not ready to run
  if (!(_readyToRun || _isRunning))
    return false;
  else
  {
    if (_stepsLeftToGo == 0) // just starting
    {
      pinMode(_motorAMinus, OUTPUT); // for multiplexing, reconfigure as output while active
      pinMode(_motorBMinus, OUTPUT); // as above
      _currentDirection = _direction;
      _stepsLeftToGo = _strokeSteps;
      _readyToRun = false;
      _isRunning = true;
      _lastStepStartTime = micros();
      // _lastSliceStartTime = _lastStepStartTime;
      // _chopper = false;
    }

    _now = micros();

    // reduce the torque by limiting the current flow to a shorter period than the whole step
    // when using a motor driver this technique was not seen to be effective outside of a narrow range around 1000 microseconds
    // jitter was seen outside of that range
    // operation was seen to be smooth and very forceful within the narrow range
    if ((_now - _lastStepStartTime) >= _torqueInterval)
      release();

    // the following two techniques were an attempt to overcome jitter observed at torque intervals outside of a narrow range
    // that range was about 900 to 1100 or so
    // the first technique uses an initial pulse followed by narrow on and off slices throughout the step period
    // this is an attempt to simulate a 'chopper' style motor driver to limit current
    // all three time periods could be variable
    // not shown to have any effect in testing
    // if ((_on && _chopper && ((_now - _lastSliceStartTime) > 100)) || (!_on && _chopper && ((_now - _lastSliceStartTime) > 600)))
    // {
    //   if (_on) release();
    //   else stepMotor(_currentStep);
    //   _on = !_on;
    //   _lastSliceStartTime = micros();
    // }
    // if (!_chopper && ((_now - _lastStepStartTime) >= _torqueInterval))
    // {
    //   release();
    //   _lastSliceStartTime = micros();
    //   _chopper = true;
    //   _on = false;
    // }
    // the second technique provides equal slices throughout the step period
    // on and off times could be varied
    // not shown to have any effect in testing
    // if (_now - _lastSliceStartTime >= offSlice + onSlice)
    // {
    //   _lastSliceStartTime = micros();
    //   stepMotor(_currentStep);
    // }
    // else
    // {
    //   if (_now - _lastSliceStartTime >= onSlice);
    //   {
    //     release();
    //   }
    // }

    // move only if the appropriate delay has passed:
    if ((_now - _lastStepStartTime) >= _stepInterval)
    {
      // _chopper = false;

      // remember when this step started
      _lastStepStartTime = _now;
      // _lastSliceStartTime = _lastStepStartTime;

      // increment or decrement the step number depending on direction
      if (_currentDirection)
      {
        //_currentStep++;
        if (++_currentStep == _stepsPerRevolution)
        {
          _currentStep = 0;
        }
      }
      else
      {
        if (_currentStep == 0)
        {
          _currentStep = _stepsPerRevolution;
        }
        _currentStep--;
      }
      // decrement the steps left to go and test for done
      if (--_stepsLeftToGo == 0) // done with the stroke if zero
      {
        release();                    // turn off the juice to reduce overheating
        pinMode(_motorAMinus, INPUT); // for multiplexing, don't want this pin acting as a sink while other coils are active
        pinMode(_motorBMinus, INPUT); // as above, done with these pins for now
        _isRunning = false;
        return true;
      }

      // step the motor to the next of the four steps
      stepMotor(_currentStep % 4);
    }
  }
  return true;
}

// moves the motor forward or backwards
void StepperMRTO::stepMotor(int thisStep)
{
  switch (thisStep)
  {
  case 0: // 1010
    digitalWrite(_motorAPlus, HIGH);
    digitalWrite(_motorAMinus, LOW);
    digitalWrite(_motorBPlus, HIGH);
    digitalWrite(_motorBMinus, LOW);
    break;
  case 1: // 0110
    digitalWrite(_motorAPlus, LOW);
    digitalWrite(_motorAMinus, HIGH);
    digitalWrite(_motorBPlus, HIGH);
    digitalWrite(_motorBMinus, LOW);
    break;
  case 2: // 0101
    digitalWrite(_motorAPlus, LOW);
    digitalWrite(_motorAMinus, HIGH);
    digitalWrite(_motorBPlus, LOW);
    digitalWrite(_motorBMinus, HIGH);
    break;
  case 3: // 1001
    digitalWrite(_motorAPlus, HIGH);
    digitalWrite(_motorAMinus, LOW);
    digitalWrite(_motorBPlus, LOW);
    digitalWrite(_motorBMinus, HIGH);
    break;
  }
}

// remove current from motor
void StepperMRTO::release(void)
{
  digitalWrite(_motorAPlus, LOW);
  digitalWrite(_motorAMinus, LOW);
  digitalWrite(_motorBPlus, LOW);
  digitalWrite(_motorBMinus, LOW);
}

int StepperMRTO::version(void)
{
  return 1;
}
