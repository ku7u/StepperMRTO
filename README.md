# StepperMRTO
StepperMRTO provides a means to control a linear stepper motor to actuate a model railroad turnout (track switch). In particular the stepper to be controlled will be a "micro" variety that is so small that it can be installed on top of the layout next to the track. Traditionally such actuators have never been stepper motors and are usually installed on the underside of the layout. This code is meant to used along with a "sketch" (program) in an Arduino Nano microcontroller.  
When configured properly the stepper can be connected to the turnout throwbar directly with no omega or Z motion absorbers required. The stepper motor will skip harmlessly if the mechanism encounters an obstruction. It will then recalibrate its position on the next commanded movement. The system also requires no position calibration at installation other than to make sure that the throw is long enough to enable reaching both stock rails and that the torque is strong enough for reliable operation. The code in StepperMRTO.cpp sets up initial conditions that should provide for good performance out of the box.  
Several aspects of the stepper operation can be controlled, namely the speed, length of throw and force (torque). These are changed within the Stepper.ino program as of now and are described therein. Stepper.ino will be improved to include a menu driven mechanism to make these changes. That will allow the changes to be made without removing the Arduino from its socket and without requiring the use of the Arduino IDE. A portable device such as a laptop, tablet or phone will be plugged into the USB connector to make the changes.  
Other additions will be made to Stepper.ino, in particular for the LED indicators. As of this date the code has been tested to drive the stepper successfully and to respond to switch inputs. The multiplexing scheme to drive up to four steppers from only 10 pins has not been tested with more than one device. The author is awaiting delivery of more devices.  
The plan going forward is to enhance Stepper.ino to include full LED indication of status at which point it will be considered "in production". At this point the steppers will be controllable by local momentary switches. They could also be controlled remotely using a device such as the LCC based rrCirkits Tower node. Any remotely controlled device that can pulse the switch inputs low will work as well.  
StepperMRTO.cpp and StepperMRTO.h should be placed in the libraries folder in the Arduino file structure. Stepper.ino should be placed in a folder of the same name placed at the top level of the Arduino file structure. It can then be accessed and run from the Arduino IDE.