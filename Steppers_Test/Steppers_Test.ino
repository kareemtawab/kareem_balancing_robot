/*
   Simple demo, should work with any driver board

   Connect STEP, DIR as indicated

   Copyright (C)2015 Laurentiu Badea

   This file may be redistributed under the terms of the MIT license.
   A copy of this license has been included with this distribution in the file LICENSE.
*/
#include <Arduino.h>
#include "BasicStepperDriver.h"

// Motor steps per revolution. Most steppers are 200 steps or 1.8 degrees/step
#define MOTOR_STEPS 200

// All the wires needed for full functionality
#define DIR1 3
#define STEP1 2
#define DIR2 5
#define STEP2 4
// Since microstepping is set externally, make sure this matches the selected mode
// 1=full step, 2=half step etc.
#define MICROSTEPS 1

// 2-wire basic config, microstepping is hardwired on the driver
BasicStepperDriver stepper1(MOTOR_STEPS, DIR1, STEP1);
BasicStepperDriver stepper2(MOTOR_STEPS, DIR2, STEP2);


void setup() {
  /*
     Set target motor RPM.
     These motors can do up to about 200rpm.
     Too high will result in a high pitched whine and the motor does not move.
  */
  stepper1.setRPM(80);
  stepper2.setRPM(80);

}

void loop() {
  /*
     Tell the driver the microstep level we selected.
     If mismatched, the motor will move at a different RPM than chosen.
  */
  stepper1.setMicrostep(MICROSTEPS);
  stepper2.setMicrostep(MICROSTEPS);


  /*
     Moving motor one full revolution using the degree notation
  */
  stepper1.rotate(360);
  stepper2.rotate(360);

  /*
     Moving motor to original position using steps
  */
  delay(750);

  stepper1.move(-200 * MICROSTEPS);
  stepper2.move(-200 * MICROSTEPS);

  delay(750);
}
