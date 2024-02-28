/*
   Uno sketch to drive a stepper motor using the AccelStepper library.
   Runs stepper back and forth between limits. (Like Bounce demo program.)
   Works with a ULN-2003 unipolar stepper driver, or a bipolar, constant voltage motor driver
   such as the L298 or TB6612, or a step/direction constant current driver like the a4988.
*/

// Include the AccelStepper Library
#include <AccelStepper.h>

// Motor Connections (constant current, step/direction bipolar motor driver)
const int dirPin = 8;
const int stepPin = 9;
int new_speed;

// Creates an instance - Pick the version you want to use and un-comment it. That's the only required change.
AccelStepper myStepper(AccelStepper::DRIVER, stepPin, dirPin);           // works for a4988 (Bipolar, constant current, step/direction driver)

void setup()
{  
   Serial.begin(115200);
   myStepper.setMaxSpeed(100000);   // this limits the value of setSpeed(). Raise it if you like.
   myStepper.setSpeed(90000);	   // runSpeed() will run the motor at this speed - set it to whatever you like.
}

void loop()
{  
    myStepper.runSpeed();   // This will run the motor forever.
}


// void setup() {
//   // set the maximum speed, acceleration factor,
//   // and the target position
//   Serial.begin(9600);
//   myStepper.setMaxSpeed(200.0);
//   myStepper.setAcceleration(50.0);
//   myStepper.moveTo(2000);
// }

// void loop() {
//   // Change direction once the motor reaches target position
//     if (myStepper.distanceToGo() == 0) { 
//         myStepper.moveTo(0);
//         myStepper.run();
//         delay(1000);
//         myStepper.moveTo(100);
//         delay(1000);
//         myStepper.run();
//     }

// //   if (!myStepper.run()) {   // run() returns true as long as the final position has not been reached and speed is not 0.
// //     Serial.print('Moving to...');
// //     myStepper.moveTo(-myStepper.currentPosition());
// //   }
// }