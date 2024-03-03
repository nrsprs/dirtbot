/*
DirtBot main demo sequence

Built for use on Arduino AVR Architecture using a MEGA 2560 board

Developed for Professors Caldwell, Tawaf, and Zoyhofski
Class of 2024, RMET
Author: Nick Spears
*/


#include <AccelStepper.h>
#include <LiquidCrystal.h> 
#include <Wire.h>
#include <Stepper.h>
#include <elapsedMillis.h>
#include <EnableInterrupt.h>
#include <QuadratureEncoder.h>
#include <InputDebounce.h>
#include <Dictionary.h>


void setup()                     // init 
{  
    Serial.begin(115200);
}


void loop()                     // main 
{
    // Initalize stepper motor objects
    typedef struct {
        int dirPin;
        int stepPin;
        } pinDictionary;

        pinDictionary PinDict[] = {
        {8, 9},                  // Stepper 0 : X-Axis motor pins
        {0, 0},                  // Stepper 1: Y-Axis motor pins
        {0, 0},                  // Stepper 2: Reservoir metering motor pins
        {0, 0}                   // Stepper 3: Auger motor pins
        };

    AccelStepper stepper0(AccelStepper::DRIVER, PinDict[0].dirPin, PinDict[0].stepPin);             // Stepper0 object
    AccelStepper stepper1(AccelStepper::DRIVER, PinDict[1].dirPin, PinDict[1].stepPin);             // Stepper1 object
    AccelStepper stepper2(AccelStepper::DRIVER, PinDict[2].dirPin, PinDict[2].stepPin);             // Stepper2 object
    AccelStepper stepper3(AccelStepper::DRIVER, PinDict[3].dirPin, PinDict[3].stepPin);             // Stepper3 object

    // for a4988 (Bipolar, constant current, step/direction driver)
    // Motor Connections (constant current, step/direction bipolar motor driver)
    
    int new_speed;
    stepper0.setMaxSpeed(100000);   // this limits the value of setSpeed(). Raise it if you like.
    stepper0.setSpeed(90000);	   // runSpeed() will run the motor at this speed - set it to whatever you like.
    stepper0.runSpeed();   // This will run the motor forever.
}