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


AccelStepper initStepper(int indx){
    // Motor Connections for constant current, step/direction bipolar motor driver
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

    AccelStepper stepper(AccelStepper::DRIVER, PinDict[indx].dirPin, PinDict[indx].stepPin);             
    // for a4988 (Bipolar, constant current, step/direction driver)
    
    return stepper;
}

void loop()                     // main 
{
    // Initalize stepper motor objects:
    AccelStepper stepper0 = initStepper(0);                                                         // Stepper0 object (X-Axis motor)
    AccelStepper stepper1 = initStepper(1);                                                         // Stepper1 object (Y-Axis motor)
    AccelStepper stepper2 = initStepper(2);                                                         // Stepper2 object (Reservoir metering motor)
    AccelStepper stepper3 = initStepper(3);                                                         // Stepper3 object (Auger motor)

    stepper0.setMaxSpeed(100000);   // this limits the value of setSpeed(). Raise it if you like.
    stepper0.setSpeed(90000);	   // runSpeed() will run the motor at this speed - set it to whatever you like.
    stepper0.runSpeed();   // This will run the motor forever.

    return;
}