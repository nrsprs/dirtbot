/*
DirtBot main demo sequence

Built for use on Arduino AVR Architecture using a MEGA 2560 board

Developed for Professors Caldwell, Tawaf, and Zoyhofski
Class of 2024, RMET
Author: Nick Spears
*/

#include <Arduino.h>
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
    // for Bipolar, constant current, step/direction driver
    return stepper;
}

void loop()                     // main 
{
    // Initalize stepper motor objects:
    AccelStepper stepper0 = initStepper(0);                                                         // Stepper0 (X-Axis motor)
    AccelStepper stepper1 = initStepper(1);                                                         // Stepper1 (Y-Axis motor)
    AccelStepper stepper2 = initStepper(2);                                                         // Stepper2 (Reservoir metering motor)
    AccelStepper stepper3 = initStepper(3);                                                         // Stepper3 (Auger motor)

    stepper0.setMaxSpeed(10000);                           // limits setSpeed(), measured in steps / second
    stepper0.setSpeed(2000);	                            // runSpeed() will run the motor at this speed, measured in steps / second
    stepper0.setAcceleration(1000);                         // motor acceleration in steps/s^2
    stepper0.disableOutputs();                              // sets pins to LOW and prevents motors from drawing current 
    stepper0.setCurrentPosition(0);                         // sets current position to 0 steps after switch trigger
    delay(1);
    stepper0.enableOutputs();                               // sets all pins 

    while (1) {
        stepper0.runSpeed();
    } 

    exit (0);
}