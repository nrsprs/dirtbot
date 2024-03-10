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
    Serial.println("Serial Begun.");

    // Assign Pins: 
    pinMode(1, INPUT);             // Encoder SW 1, testing
    pinMode(2, INPUT);             // DT
    pinMode(3, INPUT);             //CLK
    pinMode(50, INPUT);             // LS1 (x-axis)
    pinMode(51, INPUT);             // LS2 (y-axis)
}


AccelStepper initStepper(int indx){
    // Motor Connections for constant current, step/direction bipolar motor driver
    typedef struct {
        int stepPin;
        int dirPin;
        } pinDictionary;

        pinDictionary PinDict[] = {
        {8, 9},                    // Stepper 0 : X-Axis motor pins
        {23, 24},                  // Stepper 1: Y-Axis motor pins
        {25, 26},                  // Stepper 2: Reservoir metering motor pins
        {27, 28}                   // Stepper 3: Auger motor pins
        };
    AccelStepper stepper(AccelStepper::DRIVER, PinDict[indx].stepPin, PinDict[indx].dirPin);
    Serial.println(String("Index: ") + String(indx));
    Serial.println(String("DirPin: ")+String(PinDict[indx].dirPin)+String(" StepPin: ")+ String(PinDict[indx].stepPin));
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

    while(1) {
        int buttonState = digitalRead(1);
        if (buttonState == 1){ 
            Serial.println(String("Pin 1: ") + String(buttonState));
        }
    }

    /* Working stepper code: will run to +1600 steps then to -1600 steps continually. */
    // Run stepper +/- 1600
    // int pos = 1600;
    // stepper0.setMaxSpeed(4000);
    // stepper0.setAcceleration(1000);
    // while (1) {
    //     if (stepper0.distanceToGo() == 0) {
    //         delay(500); // ms
    //         pos = -pos;
    //         stepper0.moveTo(pos);
    //     }
    //     bool isrunning = stepper0.run();
    // }
    exit (0);
}
