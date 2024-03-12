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
#include <Encoder.h>
#include <InputDebounce.h>
#include <Dictionary.h>


Encoder encX(10, 11);           // Pins 10 & 11 are for X-Axis encoder (intr 4 & 5)
Encoder encY(12, 13);           // Pins 12 & 13 are for Y-Axis encoder (intr 6 & 7)
Encoder enc3(18, 19);           // Pins 18 & 19 are for the hopper encoder (intr 3 & 2)
Encoder enc4(20, 21);           // Pins 20 & 21 are for the auger encoder (intr 1 & 0)

int encXPosOld = -999, encYPosOld = -999, enc3PosOld = -999, enc4PosOld = -999;              // Have to make this global & dynamic, unfortunately..
volatile bool encXReq, encYReq, enc3Req, enc4Req; 


void interpretEncoder();


void setup()                     // init 
{  
    // Begin serial:
    Serial.begin(9600);
    Serial.println("Serial Begun.");

    // Attach interrupt pins for encoder:
    attachInterrupt(4, interpretEncoder, CHANGE);

    // Assign Pin Modes: 
    pinMode(LED_BUILTIN, OUTPUT);       // Built-in LED
}


AccelStepper initStepper(int indx) {
    // Motor Connections for constant current, step/direction bipolar motor driver
    typedef struct {
        const int stepPin;
        const int dirPin;
        } pinDictionary;

        pinDictionary PinDict[] = {
        {21, 22},                    // Stepper 0 : X-Axis motor pins
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


/* ### debounce a digital input
Takes a InputDebounce object and returns 1 or 0 depending on status. 
Meant to be called multiple times, saves on interrupt pins. 
### Returns :: Bool
*/
bool debounce(InputDebounce& switch_object) {
    // Reset vars for switches:
    static unsigned int LS1_Count = 0;
    static unsigned int LS1_OnTimeLast = 0;
    bool result = 0;

    unsigned long now = millis();
    unsigned int buttonTest_OnTime = switch_object.process(now);

    // Handle input button
    if(buttonTest_OnTime) {
        // Save current on-time (button pressed) for release
        LS1_OnTimeLast = buttonTest_OnTime;
        // Check for state change
        unsigned int count = switch_object.getStatePressedCount();
        if(LS1_Count != count) {
        LS1_Count = count;
        // Handle pressed state
        digitalWrite(LED_BUILTIN, HIGH);                 // Turn on built_in LED
        result = 1;
        Serial.print("HIGH");
        }
        else {
        // Handle still pressed state
        Serial.print("HIGH still pressed");
        }
        Serial.print(" (");
        Serial.print(buttonTest_OnTime);
        Serial.println("ms)");
    }
    else {
        if(LS1_OnTimeLast) {
        // Handle released state
        digitalWrite(LED_BUILTIN, LOW);                 // Turn off built_in LED
        result = 0;
        Serial.print("LOW (last on-time: HIGH ");
        Serial.print(LS1_OnTimeLast);
        Serial.println("ms)");
        LS1_OnTimeLast = 0;                             // Reset 
        }
    }
    return result; 
}


void interpretEncoder() {
    if (encXReq == true) {      // Read for X-Axis Encoder:
        long newPositionX = encX.read();
        if (newPositionX != encXPosOld) {
            encXPosOld = newPositionX;
            Serial.println(encXPosOld);
        }
    } 
    if (encYReq == true) {
        long newPositionY = encY.read();
        if (newPositionY != encYPosOld) {
            encYPosOld = newPositionY;
            Serial.println(encYPosOld);
        }
    }
    if (enc3Req == true) {
        long newPosition3 = enc3.read();
        if (newPosition3 != enc3PosOld) {
            enc3PosOld = newPosition3;
            Serial.println(enc3PosOld);
        }
    }
    if (enc4Req == true) {
        long newPosition4 = enc4.read();
        if (newPosition4 != enc4PosOld) {
            enc4PosOld = newPosition4;
            Serial.println(enc4PosOld);
        }
    }
}


void loop() {                     // main 
    // Initalize stepper motor objects:
    AccelStepper stepper0 = initStepper(0);                                                         // Stepper0 (X-Axis motor)
    AccelStepper stepper1 = initStepper(1);                                                         // Stepper1 (Y-Axis motor)
    AccelStepper stepper2 = initStepper(2);                                                         // Stepper2 (Reservoir metering motor)
    AccelStepper stepper3 = initStepper(3);                                                         // Stepper3 (Auger motor)

    // Initalize limit switches:
    #define BUTTON_DB_DELAY 100     // ms
    // Create input DB object:
    static InputDebounce limitSwitch1;
    static InputDebounce limitSwitch2;
    // Setup debounced pull-down pin:
    limitSwitch1.setup(12, BUTTON_DB_DELAY, InputDebounce::PIM_EXT_PULL_DOWN_RES);          // Pin 12
    limitSwitch2.setup(13, BUTTON_DB_DELAY, InputDebounce::PIM_EXT_PULL_DOWN_RES);          // Pin 13
    

    /* Working stepper code: will run to +1600 steps then to -1600 steps continually. */
    // Run stepper +/- 1600
    int pos = 1600;
    stepper0.setMaxSpeed(4000);
    stepper0.setAcceleration(1000);
    while (true) {
        if (stepper0.distanceToGo() == 0) {
            delay(500); // ms
            pos = -pos;
            stepper0.moveTo(pos);
        }
        bool isrunning = stepper0.run();
        bool ls1Status = debounce(limitSwitch1);
        Serial.println("Limit Switch 1 Status" + String(ls1Status));
    }
    exit (0);
}