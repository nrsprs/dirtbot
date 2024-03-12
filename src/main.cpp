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


void setup()                     // init 
{  
    // Begin serial:
    Serial.begin(9600);
    Serial.println("Serial Begun.");

    // Assign Pin Modes: 
    pinMode(LED_BUILTIN, OUTPUT);       // Built-in LED
    // Establish LCD:
    pinMode(A14,OUTPUT);
    pinMode(A13,OUTPUT);
    pinMode(A4,OUTPUT);
    pinMode(A0,OUTPUT);
    pinMode(A2,OUTPUT);
    pinMode(A1,OUTPUT);
    digitalWrite(A14,LOW); 
    digitalWrite(A13,HIGH); 
    digitalWrite(A4,LOW); 
    digitalWrite(A0,LOW);
    digitalWrite(A2,LOW);
    digitalWrite(A1,HIGH);
}


AccelStepper initStepper(int indx) {
    // Motor Connections for constant current, step/direction bipolar motor driver
    typedef struct {
        const int stepPin;
        const int dirPin;
        } pinDictionary;

        pinDictionary PinDict[] = {
        {22, 23},                    // Stepper 0 : X-Axis motor pins
        {24, 25},                  // Stepper 1: Y-Axis motor pins
        {26, 27},                  // Stepper 2: Reservoir metering motor pins
        {28, 29}                   // Stepper 3: Auger motor pins
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


long encoderSteps(Encoder& encoder, long prevPos) {
    // Returns encoder steps given previous steps. 
    volatile long newPos = encoder.read();
    if (newPos != prevPos) {
        prevPos = newPos;
    }
    return newPos;
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
    limitSwitch1.setup(4, BUTTON_DB_DELAY, InputDebounce::PIM_EXT_PULL_DOWN_RES);          // Pin 4
    limitSwitch2.setup(5, BUTTON_DB_DELAY, InputDebounce::PIM_EXT_PULL_DOWN_RES);          // Pin 5


    // Initalize encoder objects:
    static Encoder encX(20, 21);           // Pins 20 & 21 are for X-Axis encoder (intr 1 & 0)
    static Encoder encY(18, 19);           // Pins 18 & 19 are for Y-Axis encoder (intr 2 & 3)
    static Encoder enc3(2, 3);             // Pins 2 & 3 are for the hopper encoder (intr 4 & 5)
    // static Encoder enc4(??, ??);        // Pins ?? & ?? are for the auger encoder (intr ?? & ??)
    long encXPos = -99, encYPos = -99, enc3Pos = -99, enc4Pos = -99;
    

    // Initalize LCD:
    const int rs = A3, en = A5, d4 = A9, d5 = A10, d6 = A11, d7 = A12;
    LiquidCrystal lcd(rs, en, d4, d5, d6, d7);
    lcd.begin(16, 2);
    lcd.print("DIRTBOT  STARTED");
    

    // Demoing Encoders:
    while (true) {
        enc3Pos = encoderSteps(enc3, enc3Pos);
        lcd.setCursor(0, 1);
        lcd.println(String(enc3Pos) + " STEPS          ");

    }

    /* Working stepper code: will run to +1600 steps then to -1600 steps continually. */
    // Run stepper +/- 1600
    volatile int pos = 1600;
    stepper0.setMaxSpeed(4000);
    stepper0.setAcceleration(1000);
    while (true) {
        if (stepper0.distanceToGo() == 0) {
            delay(500); // ms
            pos = -pos;
            stepper0.moveTo(pos);
        }
        volatile bool isrunning = stepper0.run();
        volatile bool ls1Status = debounce(limitSwitch1);
        Serial.println("Limit Switch 1 Status" + String(ls1Status));
    }
    exit (0);
}