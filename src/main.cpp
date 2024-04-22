/*
DirtBot main demo sequence

Built for use on Arduino AVR Architecture using a MEGA 2560 board

Developed for Professors Caldwell, Tawaf, and Zoyhofski
Class of 2024, RMET

Author :: Nick Spears
Version :: 1.0
*/

#include <Arduino.h>
#include <AccelStepper.h>
#include <Debounce.h>
#include <Dictionary.h>
#include <elapsedMillis.h>
#include <Encoder.h>
#include <EncoderSteps.h>
#include <eStopTrigger.h>
#include <HomeAxis.h>
#include <InitUserInput.h>
#include <InputDebounce.h>
#include <LiquidCrystal.h>
#include <MoveAxis.h>
#include <RunAuger.h>
#include <RunHopper.h>
#include <StartAnimation.h>
#include <Wire.h>
#include <Vector.h>

#define elif else if
#pragma clang diagnostic ignored "-Wunused-variable"


void setup()                     // init 
{  
    // Begin serial:
    Serial.begin(250000);
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
        {22, 23},                  // Stepper 0 : X-Axis motor pins
        {24, 25},                  // Stepper 1: Y-Axis motor pins
        {26, 27},                  // Stepper 2: Reservoir metering motor pins
        {28, 29}                   // Stepper 3: Auger motor pins
        };
    AccelStepper stepper(AccelStepper::DRIVER, PinDict[indx].stepPin, PinDict[indx].dirPin);
    Serial.println(String("Index: ") + String(indx));
    Serial.println(String("DirPin: ") + String(PinDict[indx].dirPin) + String(" StepPin: ") + String(PinDict[indx].stepPin));
    // for Bipolar, constant current, step/direction driver
    return stepper;
}


// Roundup double d
double round(double d)
{return floor(d + 0.5);}


void sensorDemo(LiquidCrystal& lcd, Encoder& encX, InputDebounce& limitSwitch1) {
    // Demoing Sensors:
    lcd.clear();
    int encXPos = -99;
    while (true) {
        encXPos = EncoderSteps(encX, encXPos);
        lcd.setCursor(0, 0);
        lcd.print("ENC: " + String(encXPos));

        // Check LS Status:
        bool ls1Status = Debounce(limitSwitch1);
        lcd.setCursor(0,1);
        lcd.print("LS: " + String(ls1Status));
        if (ls1Status == 1) {Serial.println("LS STATUS: "+String(ls1Status)); delay(100);}
    }
}


void directControlDemo(AccelStepper& stepper, LiquidCrystal& lcd, Encoder& encX) {
    // Set baud rate to 250000 for accurate stepping calls. 
    // Direct encoder to motor position control test: 
    volatile int pos = 0;
    volatile int encXPos = -99;
    stepper.setSpeed(1000);
    stepper.setMaxSpeed(2000);
    stepper.setAcceleration(800);

    while (true) {
        // Get encoder status:
        encXPos = EncoderSteps(encX, encXPos);
        // If encoder is in negative space, move backwards. 
        if (pos < 0) {stepper.moveTo(-pos);}
        else {stepper.moveTo(pos);}
        pos = encXPos * 20;

        // Set and display LCD:
        lcd.setCursor(0,0);
        lcd.print("ENC: " + String(encXPos));
        lcd.setCursor(0,1);
        lcd.print("STP: " + String(pos));
        lcd.setCursor(8,0);
        lcd.print("DTG: " + String(stepper.distanceToGo()));
        Serial.println("STP Distance To Go: " + String(stepper.distanceToGo()));
        Serial.println("ENC Pos: " + String(pos));
        Serial.println("Stepper Speed: " + String(stepper.speed()));       // Returns most recent speed in steps/s
        if (stepper.distanceToGo() != 0) {
            stepper.run();
        }
    }
}



void loop() {                     // main
    // Initalize stepper motor objects:
    AccelStepper stepper0 = initStepper(0);                                      // X-Axis motor, pins 22, 23
    AccelStepper stepper1 = initStepper(1);                                      // Y-Axis motor, pins 24, 25
    AccelStepper stepper2 = initStepper(2);                                      // Hopper motor, pins 26, 27
    AccelStepper stepper3 = initStepper(3);                                      // Auger motor, pins 28, 29


    // Initalize limit switches:
    #define BUTTON_DB_DELAY 100     // ms
    int userPBPin = 2;                      // Pin 2
    // Create input DB object:
    static InputDebounce limitSwitch1;      // Pin 4
    static InputDebounce limitSwitch2;      // Pin 5
    static InputDebounce userPushButton;    // Pin 2
    // Set A15 to HI for an extra +5 VCC pin:
    pinMode(A15,OUTPUT);
    digitalWrite(A15, HIGH);
    // Setup debounced pull-down pin:
    limitSwitch1.setup(4, BUTTON_DB_DELAY, InputDebounce::PIM_EXT_PULL_DOWN_RES);          // Pin 4
    limitSwitch2.setup(5, BUTTON_DB_DELAY, InputDebounce::PIM_EXT_PULL_DOWN_RES);          // Pin 5
    userPushButton.setup(userPBPin, BUTTON_DB_DELAY, InputDebounce::PIM_INT_PULL_UP_RES);          // Pin 2 (intr 4)


    // Initalize encoder objects:
    static Encoder encX(20, 21);           // Pins 20 & 21 are for X-Axis encoder (intr 1 & 0)
    static Encoder encY(18, 19);           // Pins 18 & 19 are for Y-Axis encoder (intr 2 & 3)
    static Encoder enc3(16, 17);           // Pins 16 & 17 are for the hopper encoder (no intr)
    static Encoder enc4(14, 15);           // Pins 14 & 15 are for the auger encoder (no intr)
    static Encoder userEncKnob(12, 13);    // Pins 12 & 13 are for the user input encoder (no intr)
    volatile long encXPos = -99, encYPos = -99, enc3Pos = -99, enc4Pos = -99;
    

    // Initalize LCD:
    const int rs = A3, en = A5, d4 = A9, d5 = A10, d6 = A11, d7 = A12;
    LiquidCrystal lcd(rs, en, d4, d5, d6, d7);
    lcd.begin(16, 2);
    lcd.backlight();
    /*====  TESTING CONFIGURATIONS  ====*/


    // Sensor demo to make sure that your sensors actually work:
    // sensorDemo(lcd, userEncKnob, userPushButton);


    // Direct encoder to motor position control test: 
    // directControlDemo(stepper1, lcd, userEncKnob);


    // runAuger() test:
    // RunAuger(stepper0, encX);
    // exit(0);


    // RunHopper() test: 
    // RunHopper(stepper0, encX);

    // Home Axis Test:
    if (1==0) {
        long home_pos_stepper = 1.2;
        long home_pos_enc = 0.5;
        // Control Loop Demo:
        lcd.clear();
        lcd.setCursor(0,0);
        lcd.print("HOMING X-AXIS...");
        //          |0123456789ABCDEF|

        static_cast<int>(home_pos_stepper);
        static_cast<int>(home_pos_enc);
        lcd.setCursor(0,1);
        lcd.print("STP: " + String(home_pos_stepper) + " ENC: " + String(home_pos_enc));

        // Call homeAxis:
        home_pos_stepper, home_pos_enc = HomeAxis(stepper1, encX, limitSwitch1, 1);      // Testing home on X-axis, move CW to get to home.
        Serial.print("Finished Home Sequence for X Axis...");
        static_cast<int>(home_pos_stepper); //type: ignore
        static_cast<int>(home_pos_enc);
        lcd.clear();
        lcd.setCursor(0,0);
        lcd.print("Homing Done...");
        lcd.setCursor(0,1);
        lcd.print("STP: " + String(home_pos_stepper) + " ENC: " + String(home_pos_enc));

        delay(5000);
        Serial.print("DONE");
        while (1) {}

        Serial.println("Exit...");
        exit(0);
    }


    /*
    Guidelines for calling stepper.run():
     * * Minimize math, LCD, Serial, and encoder calls while in loop that calls stepper.run()
     * * Maximize speed of stepper by maintaining at least as many calls as the pwm rate
     * * Increase baud rate to 250000 to reduce lost steps
    
    Motor Direction Map: 
    auger: CW 
    hopper: CCW
    x-axis: CW to home
    y-axis: CCW to home
    */


   /*====  START OF ROUTINE  =====*/

    // Start Animation: 
    bool startup_animation = 0;
    if (startup_animation == 1) {StartAnimation(lcd);}

    // Get user input and number of plugs in the X and Y direction:
    Vector<int> processParams = InitUserInput(lcd, userEncKnob, userPushButton);
    Serial.println("User Output X: " + String(processParams[0]));
    Serial.println("User Output Y: " + String(processParams[1]));
    delay(3000);

    // Set the user push button as a stop-everything interrupt: 
    // ! This has to be after any not eStop user inputs are made; meaning that the interrupt needs to be unattached after the routine
    pinMode(userPBPin, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(userPBPin), eStopTrigger, CHANGE);

    /*===  STATE MACHINE  ===*/

    /* 
        start
        home y axis
        home x axis
        move auger under hopper 
        call hopper
        while (y=0; y++; y<=processParams[1]) {
            while (x=0; x++; x<=processParams[0]) {
                print(moving to x, y)
                moveAxis (distance between home and plug 1,1)
                runAuger()
                moveAxis (-distance to home)
            }
        }
    */

    bool routine_is_done = 0;
    while (routine_is_done == 0) {
        // Home y axis, move in the CW dir:                     ! FIX DIR IN TESTING !
        HomeAxis(stepper1, encY, limitSwitch2, -1);
        // Home x axis, move in the CCW dir:                    ! FIX DIR IN TESTING !
        HomeAxis(stepper0, encX, limitSwitch1, 1);

        // Move the auger under the hopper: 
        MoveAxis(stepper0, encX, limitSwitch2, 20);     //      ! DOUBLE CHECK THE LINEAR DISTANCE OUTPUT !
    } 

    exit (0);
}