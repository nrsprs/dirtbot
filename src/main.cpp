/*
DirtBot SPI

Built for use on Arduino AVR Architecture using an UNO R3 board.

Developed for Professors Caldwell, Tawaf, and Zoyhofski
Class of 2024, RMET

Author :: Nick Spears
Version :: 1.0
*/

#include <Arduino.h>
#include <Debounce.h>

#pragma clang diagnostic ignored "-Wunused-variable"


void setup()                     // init 
{  
    // Begin serial:
    Serial.begin(250000);
    Serial.println("Serial Begun.");

    // Assign Pin Modes: 
    pinMode(LED_BUILTIN, OUTPUT);       // Built-in LED

    // Set input pins:
    pinMode(2, INPUT);
    pinMode(3, INPUT);

    // Set outpt pins:
    pinMode(4, OUTPUT);
    pinMode(5, OUTPUT);
    pinMode(6, OUTPUT); 
}


void loop() {                     // main
    // Assign debounce inputs:
    // static InputDebounce augerPin;          // Pin 2
    // static InputDebounce hopperPin;         // Pin 3
    
    // Setup debounced pull-down pin:
    // const int BUTTON_DB_DELAY = 20;
    // augerPin.setup(2, BUTTON_DB_DELAY, InputDebounce::PIM_EXT_PULL_DOWN_RES);                 // auger pin 2
    // hopperPin.setup(3, BUTTON_DB_DELAY, InputDebounce::PIM_INT_PULL_UP_RES);                 // hopper pin 3
    
    bool augerStatus = 0;
    bool hopperStatus = 0;

    while (1) {
        // Check for inputs:
        // augerStatus = Debounce(augerPin);
        // hopperStatus = Debounce(hopperPin);
        augerStatus = digitalRead(2);
        hopperStatus = digitalRead(3);

        // Handle auger:
        if (augerStatus == 1) {
            digitalWrite(4, HIGH);
            Serial.println(String(augerStatus));
        }
        if (augerStatus == 0) {
            digitalWrite(4, LOW);
        }

        // Handle hopper:
        if (hopperStatus == 1) {
            digitalWrite(5, HIGH);
            digitalWrite(6, HIGH);
        }
        if (hopperStatus == 0) {
            digitalWrite(5, LOW);
            digitalWrite(6, LOW);
        }
    }
}