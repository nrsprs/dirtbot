#include <Arduino.h>
#include <Encoder.h>
#include <LiquidCrystal.h>


Encoder myEnc(2, 3);
long oldPosition  = -99;

const int rs = A3, en = A5, d4 = A9, d5 = A10, d6 = A11, d7 = A12;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);                              // throws err here:


void interpretEncoder();

void setup() {
    Serial.begin(9600);
    Serial.println("Basic Encoder Test:");
    attachInterrupt(digitalPinToInterrupt(2), interpretEncoder, CHANGE);
    attachInterrupt(digitalPinToInterrupt(3), interpretEncoder, CHANGE);

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
    lcd.begin(16, 2);
    lcd.print("Encoder Test:");
}

void interpretEncoder() {
    noInterrupts();
    long newPosition = myEnc.read();
    interrupts();
    if (newPosition != oldPosition) {
        oldPosition = newPosition;
        Serial.println(newPosition);
    }
}

void loop() {
    lcd.setCursor(0, 1);
    lcd.print(String(oldPosition) + " " + " " + " ");
}