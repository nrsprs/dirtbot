// Code: https://www.hackster.io/guptaaryan1010/easiest-way-to-connect-lcd-screen-to-arduino-mega-973682
// Why we select cu port: https://stackoverflow.com/questions/8632586/whats-the-difference-between-dev-tty-and-dev-cu-on-macos
// PIO build configs api: https://docs.platformio.org/en/latest/projectconf/build_configurations.html

// For running a 16 pin analog LCD display on the R3 Mega
#include <Arduino.h>
#include <LiquidCrystal.h>
// initialize the library by associating any needed LCD interface pin
// with the arduino pin number it is connected to
const int rs = A3, en = A5, d4 = A9, d5 = A10, d6 = A11, d7 = A12;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);                              // throws err here:


void setup() {
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

 // Print a message to the LCD.
 lcd.print("hello, world!");
}


void loop() {
 // set the cursor to column 0, line 1
 // (note: line 1 is the second row, since counting begins with 0):
 lcd.setCursor(0, 1);
 // print the number of seconds since reset:
 lcd.print(millis() / 1000);
}