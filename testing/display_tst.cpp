// https://www.hackster.io/guptaaryan1010/easiest-way-to-connect-lcd-screen-to-arduino-mega-973682

// For running a 16 pin analog LCD display on the R3 Mega
#include <Arduino.h>
#include "LiquidCrystal/src/LiquidCrystal.h"
// initialize the library by associating any needed LCD interface pin
// with the arduino pin number it is connected to
const int rs = 12, en = 11, d4 = 5, d5 = 4, d6 = 3, d7 = 2;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);
void setup() {
 // set up the LCD's number of columns and rows:
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