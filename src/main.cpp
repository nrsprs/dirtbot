#include <Arduino.h>
#include "MotorDriver/src/MotorDriver.h"

// put function declarations here:
int myFunction(int, int);

void setup() {
  // put your setup code here, to run once:
  int result = myFunction(2, 3);
  Serial.print(result);
}

void loop() {
  // put your main code here, to run repeatedly:w

  // just Blink.ino:
  digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(1000);                       // wait for a second
  digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
  delay(1000);                       // wait for a second
  Serial.print("Blink\n");
}

// put function definitions here:
int myFunction(int x, int y) {
  return x + y;
}