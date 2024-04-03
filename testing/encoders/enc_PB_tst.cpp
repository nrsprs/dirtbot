#include <Arduino.h>

#define Switch 11
#define delay1 500

void setup() {
    Serial.begin(9600);
    Serial.println("Serial Begun.");
    pinMode (Switch, INPUT_PULLUP); // Set the switch as input, so that when not pressed the value equals one (1) Serial.begin(9600); }
}

void loop() {

    Serial.print("Encoder Switch = ");
    Serial.println(digitalRead(Switch)); // print current value of Switch
    delay(delay1);

}