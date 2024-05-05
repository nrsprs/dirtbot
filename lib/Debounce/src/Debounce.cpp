#include "Debounce.h"

bool Debounce(InputDebounce& switch_object) {
    // Reset vars for switches:
    static unsigned int LS1_Count = 0;
    static unsigned int LS1_OnTimeLast = 0;
    bool result = 0;

    unsigned long now = millis();
    unsigned int buttonTest_OnTime = switch_object.process(now);

    // Handle input button
    if (buttonTest_OnTime) {
        // Save current on-time (button pressed) for release
        LS1_OnTimeLast = buttonTest_OnTime;
        // Check for state change
        unsigned int count = switch_object.getStatePressedCount();
        if (LS1_Count != count) {
            LS1_Count = count;
            // Handle pressed state
            digitalWrite(LED_BUILTIN, HIGH);                 // Turn on built_in LED
            result = 1;
            // Serial.println("HIGH");
        }
        else {
            // Handle still pressed state
            // Serial.println("HIGH still pressed (" + String(buttonTest_OnTime) + "ms)");
        }
    }
    else {
        if (LS1_OnTimeLast) {
            // Handle released state
            digitalWrite(LED_BUILTIN, LOW);                 // Turn off built_in LED
            result = 0;
            // Serial.print("LOW (last on-time: HIGH (" + String(LS1_OnTimeLast) + ")");
            LS1_OnTimeLast = 0;                             // Reset 
        }
    }
    return result; 
}