#include "eStopTrigger.h"

void eStopTrigger() {
    // Re-init LCD:
    const int rs = A3, en = A5, d4 = A9, d5 = A10, d6 = A11, d7 = A12;
    LiquidCrystal lcd(rs, en, d4, d5, d6, d7);
    lcd.begin(16, 2);

    // Assume user input knob is on pins 12 & 13:
    static Encoder userEncKnob(12, 13);    // Pins 12 & 13 are for the user input encoder (no intr)

    // Assume user input button is on pin 2:
    static InputDebounce userPushButton;    // Pin 2

    lcd.clear();
    volatile int encPos = -99;
    volatile int prevEncPos = 0;
    volatile bool restart_confirmation = 0;
    volatile bool buttonPress = 0;

    /*
    LCD Char Placement:
   =================
   0[DIRTBOT PAUSED]
   1 RESUME? [] NO
   1 RESUME? [] YES 
   =0123456789ABCDEF
    */
    while (restart_confirmation == 0) {
        encPos = 0;
        lcd.setCursor(0,0);
        lcd.print("[DIRTBOT PAUSED]");
        lcd.setCursor(0,1);
        
        encPos = EncoderSteps(userEncKnob, prevEncPos);
        if (encPos <= 0) {       // "NO" state
            lcd.setCursor(0,1);
            lcd.print(" RESUME? [] NO  ");
        }
        if (encPos > 0) {      // "YES" state
            lcd.setCursor(0,1);
            lcd.print(" RESUME? [] YES ");
            buttonPress = Debounce(userPushButton);
        }

        if (buttonPress == 1) {        // ON Pressed state
            lcd.clear();
            lcd.setCursor(4,0);
            lcd.print("RECEIVED:");
            lcd.setCursor(7,1);
            lcd.print("YES");
            delay (2000);
            prevEncPos = 0;
            lcd.clear();
            encPos = 0;
            restart_confirmation = 1;       // Allow an exit to while loop
        }
    }
}