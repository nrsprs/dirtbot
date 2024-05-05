#include "InitUserInput.h"

Vector<int> InitUserInput(LiquidCrystal& lcd, Encoder& encoder, InputDebounce& pushButton) {
    // Get pin and reset pin config from interrupt config:
    int pbPin = pushButton.getPinIn();
    pushButton.setup(pbPin, 100, InputDebounce::PIM_INT_PULL_UP_RES);
    Serial.println("Set pin location: " + String(pbPin));

    // User Input & Start Screen:
    lcd.clear();
    int storage_array[2];
    Vector<int> processParams(storage_array);
    volatile int encPos = -99;
    volatile bool start_cmd = 0;
    volatile bool dirt_filled = 0;
    volatile bool tray_size_x = 0;
    int x_tray_cnt;
    volatile bool tray_size_y = 0;
    int y_tray_cnt;
    volatile bool start_confirmation = 0;
    int _testing = 0;
    
    while (start_cmd == 0) {
        // Q: Dirt filled state:
        volatile bool pbStatus;
        volatile int prevEncPos = 0;
        while (dirt_filled == 0) {
            volatile bool dirt_fill_yes = 0;
            encPos = 0;
            lcd.setCursor(0, 0);
            lcd.print("IS DIRT FILLED? ");
            encPos = -EncoderSteps(encoder, prevEncPos);
            
            if (encPos <= 0) {       // "NO" state
                lcd.setCursor(0,1);
                lcd.print("  [] NO    YES  ");
            }
            if (encPos > 0) {      // "YES" state
                lcd.setCursor(0,1);
                lcd.print("     NO [] YES  ");
                dirt_fill_yes = 1;
                pbStatus = Debounce(pushButton);
            }

            if (_testing >= 1) {
            pbStatus = 1;
            dirt_fill_yes = 1;
            }
            if ((pbStatus == 1) && (dirt_fill_yes == 1)) {        // ON Pressed state
                lcd.clear();
                lcd.setCursor(4,0);
                lcd.print("RECEIVED:");
                lcd.setCursor(7,1);
                lcd.print("YES");
                dirt_filled = 1;            // End prompt
                delay (2000);
                prevEncPos = 0;
                lcd.clear();
                encPos = 0;
            }
        }
    while (tray_size_x == 0) {
        // Q: TRAY X SIZE? 
        lcd.setCursor(0, 0);
        lcd.print("  TRAY X SIZE:    ");
        encPos = -EncoderSteps(encoder, encPos);
        
        if (encPos <= 0) {       // Num trays is less than 0, just set to 0..
            x_tray_cnt = 0;
            encPos = 0;
            prevEncPos = 0;
            lcd.setCursor(0,1);
            lcd.print("    [" + String(x_tray_cnt) + "] PLUGS    ");
        }
        if (encPos > 0) {                                   //  Increment counter for number of trays: 
            x_tray_cnt = encPos/4;                          // 1 Increment is 4 ticks 
            if (x_tray_cnt >= 4) {x_tray_cnt = 4;}
            lcd.setCursor(0,1);
            lcd.print("    [" + String(x_tray_cnt) + "] PLUGS    ");
            pbStatus = Debounce(pushButton);
        }

        if (_testing >= 2) {
            pbStatus = 1;
            x_tray_cnt = 2;
        }
        if ((pbStatus == 1) && (x_tray_cnt >= 0)) {        // ON Pressed state
            lcd.clear();
            lcd.setCursor(4,0);
            lcd.print("RECEIVED:");
            lcd.setCursor(4,1);
            lcd.print("[" + String(x_tray_cnt) + "] PLUGS    ");
            tray_size_x = 1;                                // End prompt
            processParams.push_back(x_tray_cnt);
            delay (2000);
            lcd.clear();
            encPos = 0;
        }
    }
    encPos = 0;
    while (tray_size_y == 0) {
        // Q: TRAY Y SIZE? 
        lcd.setCursor(0, 0);
        lcd.print("  TRAY Y SIZE:    ");
        encPos = -EncoderSteps(encoder, encPos);
        
        if (encPos <= 0) {       // Num trays is less than 0, just set to 0..
            y_tray_cnt = 0;
            encPos = 0;
            prevEncPos = 0;
            lcd.setCursor(0,1);
            lcd.print("    [" + String(y_tray_cnt) + "] PLUGS    ");
        }
        if (encPos > 0) {                                   //  Increment counter for number of trays: 
            y_tray_cnt = encPos/4;                          // 1 Increment is 4 ticks 
            if (y_tray_cnt >= 3) {y_tray_cnt = 3;}         // Keep the tray count maxxed at 4
            lcd.setCursor(0,1);
            lcd.print("    [" + String(y_tray_cnt) + "] PLUGS    ");
            pbStatus = Debounce(pushButton);
        }

        if (_testing >= 3) {
            pbStatus = 1;
            y_tray_cnt = 3;
        }
        if ((pbStatus == 1) && (y_tray_cnt >= 0)) {         // ON Pressed state
            lcd.clear();
            lcd.setCursor(4,0);
            lcd.print("RECEIVED:");
            lcd.setCursor(4,1);
            lcd.print("[" + String(y_tray_cnt) + "] PLUGS    ");
            tray_size_y = 1;                                // End prompt
            processParams.push_back(y_tray_cnt);            // Insert y count to return vector
            delay (2000);
            lcd.clear();
            encPos = 0;
            prevEncPos = 0;
        }
    }
    while (start_confirmation == 0) {
        volatile bool start_confirmed = 0;
        encPos = 0;
        lcd.setCursor(0, 0);
        lcd.print("START DIRTBOT? ");
        encPos = -EncoderSteps(encoder, prevEncPos);
        
        if (encPos <= 0) {       // "NO" state
            lcd.setCursor(0,1);
            lcd.print("  [] NO    YES  ");
        }
        if (encPos > 0) {      // "YES" state
            lcd.setCursor(0,1);
            lcd.print("     NO [] YES  ");
            start_confirmed = 1;
            pbStatus = Debounce(pushButton);
        }

        if (_testing >= 4) {
            pbStatus = 1;
            start_confirmed = 1;
        }
        if ((pbStatus == 1) && (start_confirmed == 1)) {        // ON Pressed state
            lcd.clear();
            lcd.setCursor(4,0);
            lcd.print("RECEIVED:");
            lcd.setCursor(7,1);
            lcd.print("YES");
            start_confirmation = 1;            // End prompt
            start_cmd = 1;
            delay (2000);
            prevEncPos = 0;
            lcd.clear();
            encPos = 0;
        }
    } 
    }
    return processParams;
}
