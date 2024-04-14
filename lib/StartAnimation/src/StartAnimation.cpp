#include "StartAnimation.h"

void StartAnimation(LiquidCrystal& lcd) {
    lcd.setCursor(0,0);
    lcd.print("DIRTBOT  BOOTING");
    lcd.setCursor(0,1);
    for (int _i = 0; _i <= 16; ++_i) {
        lcd.print(".");
        lcd.setCursor(_i,1);
        digitalWrite(LED_BUILTIN, HIGH);
        delay(200);
        digitalWrite(LED_BUILTIN, LOW);
        delay(50);
    }
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print(" DIRTBOT  READY");
    lcd.setCursor(0,1);
    delay(2000);
    lcd.clear();
}