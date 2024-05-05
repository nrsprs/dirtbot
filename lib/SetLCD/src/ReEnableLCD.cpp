#include <ReEnableLCD.h>

// Re-Enable LCD after turning off momentarily. 
void ReEnableLCD(LiquidCrystal& lcd) {
    digitalWrite(A14,LOW); 
    digitalWrite(A13,HIGH); 
    digitalWrite(A4,LOW); 
    digitalWrite(A0,LOW);
    digitalWrite(A2,LOW);
    digitalWrite(A1,HIGH);

    lcd.begin(16, 2);
    lcd.backlight();
}
