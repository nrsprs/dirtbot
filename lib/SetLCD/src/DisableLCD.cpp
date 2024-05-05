#include <DisableLCD.h>

// Set the LCD display pins all to false to disable display momentarily. 
void DisableLCD(){
    digitalWrite(A14,HIGH); 
    digitalWrite(A13,LOW); 
    digitalWrite(A4,HIGH); 
    digitalWrite(A0,HIGH);
    digitalWrite(A2,HIGH);
    digitalWrite(A1,LOW);
}