#include "RunHopper.h"

void RunHopper(AccelStepper& stepper, Encoder& encoder) {
    const int encoder_ppr = 20*4;           // 20 ppr encoder, res of 4/ppr using encoder.h
    const int driver_ppr = 400;             // set on stepper controller
    const float num_revs = 0.2;
    long steps = driver_ppr * num_revs;
    long finalEncPos = encoder_ppr * num_revs;
    long encPos = 0;
    
    // Set gpio for vibratory hopper: 
    int hopperPinA = 36;  
    int hopperPinB = 37;     
                   // Pin 36
    pinMode(hopperPinA,OUTPUT);
    pinMode(hopperPinB,OUTPUT);
    digitalWrite(hopperPinA,HIGH);
    digitalWrite(hopperPinB,HIGH);
    delay(2000);                            // Delay for 2 seconds
    digitalWrite(hopperPinA,LOW);
    digitalWrite(hopperPinB, LOW);

    // Call stepper and set params:
    const float vel = 500.0;
    stepper.setAcceleration(-vel/2);        // Hopper runs CCW
    stepper.setMaxSpeed(-vel*1.2); 
    stepper.setSpeed(vel);
    stepper.setCurrentPosition(0);
    stepper.moveTo(steps);

    Serial.println("Moving hopper by: " + String(steps) + " steps.");

    while ((stepper.distanceToGo() != 0) && (encPos < finalEncPos)) {
        stepper.run();
        encPos = EncoderSteps(encoder, encPos);
    }
    
    // Reset gpio for vibratory hopper: 
    digitalWrite(15,LOW); 
}