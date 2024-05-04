#include "RunAuger.h"

void RunAuger(AccelStepper& stepper, Encoder& encoder) {
    const int encoder_ppr = 20*4;           // 20 ppr encoder, res of 4/ppr using encoder.h
    const int driver_ppr = 400;             // set on stepper controller
    const float num_revs = 12;                                                //## Testing:: need to validate this nunmber.                       
    float steps = -driver_ppr * num_revs;
    float finalEncPos = encoder_ppr * num_revs;
    long encPos = 0;
    
    // Set gpio for vibratory auger: 
    int hopperPin = 38;                      // Pin 37
    pinMode(hopperPin,OUTPUT);
    digitalWrite(hopperPin,HIGH); 
    delay(5000);                            // Delay for 2 seconds
    digitalWrite(hopperPin,LOW);

    // Call stepper and set params:
    const float vel = 1000.0;
    stepper.setAcceleration(vel);        // Auger runs CW
    stepper.setMaxSpeed(vel*1.2); 
    stepper.setSpeed(vel);
    stepper.setCurrentPosition(0);
    stepper.moveTo(steps);

    Serial.println("Moving hopper by: " + String(steps) + " steps.");

    while ((stepper.distanceToGo() != 0) && (encPos < finalEncPos)) {
        stepper.run();
        encPos = EncoderSteps(encoder, encPos);
    }
}