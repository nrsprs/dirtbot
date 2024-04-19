#include "HomeAxis.h"

long HomeAxis(AccelStepper& stepper, Encoder& encoder, InputDebounce& switchObj, int rotDir) {
    // Determine rotation direction for home | CW is pos, CCW is neg
    const float homeVel = 300.0;                                                    // TESTING: update speed
    volatile long encoderPos = 0;
    stepper.setAcceleration(homeVel*rotDir/2);
    stepper.setMaxSpeed(homeVel*rotDir*1.2);
    stepper.setSpeed(homeVel*rotDir);
    
    Serial.println("Running stepper...");
    // Move in direction of home:
    while (Debounce(switchObj) != 1) {           // Testing:: check if switch is active high or low in circuit, result bit still is 1..
        stepper.runSpeed();
        encoderPos = EncoderSteps(encoder, encoderPos);
    }
    Serial.println("Switch triggered");
    // Switch has been hit, reset positions:
    encoderPos = 0;
    stepper.setCurrentPosition(0);
    // Move off by 80 steps, return and wait for switch to be hit:
    stepper.moveTo(-80*rotDir);
    delay(200);
    stepper.setSpeed(homeVel*rotDir/1.5);
    while ((stepper.distanceToGo() != 0) && (Debounce(switchObj) != 1)) {stepper.run();}
    // Send back to home:
    stepper.moveTo(100*rotDir);
    delay(200);
    while ((stepper.distanceToGo() != 0) && (Debounce(switchObj) != 1)) {stepper.run();}
    // Has hit home again, move off 60 steps and return encoder position:
    stepper.moveTo(60*rotDir);
    encoderPos = 0;
    while (stepper.distanceToGo() != 0) {
        stepper.run();
        encoderPos = EncoderSteps(encoder, encoderPos);
    }    
    Serial.println("Ending function, stepper pos: " + String(stepper.currentPosition()) + " encoder pos: " + encoderPos);
    return stepper.currentPosition(), encoderPos;
}