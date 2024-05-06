#include <MoveAxis.h>

float MoveAxis(AccelStepper& stepper, Encoder& encoder, InputDebounce& switchObj, double linearDist) {
    // Convert linear distance (in mm) to rotations:
    const float wheel_radius = 13.97;    // mm
    const int driver_ppr = 400;           // set on stepper controller
    const float rev_rads = 6.283185307179586476925286766559;
    volatile long encoderPos = 0;
    int rotDir = 0;
    const int encoder_ppr = 20*4;           // 20 ppr encoder, res of 4/ppr using encoder.h
    float result = 0;

    // Calculate angle needed to be reached: 
    float angle_rads = linearDist/wheel_radius;
    //Convert from rads to revs:
    float angle_revs = angle_rads/rev_rads;
    // Calculate pulse distance:
    float distance_pulses = angle_revs * driver_ppr;
    // Calculate encoder pulse position required: 
    long finalEncPos = int(round(angle_revs)) * encoder_ppr;

    if (linearDist <= 0) {          // Positive distance corresponds to CW direction
        rotDir = -1;}           // Set CCW
    else {rotDir = 1;}          // Set CW

    // Call stepper and set params:
    const float vel = 500.0;
    stepper.setAcceleration(vel*rotDir/2);
    stepper.setMaxSpeed(vel*1.2*rotDir); 
    stepper.move(distance_pulses);

    Serial.println("Moving axis by: " + String(distance_pulses) + " pulses");

    // Move to angular position:
    while (Debounce(switchObj) != 1) {
        // Check encoder pos: 
        if (rotDir == 1) {       // Check if axis should be moving positively:
            stepper.setSpeed(vel);
            if (stepper.distanceToGo() != 0) {
                stepper.run();
                encoderPos = EncoderSteps(encoder, encoderPos);
            }
        }
        if (rotDir == -1) {     // Check if the axis should be moving negatively:
            stepper.setSpeed(-vel);
            if (stepper.distanceToGo() != 0) {
                stepper.run();
                encoderPos = EncoderSteps(encoder, encoderPos);
            }
        }
        if (stepper.distanceToGo() == 0) {
            break;
        }
    }
    // Moved to position, calculate error:
    int err_ppr = finalEncPos - encoderPos;
    float err_revs = err_ppr / encoder_ppr;
    float err_rads = err_revs * rev_rads; 
    result = err_rads * wheel_radius;
    Serial.println("Error in mm: " + String(result));

    return result;
}