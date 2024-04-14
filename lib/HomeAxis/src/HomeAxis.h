#ifndef HOME_AXIS_H
#define HOME_AXIS_H

#include <Encoder.h>
#include <AccelStepper.h>
#include <InputDebounce.h>
#include <Debounce.h>
#include <EncoderSteps.h>

/*
Closed loop single axis control scheme: 

call axis, set home:
double homeXAxis(stepperX, encX, LSX) {
    determine direction stepper needs to rotate to get close to home (pos/neg; cc/ccw)
    set stepper to half speed 
    drive stepper towards LS
    check encoder position
    check LS
    if LS is triggered:
        stop stepper
        reset encoder position to 0
        roll off in ~+80 steps opposite direction for double-tap
        roll back ~-80 steps, slowly
        if LS is triggered:
            stop stepper
            reset encoder position to 0
            roll off by ~+40 steps to release LS
    return stepper currentPosition, encoderPos
} */
long HomeAxis(AccelStepper& stepper, Encoder& encoder, InputDebounce& switchObj, int rotDir);

#endif