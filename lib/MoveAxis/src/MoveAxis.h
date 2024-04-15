#ifndef MOVE_AXIS_H
#define MOVE_AXIS_H

#include <AccelStepper.h>
#include <Encoder.h>
#include <InputDebounce.h>
#include <Debounce.h>
#include <EncoderSteps.h>

/*
Call axis, move a distance, direction and size should be calculated before calling:

Params :: AccelStepper stepper, Encoder encoder, InputDebounce switchObj, double linearDist
Returns :: float result

Function pesudo:
bool moveXAxis(stepperX, encX, LSX, position) {
    ## Angular to linear: https://pressbooks.bccampus.ca/humanbiomechanics/chapter/6-1-rotation-angle-and-angular-velocity-2/
    ## wtf is a microstep: https://www.linearmotiontips.com/microstepping-basics/#:~:text=Microstepping%20control%20divides%20each%20full,or%2051%2C200%20microsteps%20per%20revolution.

    # linear distance = radius * angle(rads)
    
    wheel radius = 13.97    # mm 
    driver pulse per rev = 400 
    1 rev = 2 pi # rads
    
    ## what angle do I need to spin to? 
    angle_rads = position (->mm) / 13.97 mm
    # Convert from rads to revolutions:
    angle_revs = angle_rads / 2*pi
    distance_in_pulses = angle_revs * 400 # ppr
    
    # Calculate encoder pulse position:

    while encoder position has not reached goal ()
    ## Move motor n pulses:

    ## Check encoder position after every pulse moved, make sure encoder position is <= the
} */
float MoveAxis(AccelStepper& stepper, Encoder& encoder, InputDebounce& switchObj, double linearDist);

#endif