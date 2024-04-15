#ifndef RUN_HOPPER_H
#define RUN_HOPPER_H

#include <AccelStepper.h>
#include <Encoder.h>
#include <EncoderSteps.h>

/*
Run the hopper once to the number of revolutions needed to dispense the whole volume of soil. 
Params :: AccelStepper stepper, Encoder encoder

Returns :: None
*/
void RunHopper(AccelStepper& stepper, Encoder& encoder);

#endif