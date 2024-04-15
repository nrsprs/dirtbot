#ifndef RUN_AUGER_H
#define RUN_AUGER_H

#include <AccelStepper.h>
#include <Encoder.h>
#include <EncoderSteps.h>

/*
Run the auger once to half the number of revolutions needed to dispense the whole volume of soil. 
Params :: AccelStepper stepper, Encoder encoder

Returns :: None
*/
void RunAuger(AccelStepper& stepper, Encoder& encoder);

#endif