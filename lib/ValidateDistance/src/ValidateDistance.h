#ifndef VALIDATE_DISTANCE_H
#define VALIDATE_DISTANCE_H

#include <AccelStepper.h>
#include <Encoder.h>
#include <EncoderSteps.h>
#include <MoveAxis.h>

/*
Run a certain distance forwards and then backwards using MoveAxis.

Params :: stepper, encoder, switchObj

Returns :: None
*/
void ValidateDistance(AccelStepper& stepper, Encoder& encoder, InputDebounce& switchObj);

#endif