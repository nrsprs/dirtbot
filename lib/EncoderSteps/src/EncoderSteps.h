#ifndef ENCODER_STEPS_H
#define ENCODER_STEPS_H

#include <Encoder.h>

/* Calculates whether encoder has stepped. 
Encoder position calculation is 4x PPR, so a 20 PPR encoder will have 80 steps.

Params :: Encoder object, long previous encoder position

Returns :: Updated encoder step position*/
long EncoderSteps(Encoder& encoder, long prevPos);

#endif