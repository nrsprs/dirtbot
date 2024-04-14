#include "EncoderSteps.h"

long EncoderSteps(Encoder& encoder, long prevPos) {
    volatile long newPos = encoder.read();
    if (newPos != prevPos) {
        prevPos = newPos;
    }
    return newPos;
}