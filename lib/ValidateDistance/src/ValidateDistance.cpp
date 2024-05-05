#include <ValidateDistance.h>

void ValidateDistance(AccelStepper& stepper, Encoder& encoder, InputDebounce& switchObj) { 
    double dist = 20;
    MoveAxis(stepper, encoder, switchObj, dist);
    delay(1);
    MoveAxis(stepper, encoder, switchObj, -dist);
    exit(0);
}