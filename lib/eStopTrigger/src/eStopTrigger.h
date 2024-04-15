#ifndef ESTOP_TRIGGER_H
#define ESTOP_TRIGGER_H

#include <LiquidCrystal.h>
#include <InputDebounce.h>
#include <Encoder.h>
#include <EncoderSteps.h>
#include <Debounce.h>

/*
Routine to run when the e-stop button on the UI is triggered.
Params :: LiquidCrytstal LCD 

Returns :: None

Stop motors & current operation by creating a blocking function that requires input by the user. 


*/
void eStopTrigger();

#endif