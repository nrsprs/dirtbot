#ifndef INIT_USER_INPUT_H
#define INIT_USER_INPUT_H

#include <Encoder.h>
#include <Vector.h>
#include <LiquidCrystal.h>
#include <InputDebounce.h>
#include <EncoderSteps.h>
#include <Debounce.h>

/*
Interface for receiving the number of plugs in the X and Y directions & blocks until user is ready. 

Returns :: <Vector> (int: number of x trays, number of y trays)
*/
Vector<int> InitUserInput(LiquidCrystal& lcd, Encoder& encoder, InputDebounce& pushButton);

#endif