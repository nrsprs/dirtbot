#ifndef START_ANIMATION_H
#define START_ANIMATION_H

#include <LiquidCrystal.h>

/*
Startup Animation for DirtBot that appears on the 2x16 char LCD.

Params :: LiquidCrystal lcd

Returns :: Vector<int> processParams
*/
void StartAnimation(LiquidCrystal& lcd);
#endif