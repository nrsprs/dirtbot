#ifndef DEBOUNCE_H
#define DEBOUNCE_H

#include <InputDebounce.h>

/* ### Debounce A Digital Input
Takes a InputDebounce object and returns 1 or 0 depending on status. 
Meant to be called multiple times, saves on interrupt pins.
Params :: InputDebounce swich 
Returns :: Bool
*/
bool Debounce(InputDebounce& switch_object);

#endif