#ifndef BUMPER_H
#define BUMPER_H

// Set pins as an inputs
void Bumper_Init(void);

// Return 1 if either bumper is pressed, 0 otherwise
unsigned char Bumper_IsPressed(void);

#endif