#ifndef BUMPER_H
#define BUMPER_H

#include <stdint.h>

// Set pins as inputs
void Bumper_Init(void);

// Start bumper recovery and begin backing up
void StartBumperRecovery(uint32_t now);

// Return 1 if either front bumper is pressed, 0 otherwise
unsigned char Bumper_IsPressed(void);
unsigned char Bumper_RearIsPressed(void); // *rear

// Return 1 once per press, then wait until bumper is released
unsigned char Bumper_NewPress(void);
unsigned char Bumper_RearNewPress(void);

// Return 1 after the straight backup step finishes
unsigned char RunBumperRecovery(uint32_t now);

// Back off in the selected direction
void StartBumperEscape(uint32_t now, unsigned char reverse);

#endif
