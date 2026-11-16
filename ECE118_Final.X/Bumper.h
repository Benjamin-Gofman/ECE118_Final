#ifndef BUMPER_H
#define BUMPER_H

#include <stdint.h>

// Set pins as inputs
void Bumper_Init(void);

// Start bumper recovery and begin backing up
void StartBumperRecovery(uint32_t now);

// Return 1 if either bumper is pressed, 0 otherwise
unsigned char Bumper_IsPressed(void);

// Return 1 once per press, then wait until bumper is released
unsigned char Bumper_NewPress(void);

// Return 1 when bumper recovery is finished
unsigned char RunBumperRecovery(uint32_t now);

#endif
