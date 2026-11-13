#ifndef BUMPER_H
#define BUMPER_H

#include <stdint.h>

// Set pins as inputs
void Bumper_Init(void);

// Return 1 if either bumper is pressed, 0 otherwise
unsigned char Bumper_IsPressed(void);

// Start bumper recovery and begin backing up
void StartBumperRecovery(uint32_t now);

// Return 1 when ready to shoot the opponent
unsigned char Bumper_IsShooting(void);

// Continue recovery after shooting finishes
void Bumper_ShootingComplete(uint32_t now);

// Return 1 when bumper recovery is finished
unsigned char RunBumperRecovery(uint32_t now);

#endif
