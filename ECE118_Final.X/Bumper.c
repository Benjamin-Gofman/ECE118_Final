#include <xc.h>
#include <IO_Ports.h>
#include "Bumper.h"

#define LEFT_BUMPER_TRIS  PORTY03_TRIS
#define LEFT_BUMPER_BIT   PORTY03_BIT
#define RIGHT_BUMPER_TRIS PORTY05_TRIS
#define RIGHT_BUMPER_BIT  PORTY05_BIT

void Bumper_Init(void) {
    // Set pins as an inputs
    LEFT_BUMPER_TRIS = 1;
    RIGHT_BUMPER_TRIS = 1;
}

unsigned char Bumper_IsPressed(void) {
    // Return 1 if either bumper is pressed, 0 otherwise
    return LEFT_BUMPER_BIT == 0 || RIGHT_BUMPER_BIT == 0;
}