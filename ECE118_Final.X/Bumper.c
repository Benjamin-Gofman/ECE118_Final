#include <xc.h>
#include <IO_Ports.h>
#include "Bumper.h"
#include "Motor.h"

#define LEFT_BUMPER_TRIS  PORTY03_TRIS
#define LEFT_BUMPER_BIT   PORTY03_BIT
#define RIGHT_BUMPER_TRIS PORTY05_TRIS
#define RIGHT_BUMPER_BIT  PORTY05_BIT

#define BACKUP_MS   600
#define TURN_MS     600
#define FORWARD_MS 1000

typedef enum {
    BACK_UP,
    GO_RIGHT,
    GO_FORWARD,
    GO_LEFT
} BumperState;

static BumperState bumperState = BACK_UP;
static uint32_t stepStart = 0;
static unsigned char previousBumperPressed = 0;

void Bumper_Init(void) {
    // Set pins as inputs
    LEFT_BUMPER_TRIS = 1;
    RIGHT_BUMPER_TRIS = 1;
    previousBumperPressed = 0;
}

unsigned char Bumper_IsPressed(void) {
    // Return 1 if either bumper is pressed, 0 otherwise
    return LEFT_BUMPER_BIT == 0 || RIGHT_BUMPER_BIT == 0;
}

unsigned char Bumper_NewPress(void) {
    // Check for a new bumper press
    unsigned char pressed = Bumper_IsPressed();
    unsigned char newPress = pressed && !previousBumperPressed;
    previousBumperPressed = pressed;
    return newPress;
}

void StartBumperRecovery(uint32_t now) {
    bumperState = BACK_UP;

    // Stop and back up
    Motor_Stop();
    Motor_Reverse(SLOW_PWM);

    // Start the backup timer
    stepStart = now;
}

unsigned char RunBumperRecovery(uint32_t now) {
    switch (bumperState) {
        case BACK_UP:
            if ((uint32_t) (now - stepStart) >= BACKUP_MS) {
                // Back up, then turn right around the obstacle.
                Motor_Stop();
                Motor_Forward(SLOW_PWM);
                Motor_SetRightSpeed(0);
                bumperState = GO_RIGHT;
                // Start the right-turn timer
                stepStart = now;
            }
            break;

        case GO_RIGHT:
            if ((uint32_t) (now - stepStart) >= TURN_MS) {
                // Start driving forward with both wheels
                Motor_Forward(SLOW_PWM);
                bumperState = GO_FORWARD;
                // Start the forward timer
                stepStart = now;
            }
            break;

        case GO_FORWARD:
            if ((uint32_t) (now - stepStart) >= FORWARD_MS) {
                // Stop the left wheel to turn left
                Motor_SetLeftSpeed(0);
                bumperState = GO_LEFT;
                // Start the left-turn timer
                stepStart = now;
            }
            break;

        case GO_LEFT:
            if ((uint32_t) (now - stepStart) >= TURN_MS) {
                // Finish recovery and look for tape 
                Motor_Stop();
                return 1;
            }
            break;
    }

    return 0;
}
