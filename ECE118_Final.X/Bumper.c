#include <xc.h>
#include <IO_Ports.h>
#include "Bumper.h"
#include "Motor.h"

#define LEFT_BUMPER_TRIS  PORTW07_TRIS
#define LEFT_BUMPER_BIT   PORTW07_BIT
#define RIGHT_BUMPER_TRIS PORTW05_TRIS
#define RIGHT_BUMPER_BIT  PORTW05_BIT

#define REAR_BUMPER_TRIS  PORTV04_TRIS
#define REAR_BUMPER_BIT   PORTV04_BIT

#define BACKUP_MS 600u

static uint32_t stepStart = 0u;
static unsigned char escapeInReverse = 1u;
static unsigned char previousBumperPressed = 0u;
static unsigned char previousRearPressed = 0u;

void Bumper_Init(void) {
    // Configure all bumper pins as inputs
    LEFT_BUMPER_TRIS = 1;
    RIGHT_BUMPER_TRIS = 1;
    REAR_BUMPER_TRIS = 1;

    previousBumperPressed = 0u;
    previousRearPressed = 0u;
}

static unsigned char ReadPressedBumpers(void) {
    // 0V when pressed, 3.3V when released
    unsigned char leftPressed = (LEFT_BUMPER_BIT == 0u);
    unsigned char rightPressed = (RIGHT_BUMPER_BIT == 0u);

    // 10 = left only, 01 = right only, 11 = both
    return (unsigned char) ((leftPressed << 1) | rightPressed);
}

unsigned char Bumper_IsPressed(void) {
    // Return 1 if either front bumper is currently pressed
    return ReadPressedBumpers() != 0u;
}

unsigned char Bumper_NewPress(void) {
    unsigned char pressed = ReadPressedBumpers();

    // Check if either front bumper changed from released to pressed
    unsigned char newPress =
            (pressed & (unsigned char) ~previousBumperPressed) != 0;

    previousBumperPressed = pressed;
    return newPress;
}

unsigned char Bumper_RearIsPressed(void) {
    // Return 1 if the rear bumper is currently pressed
    return REAR_BUMPER_BIT == 0u;
}

unsigned char Bumper_RearNewPress(void) {
    unsigned char pressed = Bumper_RearIsPressed();

    // Check if the rear bumper changed from released to pressed
    unsigned char newPress = pressed && !previousRearPressed;

    previousRearPressed = pressed;
    return newPress;
}

void StartBumperEscape(uint32_t now, unsigned char reverse) {
    // Start backing off in the selected direction
    stepStart = now;
    escapeInReverse = reverse;
    Motor_Stop();

    if (escapeInReverse) {
        Motor_Reverse(SLOW_PWM);
    } else {
        Motor_Forward(SLOW_PWM);
    }
}

void StartBumperRecovery(uint32_t now) {
    // Start backing off in reverse
    StartBumperEscape(now, 1);
}

unsigned char RunBumperRecovery(uint32_t now) {
    // Continue backing off and return 1 when finished
    if (escapeInReverse) {
        Motor_Reverse(SLOW_PWM);
    } else {
        Motor_Forward(SLOW_PWM);
    }

    // Return 1 if the backup time has passed
    return (uint32_t) (now - stepStart) >= BACKUP_MS;
}