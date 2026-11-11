/*
 * Controls three servos independently
 */

#include <RC_Servo.h>
#include "Servo.h"

// Servo signal pins
#define SERVO_1_PIN RC_PORTX03
#define SERVO_2_PIN RC_PORTX04
#define SERVO_3_PIN RC_PORTW07

// Servo 1 positions in microseconds
#define SERVO_1_CLOSED_PULSE 1500u
#define SERVO_1_OPEN_PULSE   1700u

// Servo 2 positions in microseconds
#define SERVO_2_CLOSED_PULSE 1500u
#define SERVO_2_OPEN_PULSE   1700u

// Servo 3 positions in microseconds
#define SERVO_3_CLOSED_PULSE 1500u
#define SERVO_3_OPEN_PULSE   1700u

void Servo1_Open(void) {
    // Move servo 1 to its open position
    RC_SetPulseTime(SERVO_1_PIN, SERVO_1_OPEN_PULSE);
}

void Servo1_Close(void) {
    // Move servo 1 to its closed position
    RC_SetPulseTime(SERVO_1_PIN, SERVO_1_CLOSED_PULSE);
}

void Servo2_Open(void) {
    // Move servo 2 to its open position
    RC_SetPulseTime(SERVO_2_PIN, SERVO_2_OPEN_PULSE);
}

void Servo2_Close(void) {
    // Move servo 2 to its closed position
    RC_SetPulseTime(SERVO_2_PIN, SERVO_2_CLOSED_PULSE);
}

void Servo3_Open(void) {
    // Move servo 3 to its open position
    RC_SetPulseTime(SERVO_3_PIN, SERVO_3_OPEN_PULSE);
}

void Servo3_Close(void) {
    // Move servo 3 to its closed position
    RC_SetPulseTime(SERVO_3_PIN, SERVO_3_CLOSED_PULSE);
}

void Servo_Init(void) {
    // Initialize the servo library 
    RC_Init();

    // Register the three servo signal pins
    RC_AddPins(SERVO_1_PIN | SERVO_2_PIN | SERVO_3_PIN);

    // Start closed
    Servo1_Close();
    Servo2_Close();
    Servo3_Close();
}