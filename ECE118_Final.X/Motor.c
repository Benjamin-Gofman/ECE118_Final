/*
 * Controls both drive motors through the L298 H-bridge,
 * allowing each motor to run at its own speed and direction.
 *
 * Uno32 -> L298
 * Left motor:
 * Y10 -> ENA
 * Y06 -> IN1
 * Y07 -> IN2
 *
 * Right motor:
 * Y12 -> ENB
 * Y08 -> IN3
 * Y09 -> IN4
 *
 * Connect all grounds together.
 */

#include <xc.h>
#include <IO_Ports.h>
#include <pwm.h>
#include "Motor.h"

// Left motor pins
#define LEFT_PWM       PWM_PORTY10
#define LEFT_IN1_TRIS  PORTY06_TRIS
#define LEFT_IN1_LAT   PORTY06_LAT
#define LEFT_IN2_TRIS  PORTY07_TRIS
#define LEFT_IN2_LAT   PORTY07_LAT

// Right motor pins
#define RIGHT_PWM      PWM_PORTY12
#define RIGHT_IN3_TRIS PORTY08_TRIS
#define RIGHT_IN3_LAT  PORTY08_LAT
#define RIGHT_IN4_TRIS PORTY09_TRIS
#define RIGHT_IN4_LAT  PORTY09_LAT

void Motor_Init(void) {
    // Set all direction pins as outputs
    LEFT_IN1_TRIS = 0;
    LEFT_IN2_TRIS = 0;
    RIGHT_IN3_TRIS = 0;
    RIGHT_IN4_TRIS = 0;

    // Initialize direction inputs
    LEFT_IN1_LAT = 0;
    LEFT_IN2_LAT = 0;
    RIGHT_IN3_LAT = 0;
    RIGHT_IN4_LAT = 0;

    // Initialize PWM for both motors
    PWM_Init();
    PWM_SetFrequency(1000);

    // Add PWM pins for left and right motors
    PWM_AddPins(LEFT_PWM | RIGHT_PWM);

    // Start with both motors stopped
    PWM_SetDutyCycle(LEFT_PWM, 0);
    PWM_SetDutyCycle(RIGHT_PWM, 0);
}

void Motor_SetLeftDirection(unsigned char forward) {
    // 10 = forward, 01 = reverse
    LEFT_IN1_LAT = forward;
    LEFT_IN2_LAT = !forward;
}

void Motor_SetRightDirection(unsigned char forward) {
    // 10 = forward, 01 = reverse
    RIGHT_IN3_LAT = forward;
    RIGHT_IN4_LAT = !forward;
}

void Motor_SetLeftSpeed(unsigned int speed) {
    // Limit speed to 100% PWM
    if (speed > MAX_PWM) {
        speed = MAX_PWM;
    }

    PWM_SetDutyCycle(LEFT_PWM, speed);
}

void Motor_SetRightSpeed(unsigned int speed) {
    // Limit speed to 100% PWM
    if (speed > MAX_PWM) {
        speed = MAX_PWM;
    }

    PWM_SetDutyCycle(RIGHT_PWM, speed);
}

void Motor_Stop(void) {
    // Stop both motors
    PWM_SetDutyCycle(LEFT_PWM, 0); // Left motor off
    PWM_SetDutyCycle(RIGHT_PWM, 0); // Right motor off
}
