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

#include <stdio.h>
#include <xc.h>
#include <BOARD.h>
#include <pwm.h>
#include <IO_Ports.h>
#include <math.h>
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

char Motor_Init(void) {
    // Initialize PWM for both motors
    if (PWM_Init() == ERROR) {
        printf("\n\rERROR: PWM_Init");
        return ERROR;
    }

    // Add PWM pins for left and right motors
    if (PWM_AddPins(LEFT_PWM | RIGHT_PWM) == ERROR) {
        printf("\n\rERROR: PWM_AddPins");
        return ERROR;
    }

    // Initialize: motors = stopped, direction = forward
    Motor_Stop();
    Motor_SetLeftDirection(FORWARD);
    Motor_SetRightDirection(FORWARD);

    // Set all direction pins as outputs
    LEFT_IN1_TRIS = 0;
    LEFT_IN2_TRIS = 0;
    RIGHT_IN3_TRIS = 0;
    RIGHT_IN4_TRIS = 0;

    return SUCCESS;
}

void Motor_SetLeftDirection(unsigned int forward) {
    // Turn off left motor
    PWM_SetDutyCycle(LEFT_PWM, 0);
    // 10 = forward, 01 = reverse
    LEFT_IN1_LAT = forward;
    LEFT_IN2_LAT = !forward;
}

void Motor_SetRightDirection(unsigned int forward) {
    PWM_SetDutyCycle(RIGHT_PWM, 0);
    RIGHT_IN3_LAT = forward;
    RIGHT_IN4_LAT = !forward;
}

void Motor_SetLeftSpeed(unsigned int speed) {
    PWM_SetDutyCycle(LEFT_PWM, fmin(speed, MAX_PWM)); // 1000
}

void Motor_SetRightSpeed(unsigned int speed) {
    PWM_SetDutyCycle(RIGHT_PWM, fmin(speed, MAX_PWM));
}

void Motor_Stop(void) {
    PWM_SetDutyCycle(LEFT_PWM, 0); // Left motor off
    PWM_SetDutyCycle(RIGHT_PWM, 0); // Right motor off
}
