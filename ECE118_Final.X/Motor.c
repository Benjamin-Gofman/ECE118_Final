/*
 * Controls both drive motors through the L298 H-bridge,
 * allowing each motor to run at its own speed and direction.
 *
 * Uno32 -> L298
 * Left motor:
 * Z06 -> ENA
 * Z11 -> IN1
 * Z09 -> IN2
 *
 * Right motor:
 * Y04 -> ENB
 * Z07 -> IN3
 * Z05 -> IN4
 *
 * Connect all grounds together.
 */

#include <xc.h>
#include <IO_Ports.h>
#include <pwm.h>
#include "Motor.h"

// Left motor pins
#define LEFT_PWM       PWM_PORTZ06
#define LEFT_IN1_TRIS  PORTZ11_TRIS
#define LEFT_IN1_LAT   PORTZ11_LAT
#define LEFT_IN2_TRIS  PORTZ09_TRIS
#define LEFT_IN2_LAT   PORTZ09_LAT

// Right motor pins
#define RIGHT_PWM      PWM_PORTY04
#define RIGHT_IN3_TRIS PORTZ07_TRIS
#define RIGHT_IN3_LAT  PORTZ07_LAT
#define RIGHT_IN4_TRIS PORTZ05_TRIS
#define RIGHT_IN4_LAT  PORTZ05_LAT

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

void Motor_Direction(unsigned char direction) {
    Motor_SetLeftDirection(direction);
    Motor_SetRightDirection(direction);
}

void Motor_SetLeftSpeed(unsigned int speed) {
    // Limit speed to 1000 PWM
    if (speed > MAX_PWM) {
        speed = MAX_PWM;
    }

    PWM_SetDutyCycle(LEFT_PWM, speed);
}

void Motor_SetRightSpeed(unsigned int speed) {
    // Limit speed to 1000 PWM
    if (speed > MAX_PWM) {
        speed = MAX_PWM;
    }

    PWM_SetDutyCycle(RIGHT_PWM, speed);
}

void Motor_Speed(unsigned int speed) {
    Motor_SetLeftSpeed(speed);
    Motor_SetRightSpeed(speed);
}

void Motor_Forward(unsigned int speed) {
    Motor_Direction(FORWARD);
    Motor_Speed(speed);
}

void Motor_Reverse(unsigned int speed) {
    Motor_Direction(REVERSE);
    Motor_Speed(speed);
}

void Motor_TurnRight(unsigned int speed) {
    Motor_SetLeftDirection(FORWARD);
    Motor_SetRightDirection(REVERSE);
    Motor_Speed(speed);
}

void Motor_TurnLeft(unsigned int speed) {
        Motor_SetLeftDirection(REVERSE);
        Motor_SetRightDirection(FORWARD);
        Motor_Speed(speed);
}

void Motor_Stop(void) {
    // Stop both motors
    PWM_SetDutyCycle(LEFT_PWM, 0); // Left motor off
    PWM_SetDutyCycle(RIGHT_PWM, 0); // Right motor off
}
