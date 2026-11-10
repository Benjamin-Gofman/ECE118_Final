/*
 * Controls both drive motors through the L298 H-bridge,
 * allowing each motor to run at its own speed and direction.
 *
 * Uno32 -> L298
 * Left motor:
 * Z06 -> ENA
 * Z03 -> IN1
 * Z05 -> IN2
 *
 * Right motor:
 * Y12 -> ENB
 * Z07 -> IN3
 * Z09 -> IN4
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

char Motor_Init(void) {

    if (PWM_Init() == ERROR) {
        printf("\n\rERROR: PWM_Init");
        return ERROR;
    }

    if (PWM_AddPins(PWM_PORTZ06 | PWM_PORTY12) == ERROR) {
        printf("\n\rERROR: PWM_AddPins");
        return ERROR;
    }

    // Initialize: motors = stopped, direction = forward
    Motor_Stop();
    Motor_SetLeftDirection(FORWARD);
    Motor_SetRightDirection(FORWARD);

    // Set all direction pins as outputs
    PORTZ03_TRIS = 0;
    PORTZ05_TRIS = 0;
    PORTZ07_TRIS = 0;
    PORTZ09_TRIS = 0;
    
    return SUCCESS;
}

void Motor_SetLeftDirection(unsigned int forward) {
    // Turn off left motor
    PWM_SetDutyCycle(PWM_PORTZ06, 0);
    // 10 = forward, 01 = reverse
    PORTZ03_LAT = forward;
    PORTZ05_LAT = !forward;
}

void Motor_SetRightDirection(unsigned int forward) {
    PWM_SetDutyCycle(PWM_PORTY12, 0);
    PORTZ07_LAT = forward;
    PORTZ09_LAT = !forward;
}

void Motor_SetLeftSpeed(unsigned int speed) {
    PWM_SetDutyCycle(PWM_PORTZ06, fmin(speed, MAX_PWM)); // 1000
}

void Motor_SetRightSpeed(unsigned int speed) {
    PWM_SetDutyCycle(PWM_PORTY12, fmin(speed, MAX_PWM));
}

void Motor_Stop(void) {
    PWM_SetDutyCycle(PWM_PORTZ06, 0); // Left motor off
    PWM_SetDutyCycle(PWM_PORTY12, 0); // Right motor off
}