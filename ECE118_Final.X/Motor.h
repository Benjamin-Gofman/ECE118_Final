/*
 * Declares the functions for controlling both drive motors.
 */

#ifndef MOTOR_H
#define MOTOR_H

#define FORWARD 1
#define REVERSE 0

char Motor_Init(void);

void Motor_SetLeftDirection(unsigned int forward);
void Motor_SetRightDirection(unsigned int forward);

void Motor_SetLeftSpeed(unsigned int speed);
void Motor_SetRightSpeed(unsigned int speed);

void Motor_Stop(void);

#endif