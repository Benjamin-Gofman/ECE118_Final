#ifndef MOTOR_H
#define MOTOR_H

#define FORWARD 1
#define REVERSE 0

// Initialize motor pins and PWM
char Motor_Init(void);

// Set motor directions
void Motor_SetLeftDirection(unsigned int forward);
void Motor_SetRightDirection(unsigned int forward);

// Set motor speeds from 0 to 1000 PWM
void Motor_SetLeftSpeed(unsigned int speed);
void Motor_SetRightSpeed(unsigned int speed);

// Stop both motors
void Motor_Stop(void);

#endif
