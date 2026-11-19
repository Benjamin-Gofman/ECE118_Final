#ifndef MOTOR_H
#define MOTOR_H

#define FORWARD 1
#define REVERSE 0

// Driving speeds (0 to 1000 PWM)
#define DRIVE_PWM 800 // 80% PWM
#define SLOW_PWM  600 // 60% PWM

// Initialize motor pins and PWM
void Motor_Init(void);

// Set motor directions
void Motor_SetLeftDirection(unsigned char forward);
void Motor_SetRightDirection(unsigned char forward);

// Set motor speeds from 0 to 1000 PWM
void Motor_SetLeftSpeed(unsigned int speed);
void Motor_SetRightSpeed(unsigned int speed);

// Set both directions
void Motor_Direction(unsigned char direction);

// Set both speeds
void Motor_Speed(unsigned int speed);

// Drive both motors forward 
void Motor_Forward(unsigned int speed);

// Drive both motors backward
void Motor_Reverse(unsigned int speed);

// Pivot right in place
void Motor_TurnRight(unsigned int speed);

// Pivot left in place
void Motor_TurnLeft(unsigned int speed);

// Stop both motors
void Motor_Stop(void);

#endif
