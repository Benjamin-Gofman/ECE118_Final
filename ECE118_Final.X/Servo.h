/*
 * Defines the three servo functions
 */

#ifndef SERVO_H
#define SERVO_H

// Initialize all three servos
void Servo_Init(void);

// Control servo 1
void Servo1_Open(void);
void Servo1_Close(void);

// Control servo 2
void Servo2_Open(void);
void Servo2_Close(void);

// Control servo 3
void Servo3_Open(void);
void Servo3_Close(void);

#endif