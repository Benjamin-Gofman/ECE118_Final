/*
 * Initializes the robot and starts navigation.
 */

#include <BOARD.h>
#include "Motor.h"
#include "TapeSensor.h"
#include "Bumper.h"
#include "Servo.h"
#include "Opponent.h"
#include "Flywheel.h"
#include "ES_Configure.h"
#include "ES_Framework.h"

int main(void) {
    BOARD_Init();
    Motor_Init();
    TapeSensor_Init();
    Bumper_Init();
    Servo_Init();
    Opponent_Init();
    Flywheel_Init();

    // Run the framework if all services initialize successfully
    if (ES_Initialize() == Success) {
        ES_Run();
    }

    // Stop all motors if the initialization fails
    Motor_Stop();
    Flywheel_Stop();

    return 1;
}
//#include <BOARD.h>
//#include "Motor.h"
//
//int main(void) {
//    BOARD_Init();
//    Motor_Init();
//
//    // Set both motors forward
//    Motor_SetLeftDirection(FORWARD);
//    Motor_SetRightDirection(FORWARD);
//
//    Motor_SetLeftSpeed(300);
//    Motor_SetRightSpeed(300);
//
//    while (1) {
//        // Keep both motors running
//    }
//
//    return 0;
//}