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
#include "Navigation.h"
    
int main(void) {
    BOARD_Init();
    Motor_Init();
    TapeSensor_Init();
    Bumper_Init();
    Servo_Init();
    Opponent_Init();
    Flywheel_Init();

    // Set both motor directions forward
    Motor_SetLeftDirection(FORWARD);
    Motor_SetRightDirection(FORWARD);

    RunNavigation();

    return 0;
}