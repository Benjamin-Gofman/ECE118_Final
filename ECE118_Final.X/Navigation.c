/*
 * Runs tape search, line following, bumper recovery,
 * tower scoring, and opponent search.
 */

#include <xc.h>
#include <cp0defs.h>
#include "Motor.h"
#include "TapeSensor.h"
#include "Bumper.h"
#include "Navigation.h"
#include "Servo.h"
#include "Opponent.h"
#include "Flywheel.h"

#define DRIVE_PWM             300u // 30% PWM
#define SLOW_PWM              150u // 15% PWM
#define TURN_MS               600u // 600 ms
#define BACKUP_MS             600u // 600 ms
#define FORWARD_MS           1000u // 1 s
#define SCAN_MS              1200u // 360-degree turn
#define SAMPLE_MS              10u // 10 ms
#define TICKS_PER_MS        40000u // Core timer at an 80 MHz system clock
#define BALL_ROLL_MS         1000u // Wait for the ball to roll down the slide
#define FEED_OPEN_MS          300u // Time for ball to pass the servo
#define FEED_CLOSE_MS         300u // Time for servo to close
#define OPPONENT_FEED_GAP_MS 1000u // Wait between feeds
#define FLYWHEEL_SPINUP_MS    500u // Get it steadily spinning

typedef enum {
    FIND_TAPE,
    FOLLOW_TAPE,
    BUMPED,
    SHOOT_AT_TOWER,
    FIND_OPPONENT,
    DONE,
    APPROACH_OPPONENT
} MovementState;

typedef enum {
    DRIVE_FORWARD,
    ADJUST,
    TURN_RIGHT
} LineState;

typedef enum {
    BACK_UP,
    SHOOT_OPPONENT,
    GO_RIGHT,
    GO_FORWARD,
    GO_LEFT
} BumperState;

static MovementState movement = FIND_TAPE;
static LineState lineState = DRIVE_FORWARD;
static BumperState bumperState = BACK_UP;

static unsigned int stepStart; // Timer count
static unsigned int towerCount = 0;
static unsigned char bannerSeen = 0;
static unsigned char opponentShots = 0;
static unsigned char ballReleased = 0;

static unsigned char TimePassed(unsigned int now, unsigned int milliseconds) {
    // Return 1 after the current movement step lasts long enough
    return (unsigned int) (now - stepStart) >= milliseconds * TICKS_PER_MS;
}

static void FindingTape(unsigned char onTape[6]) {
    // Look for black tape using the three front sensors
    if (onTape[0] || onTape[1] || onTape[2]) {
        movement = FOLLOW_TAPE;
    } else {
        // Keep driving forward
        Motor_SetLeftSpeed(DRIVE_PWM);
        Motor_SetRightSpeed(DRIVE_PWM);
    }
}

static void TapeFollowing(unsigned char onTape[6]) {
    unsigned char frontMiddle = onTape[1];
    unsigned char backMiddle = onTape[4];

    // Change state if it gets lost
    if (!frontMiddle && !backMiddle) {
        movement = FIND_TAPE;
        return;
    }

    // Both middle sensors see black tape
    if (frontMiddle && backMiddle) {
        lineState = DRIVE_FORWARD;
    } else if (frontMiddle) {
        // Adjust so the rear middle sensor sees black
        lineState = ADJUST;
    } else {
        // Turn right when the front middle sensor goes LOW
        lineState = TURN_RIGHT;
    }

    switch (lineState) {
        case DRIVE_FORWARD:
            Motor_SetLeftSpeed(DRIVE_PWM);
            Motor_SetRightSpeed(DRIVE_PWM);
            break;

        case ADJUST:
            // Use the outer rear sensors to pick a direction
            if (onTape[3] && !onTape[5]) {
                Motor_SetLeftSpeed(0);
                Motor_SetRightSpeed(SLOW_PWM);
            } else if (onTape[5] && !onTape[3]) {
                Motor_SetLeftSpeed(SLOW_PWM);
                Motor_SetRightSpeed(0);
            } else {
                // Advance slowly when neither side gives a clear direction
                Motor_SetLeftSpeed(SLOW_PWM);
                Motor_SetRightSpeed(SLOW_PWM);
            }
            break;

        case TURN_RIGHT:
            Motor_SetLeftSpeed(SLOW_PWM);
            Motor_SetRightSpeed(0);
            break;
    }
}

static void StartBumperRecovery(unsigned int now) {
    // Forget the banner
    bannerSeen = 0;
    movement = BUMPED;
    bumperState = BACK_UP;

    // Stop and back up
    Motor_Stop();
    Motor_SetLeftDirection(REVERSE);
    Motor_SetRightDirection(REVERSE);
    Motor_SetLeftSpeed(SLOW_PWM);
    Motor_SetRightSpeed(SLOW_PWM);

    // Start the backup timer
    stepStart = now;
}

static unsigned char ShootAtOpponent(unsigned int now) {
    static unsigned char feederOpen = 0;
    static unsigned char flywheelStarted = 0;

    // Start the flywheel before feeding the first ball
    if (opponentShots == 0 && !flywheelStarted) {
        Flywheel_Start();
        flywheelStarted = 1;
        stepStart = now;
        return 0;
    }

    // Wait for spin-up before feeding the first ball
    if (opponentShots == 0 && !feederOpen &&
            !TimePassed(now, FLYWHEEL_SPINUP_MS)) {
        return 0;
    }

    // Close the servo after the ball passes
    if (feederOpen && TimePassed(now, FEED_OPEN_MS)) {
        Servo3_Close();
        feederOpen = 0;
        opponentShots++;
        stepStart = now;
        return 0;
    }

    // Wait for the servo to close and delay before feeding the 2nd ball
    if (feederOpen ||
            (opponentShots == 1 && !TimePassed(now, OPPONENT_FEED_GAP_MS))) {
        return 0;
    }

    // Open the servo
    if (opponentShots < 2) {
        Servo3_Open();
        feederOpen = 1;
        stepStart = now;
        return 0;
    }

    // Close the servo before turning off the flywheel
    if (!TimePassed(now, FEED_CLOSE_MS)) {
        return 0;
    }

    Flywheel_Stop();
    flywheelStarted = 0;
    return 1;
}

static void RunBumperRecovery(unsigned int now) {
    switch (bumperState) {
        case BACK_UP:
            if (TimePassed(now, BACKUP_MS)) {
                // Back up then stop
                Motor_Stop();
                bumperState = SHOOT_OPPONENT;
            }
            break;

        case SHOOT_OPPONENT:
            // Wait until both balls have been shot
            if (!ShootAtOpponent(now)) {
                break;
            }

            // Finish the run if both parameters are met
            if (towerCount >= 2) {
                movement = DONE;
                break;
            }

            // Set both motors forward and start the right turn
            Motor_SetLeftDirection(FORWARD);
            Motor_SetRightDirection(FORWARD);
            Motor_SetLeftSpeed(SLOW_PWM);
            Motor_SetRightSpeed(0);
            bumperState = GO_RIGHT;
            // Start the right-turn timer
            stepStart = now;
            break;

        case GO_RIGHT:
            if (TimePassed(now, TURN_MS)) {
                // Start driving forward with both wheels
                Motor_SetRightSpeed(SLOW_PWM);
                bumperState = GO_FORWARD;
                // Start the forward timer
                stepStart = now;
            }
            break;

        case GO_FORWARD:
            if (TimePassed(now, FORWARD_MS)) {
                // Stop the left wheel to turn left
                Motor_SetLeftSpeed(0);
                bumperState = GO_LEFT;
                // Start the left-turn timer
                stepStart = now;
            }
            break;

        case GO_LEFT:
            if (TimePassed(now, TURN_MS)) {
                // Finish recovery and look for tape 
                Motor_Stop();
                movement = FIND_TAPE;
            }
            break;
    }
}

static void RunTower(unsigned int now) {
    Motor_Stop();

    if (!ballReleased && TimePassed(now, FEED_OPEN_MS)) {
        // Close the feeder after giving one ball time to pass
        Servo2_Close();
        ballReleased = 1;
        stepStart = now;
    }

    // Wait for the ball to roll down the slide
    if (!ballReleased || !TimePassed(now, FEED_CLOSE_MS + BALL_ROLL_MS)) {
        return;
    }

    towerCount++;

    // Reset for the next tower
    ballReleased = 0;

    if (towerCount >= 2 && opponentShots >= 2) {
        movement = DONE;
    } else if (towerCount >= 2) {
        // Time the opponent search
        Opponent_StartSearch();
        stepStart = now;
        movement = FIND_OPPONENT;
    } else {
        movement = FOLLOW_TAPE;
    }
}

static void UpdateNavigation(unsigned int now) {
    unsigned int readings[6];
    unsigned char onTape[6];
    unsigned char banner = TapeSensor_CheckBanner();

    if (movement == DONE) {
        Motor_Stop();
        return;
    }

    if (movement == BUMPED) {
        RunBumperRecovery(now);
        return;
    }

    if (movement == SHOOT_AT_TOWER) {
        RunTower(now);
        return;
    }

    if (Bumper_IsPressed()) {
        StartBumperRecovery(now);
        return;
    }

    if (movement == FIND_OPPONENT) {
        // Stop scan if beacon detected or timer runs out
        OpponentSearchResult result = SearchOpponent(TimePassed(now, SCAN_MS));

        if (result == OPPONENT_FOUND) {
            movement = APPROACH_OPPONENT;
        } else if (result == OPPONENT_NOT_FOUND) {
            movement = DONE;
        }
        return;
    }

    if (movement == APPROACH_OPPONENT) {
        // Drive toward the opponent until the bumper is pressed
        Motor_SetLeftSpeed(SLOW_PWM);
        Motor_SetRightSpeed(SLOW_PWM);
        return;
    }

    if (TapeSensor_ReadValues(readings) != SUCCESS ||
            TapeSensor_CheckTape(readings, onTape) != SUCCESS) {
        Motor_Stop();
        return;
    }

    if (movement == FIND_TAPE) {
        FindingTape(onTape);

        // Keep searching until a front sensor sees tape
        if (movement == FIND_TAPE) {
            return;
        }
    }

    if (banner) {
        // Keep following the tape
        bannerSeen = 1;
    } else if (bannerSeen) {
        // TOP goes LOW after losing the banner
        bannerSeen = 0;
        Motor_Stop();
        Servo1_Open();
        Servo2_Open();
        stepStart = now;
        movement = SHOOT_AT_TOWER;
        return;
    }

    TapeFollowing(onTape);
}

void RunNavigation(void) {
    // Save the current timer count
    unsigned int lastSample = _CP0_GET_COUNT();

    while (1) {
        unsigned int now = _CP0_GET_COUNT();

        // Read the sensors and update movement every 10 ms
        if ((unsigned int) (now - lastSample) >= SAMPLE_MS * TICKS_PER_MS) {
            lastSample = now;
            UpdateNavigation(now);
        }
    }
}