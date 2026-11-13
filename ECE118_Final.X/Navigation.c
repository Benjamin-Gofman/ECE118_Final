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
#include "ES_Framework.h"
#include "Servo.h"
#include "Opponent.h"
#include "Flywheel.h"
#include "LineFollowing.h"

#define DRIVE_PWM             300u // 30% PWM
#define SLOW_PWM              150u // 15% PWM
#define SCAN_MS              1200u // 360-degree turn
#define SAMPLE_MS              10u // Navigation update interval
#define BALL_ROLL_MS         1000u // Wait for the ball to roll down the slide
#define FEED_OPEN_MS          300u // Time for 1 ball to pass the servo
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

static MovementState movement = FIND_TAPE;

static unsigned int stepStart; // Timer count
static unsigned int towerCount = 0;
static unsigned char bannerSeen = 0;
static unsigned char opponentShots = 0;
static unsigned char ballReleased = 0;
static uint8_t navigationPriority;
static unsigned char feederOpen = 0;
static unsigned char flywheelStarted = 0;

static unsigned char TimePassed(unsigned int now, uint32_t milliseconds) {
    // Return 1 after the current movement step lasts long enough
    return (uint32_t) (now - stepStart) >= milliseconds;
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

static unsigned char ShootAtOpponent(uint32_t now) {
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

    // Close servo after ball passes
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

static void UpdateNavigation(uint32_t now) {
    unsigned int readings[6];
    unsigned char onTape[6];
    unsigned char banner = TapeSensor_CheckBanner();

    if (movement == DONE) {
        Motor_Stop();
        return;
    }

    if (movement == BUMPED) {
        // Shoot at the opponent after backing up
        if (Bumper_IsShooting() && ShootAtOpponent(now)) {
            Bumper_ShootingComplete(now);
        } else if (!Bumper_IsShooting() && RunBumperRecovery(now)) {
            // Finish recovery then look for tape
            movement = FIND_TAPE;
        }
        return;
    }

    if (movement == SHOOT_AT_TOWER) {
        RunTower(now);
        return;
    }

    if (movement == FIND_OPPONENT) {
        // Stop scan if beacon detected or timer runs out
        OpponentSearchResult result = SearchOpponent(
                TimePassed(now, SCAN_MS) ? SCAN_LIMIT_DEG : 0u);

        if (result == OPPONENT_FOUND) {
            Motor_SetLeftDirection(FORWARD);
            Motor_SetRightDirection(FORWARD);
            movement = APPROACH_OPPONENT;
        } else if (result == OPPONENT_NOT_FOUND) {
            movement = DONE;
        }
        return;
    }

    if (movement == APPROACH_OPPONENT) {
        // Drive toward the opponent until bumper is pressed
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

    // Return to tape search if both middle sensors lose the line
    if (!onTape[1] && !onTape[4]) {
        movement = FIND_TAPE;
        return;
    }

    LineFollowing(onTape);
}

/* ES_CheckEvents.c, detects bumper presses and posts events */
uint8_t CheckNavigationEvents(void) {
    static uint8_t lastPressed = 0u; // Previous bumper state
    uint8_t pressed = Bumper_IsPressed() ? 1u : 0u; // Current state

    // Detect a new bumper press
    if (pressed && !lastPressed) {
        ES_Event event = {NAV_BUMPER_PRESSED, 0u};

        // Post event to Navigation and retry if the queue is full
        if (!PostNavigation(event)) {
            return 0u;
        }

        lastPressed = pressed;
        return 1u;
    }

    // Save bumper state for next check
    lastPressed = pressed;
    return 0u;
}

/* ES_Framework.c, queues the startup event for Navigation */
uint8_t InitNavigation(uint8_t priority) {
    navigationPriority = priority;

    return PostNavigation((ES_Event) {
        ES_INIT, 0u
    });
}

/* ES_Timers.c, sends timer events to Navigation queue */
uint8_t PostNavigation(ES_Event event) {
    return ES_PostToService(navigationPriority, event);
}

/* ES_Framework.c, handles startup, bumper events, and navigation updates */
ES_Event RunNavigation(ES_Event event) {

    // Initialize the 10 ms navigation timer
    if (event.EventType == ES_INIT) {
        ES_Timer_InitTimer(NAV_TIMER, SAMPLE_MS);

        // Handle bumper press event
    } else if (event.EventType == NAV_BUMPER_PRESSED) {
        // Start recovery if not already recovering
        if (movement != BUMPED && movement != DONE) {
            // Stop shooting
            Flywheel_Stop();
            Servo2_Close();
            Servo3_Close();
            feederOpen = 0;
            flywheelStarted = 0;
            // Forget the banner
            bannerSeen = 0;

            movement = BUMPED;
            StartBumperRecovery(ES_Timer_GetTime());
        }

        // Update navigation every 10 ms and restart the timer
    } else if (event.EventType == ES_TIMEOUT &&
            event.EventParam == NAV_TIMER) {
        UpdateNavigation(ES_Timer_GetTime());
        ES_Timer_InitTimer(NAV_TIMER, SAMPLE_MS);
    }

    /* No additional events to process */
    return (ES_Event) {
        ES_NO_EVENT, 0u
    };
}
