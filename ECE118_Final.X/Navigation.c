/*
 * Runs tape search, line following, bumper recovery,
 * tower scoring, and opponent search.
 */

#include <xc.h>
#include "Motor.h"
#include "TapeSensor.h"
#include "Bumper.h"
#include "Navigation.h"
#include "ES_Framework.h"
#include "Servo.h"
#include "Opponent.h"
#include "LineFollowing.h"

#define TOWER_TARGET_COUNT      2 // Number of towers before opponent search
#define TIMER_INACTIVE          0 // Timer has not started
#define TOWER_REARM_MS       3000 // Time between tower shots
#define BANNER_CLEAR_MS       600 // Banner must disappear before detecting next tower
#define SAMPLE_MS              10 // Navigation update interval
#define BALL_ROLL_MS         1000 // Wait for the ball to roll down the slide
#define FEED_ACTION_MS        300 // Time for opening or closing feeder

typedef enum {
    FIND_TAPE,
    FOLLOW_TAPE,
    BUMPED,
    SHOOT_AT_TOWER,
    FIND_OPPONENT,
    DONE
} MovementState;

static MovementState movement = FIND_TAPE; // Current navigation state
static unsigned int bannerLowSince = TIMER_INACTIVE; // Confirm banner is no longer detected at the same tower
static unsigned int stepStart; // Timer count
static unsigned int towerCount = 0; // Number of towers scored
static unsigned int lastTowerShotAt = 0; // Prevent shooting twice at the same tower
static unsigned char bannerArmed = 1; // Prevent shooting twice at the same tower
static unsigned char ballReleased = 0; // Ball not released
static uint8_t navigationPriority;

#ifdef SIMULATOR

// Read-only state for simulator diagnostics; navigation remains in C

unsigned int Navigation_GetDiagnosticState(void) {
    return (unsigned int) movement;
}
#endif

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
        Motor_Forward(DRIVE_PWM);
    }
}

static void RunTower(unsigned int now) {
    Motor_Stop();

    if (!ballReleased && TimePassed(now, FEED_ACTION_MS)) {
        // Close the feeder after giving one ball time to pass
        Servo2_Close();
        ballReleased = 1;
        stepStart = now;
    }

    // Wait for the ball to roll down the slide
    if (!ballReleased || !TimePassed(now, FEED_ACTION_MS + BALL_ROLL_MS)) {
        return;
    }

    towerCount++;
    // A new tower must have a separate, stable white-banner sighting.
    bannerArmed = 0;

    // Reset for the next tower
    ballReleased = 0;

    if (towerCount >= TOWER_TARGET_COUNT) {
        // After two different banner/tower shots, tape navigation is finished forever.
        Opponent_StartSearch(now);
        movement = FIND_OPPONENT;
    } else {
        movement = FOLLOW_TAPE;
    }
}

static void UpdateNavigation(uint32_t now) {
    unsigned int readings[6];
    unsigned char onTape[6];
    unsigned char banner;

    if (movement == DONE) {
        Motor_Stop();
        return;
    }

    if (movement == FIND_OPPONENT) {
        OpponentStatus result = Opponent_Update(now);
        // Check if opponent task is finished or failed
        if (result == OPPONENT_FINISHED || result == OPPONENT_FAILED) {
            Motor_Stop();
            movement = DONE;
        }
        return;
    }

    if (movement == BUMPED) {
        // Recover from an obstacle without launching balls at the opponent.
        if (RunBumperRecovery(now)) {
            if (towerCount >= TOWER_TARGET_COUNT) {
                Motor_Stop();
                movement = DONE;
            } else {
                Motor_Direction(FORWARD);
                movement = FIND_TAPE;
            }
        }
        return;
    }

    if (movement == SHOOT_AT_TOWER) {
        RunTower(now);
        return;
    }

    // Read banner and tape sensors, stop if reading fails
    banner = TapeSensor_CheckBanner();
    if (TapeSensor_ReadValues(readings) != SUCCESS ||
            TapeSensor_CheckTape(readings, onTape) != SUCCESS) {
        Motor_Stop();
        return;
    }

    if (!banner) {
        if (bannerLowSince == TIMER_INACTIVE) {
            bannerLowSince = now;
        }
        // Allow shooting at the next tower after the cooldown and banner clear time
        if (!bannerArmed &&
                (uint32_t) (now - lastTowerShotAt) >= TOWER_REARM_MS &&
                (uint32_t) (now - bannerLowSince) >= BANNER_CLEAR_MS) {
            bannerArmed = 1;
        }
    } else {
        bannerLowSince = TIMER_INACTIVE;
        // Shoot only if a new tower can be detected
        if (bannerArmed && towerCount < TOWER_TARGET_COUNT) {
            bannerArmed = 0;
            Motor_Stop();
            Servo1_Open();
            Servo2_Open();
            lastTowerShotAt = now;
            stepStart = now;
            movement = SHOOT_AT_TOWER;
            return;
        }
    }

    // Search for tape until the front sensors detect it
    if (movement == FIND_TAPE) {
        FindingTape(onTape);
        if (movement == FIND_TAPE) {
            return;
        }
    }

    // Drive straight if all three front sensors see white
    if (!onTape[0] && !onTape[1] && !onTape[2]) {
        movement = FIND_TAPE;
        Motor_Forward(DRIVE_PWM);
        return;
    }

    LineFollowing(onTape);
}

/* ES_CheckEvents.c, detects bumper presses and posts events */
uint8_t CheckNavigationEvents(void) {
    // Only post one event for each physical bumper press
    if (Bumper_NewPress()) {
        ES_Event event = {NAV_BUMPER_PRESSED, 0};

        // Post event to Navigation and retry if the queue is full
        if (!PostNavigation(event)) {
            return 0;
        }

        return 1;
    }
    return 0;
}

/* ES_Framework.c, queues the startup event for Navigation */
uint8_t InitNavigation(uint8_t priority) {
    navigationPriority = priority;

    return PostNavigation((ES_Event) {
        ES_INIT, 0
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
        movement = FIND_TAPE;
        stepStart = ES_Timer_GetTime();
        bannerLowSince = TIMER_INACTIVE;
        ES_Timer_InitTimer(NAV_TIMER, SAMPLE_MS);

        // Handle bumper press event
    } else if (event.EventType == NAV_BUMPER_PRESSED) {
        // Start recovery if not already recovering
        if (movement == FIND_OPPONENT) {
            Opponent_OnBumperPress(ES_Timer_GetTime());
        } else if (movement != BUMPED && movement != DONE) {
            // Stop tower feeding while recovering from an obstacle.
            Servo2_Close();
            ballReleased = 0;
            // Forget the banner

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
        ES_NO_EVENT, 0
    };
}
