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
static unsigned int towerCount = 0; // Tower shots since the last direction change
static unsigned int lastTowerShotAt = 0; // Require the banner to clear before another shot
static unsigned char bannerArmed = 1; // Require the banner to clear before another shot
static unsigned char ballReleased = 0; // Ball not released
static unsigned char drivingInReverse = 0;
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

// Feed one ball and update the tower count

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
    // Wait for the banner to clear before allowing another shot
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
        // Finish backing off before resuming tape following
        if (RunBumperRecovery(now)) {
            movement = FIND_TAPE;
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

    if (drivingInReverse) {
        // Swap front and rear sensors and reverse left/right when driving backward
        for (unsigned int i = 0; i < 3; i++) {
            unsigned char saved = onTape[i];
            onTape[i] = onTape[5 - i];
            onTape[5 - i] = saved;
        }
    }

    // Keep moving in the current direction if no tape is detected
    if (!onTape[0] && !onTape[1] && !onTape[2]) {
        movement = FIND_TAPE;
        if (drivingInReverse) {
            Motor_Reverse(DRIVE_PWM);
        } else {
            Motor_Forward(DRIVE_PWM);
        }
        return;
    }

    movement = FOLLOW_TAPE;
    LineFollowing(onTape, drivingInReverse);
}

/* ES_CheckEvents.c, detects bumper presses and posts events */
uint8_t CheckNavigationEvents(void) {
    static uint8_t bumperEventPending = 0;
    if (Bumper_NewPress()) {
        bumperEventPending |= 1u;
    }
    if (Bumper_RearNewPress()) {
        bumperEventPending |= 2u;
    }
    if (bumperEventPending) {
        ES_Event event = {NAV_BUMPER_PRESSED, bumperEventPending};
        if (PostNavigation(event)) {
            bumperEventPending = 0;
            return 1;
        }
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
        drivingInReverse = 0;
        towerCount = 0;
        lastTowerShotAt = 0;
        bannerArmed = 1;
        ballReleased = 0;
        stepStart = ES_Timer_GetTime();
        bannerLowSince = TIMER_INACTIVE;
        ES_Timer_InitTimer(NAV_TIMER, SAMPLE_MS);

        // Handle bumper press event
    } else if (event.EventType == NAV_BUMPER_PRESSED) {
        unsigned char rearHit = (event.EventParam & 2u) != 0;
        unsigned char frontHit = event.EventParam == 0 || (event.EventParam & 1u) != 0;
        if (Bumper_IsPressed() && Bumper_RearIsPressed()) {
            Motor_Stop();
            Servo2_Close();
            movement = DONE;
        } else if (movement == FIND_OPPONENT) {
            if (rearHit && Bumper_RearIsPressed()) {
                // Rear input also protects opponent backing movements.
                Motor_Stop();
                Servo2_Close();
                movement = DONE;
            } else if (frontHit) {
                Opponent_OnBumperPress(ES_Timer_GetTime());
            }
        } else if (movement != DONE) {
            if ((frontHit && !drivingInReverse) || (rearHit && drivingInReverse)) {
                Servo2_Close();
                ballReleased = 0;
                // Front contact selects reverse. Rear contact selects forward.
                drivingInReverse = rearHit ? 0 : 1;
                // Start a new tower count in the new travel direction.
                towerCount = 0;
                bannerArmed = 0;
                bannerLowSince = TIMER_INACTIVE;
                lastTowerShotAt = ES_Timer_GetTime();
                movement = BUMPED;
                StartBumperEscape(ES_Timer_GetTime(), drivingInReverse);
            }
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
