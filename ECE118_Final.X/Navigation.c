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

#define TOWER_TARGET_COUNT       2u // Number of towers before opponent search
#define TIMER_INACTIVE           0u // Timer has not started
#define TOWER_REARM_MS        3000u // Time between tower shots
#define BANNER_CLEAR_MS        600u // Banner must disappear before detecting next tower
#define SAMPLE_MS               10u // Navigation update interval
#define BALL_ROLL_MS          1000u // Wait for the ball to roll down the slide
#define FEED_ACTION_MS         300u // Time for opening or closing feeder
#define BOUNDARY_BACKUP_MS     500u // Move away from a detected boundary
#define BOUNDARY_TURN_MS       400u // Maximum turn while checking FM/RM for alignment
#define BOUNDARY_TURN_MIN_MS   150u // Minimum turn before checking FM/RM
#define BOUNDARY_WIDE_TURN_MS  650u // Maximum turn for sharp FR/RR boundary detection
#define BOUNDARY_WIDE_MIN_MS   220u // Minimum sharp turn before checking FM/RM
#define BOUNDARY_CLEAR_SAMPLES   5u // Require stable boundary clearance before resuming

typedef enum {
    FIND_TAPE,
    FOLLOW_TAPE,
    BOUNDARY_BACK_UP,
    BOUNDARY_TURN,
    BUMPED,
    SHOOT_AT_TOWER,
    FIND_OPPONENT,
    DONE
} MovementState;

static MovementState movement = FIND_TAPE; // Current navigation state
static unsigned int bannerLowSince = TIMER_INACTIVE; // Confirm banner is no longer detected at the same tower
static unsigned int stepStart; // Timer count
static unsigned int towerCount = 0u; // Tower shots since the last direction change
static unsigned int lastTowerShotAt = 0u; // Require the banner to clear before another shot
static unsigned char bannerArmed = 1u; // Require the banner to clear before another shot
static unsigned char ballReleased = 0u; // Ball not released
static unsigned char drivingInReverse = 0u; // Current driving direction
static unsigned char boundaryTravelWasReverse = 0u; // Driving direction before boundary recovery
static unsigned char boundaryWideContact = 0u; // Indicates multiple sensors detected the boundary
static unsigned char boundaryClearSamples = 0u; // Counts consecutive readings with boundary sensors clear
static unsigned char boundaryEscapeStoppedByRearTape = 0u; // Indicates the backup stopped because the rear sensor detected tape
static uint8_t navigationPriority;

/* Returns the driving direction for the simulator */
unsigned char Navigation_IsDrivingInReverse(void) {
    return drivingInReverse;
}

/* Returns the current navigation state for the simulator */
unsigned int Navigation_GetDiagnosticState(void) {
    return (unsigned int) movement;
}

unsigned char Navigation_EscapeStoppedByRearTape(void) {
    return boundaryEscapeStoppedByRearTape; // 1 = backup stopped by tape
}

static unsigned char TimePassed(unsigned int now, uint32_t milliseconds) {
    // Return 1 after the current movement step lasts long enough
    return (uint32_t) (now - stepStart) >= milliseconds;
}

/* Check FR when moving forward, RR when moving in reverse */
static unsigned char RightBoundaryDetected(const unsigned char onTape[6],
        unsigned char reverse) {
    return onTape[reverse ? 5 : 2] != 0u;
}

/* Take a wider turn if FR/RR and another leading sensor detect tape */
static unsigned char WideBoundaryContact(const unsigned char onTape[6],
        unsigned char reverse) {
    unsigned char leftOnTape = onTape[reverse ? 3 : 0];
    unsigned char middleOnTape = onTape[reverse ? 4 : 1];
    unsigned char rightOnTape = onTape[reverse ? 5 : 2];

    return rightOnTape && (middleOnTape || leftOnTape);
}

static void BoundaryBackAway(void) {
    // Drive forward if previously driving in reverse
    if (boundaryTravelWasReverse) {
        Motor_Forward(SLOW_PWM);
    } else {
        // Drive in reverse if previously driving forward
        Motor_Reverse(SLOW_PWM);
    }
}

/* Start boundary recovery and preserve the original driving direction */
static void StartBoundaryRecovery(uint32_t now,
        const unsigned char onTape[6]) {
    boundaryTravelWasReverse = drivingInReverse; // 1 = reverse, 0 = forward
    boundaryWideContact = WideBoundaryContact(onTape, drivingInReverse);
    boundaryClearSamples = 0u;
    boundaryEscapeStoppedByRearTape = 0u;
    movement = BOUNDARY_BACK_UP;
    stepStart = now;
    Motor_Stop();
    BoundaryBackAway();
}

/* Turn right after FR detection or left after RR detection */
static void BoundaryTurnAwayFromBoundary(void) {
    unsigned int pwm = boundaryWideContact ? SLOW_PWM : SNAIL_PWM;
    if (boundaryTravelWasReverse) {
        Motor_TurnLeft(pwm);
    } else {
        Motor_TurnRight(pwm);
    }
}

/* Feed one ball and update the tower count */
static void RunTower(unsigned int now) {
    Motor_Stop();

    // Close the feeder after giving one ball time to pass
    if (!ballReleased && TimePassed(now, FEED_ACTION_MS)) {
        Servo2_Close();
        ballReleased = 1u;
        stepStart = now;
    }

    // Wait for the ball to roll down the slide
    if (!ballReleased || !TimePassed(now, FEED_ACTION_MS + BALL_ROLL_MS)) {
        return;
    }

    towerCount++;
    // Reset for the next tower
    bannerArmed = 0u;
    ballReleased = 0u;

    // Search for opponent after two tower shots
    if (towerCount >= TOWER_TARGET_COUNT) {
        Opponent_EnableBeacon(1u);
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

    // Stop the robot if the tape sensor readings are invalid
    if (TapeSensor_ReadValues(readings) != SUCCESS ||
            TapeSensor_CheckTape(readings, onTape) != SUCCESS) {
        Motor_Stop();
        return;
    }

    // Stop the robot after completing the run
    if (movement == DONE) {
        Motor_Stop();
        return;
    }

    // Ignore boundary tape during opponent search
    if (movement == FIND_OPPONENT) {
        Opponent_EnableBeacon(1u);
        OpponentStatus result = Opponent_Update(now);
        if (result == OPPONENT_FINISHED || result == OPPONENT_FAILED) {
            Motor_Stop();
            movement = DONE;
        }
        return; // Exit before checking the boundary sensors
    }

    if (movement == BOUNDARY_BACK_UP) {
        unsigned char rearTape = RightBoundaryDetected(onTape,
                !boundaryTravelWasReverse);
        // Stop backing up if the opposite boundary sensor detects tape or the timer expires
        if (rearTape || TimePassed(now, BOUNDARY_BACKUP_MS)) {
            boundaryEscapeStoppedByRearTape = rearTape;
            Motor_Stop();
            movement = BOUNDARY_TURN;
            boundaryClearSamples = 0u;
            stepStart = now;
            BoundaryTurnAwayFromBoundary();
        } else {
            BoundaryBackAway();
        }
        return;
    }

    if (movement == BOUNDARY_TURN) {
        // Use FM/RM and FR/RR to check alignment during the turn
        unsigned char middleOnTape = onTape[boundaryTravelWasReverse ? 4 : 1];
        unsigned char rightBoundary = RightBoundaryDetected(onTape,
                boundaryTravelWasReverse);
        // Select turn times based on boundary contact
        uint32_t minMs = boundaryWideContact ?
                BOUNDARY_WIDE_MIN_MS : BOUNDARY_TURN_MIN_MS;
        uint32_t maxMs = boundaryWideContact ?
                BOUNDARY_WIDE_TURN_MS : BOUNDARY_TURN_MS;

        // Confirm FM/RM detects tape while FR/RR remains clear
        if (middleOnTape && !rightBoundary) {
            if (boundaryClearSamples < BOUNDARY_CLEAR_SAMPLES) {
                boundaryClearSamples++;
            }
        } else {
            boundaryClearSamples = 0u;
        }

        if ((TimePassed(now, minMs) &&
                boundaryClearSamples >= BOUNDARY_CLEAR_SAMPLES) ||
                TimePassed(now, maxMs)) {
            // Turn completed or maximum time reached
            Motor_Stop();
            if (rightBoundary) {
                // Back away if boundary still detected
                movement = BOUNDARY_BACK_UP;
                stepStart = now;
                boundaryClearSamples = 0u;
                BoundaryBackAway();
            } else {
                // Resume tape following or searching
                Set_TapeFound(1u);
                movement = boundaryClearSamples >= BOUNDARY_CLEAR_SAMPLES ?
                        FOLLOW_TAPE : FIND_TAPE;
            }
        } else {
            // Continue turning until clearance or timeout
            BoundaryTurnAwayFromBoundary();
        }
        return;
    }

    // Start boundary recovery when FR/RR detects tape
    if (RightBoundaryDetected(onTape, drivingInReverse)) {
        StartBoundaryRecovery(now, onTape);
        return;
    }

    if (movement == BUMPED) {
        // Continue bumper recovery until completed
        if (RunBumperRecovery(now)) {
            Set_TapeFound(1u);
            movement = FIND_TAPE;
        }
        return;
    }

    if (movement == SHOOT_AT_TOWER) {
        RunTower(now);
        return;
    }

    // Read the banner sensor
    banner = TapeSensor_CheckBanner();

    if (!banner) {
        if (bannerLowSince == TIMER_INACTIVE) {
            bannerLowSince = now;
        }
        // Allow shooting at the next tower after the cooldown and banner clear time
        if (!bannerArmed &&
                (uint32_t) (now - lastTowerShotAt) >= TOWER_REARM_MS &&
                (uint32_t) (now - bannerLowSince) >= BANNER_CLEAR_MS) {
            bannerArmed = 1u;
        }
    } else {
        bannerLowSince = TIMER_INACTIVE;
        // Shoot only if a new tower can be detected
        if (bannerArmed && towerCount < TOWER_TARGET_COUNT) {
            bannerArmed = 0u;
            Motor_Stop();
            Servo1_Open();
            Servo2_Open();
            lastTowerShotAt = now;
            stepStart = now;
            movement = SHOOT_AT_TOWER;
            return;
        }
    }

    // Switch to FIND_TAPE if the line is lost, keep following otherwise
    movement = LineFollowing(onTape, drivingInReverse) ? FOLLOW_TAPE : FIND_TAPE;
}

/* ES_CheckEvents.c, detects bumper presses and posts events */
uint8_t CheckNavigationEvents(void) {
    static uint8_t bumperEventPending = 0u;
    if (Bumper_NewPress()) {
        bumperEventPending |= 1u;
    }
    if (Bumper_RearNewPress()) {
        bumperEventPending |= 2u;
    }
    if (bumperEventPending) {
        ES_Event event = {NAV_BUMPER_PRESSED, bumperEventPending};
        if (PostNavigation(event)) {
            bumperEventPending = 0u;
            return 1u;
        }
    }
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
        movement = FIND_TAPE;
        drivingInReverse = 0u;
        boundaryTravelWasReverse = 0u;
        boundaryWideContact = 0u;
        boundaryClearSamples = 0u;
        boundaryEscapeStoppedByRearTape = 0u;
        Set_TapeFound(0u);
        towerCount = 0u;
        lastTowerShotAt = 0u;
        bannerArmed = 1u;
        ballReleased = 0u;
        stepStart = ES_Timer_GetTime();
        bannerLowSince = TIMER_INACTIVE;
        ES_Timer_InitTimer(NAV_TIMER, SAMPLE_MS);

        // Handle bumper press event
    } else if (event.EventType == NAV_BUMPER_PRESSED) {
        unsigned char rearHit = (event.EventParam & 2u) != 0u;
        unsigned char frontHit = event.EventParam == 0u || (event.EventParam & 1u) != 0u;
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
        } else if (movement != DONE && movement != BOUNDARY_BACK_UP &&
                movement != BOUNDARY_TURN) {
            // Handle bumper events only when boundary recovery is inactive
            unsigned int boundaryReadings[6];
            unsigned char boundaryTape[6];
            if (TapeSensor_ReadValues(boundaryReadings) != SUCCESS ||
                    TapeSensor_CheckTape(boundaryReadings, boundaryTape) != SUCCESS) {
                Motor_Stop();
            } else if (RightBoundaryDetected(boundaryTape, drivingInReverse)) {
                StartBoundaryRecovery(ES_Timer_GetTime(), boundaryTape);
            } else if ((frontHit && !drivingInReverse) ||
                    (rearHit && drivingInReverse)) {
                Servo2_Close();
                ballReleased = 0u;
                // Front contact selects reverse. Rear contact selects forward.
                drivingInReverse = rearHit ? 0u : 1u;
                Set_TapeFound(0u);
                // Start a new tower count in the new travel direction.
                towerCount = 0u;
                bannerArmed = 0u;
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
        ES_NO_EVENT, 0u
    };
}
