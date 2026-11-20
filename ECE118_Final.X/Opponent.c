#include <xc.h>

#include "Opponent.h"
#include "Motor.h"
#include "Bumper.h"
#include "Servo.h"
#include "Flywheel.h"

#define SCAN_360_MS         6000u  // Time allowed for a full rotation
#define COLLISION_BACKOFF_MS 400u  // Time to reverse after opponent collision
#define BACKOFF_MAX_MS      1500u  // Maximum time to reverse if bumper stays pressed
#define FLYWHEEL_READY_MS   1000u  // Time for flywheels to reach speed
#define FEED_ACTION_MS       300u  // Time to open or close the feeder
#define BALL_ROLL_MS        1000u  // Time for the ball to roll down the slide

typedef enum {
    OPP_SEARCH,
    OPP_APPROACH,
    OPP_BACKOFF,
    OPP_SPINUP,
    OPP_SHOOT,
    OPP_DONE,
    OPP_FAILED
} OpponentState;

static OpponentState state = OPP_SEARCH;
static uint32_t stepStart;
static unsigned char shots = 0u;
static unsigned char ballReleased = 0u;
static unsigned char forwardAllowed = 1u;

// Returns the opponent state for the simulator

const char *Opponent_GetStateName(void) {
    static const char *const names[] = {
        "OPP_SEARCH", "OPP_APPROACH", "OPP_BACKOFF",
        "OPP_SPINUP", "OPP_SHOOT", "OPP_DONE", "OPP_FAILED"
    };
    unsigned int index = (unsigned int) state;
    return index < 7u ? names[index] : "OPP_UNKNOWN";
}

/* Initialize opponent detection and reset the search */
void Opponent_Init(void) {
    BEACON_TRIS = 1;
    state = OPP_SEARCH;
    shots = 0u;
    ballReleased = 0u;
    forwardAllowed = 1u;
}

/* Enables or disables beacon detection */
void Opponent_EnableBeacon(unsigned char allowed) {
    forwardAllowed = allowed ? 1u : 0u;
}

/* Read beacon sensor */
unsigned char OpponentBeaconDetected(void) {
    return BEACON_INPUT;
}

/* Check if enough time has passed */
static unsigned char TimePassed(uint32_t now, uint32_t milliseconds) {
    return (uint32_t) (now - stepStart) >= milliseconds;
}

/* Start searching for the opponent */
void Opponent_StartSearch(uint32_t now) {
    Motor_Stop();
    Flywheel_Stop();
    state = OPP_SEARCH;
    stepStart = now;
    shots = 0u;
    ballReleased = 0u;
}

/* Handle the bumper press and start backing off */
void Opponent_OnBumperPress(uint32_t now) {
    if (state != OPP_APPROACH) {
        return;
    }

    state = OPP_BACKOFF;
    stepStart = now;
    Motor_Reverse(SLOW_PWM);
}

/* Run opponent search, approach, backoff, and shooting */
OpponentStatus Opponent_Update(uint32_t now) {
    switch (state) {
        case OPP_SEARCH:
            // Pause the search timer until beacon detection is enabled
            if (!forwardAllowed) {
                stepStart = now;
            }
            // Rotate until the opponent is detected
            if (forwardAllowed && OpponentBeaconDetected()) {
                state = OPP_APPROACH;
                Motor_Forward(DRIVE_PWM);
            } else if (TimePassed(now, SCAN_360_MS)) {
                Motor_Stop();
                state = OPP_FAILED;
            } else {
                Motor_TurnRight(SNAIL_PWM);
            }
            break;

        case OPP_APPROACH:
            // Drive forward toward the opponent
            Motor_Forward(DRIVE_PWM);
            break;

        case OPP_BACKOFF:
            // Reverse until the bumper is released
            if (TimePassed(now, BACKOFF_MAX_MS) && Bumper_IsPressed()) {
                Motor_Stop();
                state = OPP_FAILED;
            } else if (!TimePassed(now, COLLISION_BACKOFF_MS) || Bumper_IsPressed()) {
                Motor_Reverse(SLOW_PWM);
            } else {
                Motor_Stop();
                Flywheel_Start();
                stepStart = now;
                state = OPP_SPINUP;
            }
            break;

        case OPP_SPINUP:
            // Wait for the flywheels before shooting
            Motor_Stop();
            if (TimePassed(now, FLYWHEEL_READY_MS)) {
                shots = 0u;
                ballReleased = 0u;
                Servo1_Open();
                Servo2_Open();
                stepStart = now;
                state = OPP_SHOOT;
            }
            break;

        case OPP_SHOOT:
            // Release two balls toward the opponent
            Motor_Stop();
            if (!ballReleased) {
                if (TimePassed(now, FEED_ACTION_MS)) {
                    Servo2_Close();
                    ballReleased = 1u;
                    stepStart = now;
                }
            } else if (TimePassed(now, BALL_ROLL_MS)) {
                shots++;
                if (shots >= OPPONENT_SHOT_COUNT) {
                    Motor_Stop();
                    Flywheel_Stop();
                    state = OPP_DONE;
                } else {
                    ballReleased = 0u;
                    stepStart = now;
                    Servo1_Open();
                    Servo2_Open();
                }
            }
            break;

        case OPP_DONE:
        case OPP_FAILED:
            // Stop the robot when finished or failed
            Motor_Stop();
            Flywheel_Stop();
            break;
    }

    // Return the current opponent task status
    if (state == OPP_DONE) return OPPONENT_FINISHED;
    if (state == OPP_FAILED) return OPPONENT_FAILED;
    return OPPONENT_RUNNING;
}
