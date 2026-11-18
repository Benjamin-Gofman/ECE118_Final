/*
 * ECE118 simulator adapter: CoppeliaSim IO <-> original C project.
 *
 * Navigation.c, Linefollowing.c and TapeSensor.c are compiled with the same navigation logic.
 * Python provides seven digital sensor inputs and reads signed motor PWM.
 * No navigation decisions, turn scripts, or tape logic live in this file.
 *
 * Hardware-only dependencies (PWM, servo, bumper, flywheel and ES scheduler)
 * are replaced by minimal simulator backends. These are NOT full hardware models.
 */
#include <stdint.h>
#include <stddef.h>
#include "Motor.h"
#include "TapeSensor.h"
#include "Bumper.h"
#include "Servo.h"
#include "Opponent.h"
#include "Flywheel.h"
#include "Navigation.h"
#include "ES_Framework.h"

#ifdef _WIN32
#define EXPORT __declspec(dllexport)
#else
#define EXPORT __attribute__((visibility("default")))
#endif

/* Original TapeSensor.c reads these seven simulated PIC32 input pins. */
unsigned char sim_tape_pins[7] = {0};
unsigned char sim_tape_tris[7] = {0};

static unsigned int pwm_left = 0, pwm_right = 0;
static unsigned char dir_left = FORWARD, dir_right = FORWARD;
unsigned char sim_opponent_detected = 0;
unsigned char sim_opponent_tris = 0;
/* Active-low PIC32 bumper inputs, shared by both simulated bumper pins. */
unsigned char sim_bumper_input = 1;
unsigned char sim_rear_bumper_input = 1;
unsigned char sim_rear_bumper_tris = 0;
unsigned char sim_bumper_left_tris = 0;
unsigned char sim_bumper_right_tris = 0;
static uint32_t sim_clock_ms = 0;
static uint32_t next_nav_timer_ms = 0;
static unsigned char timer_active = 0;

/* Preserve Motor.h's real API while storing PWM instead of calling hardware. */
void Motor_Init(void) { pwm_left = pwm_right = 0; dir_left = dir_right = FORWARD; }
void Motor_SetLeftDirection(unsigned char forward) { dir_left = forward ? FORWARD : REVERSE; }
void Motor_SetRightDirection(unsigned char forward) { dir_right = forward ? FORWARD : REVERSE; }
void Motor_SetLeftSpeed(unsigned int speed) { pwm_left = speed > 1000u ? 1000u : speed; }
void Motor_SetRightSpeed(unsigned int speed) { pwm_right = speed > 1000u ? 1000u : speed; }
void Motor_Stop(void) { pwm_left = pwm_right = 0; }

/* Simulated ES timer: Navigation.c schedules its existing 10 ms callbacks. */
uint32_t ES_Timer_GetTime(void) { return sim_clock_ms; }
void ES_Timer_InitTimer(uint8_t slot, uint32_t duration) {
    if (slot == NAV_TIMER) {
        next_nav_timer_ms = sim_clock_ms + duration;
        timer_active = 1;
    }
}
uint8_t ES_PostToService(uint8_t priority, ES_Event event) {
    (void)priority;
    RunNavigation(event);
    return 1u;
}

/* Servo/flywheel hardware is stubbed; bumper behavior comes from Bumper.c. */
void Servo_Init(void) {}
void Servo1_Open(void) {} void Servo1_Close(void) {}
void Servo2_Open(void) {} void Servo2_Close(void) {}
void Servo3_Open(void) {} void Servo3_Close(void) {}
/* The real Flywheel.c accesses PIC32 hardware LAT/TRIS registers.
 * Keep the hardware implementation in ECE118_Final.X for the UNO32.
 * These simulator no-op functions resolve Navigation.c references until
 * flywheel object commands are exposed and mapped in bridge.py.
 */
void Flywheel_Init(void) {}
void Flywheel_Start(void) {}
void Flywheel_Stop(void) {}
unsigned char SimHost_ReadOpponentDetected(void) { return sim_opponent_detected; }

EXPORT void SimHost_SetOpponentDetected(unsigned char detected) {
    sim_opponent_detected = detected ? 1u : 0u;
}

/* Digital contact input from the physical bumper force sensor. */
EXPORT void SimHost_SetBumperPressed(unsigned char pressed) {
    sim_bumper_input = pressed ? 0 : 1;
}

EXPORT void SimHost_SetRearBumperPressed(unsigned char pressed) {
    sim_rear_bumper_input = pressed ? 0 : 1;
}

EXPORT void SimHost_Init(void) {
    sim_opponent_detected = 0;
    sim_bumper_input = 1;
    sim_rear_bumper_input = 1;
    Motor_Init();
    TapeSensor_Init();
    Bumper_Init(); Servo_Init(); Flywheel_Init(); Opponent_Init();
    for (int i=0; i<7; ++i) sim_tape_pins[i] = 0;
    sim_clock_ms = 0;
    next_nav_timer_ms = 0;
    timer_active = 0;
    /* Run EXACTLY the existing Navigation.c initialization event. */
    RunNavigation((ES_Event){ ES_INIT, 0u });
}

/* Sensor order matches TapeSensor_ReadValues() and TOP in TapeSensor.c:
 * [0] FrontLeft [1] FrontMiddle [2] FrontRight
 * [3] RearLeft  [4] RearMiddle  [5] RearRight [6] Top/banner.
 * 1 means black tape (or banner detected), 0 means white/no banner.
 */
EXPORT void SimHost_SetSensors(unsigned char front_left, unsigned char front_middle,
                               unsigned char front_right, unsigned char rear_left,
                               unsigned char rear_middle, unsigned char rear_right,
                               unsigned char banner) {
    const unsigned char values[7] = {
        front_left, front_middle, front_right,
        rear_left, rear_middle, rear_right, banner
    };
    for (int i=0; i<7; ++i) sim_tape_pins[i] = values[i] ? 1u : 0u;
}

/* Advance the existing Navigation.c using CoppeliaSim's simulation clock. */
EXPORT void SimHost_Step(double seconds) {
    if (seconds < 0) return;
    uint32_t target_ms = (uint32_t)(seconds * 1000.0 + 0.5);
    if (target_ms < sim_clock_ms) return;
    int max_events = 1000; /* Avoid runaway if simulation time jumps. */
    while (timer_active && target_ms >= next_nav_timer_ms && max_events-- > 0) {
        sim_clock_ms = next_nav_timer_ms;
        timer_active = 0;
        /* Forward edge-triggered bumper events into the original C service. */
        CheckNavigationEvents();
        RunNavigation((ES_Event){ ES_TIMEOUT, NAV_TIMER });
    }
    sim_clock_ms = target_ms;
}

EXPORT void SimHost_GetMotors(int *left, int *right) {
    if (left)  *left  = (int)pwm_left * (dir_left == FORWARD ? +1 : -1);
    if (right) *right = (int)pwm_right * (dir_right == FORWARD ? +1 : -1);
}



/* Simulator implementations of the remaining Motor.h movement helpers.
 * Keep the hardware Motor.c unchanged; the simulator DLL uses this adapter. */
void Motor_TurnLeft(unsigned int speed) {
    Motor_SetLeftDirection(REVERSE);
    Motor_SetRightDirection(FORWARD);
    Motor_Speed(speed);
}

void Motor_TurnLeftArc(unsigned int speed) {
    Motor_Direction(FORWARD);
    Motor_SetLeftSpeed(speed / 3u);
    Motor_SetRightSpeed(speed);
}

void Motor_TurnRightArc(unsigned int speed) {
    Motor_Direction(FORWARD);
    Motor_SetLeftSpeed(speed);
    Motor_SetRightSpeed(speed / 3u);
}

/* Motor helpers, bumper edge detection, and diagnostic state names. */

void Motor_Direction(unsigned char direction) {
    Motor_SetLeftDirection(direction);
    Motor_SetRightDirection(direction);
}
void Motor_Speed(unsigned int speed) {
    Motor_SetLeftSpeed(speed);
    Motor_SetRightSpeed(speed);
}
void Motor_Forward(unsigned int speed) {
    Motor_Direction(FORWARD);
    Motor_Speed(speed);
}
void Motor_Reverse(unsigned int speed) {
    Motor_Direction(REVERSE);
    Motor_Speed(speed);
}
void Motor_TurnRight(unsigned int speed) {
    Motor_SetLeftDirection(FORWARD);
    Motor_SetRightDirection(REVERSE);
    Motor_Speed(speed);
}

extern unsigned int Navigation_GetDiagnosticState(void);
EXPORT const char *SimHost_GetStateName(void) {
    unsigned int state = Navigation_GetDiagnosticState();
    switch (state) {
        case 0: return "FIND_TAPE";
        case 1: return "FOLLOW_TAPE";
        case 2: return "BUMPER_ESCAPE";
        case 3: return "SHOOT_AT_TOWER";
        case 4: return Opponent_GetStateName();
        case 5: return "DONE";
        default: return "UNKNOWN";
    }
}
