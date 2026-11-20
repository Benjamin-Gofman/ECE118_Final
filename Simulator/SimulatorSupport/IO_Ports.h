// Simulated pins matching the current robot wiring.
#ifndef SIM_IO_PORTS_H
#define SIM_IO_PORTS_H

extern unsigned char sim_tape_pins[7];
extern unsigned char sim_tape_tris[7];
extern unsigned char sim_opponent_detected;
extern unsigned char sim_opponent_tris;
extern unsigned char sim_bumper_input;
extern unsigned char sim_bumper_left_tris;
extern unsigned char sim_bumper_right_tris;
extern unsigned char sim_rear_bumper_input;
extern unsigned char sim_rear_bumper_tris;

// Tape sensor order is FL, FM, FR, RL, RM, RR, TOP.
#define PORTW08_BIT  sim_tape_pins[0]
#define PORTW06_BIT  sim_tape_pins[1]
#define PORTW04_BIT  sim_tape_pins[2]
#define PORTY12_BIT  sim_tape_pins[3]
#define PORTY11_BIT  sim_tape_pins[4]
#define PORTY10_BIT  sim_tape_pins[5]
#define PORTY03_BIT  sim_tape_pins[6]

#define PORTW08_TRIS sim_tape_tris[0]
#define PORTW06_TRIS sim_tape_tris[1]
#define PORTW04_TRIS sim_tape_tris[2]
#define PORTY12_TRIS sim_tape_tris[3]
#define PORTY11_TRIS sim_tape_tris[4]
#define PORTY10_TRIS sim_tape_tris[5]
#define PORTY03_TRIS sim_tape_tris[6]

// Both front switches receive the front force-sensor input.
#define PORTW07_BIT  sim_bumper_input
#define PORTW05_BIT  sim_bumper_input
#define PORTW07_TRIS sim_bumper_left_tris
#define PORTW05_TRIS sim_bumper_right_tris

#define PORTV04_BIT  sim_rear_bumper_input
#define PORTV04_TRIS sim_rear_bumper_tris

/* The same BEACON_INPUT/BEACON_TRIS macros used by the PIC32 firmware
 * now read/write these virtual pins; no #ifdef is needed in Opponent.c.
 * SimHost_SetOpponentDetected() updates sim_opponent_detected from bridge.py. */
#define PORTW03_BIT  sim_opponent_detected
#define PORTW03_TRIS sim_opponent_tris

#endif
