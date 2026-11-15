/* Simulated PIC32 digital inputs. The original TapeSensor.c is unchanged. */
#ifndef SIM_IO_PORTS_H
#define SIM_IO_PORTS_H
extern unsigned char sim_tape_pins[7];
extern unsigned char sim_tape_tris[7];
/* Index order: FL, FM, FR, RL, RM, RR, TOP. */
#define PORTW05_BIT  sim_tape_pins[0]
#define PORTW04_BIT  sim_tape_pins[1]
#define PORTW03_BIT  sim_tape_pins[2]
#define PORTZ03_BIT  sim_tape_pins[3]
#define PORTZ04_BIT  sim_tape_pins[4]
#define PORTZ05_BIT  sim_tape_pins[5]
#define PORTY11_BIT  sim_tape_pins[6]
#define PORTW05_TRIS sim_tape_tris[0]
#define PORTW04_TRIS sim_tape_tris[1]
#define PORTW03_TRIS sim_tape_tris[2]
#define PORTZ03_TRIS sim_tape_tris[3]
#define PORTZ04_TRIS sim_tape_tris[4]
#define PORTZ05_TRIS sim_tape_tris[5]
#define PORTY11_TRIS sim_tape_tris[6]
#endif
