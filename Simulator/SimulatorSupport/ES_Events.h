/* Minimal event types required by the unmodified Navigation.c. */
#ifndef SIM_ES_EVENTS_H
#define SIM_ES_EVENTS_H
#include <stdint.h>
#include "ES_Configure.h"
typedef struct { ES_EventTyp_t EventType; uint16_t EventParam; } ES_Event;
#endif
