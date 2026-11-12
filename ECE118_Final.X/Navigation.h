#ifndef NAVIGATION_H
#define NAVIGATION_H

#include "ES_Configure.h"
#include "ES_Events.h"

// ES_Framework.c, ES_Initialize() calls this to save the service number and queue ES_INIT
uint8_t InitNavigation(uint8_t priority);

// ES_Timers.c, calls this to add timer events to Navigation's queue
uint8_t PostNavigation(ES_Event event);

// ES_Framework.c, ES_Run() calls this to handle one queued event
ES_Event RunNavigation(ES_Event event);

// ES_CheckEvents.c, ES_CheckUserEvents() calls this to check the bumper and post an event
uint8_t CheckNavigationEvents(void);

#endif