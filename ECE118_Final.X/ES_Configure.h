#ifndef ES_CONFIGURE_H
#define ES_CONFIGURE_H

typedef enum {
    ES_NO_EVENT,        // ES_Queue.c, queue is empty
    ES_ERROR,           // ES_Framework.c, service error
    ES_INIT,            // Navigation.c, start the first timer
    ES_TIMEOUT,         // ES_Timers.c, timer expires
    ES_TIMERACTIVE,     // ES_Timers.c, timer starts
    ES_TIMERSTOPPED,    // ES_Timers.c, timer stops
    NAV_BUMPER_PRESSED  // Navigation.c, start bumper recovery
} ES_EventTyp_t;

// Register Navigation as the only service
#define NUM_SERVICES      1               // ES_Framework.c, number of services
#define SERV_0_HEADER     "Navigation.h"  // ES_ServiceHeaders.h, includes Navigation.h
#define SERV_0_INIT       InitNavigation  // ES_Framework.c, ES_Initialize() calls InitNavigation()
#define SERV_0_RUN        RunNavigation   // ES_Framework.c, ES_Run() calls RunNavigation()
#define SERV_0_QUEUE_SIZE 9               // ES_Framework.c, queue size for service 0

// Timer slots used by Navigation
#define NAV_TIMER         0u             // Navigation.c, schedule the next 10 ms update
#define STEP_TIMER        1u             // Navigation.c, time the current maneuver
#define TIMER0_RESP_FUNC  PostNavigation // ES_Timers.c, send slot 0 events to Navigation
#define TIMER1_RESP_FUNC  PostNavigation // ES_Timers.c, send slot 1 events to Navigation

// Unused slots still listed in ES_Timers.c, Timer2PostFunc
#define TIMER_UNUSED      ((pPostFunc)0) // No post function assigned
#define TIMER2_RESP_FUNC  TIMER_UNUSED
#define TIMER3_RESP_FUNC  TIMER_UNUSED
#define TIMER4_RESP_FUNC  TIMER_UNUSED
#define TIMER5_RESP_FUNC  TIMER_UNUSED
#define TIMER6_RESP_FUNC  TIMER_UNUSED
#define TIMER7_RESP_FUNC  TIMER_UNUSED
#define TIMER8_RESP_FUNC  TIMER_UNUSED
#define TIMER9_RESP_FUNC  TIMER_UNUSED
#define TIMER10_RESP_FUNC TIMER_UNUSED
#define TIMER11_RESP_FUNC TIMER_UNUSED
#define TIMER12_RESP_FUNC TIMER_UNUSED
#define TIMER13_RESP_FUNC TIMER_UNUSED
#define TIMER14_RESP_FUNC TIMER_UNUSED
#define TIMER15_RESP_FUNC TIMER_UNUSED

// Register the bumper event checker
#define EVENT_CHECK_HEADER "Navigation.h"      // ES_CheckEvents.c, includes this header
#define EVENT_CHECK_LIST CheckNavigationEvents // ES_CheckEvents.c, calls this function

// ES_Framework.c, requires a keyboard event callback even when keyboard is disabled
#define POST_KEY_FUNC ES_PostAll 

#endif