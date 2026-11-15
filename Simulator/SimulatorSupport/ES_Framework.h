/* Simulator scheduler adapter for Navigation.c's ES Framework API. */
#ifndef SIM_ES_FRAMEWORK_H
#define SIM_ES_FRAMEWORK_H
#include <stdint.h>
#include "ES_Events.h"
uint8_t ES_PostToService(uint8_t priority, ES_Event event);
void ES_Timer_InitTimer(uint8_t slot, uint32_t milliseconds);
uint32_t ES_Timer_GetTime(void);
#endif
