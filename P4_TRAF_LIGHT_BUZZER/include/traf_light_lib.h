#ifndef TRAF_LIGHT_LIB_H
#define TRAF_LIGHT_LIB_H

#include <Arduino.h>
#include "config.h"

#define TIMER1_PERIOD_US 1000000 // Period in microseconds (unit for STM32 boards), equals to 1 second

#define DEBOUNCE_TIME 150      // Debounce time in milliseconds

#define YELLOW_SECONDS 3
#define RED_SECONDS 5
#define COOLDOWN_SECONDS 2

enum systemStateEnum
{
    GREEN,
    YELLOW,
    RED,
    COOLDOWN
};

extern systemStateEnum currentState;
extern volatile int flags;

void traffic_light_init(void);
void basic_timer_init(void);

void traffic_light_fsm_fire(void);
void basic_timer_fsm_fire(void);

#endif // TRAF_LIGHT_LIB_H