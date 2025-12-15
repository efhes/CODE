#ifndef BUZZER_LIB_H
#define BUZZER_LIB_H

#include <Arduino.h>
#include "config.h"

#define TIMER2_PERIOD_US 500000  // Period in microseconds for buzzer timer, equals to 500 milliseconds

// Our system has an intermittent beeping buzzer so we need 3 states: WAITING, ON and OFF
enum BuzzerStateEnum
{
    WAITING,
    OFF,
    ON
};

extern BuzzerStateEnum buzzerState;
extern volatile int flags;

void buzzer_init(void);

void buzzer_fsm_fire(void);

#endif // BUZZER_LIB_H
