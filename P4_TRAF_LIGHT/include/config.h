#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

#define TIMER_INTERRUPT_DEBUG 1
#define __STM32__

#include "TimerInterrupt_Generic.h"

#define LED_R_PIN PB4
#define LED_G_PIN PB3
#define LED_B_PIN PB5
#define BUTTON_PIN USER_BTN
#define BUZZER_PIN PA6

#endif // CONFIG_H