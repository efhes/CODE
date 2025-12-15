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

#define FLAG_TIMER         0x01 // Timer flag:        its 32-bit value equals to 00000000 00000000 00000000 00000001
#define FLAG_BUTTON        0x02 // Button flag:       its 32-bit value equals to 00000000 00000000 00000000 00000010
#define FLAG_COUNTDOWN     0x04 // Countdown flag:    its 32-bit value equals to 00000000 00000000 00000000 00000100

// To be completed by the student
#define FLAG_BUZZER_ACTIVE 0x00 // Buzzer flag:       its 32-bit value equals to XXXXXXXX XXXXXXXX XXXXXXXX XXXXXXXX 
#define FLAG_BUZZER_TIMER  0x00 // Buzzer timer flag: its 32-bit value equals to XXXXXXXX XXXXXXXX XXXXXXXX XXXXXXXX

#endif // CONFIG_H