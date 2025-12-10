#include <Arduino.h>
#include "traf_light_lib.h"

#define FLAG_TIMER 0x01
STM32Timer timer(TIM1);

#define TIMER1_PERIOD_MS 1000000 // microseconds (unit for STM32 boards)

systemState currentState = GREEN;

volatile int flags = 0;
volatile bool ped_request = false;
const int yellow_seconds = 3;
const int red_seconds = 5;
const int cooldown_seconds = 2;
int seconds_counter = 0;

void timer_isr(void)
{
  flags |= FLAG_TIMER;
}

int timer_timeout(void)
{
  return (flags & FLAG_TIMER);
}

void button_isr(void)
{
  if (currentState == GREEN)
  {
    ped_request = true;
  }
}

void setup()
{
  Serial.begin(9600);
  while (!Serial)
    ; // wait for serial port to connect. Needed for native USB

  delay(1000);
  Serial.println("\n[main.cpp] Initializing traffic light...");

  // RGB LED (Green at start)
  configInitRGBLed();

  // Button: internal pull-up, RISING-edge interrupt
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  attachInterrupt(BUTTON_PIN, button_isr, RISING);

  // Buzzer
  pinMode(BUZZER_PIN, OUTPUT);
  noTone(BUZZER_PIN);

  // Timer
  timer.attachInterruptInterval(TIMER1_PERIOD_MS, timer_isr);
}

void loop()
{
  if (timer_timeout())
  {
    // Clear timer flag
    flags &= ~FLAG_TIMER;

    // Update seconds counter if not zero
    if (seconds_counter > 0)
    {
      seconds_counter--;
    }

    // FSM for traffic light
    switch (currentState)
    {
    case GREEN:
      if (ped_request)
      {
        Serial.println("Button pressed! Switching to YELLOW...");

        currentState = YELLOW;
        ped_request = false;
        seconds_counter = yellow_seconds;
      }
      break;

    case YELLOW:
      setRGBLedColor(255, 255, 0); // Yellow
      if (seconds_counter == 0)
      {
        Serial.println("Switching to RED...");
        currentState = RED;
        seconds_counter = red_seconds;
      }
      break;

    case RED:
      setRGBLedColor(255, 0, 0); // Red
      Serial.println("Pedestrians can cross!");
      tone(BUZZER_PIN, 1000); // 1kHz tone
      delay(500);             // Buzzer on for 500ms
      noTone(BUZZER_PIN);     // Stop tone
      if (seconds_counter == 0)
      {
        Serial.println("Switching to COOLDOWN...");
        currentState = COOLDOWN;
        seconds_counter = cooldown_seconds;
      }
      break;

    case COOLDOWN:
      setRGBLedColor(0, 255, 0); // Green
      if (seconds_counter == 0)
      {
        currentState = GREEN;
      }
    }
  }
}