#include <Arduino.h>
#include "traf_light_lib.h"
#include "buzzer_lib.h"

volatile int flags = 0;

void setup() {
  Serial.begin(9600);
  while (!Serial)
    ; // wait for serial port to connect. Needed for native USB

  delay(1000);
  
  traffic_light_init(); // Initialize traffic light system
  basic_timer_init(); // Initialize basic timer system
  buzzer_init(); // Initialize buzzer system
  
  Serial.println("[main.cpp] Initialization complete.");
}

void loop() {
  // FSM for basic timer
  basic_timer_fsm_fire();

  // FSM for traffic light control
  traffic_light_fsm_fire();

  // FSM for buzzer control
  buzzer_fsm_fire();

  delay(10); // Small delay to avoid busy-waiting  
}