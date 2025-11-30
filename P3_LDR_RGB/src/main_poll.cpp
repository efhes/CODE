#include <Arduino.h>
#include "ldr_led_lib.h"

int totalLoops;

#define POLLING_PERIOD 1000 // milliseconds

void setup() {
	Serial.begin(9600);
  	while (!Serial);  // wait for serial port to connect. Needed for native USB
  
  	delay(1000);
  	Serial.println("\n[main_pool.cpp]Starting button ISR example...");
	
	// RGB LED
  	configInitRGBLed();

  	// Button
	pinMode(BUTTON_PIN, INPUT_PULLUP);
	
	// LDR
	pinMode(LDR_PIN, INPUT);

	currentState = LDR_OFF;
}

void loop() {
  	totalLoops++;
	
	// delay() is a blocking function, which means that during this time the microcontroller cannot respond to other events,
	// read other sensors, or execute any other task. 
	// If you need your program to be more responsive or handle multiple tasks simultaneously, consider using non-blocking techniques like millis() to control timing.
	delay(POLLING_PERIOD);

	if (digitalRead(BUTTON_PIN) == LOW) {
		while(digitalRead(BUTTON_PIN) == LOW);
		
		changeState();

		if (currentState == LDR_ON) {
			setRGBLedColor(0, 255, 0);
			Serial.println("\n[STATE] LDR ON. Starting light detection...");
		}
		else {
			setRGBLedColor(255, 0, 0);
			Serial.println("\n[STATE] LDR OFF. Stopping light detection...");
		}
	}

	if (currentState == LDR_ON) {
		uint32_t ldrV = analogRead(LDR_PIN);
		Serial.print("\nCurrent LDR Value: ");
		Serial.println(ldrV);
	}
}
