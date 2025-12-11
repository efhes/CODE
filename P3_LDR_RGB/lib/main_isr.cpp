/*
 * LDR + RGB LED with button ISR
 * - Button interrupt toggles LDR reading ON/OFF
 * - RGB LED indicates state (GREEN: LDR ON, RED: LDR OFF)
 * - LDR value read on PA1 when enabled
 */
#include <Arduino.h>
#include "ldr_led_lib.h"

volatile int button_count;
int totalLoops;

systemState currentState = LDR_OFF;
systemState prevState = currentState;

/**
 * Interrupt Service Routine (ISR) for the user button.
 *
 * Behavior:
 *  - Increments the interrupt event counter (button_count) on every trigger.
 *
 * Notes:
 *  - Keep ISRs short. Serial printing inside an ISR may increase latency and is
 *    generally discouraged on resource-constrained boards.
 *  - BUTTON_PIN is configured with INPUT_PULLUP and interrupt on RISING edge.
 */
void button_isr() {
  	button_count++;
	changeState();
}

/**
 * @brief Arduino setup routine for the LDR + RGB LED example with button ISR.
 *
 * Responsibilities:
 * - Initialize Serial at 9600 baud and wait for USB connection.
 * - Configure RGB LED pins as OUTPUT and set initial color RED (LDR OFF).
 * - Configure USER button as INPUT_PULLUP and attach RISING-edge ISR.
 * - Configure LDR pin (PA1) as analog INPUT.
 *
 * Notes:
 * - See button_isr() and changeState() for runtime behavior.
 */
void setup() {
	Serial.begin(9600);
  	while (!Serial);  // wait for serial port to connect. Needed for native USB
  
  	delay(1000);
  	Serial.println("\n[main_isr.cpp] Starting button ISR example...");
	
	// RGB LED
    configInitRGBLed();

    // Button: internal pull-up, RISING-edge interrupt
	pinMode(BUTTON_PIN, INPUT_PULLUP);
	attachInterrupt(BUTTON_PIN, button_isr, RISING);
	
	// LDR analog input
	pinMode(LDR_PIN, INPUT);
}


/**
 * @brief Main runtime loop.
 *
 * Behavior:
 * - Increments totalLoops counter each iteration.
 * - When the system state is LDR_ON, reads the analog value from LDR_PIN
 *   once per second and prints it to the Serial monitor.
 * - When the system state is LDR_OFF, the loop runs without delay, allowing
 *   quick responsiveness to button interrupts that may toggle the state.
 *
 */
void loop() {
	static uint32_t lastLdrRead = 0;
  	totalLoops++;
	if (currentState == LDR_ON) {
		// LDR is ON
		if (prevState != currentState) {
			// State just changed
			prevState = currentState;

			Serial.print("\n[ISR] Interrupt events detected: ");
			Serial.println(button_count);
			Serial.println("\n[STATE] LDR ON. Starting light detection...");
			setRGBLedColor(0, 255, 0);  // Ensure LED is GREEN (R=0, G=255, B=0)
		}
		
		// Uncomment the following line if you want to read LDR every second
		// if (millis() - lastLdrRead >= 1000) // Read LDR every second
		{ 
			lastLdrRead = millis();
			uint32_t ldrV = analogRead(LDR_PIN);
			Serial.print("\nCurrent LDR Value: ");
			Serial.println(ldrV);
		}
	}
	else {
		// LDR is OFF
		if (prevState != currentState) {
			// State just changed
			prevState = currentState;
			Serial.print("\n[ISR] Interrupt events detected: ");
			Serial.println(button_count);
			Serial.println("\n[STATE] LDR OFF. Stopping light detection...");
			setRGBLedColor(255, 0, 0);  // Ensure LED is RED (R=255, G=0, B=0)
		}
	}
}
