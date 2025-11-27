/*
 * LDR + RGB LED with debounced button ISR
 * - Button interrupt toggles LDR reading ON/OFF with simple debounce
 * - RGB LED indicates state (GREEN: LDR ON, RED: LDR OFF)
 * - LDR value read on PA1 when enabled
 */
#include <Arduino.h>

volatile uint32_t DebounceTimer = 0;
volatile int button_count;
int totalLoops;

#define LED_R_PIN PB4
#define LED_G_PIN PB3
#define LED_B_PIN PB5
#define BUTTON_PIN USER_BTN
#define LDR_PIN PA1
#define DEBOUNCE_TIME 250

enum systemState {
	LDR_ON,
	LDR_OFF
};

systemState currentState = LDR_OFF;
systemState prevState = currentState;

/**
 * Set RGB LED color via PWM duty cycle.
 *
 * Parameters:
 *  - r: Red channel intensity (0 = off, 255 = max brightness)
 *  - g: Green channel intensity (0 = off, 255 = max brightness)
 *  - b: Blue channel intensity (0 = off, 255 = max brightness)
 *
 * Notes:
 *  - Uses LED_R_PIN, LED_G_PIN, LED_B_PIN configured as PWM-capable outputs.
 */
void setRGBLedColor(uint8_t r, uint8_t g, uint8_t b) {
	analogWrite(LED_R_PIN, r);
	analogWrite(LED_G_PIN, g);
	analogWrite(LED_B_PIN, b);
}

/**
 * Toggle system state between LDR_ON and LDR_OFF.
 *
 * Effects:
 *  - Updates currentState.
 *  - Sets RGB LED to GREEN when LDR is enabled, RED when disabled.
 *  - Prints a state message to the Serial monitor.
 *
 * Notes:
 *  - This function is invoked by the button ISR (button_isr) after debouncing.
 *    Keep it short; avoid heavy processing. Serial and PWM calls may elongate
 *    ISR latency on some boards.
 */
void changeState() {
	if (currentState == LDR_OFF) {
		currentState = LDR_ON;
	} 
	else {
		currentState = LDR_OFF;
	}
}

/**
 * Interrupt Service Routine (ISR) for the user button with simple debounce.
 *
 * Behavior:
 *  - Increments the interrupt event counter (button_count) on every trigger.
 *  - If at least DEBOUNCE_TIME milliseconds have elapsed since the last accepted
 *    event, updates DebounceTimer and invokes changeState() to toggle the system
 *    between LDR_ON and LDR_OFF.
 *
 * Timing/Assumptions:
 *  - Uses millis() for timing; DEBOUNCE_TIME must be chosen appropriately for
 *    the hardware/button characteristics.
 *
 * Notes:
 *  - Keep ISRs short. Serial printing inside an ISR may increase latency and is
 *    generally discouraged on resource-constrained boards.
 *  - BUTTON_PIN is configured with INPUT_PULLUP and interrupt on RISING edge.
 */
void button_isr() {
  	button_count++;
	if (millis() - DebounceTimer >= DEBOUNCE_TIME) {
    	DebounceTimer = millis();
		changeState();
	}
}

/**
 * @brief Arduino setup routine for the LDR + RGB LED example with debounced button ISR.
 *
 * Responsibilities:
 * - Initialize Serial at 9600 baud and wait for USB connection.
 * - Configure RGB LED pins as OUTPUT and set initial color RED (LDR OFF).
 * - Configure USER button as INPUT_PULLUP and attach RISING-edge ISR.
 * - Configure LDR pin (PA1) as analog INPUT.
 *
 * Notes:
 * - The button ISR applies a simple time-based debounce using DEBOUNCE_TIME.
 * - See button_isr() and changeState() for runtime behavior.
 */
void setup() {
	Serial.begin(9600);
  	while (!Serial);  // wait for serial port to connect. Needed for native USB
  
  	delay(1000);
  	Serial.println("Starting button ISR example...");
	
	// RGB LED
	pinMode(LED_R_PIN, OUTPUT);
	pinMode(LED_G_PIN, OUTPUT);
	pinMode(LED_B_PIN, OUTPUT);
	analogWrite(LED_R_PIN, 255);
	analogWrite(LED_G_PIN, 0);
	analogWrite(LED_B_PIN, 0);
	
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
 * Notes:
 * - The delay(1000) only runs while LDR is ON. Interrupts remain active during
 *   delay(), so the button ISR can still trigger and toggle the state.
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
		
		if (millis() - lastLdrRead >= 1000) { // Read LDR every second
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
