#include <Arduino.h>
#include "ldr_led_lib.h"

systemState currentState = LDR_OFF;

/**
 * @brief Initializes and configures the RGB LED pins.
 * 
 * This function configures the RGB LED pins (red, green, and blue) as outputs
 * and sets their initial state. The LED is initialized with the red component
 * at maximum brightness (255), while green and blue components are turned off (0).
 * 
 * @note This function must be called before using any RGB LED control functions.
 * @note The pin numbers are defined by LED_R_PIN, LED_G_PIN, and LED_B_PIN macros.
 * 
 * @return void
 */
void configInitRGBLed()
{
    pinMode(LED_R_PIN, OUTPUT);
    pinMode(LED_G_PIN, OUTPUT);
    pinMode(LED_B_PIN, OUTPUT);
    analogWrite(LED_R_PIN, 255);
    analogWrite(LED_G_PIN, 0);
    analogWrite(LED_B_PIN, 0);
}

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
