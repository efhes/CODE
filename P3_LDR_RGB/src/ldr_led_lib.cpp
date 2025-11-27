#include <Arduino.h>
#include "ldr_led_lib.h"

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
