#include <Arduino.h>
#include "traf_light_lib.h"

/**
 * @brief Initializes and configures the RGB LED pins.
 *
 * This function configures the RGB LED pins (red, green, and blue) as outputs
 * and sets their initial state. The LED is initialized with the green component
 * at maximum brightness (255), while red and blue components are turned off (0).
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
	analogWrite(LED_R_PIN, 0);
	analogWrite(LED_G_PIN, 255);
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
void setRGBLedColor(uint8_t r, uint8_t g, uint8_t b)
{
	analogWrite(LED_R_PIN, r);
	analogWrite(LED_G_PIN, g);
	analogWrite(LED_B_PIN, b);
}
