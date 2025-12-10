#ifndef TRAF_LIGHT_LIB_H
#define TRAF_LIGHT_LIB_H

#include <Arduino.h>
#include "config.h"

enum systemState
{
    GREEN,
    YELLOW,
    RED,
    COOLDOWN
};

extern systemState currentState;

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
void configInitRGBLed();

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
void setRGBLedColor(uint8_t r, uint8_t g, uint8_t b);

#endif // TRAF_LIGHT_LIB_H