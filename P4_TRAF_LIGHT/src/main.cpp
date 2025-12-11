#include <Arduino.h>
#include "traf_light_lib.h"

#define FLAG_TIMER      0x01 // Timer flag:     its 32-bit value equals to 00000000 00000000 00000000 00000001
#define FLAG_BUTTON     0x02 // Button flag:    its 32-bit value equals to 00000000 00000000 00000000 00000010
#define FLAG_COUNTDOWN  0x04 // Countdown flag: its 32-bit value equals to 00000000 00000000 00000000 00000100

STM32Timer timer(TIM1); // Using TIM1
STM32Timer timer_buzzer(TIM2); // Using TIM2

#define TIMER1_PERIOD_US 1000000 // Period in microseconds (unit for STM32 boards), equals to 1 second
#define DEBOUNCE_TIME 250      // Debounce time in milliseconds
unsigned long DebounceTimer = 0;

systemState currentState = GREEN;

volatile int flags = 0;
const int yellow_seconds = 3;
const int red_seconds = 5;
const int cooldown_seconds = 2;

int seconds_counter = 0;
int ellapsed_seconds = 0; // Seconds elapsed since the beginning of the program

/**
 * @brief Interrupt service routine for timer interrupts.
 * 
 * Sets the timer flag in the global flags variable when called by the timer
 * interrupt. This function is executed in interrupt context.
 * 
 * @note This is an ISR and should execute as quickly as possible.
 * @see FLAG_TIMER
 */
void timer_isr(void) {
  flags |= FLAG_TIMER;
}

/**
 * @brief Interrupt service routine for button press detection with debouncing.
 * 
 * Handles button press events while preventing false triggers caused by electrical
 * noise (bouncing). Uses a timer-based debouncing mechanism to ensure only valid
 * button presses are registered.
 * 
 * @details When called, checks if enough time has elapsed since the last valid
 * button press (DEBOUNCE_TIME). If the debounce period has passed, updates the
 * debounce timer and sets the FLAG_BUTTON flag to signal a valid button press.
 * 
 * @note This function should be attached to a button interrupt pin.
 * @see DEBOUNCE_TIME - debounce delay in milliseconds
 * @see FLAG_BUTTON - flag indicating a valid button press occurred
 * @see DebounceTimer - global variable tracking the last valid button press time
 * @see flags - global variable holding status flags
 */
void button_isr(void) {
  if (millis() - DebounceTimer >= DEBOUNCE_TIME) {
    DebounceTimer = millis();
		flags |= FLAG_BUTTON;
	}
}
  
int timer_timeout(void) {
  return (flags & FLAG_TIMER);
}

void update_timer(void) {
  flags &= ~FLAG_TIMER; // Clears the timer flag
  
  if (seconds_counter > 0) { // Update seconds counter if not zero
    seconds_counter--;
  }
  else
    flags |= FLAG_COUNTDOWN; // Set countdown finished flag

  ellapsed_seconds++; // Increment elapsed seconds
  Serial.print("Elapsed seconds: ");
  Serial.println(ellapsed_seconds);
}

/**
 * @brief Checks if a button has been pressed.
 * 
 * This function checks the FLAG_BUTTON bit in the flags variable. If the flag
 * is set, it returns 1 to indicate a button press was detected.
 * If the flag is not set, it returns 0.
 * 
 * @return 1 if a button press was detected (flag was set), 0 otherwise.
 * 
 * @note This function uses a global or static 'flags' variable that must be
 *       managed elsewhere in the program (typically in an interrupt handler).
 */
int button_pressed(void) {
  if (flags & FLAG_BUTTON) {
    return 1;
  }
  return 0;
}

/**
 * @brief Checks if the countdown timer has finished.
 * 
 * This function checks the FLAG_COUNTDOWN bit in the flags variable. If the flag
 * is set, it returns 1 to indicate the countdown has finished.
 * If the flag is not set, it returns 0.
 * 
 * @return 1 if the countdown has finished (flag was set), 0 otherwise.
 * 
 * @note This flag is set by update_timer() when seconds_counter reaches zero.
 * @see FLAG_COUNTDOWN - flag indicating countdown completion
 * @see flags - global variable holding status flags
 */
int countdown_finished(void) {
  if (flags & FLAG_COUNTDOWN) {
    return 1;
  }
  return 0;
}

/**
 * @brief Transitions the traffic light state machine to YELLOW and updates all related hardware/state variables.
 * 
 * This function performs a state transition from GREEN to YELLOW when a pedestrian button press is detected.
 * It handles the following operations:
 * - Clears the button press flag to prevent multiple state transitions
 * - Updates the current FSM state to YELLOW
 * - Sets the RGB LED to yellow (255, 255, 0)
 * - Initializes the countdown timer for the YELLOW state duration (yellow_seconds = 3)
 * - Logs state change information to the serial port
 * 
 * @details
 * This function is part of the traffic light finite state machine (FSM). It is typically called
 * from the main loop when:
 * 1. The current state is GREEN (vehicle has right of way)
 * 2. A valid button press is detected (button_pressed() returns 1)
 * 
 * The YELLOW state indicates a transition period where:
 * - Vehicles are warned to prepare to stop
 * - Pedestrians are instructed to wait (they cannot cross)
 * - After yellow_seconds (3 seconds), the FSM transitions to RED state
 * 
 * @return void
 * 
 * @note This function must be called from the main application loop (not from an ISR).
 * @note Assumes global variables are initialized: currentState, flags, seconds_counter
 * @note The countdown is managed by update_timer() and tracked via the countdown_finished() check
 * 
 * @see update_to_red() - The next state transition (YELLOW -> RED)
 * @see button_pressed() - Function that triggers this state transition
 * @see FLAG_BUTTON - Flag that is cleared by this function
 * @see yellow_seconds - Duration constant for YELLOW state (3 seconds)
 * @see setRGBLedColor() - Hardware abstraction for LED control
 */
void update_to_yellow(void) {
  flags &= ~FLAG_BUTTON; // Clears the button flag
  currentState = YELLOW; // Switch to YELLOW state
  Serial.println("Button pressed!!!!");
  Serial.println("[TRAFFIC LIGHT][YELLOW][PEDESTRIANS WAIT]");
  setRGBLedColor(255, 255, 0); // Set LED to Yellow
  seconds_counter = yellow_seconds; // Set countdown for YELLOW state
}

/**
 * @brief Transitions the traffic light state machine to RED and updates all related hardware/state variables.
 * 
 * This function performs a state transition from YELLOW to RED when the yellow countdown timer finishes.
 * It handles the following operations:
 * - Clears the countdown flag to prevent multiple state transitions
 * - Updates the current FSM state to RED
 * - Sets the RGB LED to red (255, 0, 0)
 * - Activates the buzzer at 1kHz frequency to signal pedestrians
 * - Initializes the countdown timer for the RED state duration (red_seconds = 5)
 * - Logs state change information to the serial port
 * 
 * @details
 * This function is part of the traffic light finite state machine (FSM). It is typically called
 * from the main loop when:
 * 1. The current state is YELLOW (transition period)
 * 2. The countdown timer has finished (countdown_finished() returns 1)
 * 
 * The RED state indicates the main traffic light control phase where:
 * - Vehicles must stop (red light)
 * - Pedestrians have the right of way and can cross
 * - An audible buzzer (1kHz tone) alerts pedestrians to cross
 * - After red_seconds (5 seconds), the FSM transitions to COOLDOWN state
 * 
 * @return void
 * 
 * @note This function must be called from the main application loop (not from an ISR).
 * @note Assumes global variables are initialized: currentState, flags, seconds_counter
 * @note The countdown is managed by update_timer() and tracked via the countdown_finished() check
 * @note The buzzer tone must be disabled when transitioning out of RED state
 * 
 * @see update_to_yellow() - The previous state transition (GREEN -> YELLOW)
 * @see update_to_cooldown() - The next state transition (RED -> COOLDOWN)
 * @see countdown_finished() - Function that triggers this state transition
 * @see FLAG_COUNTDOWN - Flag that is cleared by this function
 * @see red_seconds - Duration constant for RED state (5 seconds)
 * @see setRGBLedColor() - Hardware abstraction for LED control
 * @see tone() - Function to activate the buzzer
 */
void update_to_red(void) {
  flags &= ~FLAG_COUNTDOWN; // Clears the countdown flag
  currentState = RED; // Switch to RED state
  Serial.println("Countdown finished!!!!");
  Serial.println("[TRAFFIC LIGHT][RED][PEDESTRIANS CROSS]");
  setRGBLedColor(255, 0, 0); // Set LED to Red
  seconds_counter = red_seconds; // Set countdown for RED state
  tone(BUZZER_PIN, 1000); // 1kHz tone
}

/**
 * @brief Transitions the traffic light state machine to COOLDOWN and updates all related hardware/state variables.
 * 
 * This function performs a state transition from RED to COOLDOWN when the red countdown timer finishes.
 * It handles the following operations:
 * - Clears the countdown flag to prevent multiple state transitions
 * - Updates the current FSM state to COOLDOWN
 * - Sets the RGB LED to blue (0, 0, 255)
 * - Disables the buzzer that was activated during RED state
 * - Initializes the countdown timer for the COOLDOWN state duration (cooldown_seconds = 2)
 * - Logs state change information to the serial port
 * 
 * @details
 * This function is part of the traffic light finite state machine (FSM). It is typically called
 * from the main loop when:
 * 1. The current state is RED (pedestrians crossing)
 * 2. The countdown timer has finished (countdown_finished() returns 1)
 * 
 * The COOLDOWN state represents a brief transition period where:
 * - Vehicles and pedestrians both must wait (neither has the right of way)
 * - The blue LED indicates a transitional/cooldown state
 * - The buzzer is turned off (pedestrians have finished crossing)
 * - After cooldown_seconds (2 seconds), the FSM transitions back to GREEN state
 * 
 * @return void
 * 
 * @note This function must be called from the main application loop (not from an ISR).
 * @note Assumes global variables are initialized: currentState, flags, seconds_counter
 * @note The countdown is managed by update_timer() and tracked via the countdown_finished() check
 * @note The buzzer must be disabled before transitioning to the next state
 * 
 * @see update_to_red() - The previous state transition (YELLOW -> RED)
 * @see update_to_green() - The next state transition (COOLDOWN -> GREEN)
 * @see countdown_finished() - Function that triggers this state transition
 * @see FLAG_COUNTDOWN - Flag that is cleared by this function
 * @see cooldown_seconds - Duration constant for COOLDOWN state (2 seconds)
 * @see setRGBLedColor() - Hardware abstraction for LED control
 * @see noTone() - Function to disable the buzzer
 */
void update_to_cooldown(void) {
  flags &= ~FLAG_COUNTDOWN; // Clears the countdown flag
  currentState = COOLDOWN; // Switch to COOLDOWN state
  Serial.println("Countdown finished!!!!");
  Serial.println("[TRAFFIC LIGHT][COOLDOWN][PEDESTRIANS WAIT]");
  setRGBLedColor(0, 0, 255); // Set LED to Blue
  seconds_counter = cooldown_seconds; // Set countdown for COOLDOWN state
  noTone(BUZZER_PIN); // Stop tone
}

/**
 * @brief Transitions the traffic light state machine to GREEN and updates all related hardware/state variables.
 * 
 * This function performs a state transition from COOLDOWN to GREEN when the cooldown countdown timer finishes.
 * It handles the following operations:
 * - Clears the countdown flag to prevent multiple state transitions
 * - Updates the current FSM state to GREEN
 * - Sets the RGB LED to green (0, 255, 0)
 * - Logs state change information to the serial port
 * 
 * @details
 * This function is part of the traffic light finite state machine (FSM). It is typically called
 * from the main loop when:
 * 1. The current state is COOLDOWN (transition period after pedestrians cross)
 * 2. The countdown timer has finished (countdown_finished() returns 1)
 * 
 * The GREEN state indicates the main vehicle traffic phase where:
 * - Vehicles have the right of way and can proceed
 * - Pedestrians must wait (red light for pedestrians)
 * - The traffic light cycles back to allow pedestrian button presses
 * - This state continues indefinitely until a pedestrian presses the button
 * 
 * @return void
 * 
 * @note This function must be called from the main application loop (not from an ISR).
 * @note Assumes global variables are initialized: currentState, flags
 * @note Unlike other state transitions, GREEN state has no countdown duration (runs until button pressed)
 * 
 * @see update_to_cooldown() - The previous state transition (RED -> COOLDOWN)
 * @see update_to_yellow() - The next state transition (GREEN -> YELLOW)
 * @see countdown_finished() - Function that triggers this state transition
 * @see FLAG_COUNTDOWN - Flag that is cleared by this function
 * @see button_pressed() - Function used to exit this state
 * @see setRGBLedColor() - Hardware abstraction for LED control
 */
void update_to_green(void) {
  flags &= ~FLAG_COUNTDOWN; // Clears the countdown flag
  currentState = GREEN; // Switch to GREEN state
  Serial.println("Switching to GREEN...");
  setRGBLedColor(0, 255, 0); // Set LED to Green
}

void setup() {
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

  // FSM
  update_to_green();

  // Timer
  timer.attachInterruptInterval(TIMER1_PERIOD_US, timer_isr);
}

void loop() {
  // FSM timer event handling
  if (timer_timeout()) // Timer event occurred
    update_timer(); // Clear timer flag

  // FSM for traffic light control
  switch (currentState) {
    case GREEN:
      if (button_pressed())
        update_to_yellow();
      break;

    case YELLOW:
      if (countdown_finished())
        update_to_red();
      break;

    case RED:
      if (countdown_finished())
        update_to_cooldown();
      break;

    case COOLDOWN:
      if (countdown_finished())
        update_to_green();
      break;

    default:
      Serial.println("Error: Unknown state!");
      break;
  }

  // FSM for buzzer control could be added here

  delay(10); // Small delay to avoid busy-waiting  
}