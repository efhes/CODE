#include <Arduino.h>
#include "traf_light_lib.h"

#define FLAG_TIMER         0x01 // Timer flag:        its 32-bit value equals to 00000000 00000000 00000000 00000001
#define FLAG_BUTTON        0x02 // Button flag:       its 32-bit value equals to 00000000 00000000 00000000 00000010
#define FLAG_COUNTDOWN     0x04 // Countdown flag:    its 32-bit value equals to 00000000 00000000 00000000 00000100
#define FLAG_BUZZER_ACTIVE 0x08 // Buzzer flag:       its 32-bit value equals to 00000000 00000000 00000000 00001000
#define FLAG_BUZZER_TIMER  0x10 // Buzzer timer flag: its 32-bit value equals to 00000000 00000000 00000000 00010000

STM32Timer timer(TIM1); // Using TIM1
STM32Timer timer_buzzer(TIM2); // Using TIM2

#define TIMER1_PERIOD_US 1000000 // Period in microseconds (unit for STM32 boards), equals to 1 second
#define TIMER2_PERIOD_US 800000  // Period in microseconds for buzzer timer, equals to 800 milliseconds
#define DEBOUNCE_TIME 250      // Debounce time in milliseconds
unsigned long DebounceTimer = 0;

systemState currentState = GREEN;

enum BuzzerStateEnum
{
    WAITING,
    OFF,
    ON
};

BuzzerStateEnum buzzerState = WAITING;
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

void timer_buzzer_isr(void) {
  flags |= FLAG_BUZZER_TIMER;
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

/**
 * @brief Checks if the main system timer has timed out.
 * 
 * This function checks the FLAG_TIMER bit in the flags variable. If the flag
 * is set, it returns 1 to indicate a timer interrupt has occurred.
 * If the flag is not set, it returns 0.
 * 
 * @return 1 if the timer has timed out (flag was set), 0 otherwise.
 * 
 * @details The main system timer is configured to interrupt every 1 second (TIMER1_PERIOD_US).
 * When a timer interrupt occurs, the timer_isr() function sets the FLAG_TIMER flag.
 * This function checks that flag to determine if a timer event should be processed in the main loop.
 * 
 * @note This flag is set by timer_isr() when the main timer interrupt occurs.
 * @note This function is checked in the main loop to trigger the update_timer() function.
 * 
 * @see FLAG_TIMER - flag indicating timer interrupt occurred
 * @see timer_isr() - ISR that sets this flag
 * @see update_timer() - Function typically called when this returns 1
 * @see flags - global variable holding status flags
 * @see TIMER1_PERIOD_US - Timer period configuration (1 second)
 */
int timer_timeout(void) {
  return (flags & FLAG_TIMER);
}

/**
 * @brief Handles timer tick events and manages the traffic light state countdown.
 * 
 * This function is called from the main loop when the main system timer times out (every 1 second).
 * It performs the following operations:
 * - Clears the FLAG_TIMER flag to indicate the timer event has been processed
 * - Decrements the seconds_counter if it is greater than zero
 * - Sets the FLAG_COUNTDOWN flag when seconds_counter reaches zero to signal countdown completion
 * - Increments the ellapsed_seconds counter to track total elapsed time since program start
 * - Logs the elapsed seconds to the serial port for debugging/monitoring
 * 
 * @details
 * This function implements the core timing mechanism for the traffic light FSM. It is responsible
 * for managing state duration countdowns (YELLOW = 3 seconds, RED = 5 seconds, COOLDOWN = 2 seconds).
 * The countdown works as follows:
 * - A state is entered with seconds_counter set to its duration
 * - Each timer tick decrements seconds_counter
 * - When seconds_counter reaches 0, FLAG_COUNTDOWN is set to trigger the next state transition
 * 
 * The ellapsed_seconds counter tracks total program runtime for monitoring and debugging purposes.
 * 
 * @return void
 * 
 * @note This function should be called from the main loop immediately after timer_timeout() returns true.
 * @note This function must NOT be called from an ISR (it is called from the main loop, not interrupt context).
 * @note The seconds_counter variable is set by state transition functions (e.g., update_to_yellow(), update_to_red()).
 * 
 * @see timer_timeout() - Function that checks if this timer handler should be called
 * @see FLAG_TIMER - Flag that is cleared by this function
 * @see FLAG_COUNTDOWN - Flag that is set when countdown reaches zero
 * @see seconds_counter - Countdown timer for current state duration
 * @see ellapsed_seconds - Total elapsed time since program start
 * @see TIMER1_PERIOD_US - Timer interrupt period (1 second)
 * @see timer_isr() - ISR that triggers this function via FLAG_TIMER
 */
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
  flags |= FLAG_BUZZER_ACTIVE; // Set buzzer active flag
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
  flags &= ~FLAG_BUZZER_ACTIVE; // Clear buzzer active flag
}

/**
 * @brief Checks if the buzzer timer has timed out.
 * 
 * This function checks the FLAG_BUZZER_TIMER bit in the flags variable. If the flag
 * is set, it returns 1 to indicate the buzzer timer has expired.
 * If the flag is not set, it returns 0.
 * 
 * @return 1 if the buzzer timer has timed out (flag was set), 0 otherwise.
 * 
 * @details The buzzer timer is an 800ms interval timer (configured in setup()) that controls
 * the buzzer beeping pattern. When this timeout occurs, the buzzer FSM transitions between
 * ON and OFF states to create an intermittent beeping effect.
 * 
 * @note This flag is set by timer_buzzer_isr() when the buzzer timer interrupt occurs.
 * @note This function is used in the buzzer FSM to manage state transitions.
 * 
 * @see FLAG_BUZZER_TIMER - flag indicating buzzer timer timeout
 * @see timer_buzzer_isr() - ISR that sets this flag
 * @see flags - global variable holding status flags
 */
int buzzer_timer_timeout(void) {
  return (flags & FLAG_BUZZER_TIMER);
}

/**
 * @brief Checks if the buzzer should transition to ON (active) state.
 * 
 * This function checks the FLAG_BUZZER_ACTIVE bit in the flags variable. If the flag
 * is set, it returns 1 to indicate the buzzer should be activated.
 * If the flag is not set, it returns 0.
 * 
 * @return 1 if the buzzer should activate (FLAG_BUZZER_ACTIVE is set), 0 otherwise.
 * 
 * @details This condition is checked in the buzzer FSM to transition from WAITING or OFF
 * states to the ON state when the traffic light enters RED state.
 * 
 * @note This flag is set by update_to_red() when RED state is entered.
 * @note This flag is cleared by update_to_cooldown() when leaving RED state.
 * @see FLAG_BUZZER_ACTIVE - flag indicating buzzer should be active
 * @see update_to_red() - Sets this flag
 * @see update_to_cooldown() - Clears this flag
 */
int transition_to_buzzer_active(void) {
  return (flags & FLAG_BUZZER_ACTIVE);
}

/**
 * @brief Checks if the buzzer should transition to WAITING (inactive) state.
 * 
 * This function checks the FLAG_BUZZER_ACTIVE bit in the flags variable. If the flag
 * is NOT set, it returns 1 to indicate the buzzer should transition to WAITING state.
 * If the flag is set, it returns 0.
 * 
 * @return 1 if the buzzer should transition to WAITING (FLAG_BUZZER_ACTIVE is NOT set), 0 otherwise.
 * 
 * @details This condition is checked in the buzzer FSM to transition from ON or OFF
 * states back to the WAITING state when the traffic light leaves RED state.
 * 
 * @note This function uses the inverse logic of FLAG_BUZZER_ACTIVE for simplicity,
 *       allowing us to use the same flag for both active and waiting state checks.
 * @note This flag is cleared by update_to_cooldown() when leaving RED state.
 * @see FLAG_BUZZER_ACTIVE - flag indicating buzzer should be active
 * @see update_to_red() - Sets FLAG_BUZZER_ACTIVE
 * @see update_to_cooldown() - Clears FLAG_BUZZER_ACTIVE
 */
int transition_to_buzzer_waiting(void) {
  return !(flags & FLAG_BUZZER_ACTIVE); // For simplicity, we use the same flag; in this case we check the opposite condition
}

/**
 * @brief Transitions the buzzer FSM to OFF state and manages the buzzer timer.
 * 
 * This function performs a state transition in the buzzer finite state machine from ON to OFF state.
 * It handles the following operations:
 * - Updates the buzzer FSM state to OFF
 * - Deactivates the buzzer sound (stops the 1kHz tone)
 * - Enables the buzzer timer to track the OFF duration
 * - Clears the buzzer timer flag to indicate the timer has been restarted
 * 
 * @details
 * This function is part of the buzzer finite state machine (FSM) that creates an intermittent
 * beeping pattern. It is typically called from the main loop when:
 * 1. The buzzer is currently in ON state (actively beeping)
 * 2. The buzzer timer has timed out (800ms interval), indicating it's time to turn OFF
 * 
 * The OFF state represents a silent period in the buzzer beeping cycle where:
 * - The buzzer sound is deactivated
 * - The timer is restarted to measure the OFF duration (800ms)
 * - After the timer expires, the FSM transitions back to ON state to resume beeping
 * - This creates an alternating ON/OFF pattern (on for 800ms, off for 800ms, etc.)
 * 
 * @return void
 * 
 * @note This function must be called from the main application loop (not from an ISR).
 * @note The OFF state duration is controlled by the 800ms buzzer timer interval.
 * @note This transition occurs only when the traffic light is in RED state (FLAG_BUZZER_ACTIVE is set).
 * 
 * @see update_to_buzzer_on() - The complementary ON state transition
 * @see update_to_buzzer_waiting() - Transitions buzzer FSM back to idle state
 * @see buzzer_timer_timeout() - Condition that triggers this transition
 * @see FLAG_BUZZER_TIMER - Flag cleared when timer is restarted
 * @see timer_buzzer - The 800ms interval timer controlling buzzer timing
 * @see noTone() - Function to deactivate the buzzer
 */
void update_to_buzzer_off(void) {
  buzzerState = OFF;
  noTone(BUZZER_PIN); // Deactivate buzzer
  // We restart the buzzer timer
  timer_buzzer.enableTimer();
  flags &= ~FLAG_BUZZER_TIMER; // Clear TIMER flag, buzzer timer started
}

/**
 * @brief Transitions the buzzer FSM to ON state and manages the buzzer timer.
 * 
 * This function performs a state transition in the buzzer finite state machine from OFF or WAITING to ON state.
 * It handles the following operations:
 * - Updates the buzzer FSM state to ON
 * - Activates the buzzer sound at 1kHz frequency
 * - Enables the buzzer timer to track the ON duration
 * - Clears the buzzer timer flag to indicate the timer has been restarted
 * 
 * @details
 * This function is part of the buzzer finite state machine (FSM) that creates an intermittent
 * beeping pattern. It is typically called from the main loop when:
 * 1. The buzzer is in WAITING state and FLAG_BUZZER_ACTIVE becomes set (traffic light enters RED state)
 * 2. The buzzer is in OFF state and the buzzer timer has timed out (800ms interval), indicating it's time to turn ON
 * 
 * The ON state represents an active beeping period in the buzzer cycle where:
 * - The buzzer sound is activated at 1kHz frequency
 * - The timer is restarted to measure the ON duration (800ms)
 * - After the timer expires, the FSM transitions to OFF state to create a silent period
 * - This creates an alternating ON/OFF pattern (on for 800ms, off for 800ms, etc.)
 * 
 * @return void
 * 
 * @note This function must be called from the main application loop (not from an ISR).
 * @note The ON state duration is controlled by the 800ms buzzer timer interval.
 * @note This transition occurs only when the traffic light is in RED state (FLAG_BUZZER_ACTIVE is set).
 * 
 * @see update_to_buzzer_off() - The complementary OFF state transition
 * @see update_to_buzzer_waiting() - Transitions buzzer FSM back to idle state
 * @see buzzer_timer_timeout() - Condition that triggers transition from OFF to ON
 * @see transition_to_buzzer_active() - Condition that triggers transition from WAITING to ON
 * @see FLAG_BUZZER_TIMER - Flag cleared when timer is restarted
 * @see timer_buzzer - The 800ms interval timer controlling buzzer timing
 * @see tone() - Function to activate the buzzer at specified frequency
 */
void update_to_buzzer_on(void) {
  buzzerState = ON;
  tone(BUZZER_PIN, 1000); // Activate buzzer at 1kHz
  // We start the buzzer timer
  timer_buzzer.enableTimer();
  flags &= ~FLAG_BUZZER_TIMER; // Clear TIMER flag, buzzer timer started
}

/**
 * @brief Transitions the buzzer FSM to WAITING state and stops the buzzer timer.
 * 
 * This function performs a state transition in the buzzer finite state machine to WAITING state.
 * It handles the following operations:
 * - Updates the buzzer FSM state to WAITING
 * - Deactivates the buzzer sound (stops any active tone)
 * - Disables the buzzer timer to stop measuring intervals
 * - Clears the buzzer timer flag to indicate the timer has been stopped
 * 
 * @details
 * This function is part of the buzzer finite state machine (FSM) that manages buzzer behavior.
 * It is typically called from the main loop when:
 * 1. The buzzer is in ON or OFF state and FLAG_BUZZER_ACTIVE becomes cleared (traffic light leaves RED state)
 * 2. The traffic light transitions away from RED state (via update_to_cooldown())
 * 
 * The WAITING state represents an inactive/idle state for the buzzer where:
 * - The buzzer sound is completely deactivated
 * - The buzzer timer is stopped to save resources
 * - The buzzer waits for FLAG_BUZZER_ACTIVE to be set again (RED state entry)
 * - No periodic beeping occurs during this state
 * 
 * @return void
 * 
 * @note This function must be called from the main application loop (not from an ISR).
 * @note This transition occurs when the traffic light leaves the RED state.
 * @note The buzzer timer is disabled during WAITING state to conserve CPU resources.
 * 
 * @see update_to_buzzer_on() - Transitions to ON state when buzzer activation is needed
 * @see update_to_buzzer_off() - Transitions to OFF state for silent periods during beeping
 * @see transition_to_buzzer_waiting() - Condition that triggers this transition
 * @see FLAG_BUZZER_ACTIVE - Flag that controls buzzer activation
 * @see update_to_cooldown() - Function that triggers this transition by clearing FLAG_BUZZER_ACTIVE
 * @see timer_buzzer - The 800ms interval timer that is disabled in this function
 * @see noTone() - Function to deactivate the buzzer
 */
void update_to_buzzer_waiting(void) {
  buzzerState = WAITING;
  noTone(BUZZER_PIN); // Deactivate buzzer
  timer_buzzer.disableTimer(); // Stop buzzer timer
  flags &= ~FLAG_BUZZER_TIMER; // Clear TIMER flag to indicate buzzer timer stopped
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

  // Buzzer Timer
  timer_buzzer.attachInterruptInterval(TIMER2_PERIOD_US, timer_buzzer_isr); // 800ms
  timer_buzzer.disableTimer(); // Starts disabled
  buzzerState = WAITING; // Initial buzzer state
}

void loop() {
  // FSM timer event handling
  if (timer_timeout()) {
    update_timer(); // Clear timer flag
  }

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

  // FSM for buzzer control
  // Our system has an intermittent beeping buzzer so we need 3 states: WAITING, ON and OFF
  // We use the existing timer_buzzer timer and flags to manage buzzer state
  // We implement the solution
  switch (buzzerState) {
    case WAITING: // Buzzer WAITING state
      if (transition_to_buzzer_active()) { // Check if buzzer activation condition is met
        update_to_buzzer_on(); // Transition to ON state
      }
      break;

    case ON: // Buzzer ON state
      if (buzzer_timer_timeout()) { // Check if buzzer deactivation condition is met
        update_to_buzzer_off(); // Transition to OFF state
      }
      else if(transition_to_buzzer_waiting()) {
        update_to_buzzer_waiting(); // Transition to WAITING state
      }
      break;

    case OFF: // Buzzer OFF state
      if(buzzer_timer_timeout()) { // Check if buzzer activation condition is met
        update_to_buzzer_on(); // Transition to ON state
      }
      else if(transition_to_buzzer_waiting()) { // Check if can transition to WAITING state
        update_to_buzzer_waiting(); // Transition to WAITING state
      }
      break;

    default:
      Serial.println("Error: Unknown buzzer state!");
      break;
  }

  delay(10); // Small delay to avoid busy-waiting  
}