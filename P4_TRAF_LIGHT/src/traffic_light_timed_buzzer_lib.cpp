#include <Arduino.h>
#include "traf_light_lib.h"

systemStateEnum currentState = GREEN;
STM32Timer timer(TIM1); // Using TIM1
unsigned long DebounceTimer = 0;
int seconds_counter = 0; // Countdown timer for current state duration
int ellapsed_seconds = 0; // Seconds elapsed since the beginning of the program

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
 * - Initializes the countdown timer for the YELLOW state duration (YELLOW_SECONDS = 3)
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
 * - After YELLOW_SECONDS (3 seconds), the FSM transitions to RED state
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
 * @see YELLOW_SECONDS - Duration constant for YELLOW state (3 seconds)
 * @see setRGBLedColor() - Hardware abstraction for LED control
 */
void update_to_yellow(void) {
  flags &= ~FLAG_BUTTON; // Clears the button flag
  currentState = YELLOW; // Switch to YELLOW state
  Serial.println("Button pressed!!!!");
  Serial.println("[TRAFFIC LIGHT][YELLOW][PEDESTRIANS WAIT]");
  setRGBLedColor(255, 255, 0); // Set LED to Yellow
  seconds_counter = YELLOW_SECONDS; // Set countdown for YELLOW state
  flags &= ~FLAG_COUNTDOWN; // Clears the countdown flag
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
 * - Initializes the countdown timer for the RED state duration (RED_SECONDS = 5)
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
 * - After RED_SECONDS (5 seconds), the FSM transitions to COOLDOWN state
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
 * @see RED_SECONDS - Duration constant for RED state (5 seconds)
 * @see setRGBLedColor() - Hardware abstraction for LED control
 * @see tone() - Function to activate the buzzer
 */
void update_to_red(void) {
  currentState = RED; // Switch to RED state
  Serial.println("Countdown finished!!!!");
  Serial.println("[TRAFFIC LIGHT][RED][PEDESTRIANS CROSS]");
  setRGBLedColor(255, 0, 0); // Set LED to Red
  seconds_counter = RED_SECONDS; // Set countdown for RED state
  flags &= ~FLAG_COUNTDOWN; // Clears the countdown flag
  flags |= FLAG_BUZZER_ACTIVE; // Set buzzer active flag
  flags &= ~FLAG_BUTTON; // Clears the button flag to ignore new presses during RED state
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
 * - Initializes the countdown timer for the COOLDOWN state duration (COOLDOWN_SECONDS = 2)
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
 * - After COOLDOWN_SECONDS (2 seconds), the FSM transitions back to GREEN state
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
 * @see COOLDOWN_SECONDS - Duration constant for COOLDOWN state (2 seconds)
 * @see setRGBLedColor() - Hardware abstraction for LED control
 * @see noTone() - Function to disable the buzzer
 */
void update_to_cooldown(void) {
  currentState = COOLDOWN; // Switch to COOLDOWN state
  Serial.println("Countdown finished!!!!");
  Serial.println("[TRAFFIC LIGHT][COOLDOWN][PEDESTRIANS WAIT]");
  setRGBLedColor(0, 0, 255); // Set LED to Blue
  seconds_counter = COOLDOWN_SECONDS; // Set countdown for COOLDOWN state
  flags &= ~FLAG_COUNTDOWN; // Clears the countdown flag
  flags &= ~FLAG_BUZZER_ACTIVE; // Clear buzzer active flag
  flags &= ~FLAG_BUTTON; // Clears the button flag to ignore new presses during COOLDOWN state
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

/**
 * @brief Processes timer interrupt events for the traffic light FSM.
 * 
 * This function handles timer-related events that drive the traffic light state machine.
 * It checks if the main system timer has timed out and, if so, calls update_timer() to
 * process the timer tick and manage the state countdown mechanism.
 * 
 * @details
 * This function is part of the event processing layer of the traffic light FSM. It should
 * be called from the main application loop to check for and process timer events. The function:
 * - Checks if the FLAG_TIMER flag is set (indicating a timer interrupt occurred)
 * - Calls update_timer() to handle the timer tick, which:
 *   - Clears the FLAG_TIMER flag
 *   - Decrements the seconds_counter
 *   - Sets FLAG_COUNTDOWN when the countdown reaches zero
 *   - Increments the elapsed time counter
 * 
 * The timer is configured to interrupt every TIMER1_PERIOD_US (1 second), providing
 * periodic events for state duration management in the traffic light FSM.
 * 
 * @return void
 * 
 * @note This function must be called from the main application loop (not from an ISR).
 * @note This is a wrapper function that bridges the ISR-level timer interrupt to the
 *       main-loop FSM processing logic.
 * @note Should be called before traffic_light_fsm_fire() in the main loop to ensure
 *       timer events are processed before state transition logic runs.
 * 
 * @see timer_timeout() - Checks if the timer flag is set
 * @see update_timer() - Handles timer tick processing and flag clearing
 * @see FLAG_TIMER - Flag set by timer_isr() when interrupt occurs
 * @see timer_isr() - ISR that sets FLAG_TIMER
 * @see TIMER1_PERIOD_US - Timer interrupt period (1 second)
 * @see traffic_light_fsm_fire() - Main FSM logic function (called after this)
 * @see seconds_counter - Countdown timer managed by update_timer()
 * @see ellapsed_seconds - Total elapsed time tracked by update_timer()
 */
void basic_timer_fsm_fire(void) {
	// FSM timer event handling
	if (timer_timeout()) {
		update_timer(); // Clear timer flag
	}
}

/**
 * @brief Initializes and configures the traffic light system hardware and FSM state.
 * 
 * This function sets up all hardware components required for the traffic light operation:
 * - Configures the RGB LED with green as the initial state (vehicles have right of way)
 * - Initializes the pedestrian button with internal pull-up and rising-edge interrupt detection
 * - Sets the traffic light FSM to its initial GREEN state
 * 
 * @details
 * This function performs the following initialization steps:
 * 1. Logs initialization message to serial port for debugging
 * 2. Configures RGB LED pins as outputs and sets initial color to green (0, 255, 0)
 * 3. Configures the button pin with internal pull-up resistor (active-low logic)
 * 4. Attaches the button_isr() interrupt handler to detect rising-edge button presses
 * 5. Transitions the FSM to GREEN state by calling update_to_green()
 * 
 * The traffic light is ready to operate after this function returns:
 * - Vehicles can proceed (green light)
 * - The button interrupt is armed and ready to detect pedestrian presses
 * - The timer-driven FSM can begin processing events
 * 
 * @return void
 * 
 * @note This function must be called during system initialization, typically from the main setup() function.
 * @note Must be called after basic_timer_init() to ensure the timer infrastructure is ready.
 * @note Must be called after Serial is initialized to ensure debug output works correctly.
 * @note The button uses RISING-edge detection because it has an internal pull-up (inactive state = HIGH).
 * @note After this function returns, call basic_timer_fsm_fire() and traffic_light_fsm_fire() in the main loop.
 * 
 * @see configInitRGBLed() - Initializes RGB LED pins and sets to green
 * @see update_to_green() - Sets FSM to GREEN state and initializes LED
 * @see button_isr() - Interrupt handler attached to BUTTON_PIN
 * @see basic_timer_init() - Should be called before this function
 * @see traffic_light_fsm_fire() - Main FSM logic (called after this from main loop)
 * @see BUTTON_PIN - Button input pin macro
 * @see DEBOUNCE_TIME - Debounce delay for button (managed by button_isr)
 */
void traffic_light_init(void) {
	Serial.println("\n[main.cpp] Initializing traffic light...");
	
	// RGB LED (Green at start)
	configInitRGBLed();

	// Button: internal pull-up, RISING-edge interrupt
	pinMode(BUTTON_PIN, INPUT_PULLUP);
	attachInterrupt(BUTTON_PIN, button_isr, RISING);

	update_to_green(); // Initial state
}

/**
 * @brief Initializes and configures the STM32 timer hardware for traffic light operation.
 * 
 * This function sets up Timer 1 (TIM1) with a periodic interrupt that fires every TIMER1_PERIOD_US microseconds
 * (configured for 1 second intervals). The timer generates interrupts that drive the traffic light state machine
 * timing mechanism.
 * 
 * @details
 * This function performs the following operations:
 * - Prints an initialization status message to the serial port for debugging
 * - Attaches the timer_isr() interrupt service routine to the timer's interrupt event
 * - Configures the timer to generate periodic interrupts at TIMER1_PERIOD_US interval
 * 
 * The timer provides the heartbeat for the traffic light FSM by generating regular tick events that:
 * - Decrement the state countdown timers (seconds_counter)
 * - Trigger state transitions when countdowns complete
 * - Track total elapsed time since program start (ellapsed_seconds)
 * 
 * @return void
 * 
 * @note This function must be called during system initialization, typically from the main setup() function.
 * @note Must be called after the serial port is initialized to ensure debug output works correctly.
 * @note The timer is configured to use TIM1 (Timer 1) on the STM32 microcontroller.
 * @note The STM32Timer object must be instantiated globally before calling this function.
 * @note Should be called before traffic_light_init() to ensure timing infrastructure is ready.
 * 
 * @see timer - Global STM32Timer object for TIM1
 * @see TIMER1_PERIOD_US - Timer period in microseconds (1 second = 1,000,000 µs)
 * @see timer_isr() - Interrupt service routine called on each timer tick
 * @see FLAG_TIMER - Flag set by timer_isr() to signal timer event to main loop
 * @see update_timer() - Main loop function that processes timer events
 * @see basic_timer_fsm_fire() - Wrapper that calls update_timer() from main loop
 * @see traffic_light_init() - Should be called after this function
 */
void basic_timer_init(void) {
	Serial.println("\n[main.cpp] Initializing basic timer...");

	// Timer
	timer.attachInterruptInterval(TIMER1_PERIOD_US, timer_isr); // Timer period: 1 second
}

/**
 * @brief Main finite state machine engine for traffic light control.
 * 
 * This function implements the core traffic light state machine logic. It processes
 * the current state and evaluates conditions to determine if a state transition should occur.
 * This function should be called continuously from the main application loop.
 * 
 * @details
 * The traffic light FSM implements a 4-state cycle:
 * 
 * - GREEN: Vehicles have right of way. Waits for pedestrian button press.
 *   Transition: GREEN -> YELLOW when button_pressed() returns true
 * 
 * - YELLOW: Warning state. Vehicles prepare to stop, pedestrians wait.
 *   Duration: YELLOW_SECONDS (3 seconds)
 *   Transition: YELLOW -> RED when countdown_finished() returns true
 * 
 * - RED: Pedestrian crossing phase. Vehicles stopped, pedestrians cross.
 *   Duration: RED_SECONDS (5 seconds)
 *   Buzzer active during this state to alert pedestrians
 *   Transition: RED -> COOLDOWN when countdown_finished() returns true
 * 
 * - COOLDOWN: Transition/cool-down period. Both vehicles and pedestrians wait.
 *   Duration: COOLDOWN_SECONDS (2 seconds)
 *   Buzzer disabled during this state
 *   Transition: COOLDOWN -> GREEN when countdown_finished() returns true
 * 
 * State transitions are triggered by external events:
 * - Button press from pedestrian (via button_isr())
 * - Countdown timer completion (via update_timer() and FLAG_COUNTDOWN)
 * 
 * @return void
 * 
 * @note This function must be called from the main application loop continuously.
 * @note This function should NOT be called from an ISR (it is a main-loop function).
 * @note Depends on global state variables: currentState, flags
 * @note The actual state transition logic is implemented in update_to_*() functions:
 *       - update_to_yellow()
 *       - update_to_red()
 *       - update_to_cooldown()
 *       - update_to_green()
 * 
 * @see currentState - Global variable holding current FSM state
 * @see button_pressed() - Checks for pedestrian button press
 * @see countdown_finished() - Checks if state countdown has finished
 * @see update_to_yellow() - GREEN -> YELLOW transition
 * @see update_to_red() - YELLOW -> RED transition
 * @see update_to_cooldown() - RED -> COOLDOWN transition
 * @see update_to_green() - COOLDOWN -> GREEN transition
 * @see traffic_light_init() - Initializes the FSM
 * @see basic_timer_fsm_fire() - Timer event handler (processes timing events)
 */
void traffic_light_fsm_fire(void) {
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
}
