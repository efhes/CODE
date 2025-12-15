#include "buzzer_lib.h"

BuzzerStateEnum buzzerState = WAITING;
STM32Timer timer_buzzer(TIM2); // Using TIM2

void buzzer_init(void) {
  Serial.println("\n[main.cpp] Initializing buzzer...");

  // Buzzer
  pinMode(BUZZER_PIN, OUTPUT);
  noTone(BUZZER_PIN);

  // Initial buzzer state
  buzzerState = WAITING;
}

/**
 * @brief Interrupt Service Routine (ISR) for the buzzer timer.
 * 
 * This function is the ISR that is called when the buzzer timer expires. It is configured
 * to be triggered at regular 500ms intervals to control the buzzer beeping pattern.
 * 
 * The ISR sets the FLAG_BUZZER_TIMER flag in the global flags variable to signal that
 * the timer has expired. This flag is then checked in the main application loop by the
 * buzzer_timer_timeout() function to manage state transitions in the buzzer FSM.
 * 
 * @return void
 * 
 * @details
 * This function is called automatically by the hardware timer interrupt when the 500ms
 * interval elapses. It performs a minimal operation (flag setting) to keep ISR execution
 * time short. The actual buzzer state transition logic is handled in the main loop, not
 * in the ISR itself.
 * 
 * The ISR is part of the buzzer finite state machine (FSM) timing mechanism:
 * - Each time the ISR fires, it sets FLAG_BUZZER_TIMER to signal a timer event
 * - The main loop checks this flag with buzzer_timer_timeout() function
 * - When transitioning states (ON to OFF or OFF to ON), the flag is cleared
 * - This creates the 500ms ON/OFF beeping pattern
 * 
 * @note This is an ISR (Interrupt Service Routine) - it should be kept as short and fast as possible.
 * @note This function is registered with the buzzer timer during setup().
 * @note Do NOT call this function directly from application code - it is called by the timer interrupt.
 * @note The buzzer timer interval (500ms) is configured in setup().
 * 
 * @see FLAG_BUZZER_TIMER - Flag that is set by this ISR
 * @see buzzer_timer_timeout() - Function that checks if this flag is set
 * @see update_to_buzzer_on() - Clears this flag when transitioning to ON
 * @see update_to_buzzer_off() - Clears this flag when transitioning to OFF
 * @see timer_buzzer - The 500ms interval timer that triggers this ISR
 */
void timer_buzzer_isr(void) {
  // To be completed by the student
  
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
 * @details The buzzer timer is an 500ms interval timer (configured in setup()) that controls
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
  // To be completed by the student

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
 * 2. The buzzer timer has timed out (500ms interval), indicating it's time to turn OFF
 * 
 * The OFF state represents a silent period in the buzzer beeping cycle where:
 * - The buzzer sound is deactivated
 * - The timer is restarted to measure the OFF duration (500ms)
 * - After the timer expires, the FSM transitions back to ON state to resume beeping
 * - This creates an alternating ON/OFF pattern (on for 500ms, off for 500ms, etc.)
 * 
 * @return void
 * 
 * @note This function must be called from the main application loop (not from an ISR).
 * @note The OFF state duration is controlled by the 500ms buzzer timer interval.
 * @note This transition occurs only when the traffic light is in RED state (FLAG_BUZZER_ACTIVE is set).
 * 
 * @see update_to_buzzer_on() - The complementary ON state transition
 * @see update_to_buzzer_waiting() - Transitions buzzer FSM back to idle state
 * @see buzzer_timer_timeout() - Condition that triggers this transition
 * @see FLAG_BUZZER_TIMER - Flag cleared when timer is restarted
 * @see timer_buzzer - The 500ms interval timer controlling buzzer timing
 * @see noTone() - Function to deactivate the buzzer
 */
void update_to_buzzer_off(void) {
  buzzerState = OFF;
  noTone(BUZZER_PIN); // Deactivate buzzer
  Serial.println("[BUZZER][OFF]!!!!");
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
 * 2. The buzzer is in OFF state and the buzzer timer has timed out (500ms interval), indicating it's time to turn ON
 * 
 * The ON state represents an active beeping period in the buzzer cycle where:
 * - The buzzer sound is activated at 1kHz frequency
 * - The timer is restarted to measure the ON duration (500ms)
 * - After the timer expires, the FSM transitions to OFF state to create a silent period
 * - This creates an alternating ON/OFF pattern (on for 500ms, off for 500ms, etc.)
 * 
 * @return void
 * 
 * @note This function must be called from the main application loop (not from an ISR).
 * @note The ON state duration is controlled by the 500ms buzzer timer interval.
 * @note This transition occurs only when the traffic light is in RED state (FLAG_BUZZER_ACTIVE is set).
 * 
 * @see update_to_buzzer_off() - The complementary OFF state transition
 * @see update_to_buzzer_waiting() - Transitions buzzer FSM back to idle state
 * @see buzzer_timer_timeout() - Condition that triggers transition from OFF to ON
 * @see transition_to_buzzer_active() - Condition that triggers transition from WAITING to ON
 * @see FLAG_BUZZER_TIMER - Flag cleared when timer is restarted
 * @see timer_buzzer - The 500ms interval timer controlling buzzer timing
 * @see tone() - Function to activate the buzzer at specified frequency
 */
void update_to_buzzer_on(void) {
  buzzerState = ON;
  tone(BUZZER_PIN, 1000); // Activate buzzer at 1kHz
  Serial.println("[BUZZER][ON]!!!!");
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
 * @see timer_buzzer - The 500ms interval timer that is disabled in this function
 * @see noTone() - Function to deactivate the buzzer
 */
void update_to_buzzer_waiting(void) {
  // To be completed by the student
}

void buzzer_fsm_fire(void) {
  switch (buzzerState) {
    case WAITING: // Buzzer WAITING state
      if (transition_to_buzzer_active()) { // Check if buzzer activation condition is met
        Serial.println("[BUZZER][ACTIVE]!!!!");
        // To be completed by the student

      }
      break;

    case ON: // Buzzer ON state
      // To be completed by the student

      break;

    case OFF: // Buzzer OFF state
      // To be completed by the student

      break;

    default:
      Serial.println("Error: Unknown buzzer state!");
      break;
  }
}