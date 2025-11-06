#pragma once

#include <stdint.h>

// ------------------------------------------------
// NOTE: State machine graph in docs/state_machine!
// ------------------------------------------------

/**
 * @brief All possible states of DoodleBot.
 * 
 * Each value corresponds to a state handler function in the state machine. They are specified here for debugging purposes.
 */
enum class State {
    Init = 0,
    Actuate,
    Idle,
    Error
};

/**
 * @brief External events the state machine can process.
 */
enum class Event {
    HardwareReset = 0,
    InitSuccess,
    ReceiveVldCmd,
    ReceiveInvldCmd,
    EmptyCmdQueue,
    BadActuation,
    InitError
};

/**
 * @brief Commands (outputs) produced by the state machine.
 */
enum class Command {
    None = 0,      ///< No command to execute
    Initialize,    ///< Initialize hardware/system
    ProcessCommand,///< Process received command
    HandleError,   ///< Handle error condition
    Sleep          ///< Put device to sleep/idle
};

/**
 * @brief DoodleBot's high-level FSM.
 * 
 * Simple state machine with enum-based states for easy command dispatching.
 * Inputs: Events
 * Outputs: Commands
 */
class DoodleBotState {
    public:
        /**
         * @brief Construct new DoodleBotState in Init state.
         */
        DoodleBotState();

        /**
         * @brief Process a new event and transition states.
         * 
         * @param e Event to process.
         * @return Command to be executed by caller.
         */
        Command processEvent(Event e);

        /**
         * @brief Get current state.
         * 
         * @return State current state.
         */
        State getCurrentState() const;

        /**
         * @brief Check if state machine is in error state.
         * 
         * @return true if in error state, false otherwise.
         */
        bool isInError() const;

    private:
        State currentState;         ///< Current state
        
        /**
         * @brief Transition to new state and return associated command.
         * 
         * @param newState State to transition to.
         * @return Command associated with the new state.
         */
        Command transitionTo(State newState);
};
