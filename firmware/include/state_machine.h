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
enum class State : uint8_t {
    Init,
    Idle,
    Draw,
    Erase,
    Pause,
    Error
};

/**
 * @brief External events the state machine can process.
 */
enum class Event : uint8_t {
    CmdDraw,
    CmdErase,
    CmdPause,
    CmdResume,
    Fault,
    Reset,
    Done,
    Ready
};

/**
 * @brief Commands (outputs) produced by the state machine.
 */
enum class Command : uint8_t {
    None,
    StartDraw,
    StartErase,
    DeviceSleep,
    HaltMotion,
    ResumeMotion
};

/**
 * @brief DoodleBot's high-level FSM.
 * 
 * Inputs: Events
 * Outputs: Commands
 */
class DoodleBotState {
    public:
        /**
         * @brief Type alias for a pointer to a state handler function.
         */
        using StateHandler = void (DoodleBotState::*)(Event);

        /**
         * @brief Construct new DoodleBotState in IDLE.
         */
        DoodleBotState();

        /**
         * @brief Process a new event.
         * 
         * @param e Event to process.
         */
        void disbatch(Event e);

        /**
         * @brief Fetch last issued command.
         * 
         * @return Command to be executed by caller (could be none).
         */
        Command lastCommand() const;

        /**
         * @brief fetch current state.
         * 
         * For logging and debugging.
         * 
         * @return State current state.
         */
        State currentStateEnum() const;


    private:
        StateHandler currentState;  ///< Pointer to current state's member function
        State currentEnum;          ///< Current state's enum value
        Command pendingCommand;     ///< Most recent command sent

        /**
         * @brief set data members to the next state's values based on state handler.
         */
        void transition(StateHandler next, Command cmd, State s);

        // --- state handlers ---
        void stateInit(Event e);
        void stateIdle(Event e);
        void stateDraw(Event e);
        void stateErase(Event e);
        void statePause(Event e);
        void stateError(Event e);

};
