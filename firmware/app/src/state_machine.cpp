#include "state_machine.h"

DoodleBotState::DoodleBotState() : currentState(State::Init) {}

Command DoodleBotState::processEvent(Event e) {
    switch (currentState) {
        case State::Init:
            switch (e) {
                case Event::InitSuccess:
                    return transitionTo(State::Idle);
                case Event::InitError:
                    return transitionTo(State::Error);
                default:
                    return Command::None;
            }

        case State::Idle:
            switch (e) {
                case Event::ReceiveVldCmd:
                    return transitionTo(State::Actuate);
                case Event::ReceiveInvldCmd:
                    return transitionTo(State::Error);
                default:
                    return Command::None;
            }

        case State::Actuate:
            switch (e) {
                case Event::EmptyCmdQueue:
                    return transitionTo(State::Idle);
                case Event::BadActuation:
                    return transitionTo(State::Error);
                default:
                    return Command::None;
            }

        case State::Error:
            switch (e) {
                case Event::HardwareReset:
                    return transitionTo(State::Init);
                default:
                    return Command::None;
            }
    }
    
    return Command::None;
}

State DoodleBotState::getCurrentState() const {
    return currentState;
}

bool DoodleBotState::isInError() const {
    return currentState == State::Error;
}

Command DoodleBotState::transitionTo(State newState) {
    currentState = newState;
    
    // Return the command associated with entering this state
    switch (newState) {
        case State::Init:
            return Command::Initialize;
        case State::Idle:
            return Command::Sleep;
        case State::Actuate:
            return Command::ProcessCommand;
        case State::Error:
            return Command::HandleError;
    }
    
    return Command::None;
}
