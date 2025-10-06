#include "state_machine.h"
#include <stdio.h>


DoodleBotState::DoodleBotState() :
    currentState(&DoodleBotState::stateIdle),
    currentEnum(State::Idle),
    pendingCommand(Command::DeviceSleep) {}


void DoodleBotState::disbatch(Event e) {
    return (this->*currentState)(e);
}


Command DoodleBotState::lastCommand() const {
    return pendingCommand;
}


State DoodleBotState::currentStateEnum() const {
    return currentEnum;
}


void DoodleBotState::transition(StateHandler next, Command cmd, State s) {
    currentState = next;
    currentEnum = s;
    pendingCommand = cmd;
}

// --------------------------
// state function definitions
// --------------------------

void DoodleBotState::stateInit(Event e) {
    if (e == Event::InitDone) {
        transition(&DoodleBotState::stateIdle, Command::DeviceSleep, State::Idle);
    }
}

void DoodleBotState::stateIdle(Event e) {
    if (e == Event::EraseAcknowledge) {
        transition(&DoodleBotState::stateErase, Command::StartErase, State::Erase);
    } else if (e == Event::DrawRequest) {
        transition(&DoodleBotState::stateDraw, Command::StartDraw, State::Draw);
    } else if (e == Event::PauseRequest) {
        transition(&DoodleBotState::statePause, Command::Pause, State::Pause);
    } else if (e == Event::ErrorDetected) {
        transition(&DoodleBotState::stateError, Command::HandleError, State::Error);
    }
}


void DoodleBotState::stateErasing(Event e) {
    if (e == Event::TransformationDone) {
        transition(&DoodleBotState::stateIdle, Command::DeviceSleep, State::Idle);        
    }
}