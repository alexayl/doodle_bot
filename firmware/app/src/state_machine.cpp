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

void DoodleBotState::stateIdle(Event e) {
    if (e == Event::CmdErase) {
        transition(&DoodleBotState::stateErase, Command::StartErase, State::Erase);
    }
}


void DoodleBotState::stateErase(Event e) {
    if (e == Event::Done) {
        transition(&DoodleBotState::stateIdle, Command::DeviceSleep, State::Idle);        
    }
}