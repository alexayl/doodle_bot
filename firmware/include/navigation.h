#pragma once

#include "instruction_parser.h"
#include "peripheral_wrappers.h"

#define DEBUG_NAV

/* Functions */

class InstructionHandler {
public:
    virtual int consumeInstruction() = 0;
};

class Navigator : InstructionHandler {
public:
    int consumeInstruction() override;
};

class PeripheralMover : InstructionHandler {
public:
    int consumeInstruction() override;
protected:
    Peripheral peripheral_;
};

void nav_thread(void *, void *, void *);
