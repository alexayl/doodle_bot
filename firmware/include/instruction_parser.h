#pragma once

#include <stddef.h>
#include <stdint.h>
#include <string.h>
#include <ctype.h>
#include <stdlib.h>

//-------------------------
// INSTRUCTION PARSING
// ------------------------

class InstructionParser {
public:
    static constexpr uint8_t MAX_ARGS = 8;

    struct Arg {
        char letter;   // e.g., 'X', 'Y', 'Z', 'F', etc.
        float value;
    };

    struct GCodeCmd {
        char code;      // 'G' or 'M'
        int number;     // e.g. 91, 0, 280
        Arg args[MAX_ARGS];
        uint8_t argc;
    };

    enum Instruction : uint8_t {
        G91 = 0,    // relative positioning
        G0,         // rapid move
        G1,         // idk what tom is using
        M280        // move servos
    };

    InstructionParser() = default;

    /** Parse a single G-code line (e.g. "G0 X10 Y-5 F1000") */
    bool parseLine(const char* line, GCodeCmd& outCmd);

    /** Utility: check if parsed command matches a known instruction */
    static bool isSupported(const GCodeCmd& cmd);
};