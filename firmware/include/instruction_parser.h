#pragma once

#include <stddef.h>
#include <stdint.h>
#include <string.h>
#include <ctype.h>
#include <stdlib.h>
#include "config.h"

//-------------------------
// INSTRUCTION PARSING
// ------------------------

class InstructionParser {
public:
    uint8_t expected_packet_id = 0;

    static constexpr uint8_t MAX_ARGS = 8;

    struct Arg {
        char letter;   // e.g., 'X', 'Y', 'Z', 'F', etc.
        float value;
    };

    struct GCodeCmd {
        uint8_t packet_id; // Packet ID of this command
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


    /**
     * @brief Parse a single G-code line into a GCodeCmd structure
     * @param line The input G-code line as a C-string
     * @param outCmd The output GCodeCmd structure to populate
     * @return 0 if successful parse, and -EINVAL otherwise
     */
    int parseLine(const char* line, GCodeCmd& outCmd);

    /**
     * @brief Check if the parsed command matches a known instruction
     * @param cmd The GCode command to check
     * @return 0 if supported, -EINVAL otherwise
     */
    static int isSupported(const GCodeCmd& cmd);

    /**
     * @brief Reset parser state (e.g., on BLE disconnect)
     */
    void reset() {
        expected_packet_id = 0;
    }
};