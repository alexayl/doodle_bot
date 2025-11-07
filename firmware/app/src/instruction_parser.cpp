#include "instruction_parser.h"
#include <zephyr/sys/printk.h>
#include <errno.h>

//-------------------------
// INSTRUCTION PARSING
// ------------------------

// TODO: Make the language supported more modular
// Don't hardcode expected character and numbers
int InstructionParser::parseLine(const char* line, GCodeCmd& outCmd) {
    outCmd = {}; // reset
    const char* ptr = line;

    // find and validate packet id
    uint8_t packet_id = (uint8_t)*ptr;
    if (packet_id == expected_packet_id) {
        outCmd.packet_id = packet_id;
        expected_packet_id++; // increment if successful
    } else {
        printk("Packet ID mismatch: expected %d, got %d\n", expected_packet_id, packet_id);
        return -EINVAL;
    }

    ptr++;

    // validate command code
    outCmd.code = toupper(*ptr++);
    if (outCmd.code != 'G' && outCmd.code != 'M') {
        printk("Invalid command code: %c\n", outCmd.code);
        return -EINVAL;
    }

    // read the numeric command number and advance pointer
    outCmd.number = atoi(ptr);
    while (isdigit(*ptr)) ptr++;
    outCmd.argc = 0;

    // parse arguments
    while (*ptr && outCmd.argc < MAX_ARGS) {
        while (isspace(*ptr)) ptr++;
        if (!*ptr) break;
        if (!isalpha(*ptr)) { ptr++; continue; }

        Arg& arg = outCmd.args[outCmd.argc++];
        arg.letter = toupper(*ptr++);
        arg.value = (float)atof(ptr);
        // Skip past the number
        if (*ptr == '-' || *ptr == '+') ptr++;
        while (isdigit(*ptr) || *ptr == '.') ptr++;
    }

    #ifdef DEBUG_INSTRUCTION_PARSER
        // TODO: Debug print the parsed command

    #endif
    
    // validate that the command is supported
    return isSupported(outCmd);
}

int InstructionParser::isSupported(const GCodeCmd& cmd) {
    if (cmd.code == 'G' && (cmd.number == 91 || cmd.number == 1))
        return 0;
    if (cmd.code == 'M' && cmd.number == 280)
        return 0;
    return -EINVAL;
}