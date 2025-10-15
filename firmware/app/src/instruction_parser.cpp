#include "instruction_parser.h"

//-------------------------
// INSTRUCTION PARSING
// ------------------------

bool InstructionParser::parseLine(const char* line, GCodeCmd& outCmd) {
    outCmd = {}; // reset
    const char* ptr = line;

    // skip whitespace
    while (isspace(*ptr)) ptr++;

    // first char must be G or M
    outCmd.code = toupper(*ptr++);
    if (outCmd.code != 'G' && outCmd.code != 'M')
        return false;

    // read the numeric command number (e.g. 0, 91, 280)
    outCmd.number = atoi(ptr);

    // skip numeric part
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
    return true;
}

bool InstructionParser::isSupported(const GCodeCmd& cmd) {
    if (cmd.code == 'G' && (cmd.number == 91 || cmd.number == 0))
        return true;
    if (cmd.code == 'M' && cmd.number == 280)
        return true;
    return false;
}