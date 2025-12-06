#include <zephyr/ztest.h>
#include <string.h>
#include <math.h>
#include "instruction_parser.h"


struct Testcase {
    const char* input_gcode;
    int input_pid;

    int expected_ret;
    uint8_t expected_pid;
    InstructionParser::GCodeCmd expected_cmd;
};

Testcase testcases[] = {
    // Valid cases
    {"G0 X10 Y20\n", 0, 0, 0, {'G', 0,   { {'X', 10.0f}, {'Y', 20.0f} }, 2 }},
    {"M280 S90\n",   0, 0, 0, {'M', 280, { {'S', 90.0f} }, 1 }},
    {"G91\n",        0, 0, 0, {'G', 91,  {}, 0 }},
    {"G0\n",         0, 0, 0, {'G', 0,   {}, 0 }},
    {"M280 S0\n",    0, 0, 0, {'M', 280, { {'S', 0.0f} }, 1 }},

    // Valid with extra whitespace
    {"  G0   X5   Y6  \n", 0, 0, 0, {'G', 0, { {'X', 5.0f}, {'Y', 6.0f} }, 2 }},
    
    // Valid with negative and decimal values
    {"G0 X-1.5 Y2.25\n", 0, 0, 0, {'G', 0, { {'X', -1.5f}, {'Y', 2.25f} }, 2 }},
    
    // Valid with multiple arguments
    {"G0 X10 Y20 Z30 F100\n", 0, 0, 0, {'G', 0, { {'X', 10.0f}, {'Y', 20.0f}, {'Z', 30.0f}, {'F', 100.0f} }, 4 }},
    
    // Valid without spaces between command and arguments
    {"G0X10Y20\n", 0, 0, 0, {'G', 0, { {'X', 10.0f}, {'Y', 20.0f} }, 2 }},
    
    // Invalid cases - wrong command code (not G or M)
    {"A123\n", 0, -22, 0, {'A', 123, {}, 0 }},
    {"B0 X10\n", 0, -22, 0, {'B', 0, {}, 0 }},
    
    // Invalid cases - unsupported G/M commands
    {"G2 X10\n", 0, -22, 0, {'G', 2, { {'X', 10.0f} }, 1 }},
    {"M100\n", 0, -22, 0, {'M', 100, {}, 0 }},
    {"G99\n", 0, -22, 0, {'G', 99, {}, 0 }},
    
    // Valid with missing argument values (defaults to 0)
    {"G0 X10 Y\n", 0, 0, 0, {'G', 0, { {'X', 10.0f}, {'Y', 0.0f} }, 2 }},
    
    // Valid with non-numeric values (atof returns 0)
    {"G0 Xabc\n", 0, 0, 0, {'G', 0, { {'X', 0.0f} }, 1 }},
    
    // Valid with lowercase letters (should be uppercased)
    {"g0 x5 y6\n", 0, 0, 0, {'G', 0, { {'X', 5.0f}, {'Y', 6.0f} }, 2 }},
    {"m280 s45\n", 0, 0, 0, {'M', 280, { {'S', 45.0f} }, 1 }},
};
const size_t num_testcases = sizeof(testcases) / sizeof(testcases[0]);


ZTEST(instruction_parser_tests, sanity_check) {
    int a = 2;
    int b = 3;
    zassert_equal(a + b, 5, "Addition failed");
}

ZTEST(instruction_parser_tests, parse_testcases) {

    for (size_t t = 0; t < num_testcases; ++t) {
        const Testcase* tc = &testcases[t];

        InstructionParser instruction_parser;
        InstructionParser::GCodeCmd parsed_cmd;

        char input[120];
        snprintf(input, sizeof(input), "%c%s", tc->input_pid, tc->input_gcode);
        int ret = instruction_parser.parseLine(input, parsed_cmd);

        // return value
        zassert_equal(ret, tc->expected_ret, "return value diff: testcase %s", tc->input_gcode);

        if (tc->expected_ret != 0) continue;
        
        // code
        zassert_equal(parsed_cmd.code, tc->expected_cmd.code, "expected code: %d | actual ret: %d", tc->expected_cmd.code, parsed_cmd.code);

        // command number
        zassert_equal(parsed_cmd.number, tc->expected_cmd.number, "expected number: %d | actual ret: %d", tc->expected_cmd.number, parsed_cmd.number);

        // arguments
        for (uint8_t i = 0; i < tc->expected_cmd.argc; ++i) {
            zassert_equal(parsed_cmd.args[i].letter, tc->expected_cmd.args[i].letter, "Arg letter mismatch for input: %s", tc->input_gcode);
            zassert_equal(parsed_cmd.args[i].value, tc->expected_cmd.args[i].value, "Arg value mismatch for input: %s", tc->input_gcode);
        }
    }
}

ZTEST_SUITE(instruction_parser_tests, NULL, NULL, NULL, NULL, NULL);
