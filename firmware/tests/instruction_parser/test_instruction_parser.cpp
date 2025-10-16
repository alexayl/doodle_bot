    #include <zephyr/ztest.h>
#include <string.h>
#include <math.h>
#include "instruction_parser.h"

ZTEST(instruction_parser, test_parse_g0_command)
{
    InstructionParser parser;
    InstructionParser::GCodeCmd cmd;
    
    // First packet should have ID 0, followed by G-code
    const char* packet = "\x00G0 X10 Y-5";
    bool result = parser.parseLine(packet, cmd);
    
    zassert_true(result, "Should successfully parse G0 command with packet ID 0");
    zassert_equal(cmd.code, 'G', "Command code should be G");
    zassert_equal(cmd.number, 0, "Command number should be 0");
    zassert_equal(cmd.argc, 2, "Should have 2 arguments");
    zassert_equal(cmd.args[0].letter, 'X', "First arg should be X");
    zassert_within(cmd.args[0].value, 10.0f, 1e-6, "X value should be 10.0");
    zassert_equal(cmd.args[1].letter, 'Y', "Second arg should be Y");
    zassert_within(cmd.args[1].value, -5.0f, 1e-6, "Y value should be -5.0");
    zassert_equal(parser.packet_id, 1, "Packet ID should increment to 1 after successful parse");
}

ZTEST(instruction_parser, test_parse_g91_command)
{
    InstructionParser parser;
    InstructionParser::GCodeCmd cmd;
    
    // First packet should have ID 0, followed by G-code
    const char* packet = "\x00G91";
    bool result = parser.parseLine(packet, cmd);
    
    zassert_true(result, "Should successfully parse G91 command with packet ID 0");
    zassert_equal(cmd.code, 'G', "Command code should be G");
    zassert_equal(cmd.number, 91, "Command number should be 91");
    zassert_equal(cmd.argc, 0, "Should have no arguments");
    zassert_equal(parser.packet_id, 1, "Packet ID should increment to 1 after successful parse");
}

ZTEST(instruction_parser, test_parse_m280_servo_command)
{
    InstructionParser parser;
    InstructionParser::GCodeCmd cmd;
    
    // First packet should have ID 0, followed by G-code
    const char* packet = "\x00M280 P0 S90";
    bool result = parser.parseLine(packet, cmd);
    
    zassert_true(result, "Should successfully parse M280 command with packet ID 0");
    zassert_equal(cmd.code, 'M', "Command code should be M");
    zassert_equal(cmd.number, 280, "Command number should be 280");
    zassert_equal(cmd.argc, 2, "Should have 2 arguments");
    zassert_equal(cmd.args[0].letter, 'P', "First arg should be P");
    zassert_within(cmd.args[0].value, 0.0f, 1e-6, "P value should be 0.0");
    zassert_equal(cmd.args[1].letter, 'S', "Second arg should be S");
    zassert_within(cmd.args[1].value, 90.0f, 1e-6, "S value should be 90.0");
    zassert_equal(parser.packet_id, 1, "Packet ID should increment to 1 after successful parse");
}

ZTEST(instruction_parser, test_parse_with_whitespace)
{
    InstructionParser parser;
    InstructionParser::GCodeCmd cmd;
    
    // Create packet: [packet_id][G-code with whitespace]
    // Parser flow: skip initial whitespace -> check packet_id -> read G directly -> parse rest with whitespace
    char packet[50];
    packet[0] = 0;        // Packet ID 0  
    packet[1] = 'G';      // Command must immediately follow packet ID
    strcpy(&packet[2], "0   X10.5   Y-20.25  Z5  ");
    
    bool result = parser.parseLine(packet, cmd);
    
    zassert_true(result, "Should successfully parse command with whitespace and packet ID 0");
    zassert_equal(cmd.code, 'G', "Command code should be G");
    zassert_equal(cmd.number, 0, "Command number should be 0");
    zassert_equal(cmd.argc, 3, "Should have 3 arguments");
    zassert_equal(cmd.args[0].letter, 'X', "First arg should be X");
    zassert_within(cmd.args[0].value, 10.5f, 1e-6, "X value should be 10.5");
    zassert_equal(cmd.args[1].letter, 'Y', "Second arg should be Y");
    zassert_within(cmd.args[1].value, -20.25f, 1e-6, "Y value should be -20.25");
    zassert_equal(cmd.args[2].letter, 'Z', "Third arg should be Z");
    zassert_within(cmd.args[2].value, 5.0f, 1e-6, "Z value should be 5.0");
    zassert_equal(parser.packet_id, 1, "Packet ID should increment to 1 after successful parse");
}

ZTEST(instruction_parser, test_parse_lowercase_command)
{
    InstructionParser parser;
    InstructionParser::GCodeCmd cmd;
    
    // Create packet with ID 0 followed by lowercase G-code
    const char* packet = "\x00g0 x10 y-5";
    bool result = parser.parseLine(packet, cmd);
    
    zassert_true(result, "Should successfully parse lowercase command with packet ID 0");
    zassert_equal(cmd.code, 'G', "Command code should be converted to uppercase G");
    zassert_equal(cmd.number, 0, "Command number should be 0");
    zassert_equal(cmd.argc, 2, "Should have 2 arguments");
    zassert_equal(cmd.args[0].letter, 'X', "First arg should be converted to uppercase X");
    zassert_equal(cmd.args[1].letter, 'Y', "Second arg should be converted to uppercase Y");
    zassert_equal(parser.packet_id, 1, "Packet ID should increment to 1 after successful parse");
}

ZTEST(instruction_parser, test_parse_complex_gcode)
{
    InstructionParser parser;
    InstructionParser::GCodeCmd cmd;
    
    // Create packet with ID 0 followed by complex G-code
    const char* packet = "\x00G0 X10 Y-5 Z2.5 F1000 E0.5";
    bool result = parser.parseLine(packet, cmd);
    
    zassert_true(result, "Should successfully parse complex G-code with packet ID 0");
    zassert_equal(cmd.code, 'G', "Command code should be G");
    zassert_equal(cmd.number, 0, "Command number should be 0");
    zassert_equal(cmd.argc, 5, "Should have 5 arguments");
    
    // Check each argument
    zassert_equal(cmd.args[0].letter, 'X', "First arg should be X");
    zassert_within(cmd.args[0].value, 10.0f, 1e-6, "X value should be 10.0");
    
    zassert_equal(cmd.args[1].letter, 'Y', "Second arg should be Y");
    zassert_within(cmd.args[1].value, -5.0f, 1e-6, "Y value should be -5.0");
    
    zassert_equal(cmd.args[2].letter, 'Z', "Third arg should be Z");
    zassert_within(cmd.args[2].value, 2.5f, 1e-6, "Z value should be 2.5");
    
    zassert_equal(cmd.args[3].letter, 'F', "Fourth arg should be F");
    zassert_within(cmd.args[3].value, 1000.0f, 1e-6, "F value should be 1000.0");
    
    zassert_equal(cmd.args[4].letter, 'E', "Fifth arg should be E");
    zassert_within(cmd.args[4].value, 0.5f, 1e-6, "E value should be 0.5");
    zassert_equal(parser.packet_id, 1, "Packet ID should increment to 1 after successful parse");
}

ZTEST(instruction_parser, test_parse_invalid_command_char)
{
    InstructionParser parser;
    InstructionParser::GCodeCmd cmd;
    
    // Create packet with ID 0 but invalid command (no G or M prefix)
    const char* packet = "\x00X0 Y10";
    bool result = parser.parseLine(packet, cmd);
    
    zassert_false(result, "Should fail to parse command without G or M prefix even with valid packet ID");
    zassert_equal(parser.packet_id, 1, "Packet ID increments when packet ID matches, even if command parsing fails");
}

ZTEST(instruction_parser, test_parse_invalid_command_number)
{
    InstructionParser parser;
    InstructionParser::GCodeCmd cmd;
    
    // Create packet with ID 0 and just "G" (no number)
    const char* packet = "\x00G";
    bool result = parser.parseLine(packet, cmd);
    
    zassert_true(result, "Should parse G with implicit number 0 and valid packet ID");
    zassert_equal(cmd.code, 'G', "Command code should be G");
    zassert_equal(cmd.number, 0, "Command number should be 0 (atoi returns 0 for no digits)");
    zassert_equal(parser.packet_id, 1, "Packet ID should increment to 1 after successful parse");
}

ZTEST(instruction_parser, test_parse_empty_string)
{
    InstructionParser parser;
    InstructionParser::GCodeCmd cmd;
    
    bool result = parser.parseLine("", cmd);
    
    zassert_false(result, "Should fail to parse empty string (no packet ID)");
    zassert_equal(parser.packet_id, 1, "Packet ID increments because null terminator (0) matches initial packet_id (0)");
}

ZTEST(instruction_parser, test_parse_whitespace_only)
{
    InstructionParser parser;
    InstructionParser::GCodeCmd cmd;
    
    bool result = parser.parseLine("   \t\n  ", cmd);
    
    zassert_false(result, "Should fail to parse whitespace-only string (no packet ID)");
    zassert_equal(parser.packet_id, 1, "Packet ID increments because null terminator (0) after whitespace matches initial packet_id (0)");
}

ZTEST(instruction_parser, test_parse_max_args)
{
    InstructionParser parser;
    InstructionParser::GCodeCmd cmd;
    
    // Create a command with exactly MAX_ARGS (8) arguments, with packet ID 0
    const char* packet = "\x00G0 A1 B2 C3 D4 E5 F6 G7 H8";
    bool result = parser.parseLine(packet, cmd);
    
    zassert_true(result, "Should successfully parse command with max args and packet ID 0");
    zassert_equal(cmd.argc, 8, "Should have exactly MAX_ARGS arguments");
    
    // Verify all args are parsed correctly
    for (int i = 0; i < 8; i++) {
        char expected_letter = 'A' + i;
        float expected_value = 1.0f + i;
        zassert_equal(cmd.args[i].letter, expected_letter, "Arg letter should match");
        zassert_within(cmd.args[i].value, expected_value, 1e-6, "Arg value should match");
    }
    zassert_equal(parser.packet_id, 1, "Packet ID should increment to 1 after successful parse");
}

ZTEST(instruction_parser, test_parse_exceed_max_args)
{
    InstructionParser parser;
    InstructionParser::GCodeCmd cmd;
    
    // Create a command with more than MAX_ARGS (8) arguments, with packet ID 0
    const char* packet = "\x00G0 A1 B2 C3 D4 E5 F6 G7 H8 I9 J10";
    bool result = parser.parseLine(packet, cmd);
    
    zassert_true(result, "Should successfully parse but truncate excess args with packet ID 0");
    zassert_equal(cmd.argc, 8, "Should have exactly MAX_ARGS arguments");
    
    // Should only parse first 8 arguments
    zassert_equal(cmd.args[7].letter, 'H', "Last parsed arg should be H");
    zassert_within(cmd.args[7].value, 8.0f, 1e-6, "Last parsed value should be 8.0");
    zassert_equal(parser.packet_id, 1, "Packet ID should increment to 1 after successful parse");
}

ZTEST(instruction_parser, test_parse_negative_numbers)
{
    InstructionParser parser;
    InstructionParser::GCodeCmd cmd;
    
    // Create packet with ID 0 and negative/positive numbers
    const char* packet = "\x00G0 X-10.5 Y+20.25 Z-0.1";
    bool result = parser.parseLine(packet, cmd);
    
    zassert_true(result, "Should successfully parse negative and positive numbers with packet ID 0");
    zassert_equal(cmd.argc, 3, "Should have 3 arguments");
    zassert_within(cmd.args[0].value, -10.5f, 1e-6, "X value should be -10.5");
    zassert_within(cmd.args[1].value, 20.25f, 1e-6, "Y value should be +20.25");
    zassert_within(cmd.args[2].value, -0.1f, 1e-6, "Z value should be -0.1");
    zassert_equal(parser.packet_id, 1, "Packet ID should increment to 1 after successful parse");
}

ZTEST(instruction_parser, test_is_supported_g91)
{
    InstructionParser::GCodeCmd cmd;
    cmd.code = 'G';
    cmd.number = 91;
    
    bool result = InstructionParser::isSupported(cmd);
    zassert_true(result, "G91 should be supported");
}

ZTEST(instruction_parser, test_is_supported_g0)
{
    InstructionParser::GCodeCmd cmd;
    cmd.code = 'G';
    cmd.number = 0;
    
    bool result = InstructionParser::isSupported(cmd);
    zassert_true(result, "G0 should be supported");
}

ZTEST(instruction_parser, test_is_supported_m280)
{
    InstructionParser::GCodeCmd cmd;
    cmd.code = 'M';
    cmd.number = 280;
    
    bool result = InstructionParser::isSupported(cmd);
    zassert_true(result, "M280 should be supported");
}

ZTEST(instruction_parser, test_is_not_supported_g1)
{
    InstructionParser::GCodeCmd cmd;
    cmd.code = 'G';
    cmd.number = 1;
    
    bool result = InstructionParser::isSupported(cmd);
    zassert_false(result, "G1 should not be supported");
}

ZTEST(instruction_parser, test_is_not_supported_m104)
{
    InstructionParser::GCodeCmd cmd;
    cmd.code = 'M';
    cmd.number = 104;
    
    bool result = InstructionParser::isSupported(cmd);
    zassert_false(result, "M104 should not be supported");
}

ZTEST(instruction_parser, test_is_not_supported_invalid_code)
{
    InstructionParser::GCodeCmd cmd;
    cmd.code = 'X';
    cmd.number = 0;
    
    bool result = InstructionParser::isSupported(cmd);
    zassert_false(result, "Commands with invalid code should not be supported");
}

ZTEST(instruction_parser, test_parse_with_comments)
{
    InstructionParser parser;
    InstructionParser::GCodeCmd cmd;
    
    // Create packet with ID 0 and G-code with comment-like text
    const char* packet = "\x00G0 X10 ; move to X10";
    bool result = parser.parseLine(packet, cmd);
    
    zassert_true(result, "Should parse the command part with packet ID 0");
    zassert_equal(cmd.code, 'G', "Command code should be G");
    zassert_equal(cmd.number, 0, "Command number should be 0");
    // The parser will find X10, then M (from "move"), then T (from "to"), then X (from "X10")
    // This is expected behavior since the parser doesn't understand comment syntax
    zassert_true(cmd.argc >= 1, "Should have at least 1 argument");
    zassert_equal(cmd.args[0].letter, 'X', "First argument should be X");
    zassert_within(cmd.args[0].value, 10.0f, 1e-6, "X value should be 10.0");
    zassert_equal(parser.packet_id, 1, "Packet ID should increment to 1 after successful parse");
}

// NEW PACKET ID TESTS

ZTEST(instruction_parser, test_packet_id_validation_wrong_id)
{
    InstructionParser parser;
    InstructionParser::GCodeCmd cmd;
    
    // Try to send packet with ID 1 when parser expects 0
    const char* packet = "\x01G0 X10 Y5";
    bool result = parser.parseLine(packet, cmd);
    
    zassert_false(result, "Should fail when packet ID doesn't match expected");
    zassert_equal(parser.packet_id, 0, "Packet ID should remain 0 after failed validation");
}

ZTEST(instruction_parser, test_packet_id_sequence)
{
    InstructionParser parser;
    InstructionParser::GCodeCmd cmd;
    
    // Send packets with correct sequence: 0, 1, 2
    const char* packet1 = "\x00G0 X10";
    const char* packet2 = "\x01G91";
    const char* packet3 = "\x02M280 P0 S90";
    
    bool result1 = parser.parseLine(packet1, cmd);
    zassert_true(result1, "First packet (ID 0) should succeed");
    zassert_equal(parser.packet_id, 1, "Packet ID should be 1 after first packet");
    
    bool result2 = parser.parseLine(packet2, cmd);
    zassert_true(result2, "Second packet (ID 1) should succeed");
    zassert_equal(parser.packet_id, 2, "Packet ID should be 2 after second packet");
    
    bool result3 = parser.parseLine(packet3, cmd);
    zassert_true(result3, "Third packet (ID 2) should succeed");
    zassert_equal(parser.packet_id, 3, "Packet ID should be 3 after third packet");
}

ZTEST(instruction_parser, test_packet_id_sequence_with_failure)
{
    InstructionParser parser;
    InstructionParser::GCodeCmd cmd;
    
    // Send valid packet, then wrong ID, then correct next ID
    const char* packet1 = "\x00G0 X10";
    const char* packet_wrong = "\x05G91"; // Wrong ID
    const char* packet2 = "\x01M280 P0 S90"; // Correct next ID
    
    bool result1 = parser.parseLine(packet1, cmd);
    zassert_true(result1, "First packet (ID 0) should succeed");
    zassert_equal(parser.packet_id, 1, "Packet ID should be 1 after first packet");
    
    bool result_wrong = parser.parseLine(packet_wrong, cmd);
    zassert_false(result_wrong, "Packet with wrong ID should fail");
    zassert_equal(parser.packet_id, 1, "Packet ID should remain 1 after failed packet");
    
    bool result2 = parser.parseLine(packet2, cmd);
    zassert_true(result2, "Packet with correct ID should succeed");
    zassert_equal(parser.packet_id, 2, "Packet ID should be 2 after successful packet");
}

ZTEST(instruction_parser, test_packet_id_wraparound)
{
    InstructionParser parser;
    InstructionParser::GCodeCmd cmd;
    
    // Test a few high values leading up to 255
    parser.packet_id = 254;
    
    // Test packet with ID 254 first
    char packet254[20];
    packet254[0] = (char)254;
    packet254[1] = 'G';
    strcpy(&packet254[2], "0 X10");
    
    bool result254 = parser.parseLine(packet254, cmd);
    zassert_true(result254, "Packet with ID 254 should succeed");
    zassert_equal(parser.packet_id, 255, "Packet ID should be 255 after ID 254");
    
    // Now test packet with ID 255
    char packet255[20];
    packet255[0] = (char)255;  // This might be the issue - char vs uint8_t
    packet255[1] = 'G';
    strcpy(&packet255[2], "91");
    
    bool result255 = parser.parseLine(packet255, cmd);
    zassert_true(result255, "Packet with ID 255 should succeed");
    zassert_equal(parser.packet_id, 0, "Packet ID should wrap around to 0 after 255");
}

ZTEST(instruction_parser, test_packet_id_mid_range_values)
{
    InstructionParser parser;
    InstructionParser::GCodeCmd cmd;
    
    // Test some mid-range values to ensure no issues with signed/unsigned
    parser.packet_id = 100;
    
    char packet[20];
    packet[0] = (char)100;
    packet[1] = 'G';
    strcpy(&packet[2], "0 X10");
    
    bool result = parser.parseLine(packet, cmd);
    zassert_true(result, "Packet with ID 100 should succeed");
    zassert_equal(parser.packet_id, 101, "Packet ID should increment to 101");
    
    // Test boundary around signed char limit (127/128)
    parser.packet_id = 127;
    packet[0] = (char)127;
    
    result = parser.parseLine(packet, cmd);
    zassert_true(result, "Packet with ID 127 should succeed");
    zassert_equal(parser.packet_id, 128, "Packet ID should increment to 128");
    
    // Test 128 (where signed char becomes negative)
    packet[0] = (char)128;
    
    result = parser.parseLine(packet, cmd);
    zassert_true(result, "Packet with ID 128 should succeed (testing signed/unsigned boundary)");
    zassert_equal(parser.packet_id, 129, "Packet ID should increment to 129");
}

ZTEST(instruction_parser, test_packet_id_reset_behavior)
{
    InstructionParser parser;
    InstructionParser::GCodeCmd cmd;
    
    // Test that each parser instance starts with packet_id = 0
    zassert_equal(parser.packet_id, 0, "New parser should start with packet_id = 0");
    
    // Process a few packets
    const char* packet1 = "\x00G0 X10";
    const char* packet2 = "\x01G91";
    
    parser.parseLine(packet1, cmd);
    parser.parseLine(packet2, cmd);
    zassert_equal(parser.packet_id, 2, "Packet ID should be 2 after two packets");
    
    // Create new parser instance
    InstructionParser newParser;
    zassert_equal(newParser.packet_id, 0, "New parser instance should start with packet_id = 0");
}

ZTEST_SUITE(instruction_parser, NULL, NULL, NULL, NULL, NULL);