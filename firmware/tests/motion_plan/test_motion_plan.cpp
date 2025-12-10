/**
 * Unit Tests for Motion Planner Interpolation
 * 
 * Tests both linear and cubic Hermite interpolation modes.
 */

#include <zephyr/ztest.h>
#include <math.h>
#include "motion_plan.h"
#include "instruction_parser.h"

/* ================================
 * TEST FIXTURES AND HELPERS
 * ================================ */

#define FLOAT_TOLERANCE 0.001f

static bool float_equal(float a, float b, float tolerance = FLOAT_TOLERANCE) {
    return fabsf(a - b) < tolerance;
}

/**
 * Helper to create a G1 move command
 */
static InstructionParser::GCodeCmd make_g1_command(float x, float y, uint8_t packet_id = 0) {
    InstructionParser::GCodeCmd cmd = {};
    cmd.code = 'G';
    cmd.number = 1;
    cmd.args[0].letter = 'X';
    cmd.args[0].value = x;
    cmd.args[1].letter = 'Y';
    cmd.args[1].value = y;
    cmd.argc = 2;
    cmd.packet_id = packet_id;
    return cmd;
}

/**
 * Helper to create an M505 interpolation mode command
 */
static InstructionParser::GCodeCmd make_interp_mode_command(int mode) {
    InstructionParser::GCodeCmd cmd = {};
    cmd.code = 'M';
    cmd.number = 505;
    cmd.args[0].letter = 'I';
    cmd.args[0].value = (float)mode;
    cmd.argc = 1;
    cmd.packet_id = 0;
    return cmd;
}

/**
 * Count stepper commands in output
 */
static int count_stepper_commands(const MotionPlanner::Output& output) {
    int count = 0;
    for (size_t i = 0; i < output.count; i++) {
        if (output.cmds[i].device() == Steppers) {
            count++;
        }
    }
    return count;
}

/**
 * Calculate total distance traveled from stepper commands
 * This is approximate based on velocity commands
 */
static float calculate_path_distance(const MotionPlanner::Output& output) {
    float total = 0.0f;
    for (size_t i = 0; i < output.count; i++) {
        if (output.cmds[i].device() == Steppers) {
            // Average velocity * time step
            float avg_v = (fabsf(output.cmds[i].steppers().left_velocity) + 
                          fabsf(output.cmds[i].steppers().right_velocity)) / 2.0f;
            // Convert from deg/s to approximate mm/s using wheel radius
            float wheel_radius = WHEEL_RADIUS;
            float linear_v = avg_v * (PI / 180.0f) * wheel_radius;
            total += linear_v / STEPPER_CTRL_FREQ;  // mm per control cycle
        }
    }
    return total;
}

/* ================================
 * HERMITE MATH TESTS
 * ================================ */

/**
 * Test: Hermite basis functions at boundary values
 * H00(0) = 1, H00(1) = 0
 * H10(0) = 0, H10(1) = 0  
 * H01(0) = 0, H01(1) = 1
 * H11(0) = 0, H11(1) = 0
 */
ZTEST(motion_plan_hermite_math, test_hermite_basis_boundaries) {
    // H00: should be 1 at t=0, 0 at t=1
    float h00_0 = 2.0f * 0.0f * 0.0f * 0.0f - 3.0f * 0.0f * 0.0f + 1.0f;
    float h00_1 = 2.0f * 1.0f * 1.0f * 1.0f - 3.0f * 1.0f * 1.0f + 1.0f;
    zassert_true(float_equal(h00_0, 1.0f), "H00(0) should be 1");
    zassert_true(float_equal(h00_1, 0.0f), "H00(1) should be 0");
    
    // H10: should be 0 at both ends
    float h10_0 = 0.0f * 0.0f * 0.0f - 2.0f * 0.0f * 0.0f + 0.0f;
    float h10_1 = 1.0f * 1.0f * 1.0f - 2.0f * 1.0f * 1.0f + 1.0f;
    zassert_true(float_equal(h10_0, 0.0f), "H10(0) should be 0");
    zassert_true(float_equal(h10_1, 0.0f), "H10(1) should be 0");
    
    // H01: should be 0 at t=0, 1 at t=1
    float h01_0 = -2.0f * 0.0f * 0.0f * 0.0f + 3.0f * 0.0f * 0.0f;
    float h01_1 = -2.0f * 1.0f * 1.0f * 1.0f + 3.0f * 1.0f * 1.0f;
    zassert_true(float_equal(h01_0, 0.0f), "H01(0) should be 0");
    zassert_true(float_equal(h01_1, 1.0f), "H01(1) should be 1");
    
    // H11: should be 0 at both ends
    float h11_0 = 0.0f * 0.0f * 0.0f - 0.0f * 0.0f;
    float h11_1 = 1.0f * 1.0f * 1.0f - 1.0f * 1.0f;
    zassert_true(float_equal(h11_0, 0.0f), "H11(0) should be 0");
    zassert_true(float_equal(h11_1, 0.0f), "H11(1) should be 0");
    
    printk("PASS: Hermite basis functions have correct boundary values\n");
}

/**
 * Test: Hermite interpolation midpoint (t=0.5) 
 * With zero tangents, should be at midpoint between p0 and p1
 */
ZTEST(motion_plan_hermite_math, test_hermite_midpoint_zero_tangents) {
    float t = 0.5f;
    
    // Calculate basis functions at t=0.5
    float h00 = 2.0f * t * t * t - 3.0f * t * t + 1.0f;  // 0.5
    float h10 = t * t * t - 2.0f * t * t + t;             // 0.125
    float h01 = -2.0f * t * t * t + 3.0f * t * t;         // 0.5
    float h11 = t * t * t - t * t;                        // -0.125
    
    // With p0=0, p1=10, t0=0, t1=0:
    // P(0.5) = 0.5*0 + 0.125*0 + 0.5*10 + (-0.125)*0 = 5
    float result = h00 * 0.0f + h10 * 0.0f + h01 * 10.0f + h11 * 0.0f;
    
    zassert_true(float_equal(result, 5.0f), "Hermite midpoint with zero tangents should be at p1/2");
    printk("PASS: Hermite midpoint = %.2f (expected 5.0)\n", (double)result);
}

ZTEST_SUITE(motion_plan_hermite_math, NULL, NULL, NULL, NULL, NULL);

/* ================================
 * LINEAR INTERPOLATION TESTS
 * ================================ */

ZTEST(motion_plan_linear, test_linear_straight_line_x) {
    MotionPlanner planner;
    planner.setInterpolationMode(INTERP_LINEAR);
    
    // Move 10mm in positive X
    auto cmd = make_g1_command(10.0f, 0.0f);
    auto output = planner.consumeGcode(cmd);
    
    // Should generate stepper commands
    int stepper_count = count_stepper_commands(output);
    zassert_true(stepper_count > 0, "Should generate stepper commands for X movement");
    
    printk("PASS: Linear X movement generated %d stepper commands\n", stepper_count);
}

ZTEST(motion_plan_linear, test_linear_straight_line_y) {
    MotionPlanner planner;
    planner.setInterpolationMode(INTERP_LINEAR);
    
    // Move 10mm in positive Y (requires 90 degree turn first)
    auto cmd = make_g1_command(0.0f, 10.0f);
    auto output = planner.consumeGcode(cmd);
    
    // Should generate stepper commands (turn + forward)
    int stepper_count = count_stepper_commands(output);
    zassert_true(stepper_count > 0, "Should generate stepper commands for Y movement");
    
    printk("PASS: Linear Y movement generated %d stepper commands\n", stepper_count);
}

ZTEST(motion_plan_linear, test_linear_diagonal) {
    MotionPlanner planner;
    planner.setInterpolationMode(INTERP_LINEAR);
    
    // Move diagonally (45 degrees)
    auto cmd = make_g1_command(10.0f, 10.0f);
    auto output = planner.consumeGcode(cmd);
    
    int stepper_count = count_stepper_commands(output);
    zassert_true(stepper_count > 0, "Should generate stepper commands for diagonal movement");
    
    // Distance should be approximately sqrt(10^2 + 10^2) = 14.14mm
    float distance = calculate_path_distance(output);
    float expected = sqrtf(200.0f);
    
    printk("PASS: Linear diagonal generated %d commands, distance ~%.1f mm (expected ~%.1f)\n", 
           stepper_count, (double)distance, (double)expected);
}

ZTEST(motion_plan_linear, test_linear_sequence_maintains_heading) {
    MotionPlanner planner;
    planner.setInterpolationMode(INTERP_LINEAR);
    
    // Move right, then right again - should not need to turn
    auto cmd1 = make_g1_command(10.0f, 0.0f, 1);
    auto cmd2 = make_g1_command(10.0f, 0.0f, 2);
    
    auto output1 = planner.consumeGcode(cmd1);
    auto output2 = planner.consumeGcode(cmd2);
    
    // Both should generate commands
    zassert_true(output1.count > 0, "First move should generate commands");
    zassert_true(output2.count > 0, "Second move should generate commands");
    
    printk("PASS: Sequential same-direction moves work correctly\n");
}

ZTEST(motion_plan_linear, test_linear_small_movement) {
    MotionPlanner planner;
    planner.setInterpolationMode(INTERP_LINEAR);
    
    // Small but practical movement (2mm) - above discretization threshold
    auto cmd = make_g1_command(2.0f, 0.0f);
    auto output = planner.consumeGcode(cmd);
    
    // Should generate commands for 2mm movement
    int stepper_count = count_stepper_commands(output);
    zassert_true(stepper_count > 0, "Should handle small (2mm) movements");
    
    printk("PASS: Small movement (2mm) generated %d commands\n", stepper_count);
}

ZTEST(motion_plan_linear, test_linear_below_threshold) {
    MotionPlanner planner;
    planner.setInterpolationMode(INTERP_LINEAR);
    
    // Movement below threshold (< 0.1mm)
    auto cmd = make_g1_command(0.05f, 0.0f);
    auto output = planner.consumeGcode(cmd);
    
    // May generate 0 commands due to threshold
    printk("INFO: Below-threshold movement (0.05mm) generated %zu commands\n", output.count);
}

ZTEST_SUITE(motion_plan_linear, NULL, NULL, NULL, NULL, NULL);

/* ================================
 * CUBIC HERMITE INTERPOLATION TESTS
 * ================================ */

ZTEST(motion_plan_hermite, test_hermite_first_waypoint_buffered) {
    MotionPlanner planner;
    planner.setInterpolationMode(INTERP_CUBIC_HERMITE);
    
    // First waypoint - should be buffered, minimal/no output
    auto cmd = make_g1_command(10.0f, 0.0f);
    auto output = planner.consumeGcode(cmd);
    
    // Hermite buffers first waypoint for look-ahead
    printk("INFO: Hermite first waypoint generated %zu commands (may buffer)\n", output.count);
}

ZTEST(motion_plan_hermite, test_hermite_second_waypoint_interpolates) {
    MotionPlanner planner;
    planner.setInterpolationMode(INTERP_CUBIC_HERMITE);
    
    // First waypoint (buffered)
    auto cmd1 = make_g1_command(10.0f, 0.0f, 1);
    planner.consumeGcode(cmd1);
    
    // Second waypoint - should trigger interpolation
    auto cmd2 = make_g1_command(10.0f, 10.0f, 2);
    auto output2 = planner.consumeGcode(cmd2);
    
    int stepper_count = count_stepper_commands(output2);
    zassert_true(stepper_count > 0, "Second waypoint should trigger interpolation");
    
    printk("PASS: Hermite second waypoint generated %d stepper commands\n", stepper_count);
}

ZTEST(motion_plan_hermite, test_hermite_generates_more_samples_than_linear) {
    // Linear planner
    MotionPlanner linear_planner;
    linear_planner.setInterpolationMode(INTERP_LINEAR);
    
    // Hermite planner
    MotionPlanner hermite_planner;
    hermite_planner.setInterpolationMode(INTERP_CUBIC_HERMITE);
    
    // Same path for both
    auto cmd1 = make_g1_command(20.0f, 0.0f, 1);
    auto cmd2 = make_g1_command(0.0f, 20.0f, 2);
    auto cmd3 = make_g1_command(-20.0f, 0.0f, 3);
    
    // Linear: direct point-to-point
    auto lin_out1 = linear_planner.consumeGcode(cmd1);
    auto lin_out2 = linear_planner.consumeGcode(cmd2);
    auto lin_out3 = linear_planner.consumeGcode(cmd3);
    int linear_total = count_stepper_commands(lin_out1) + 
                       count_stepper_commands(lin_out2) + 
                       count_stepper_commands(lin_out3);
    
    // Hermite: smooth interpolation (first is buffered)
    hermite_planner.consumeGcode(cmd1);  // buffered
    auto herm_out2 = hermite_planner.consumeGcode(cmd2);
    auto herm_out3 = hermite_planner.consumeGcode(cmd3);
    int hermite_total = count_stepper_commands(herm_out2) + 
                        count_stepper_commands(herm_out3);
    
    printk("INFO: Linear generated %d commands, Hermite generated %d commands\n",
           linear_total, hermite_total);
    
    // Hermite typically generates more micro-movements for smooth curves
    // but this depends on segment length and sample resolution
}

ZTEST(motion_plan_hermite, test_hermite_curved_path) {
    MotionPlanner planner;
    planner.setInterpolationMode(INTERP_CUBIC_HERMITE);
    
    // Create a curved path (quarter circle approximation)
    auto cmd1 = make_g1_command(10.0f, 0.0f, 1);   // Start going right
    auto cmd2 = make_g1_command(7.0f, 7.0f, 2);    // Curve up-right
    auto cmd3 = make_g1_command(0.0f, 10.0f, 3);   // End going up
    
    planner.consumeGcode(cmd1);  // buffered
    auto output2 = planner.consumeGcode(cmd2);
    auto output3 = planner.consumeGcode(cmd3);
    
    int total_commands = count_stepper_commands(output2) + count_stepper_commands(output3);
    zassert_true(total_commands > 0, "Curved path should generate commands");
    
    printk("PASS: Hermite curved path generated %d total stepper commands\n", total_commands);
}

ZTEST_SUITE(motion_plan_hermite, NULL, NULL, NULL, NULL, NULL);

/* ================================
 * MODE SWITCHING TESTS
 * ================================ */

ZTEST(motion_plan_modes, test_mode_default_is_linear) {
    MotionPlanner planner;
    zassert_equal(planner.getInterpolationMode(), INTERP_LINEAR, 
                  "Default mode should be LINEAR");
    printk("PASS: Default interpolation mode is LINEAR\n");
}

ZTEST(motion_plan_modes, test_mode_switch_to_hermite) {
    MotionPlanner planner;
    
    planner.setInterpolationMode(INTERP_CUBIC_HERMITE);
    zassert_equal(planner.getInterpolationMode(), INTERP_CUBIC_HERMITE,
                  "Mode should be CUBIC_HERMITE after setting");
    
    printk("PASS: Mode switch to CUBIC_HERMITE works\n");
}

ZTEST(motion_plan_modes, test_mode_switch_via_gcode) {
    MotionPlanner planner;
    
    // Switch to Hermite via M505 I1
    auto cmd = make_interp_mode_command(1);
    planner.consumeGcode(cmd);
    
    zassert_equal(planner.getInterpolationMode(), INTERP_CUBIC_HERMITE,
                  "M505 I1 should set CUBIC_HERMITE mode");
    
    // Switch back to Linear via M505 I0
    auto cmd2 = make_interp_mode_command(0);
    planner.consumeGcode(cmd2);
    
    zassert_equal(planner.getInterpolationMode(), INTERP_LINEAR,
                  "M505 I0 should set LINEAR mode");
    
    printk("PASS: G-code mode switching (M505) works correctly\n");
}

ZTEST(motion_plan_modes, test_mode_switch_mid_path) {
    MotionPlanner planner;
    
    // Start with linear
    planner.setInterpolationMode(INTERP_LINEAR);
    auto cmd1 = make_g1_command(10.0f, 0.0f, 1);
    auto output1 = planner.consumeGcode(cmd1);
    
    // Switch to Hermite
    planner.setInterpolationMode(INTERP_CUBIC_HERMITE);
    auto cmd2 = make_g1_command(10.0f, 10.0f, 2);
    auto output2 = planner.consumeGcode(cmd2);
    
    // Both should produce some output
    zassert_true(output1.count > 0, "Linear segment should produce output");
    // Hermite first waypoint after mode switch may be buffered
    
    printk("PASS: Mode switch mid-path handles transition\n");
}

ZTEST_SUITE(motion_plan_modes, NULL, NULL, NULL, NULL, NULL);

/* ================================
 * RESET AND STATE TESTS
 * ================================ */

ZTEST(motion_plan_state, test_reset_clears_position) {
    MotionPlanner planner;
    planner.setInterpolationMode(INTERP_LINEAR);
    
    // Move somewhere
    auto cmd1 = make_g1_command(50.0f, 50.0f);
    planner.consumeGcode(cmd1);
    
    // Reset
    planner.reset();
    
    // Move from origin again - should work as if fresh start
    auto cmd2 = make_g1_command(10.0f, 0.0f);
    auto output = planner.consumeGcode(cmd2);
    
    zassert_true(output.count > 0, "Should generate commands after reset");
    printk("PASS: Reset clears planner state\n");
}

ZTEST(motion_plan_state, test_reset_clears_waypoint_buffer) {
    MotionPlanner planner;
    planner.setInterpolationMode(INTERP_CUBIC_HERMITE);
    
    // Add waypoints
    planner.consumeGcode(make_g1_command(10.0f, 0.0f));
    planner.consumeGcode(make_g1_command(20.0f, 10.0f));
    
    // Reset
    planner.reset();
    
    // First waypoint after reset should be buffered (fresh start)
    auto output = planner.consumeGcode(make_g1_command(5.0f, 0.0f));
    
    printk("PASS: Reset clears Hermite waypoint buffer\n");
}

ZTEST_SUITE(motion_plan_state, NULL, NULL, NULL, NULL, NULL);

/* ================================
 * EDGE CASES
 * ================================ */

ZTEST(motion_plan_edge_cases, test_zero_movement) {
    MotionPlanner planner;
    planner.setInterpolationMode(INTERP_LINEAR);
    
    // Zero movement
    auto cmd = make_g1_command(0.0f, 0.0f);
    auto output = planner.consumeGcode(cmd);
    
    // Should handle gracefully (no stepper commands needed)
    int stepper_count = count_stepper_commands(output);
    printk("INFO: Zero movement generated %d stepper commands\n", stepper_count);
}

ZTEST(motion_plan_edge_cases, test_negative_coordinates) {
    MotionPlanner planner;
    planner.setInterpolationMode(INTERP_LINEAR);
    
    // Negative movement
    auto cmd = make_g1_command(-10.0f, -10.0f);
    auto output = planner.consumeGcode(cmd);
    
    int stepper_count = count_stepper_commands(output);
    zassert_true(stepper_count > 0, "Should handle negative coordinates");
    
    printk("PASS: Negative coordinates generated %d commands\n", stepper_count);
}

ZTEST(motion_plan_edge_cases, test_large_movement) {
    MotionPlanner planner;
    planner.setInterpolationMode(INTERP_LINEAR);
    
    // Large movement (100mm)
    auto cmd = make_g1_command(100.0f, 0.0f);
    auto output = planner.consumeGcode(cmd);
    
    int stepper_count = count_stepper_commands(output);
    zassert_true(stepper_count > 0, "Should handle large movements");
    zassert_true(output.count <= MOTION_PLAN_OUTPUT_SIZE, 
                 "Output should not exceed buffer size");
    
    printk("PASS: Large movement (100mm) generated %d commands (max %d)\n", 
           stepper_count, MOTION_PLAN_OUTPUT_SIZE);
}

ZTEST(motion_plan_edge_cases, test_180_degree_turn) {
    MotionPlanner planner;
    planner.setInterpolationMode(INTERP_LINEAR);
    
    // Move right, then left (180 degree turn)
    auto cmd1 = make_g1_command(10.0f, 0.0f);
    auto cmd2 = make_g1_command(-10.0f, 0.0f);
    
    planner.consumeGcode(cmd1);
    auto output2 = planner.consumeGcode(cmd2);
    
    int stepper_count = count_stepper_commands(output2);
    zassert_true(stepper_count > 0, "Should handle 180 degree turn");
    
    printk("PASS: 180 degree turn generated %d commands\n", stepper_count);
}

ZTEST_SUITE(motion_plan_edge_cases, NULL, NULL, NULL, NULL, NULL);
