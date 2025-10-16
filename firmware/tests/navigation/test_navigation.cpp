#include <zephyr/ztest.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "navigation.h"
#include "instruction_parser.h"

// Mock implementations for hardware functions that Navigation depends on
extern "C" {
    bool stepper_init(void) { return true; }
    void simple_led_init(void) {}
    void buzzer_init(void) {}
}

// Test data structure for CSV output
struct MovementRecord {
    int step;
    float start_x, start_y, start_heading;
    float target_x, target_y;
    float dx, dy;
    bool pen_down;
    const char* command;
};

static MovementRecord movement_log[100];
static int movement_count = 0;

// Test results tracking
struct TestResult {
    const char* test_name;
    float avg_deviation;
    float max_deviation;
    float percentage_error;
    float efficiency;
    int move_count;
};

static TestResult test_results[10];
static int test_result_count = 0;

void log_movement(int step, const Navigation& nav, const MovementDelta& delta, const char* cmd_str) {
    if (movement_count < 100) {
        MovementRecord& record = movement_log[movement_count++];
        record.step = step;
        record.start_x = nav.getCurrentPosition().x;
        record.start_y = nav.getCurrentPosition().y;
        record.start_heading = nav.getCurrentPosition().heading_degrees;
        record.target_x = nav.getTargetPosition().x;
        record.target_y = nav.getTargetPosition().y;
        record.dx = delta.dx;
        record.dy = delta.dy;
        record.pen_down = delta.pen_down;
        record.command = cmd_str;
    }
}

void analyze_path_deviation(const char* test_name) {
    printk("\n=== DEVIATION ANALYSIS for %s ===\n", test_name);
    
    float total_deviation = 0.0f;
    float max_deviation = 0.0f;
    int pen_down_moves = 0;
    float expected_step_distance = 0.0f; // Declare here for circle pattern
    
    // Also analyze differential drive commands
    printk("\n--- Differential Drive Analysis ---\n");
    
    // Calculate deviations based on test pattern
    if (strcmp(test_name, "square") == 0) {
        // Expected square pattern: right 50, up 50, left 50, down 50
        float expected_moves[][2] = {{50.0f, 0.0f}, {0.0f, 50.0f}, {-50.0f, 0.0f}, {0.0f, -50.0f}};
        int expected_idx = 0;
        
        for (int i = 0; i < movement_count && expected_idx < 4; i++) {
            MovementRecord& r = movement_log[i];
            if (r.pen_down && (r.dx != 0.0f || r.dy != 0.0f)) {
                float expected_dx = expected_moves[expected_idx][0];
                float expected_dy = expected_moves[expected_idx][1];
                float deviation = sqrtf((r.dx - expected_dx) * (r.dx - expected_dx) + 
                                      (r.dy - expected_dy) * (r.dy - expected_dy));
                total_deviation += deviation;
                if (deviation > max_deviation) max_deviation = deviation;
                
                // Calculate wheel commands for this movement
                Navigation nav; // Create temporary nav for calculations
                nav.setDriveParams(100.0f, 50.0f, 200); // Example: 100mm wheelbase, 50mm wheels, 200 steps/rev
                WheelCommands wheel_cmd = nav.calculateWheelCommands(r.dx, r.dy);
                
                // Calculate theoretical vs actual movement efficiency
                float theoretical_distance = sqrtf(expected_dx * expected_dx + expected_dy * expected_dy);
                float actual_distance = sqrtf(r.dx * r.dx + r.dy * r.dy);
                float move_efficiency = (theoretical_distance > 0) ? (actual_distance / theoretical_distance) * 100.0f : 100.0f;
                int eff_int = (int)move_efficiency;
                int eff_frac = (int)((move_efficiency - eff_int) * 100);
                
                printk("Move %d: dx=%d.%02d dy=%d.%02d -> L:%d R:%d steps (eff: %d.%02d%%)\n", 
                       expected_idx + 1,
                       (int)r.dx, (int)((fabsf(r.dx) - (int)fabsf(r.dx)) * 100),
                       (int)r.dy, (int)((fabsf(r.dy) - (int)fabsf(r.dy)) * 100),
                       wheel_cmd.left_steps, wheel_cmd.right_steps,
                       eff_int, eff_frac);
                
                pen_down_moves++;
                expected_idx++;
            }
        }
    } else if (strcmp(test_name, "circle") == 0) {
        // For circle, check if all movements are roughly the same distance (consistent step size)
        // Calculate expected step distance from first movement
        float expected_step_distance = 0.0f;
        bool first_move = true;
        float total_step_distance = 0.0f;
        int step_count = 0;
        
        for (int i = 0; i < movement_count; i++) {
            MovementRecord& r = movement_log[i];
            if (r.pen_down && (r.dx != 0.0f || r.dy != 0.0f)) {
                float step_distance = sqrtf(r.dx * r.dx + r.dy * r.dy);
                total_step_distance += step_distance;
                step_count++;
                
                if (first_move) {
                    expected_step_distance = step_distance;
                    first_move = false;
                } else {
                    float deviation = fabsf(step_distance - expected_step_distance);
                    total_deviation += deviation;
                    if (deviation > max_deviation) max_deviation = deviation;
                }
                pen_down_moves++;
            }
        }
        
        // Use average step distance for percentage calculation
        if (step_count > 0) {
            expected_step_distance = total_step_distance / step_count;
        }
    } else if (strcmp(test_name, "star") == 0) {
        // For star pattern, expect alternating long/short movements
        // Calculate average of actual movements since star has varying distances
        float total_movement_distance = 0.0f;
        for (int i = 0; i < movement_count; i++) {
            MovementRecord& r = movement_log[i];
            if (r.pen_down && (r.dx != 0.0f || r.dy != 0.0f)) {
                float movement_distance = sqrtf(r.dx * r.dx + r.dy * r.dy);
                total_movement_distance += movement_distance;
                pen_down_moves++;
            }
        }
        // For star, deviation is just variance from average movement
        if (pen_down_moves > 0) {
            float avg_movement = total_movement_distance / pen_down_moves;
            for (int i = 0; i < movement_count; i++) {
                MovementRecord& r = movement_log[i];
                if (r.pen_down && (r.dx != 0.0f || r.dy != 0.0f)) {
                    float movement_distance = sqrtf(r.dx * r.dx + r.dy * r.dy);
                    float deviation = fabsf(movement_distance - avg_movement);
                    total_deviation += deviation;
                    if (deviation > max_deviation) max_deviation = deviation;
                }
            }
            expected_step_distance = avg_movement; // Use for percentage calculation
        }
    } else {
        // For other patterns, only analyze actual pen-down drawing movements
        for (int i = 0; i < movement_count; i++) {
            MovementRecord& r = movement_log[i];
            if (r.pen_down && (r.dx != 0.0f || r.dy != 0.0f)) {
                // For mixed modes, just check that movements are executing correctly
                // Don't calculate position deviation since it switches coordinate systems
                float movement_distance = sqrtf(r.dx * r.dx + r.dy * r.dy);
                float expected_movement = 10.0f; // Expected relative movement size
                float deviation = fabsf(movement_distance - expected_movement);
                total_deviation += deviation;
                if (deviation > max_deviation) max_deviation = deviation;
                pen_down_moves++;
            }
        }
    }
    
    if (pen_down_moves > 0) {
        float avg_deviation = total_deviation / pen_down_moves;
        
        // Calculate percentage deviation based on pattern
        float expected_distance = 0.0f;
        float percentage_deviation = 0.0f;
        
        if (strcmp(test_name, "square") == 0) {
            expected_distance = 50.0f; // Each side of square is 50 units
        } else if (strcmp(test_name, "circle") == 0) {
            expected_distance = expected_step_distance; // Use calculated average step distance
        } else if (strcmp(test_name, "star") == 0) {
            expected_distance = expected_step_distance; // Use calculated average movement distance
        } else if (strcmp(test_name, "mixed_modes") == 0) {
            // For mixed modes, calculate average of actual movements
            float total_distance = 0.0f;
            int move_count = 0;
            for (int i = 0; i < movement_count; i++) {
                MovementRecord& r = movement_log[i];
                if (r.pen_down && (r.dx != 0.0f || r.dy != 0.0f)) {
                    total_distance += sqrtf(r.dx * r.dx + r.dy * r.dy);
                    move_count++;
                }
            }
            expected_distance = (move_count > 0) ? total_distance / move_count : 10.0f;
        } else {
            expected_distance = 20.0f; // Generic estimate
        }
        
        if (expected_distance > 0.0f) {
            percentage_deviation = (avg_deviation / expected_distance) * 100.0f;
        } else {
            percentage_deviation = 0.0f; // No expected distance means perfect accuracy
        }
        
        // Print integer and fractional parts separately (Zephyr printk limitation)
        int avg_int = (int)avg_deviation;
        int avg_frac = (int)((avg_deviation - avg_int) * 1000);
        int max_int = (int)max_deviation;
        int max_frac = (int)((max_deviation - max_int) * 1000);
        int percent_int = (int)percentage_deviation;
        int percent_frac = (int)((percentage_deviation - percent_int) * 100);
        
        printk("Pattern: %s\n", test_name);
        printk("Total moves analyzed: %d\n", pen_down_moves);
        printk("Average deviation: %d.%03d units\n", avg_int, avg_frac);
        printk("Maximum deviation: %d.%03d units\n", max_int, max_frac);
        printk("Percentage error: %d.%02d%%\n", percent_int, percent_frac);
        
        // Accuracy rating based on percentage
        const char* rating;
        if (percentage_deviation < 1.0f) rating = "EXCELLENT (<1%)";
        else if (percentage_deviation < 5.0f) rating = "GOOD (<5%)";
        else if (percentage_deviation < 10.0f) rating = "FAIR (<10%)";
        else if (percentage_deviation < 20.0f) rating = "POOR (<20%)";
        else rating = "VERY POOR (>20%)";
        
        printk("Accuracy rating: %s\n", rating);
        
        // Calculate efficiency metrics
        float total_expected_distance = expected_distance * pen_down_moves;
        float efficiency = ((total_expected_distance - total_deviation) / total_expected_distance) * 100.0f;
        int eff_int = (int)efficiency;
        int eff_frac = (int)((efficiency - eff_int) * 100);
        printk("Movement efficiency: %d.%02d%%\n", eff_int, eff_frac);
        
        // Store test results for overall summary
        if (test_result_count < 10) {
            TestResult& result = test_results[test_result_count++];
            result.test_name = test_name;
            result.avg_deviation = avg_deviation;
            result.max_deviation = max_deviation;
            result.percentage_error = percentage_deviation;
            result.efficiency = efficiency;
            result.move_count = pen_down_moves;
        }
        
    } else {
        printk("No pen-down movements found for analysis\n");
    }
    
    printk("=== END ANALYSIS ===\n\n");
}

void print_overall_accuracy_summary() {
    if (test_result_count == 0) {
        printk("No test results to summarize\n");
        return;
    }
    
    printk("\n======================================================================\n");
    printk("OVERALL NAVIGATION ACCURACY SUMMARY\n");
    printk("======================================================================\n");
    
    float total_percentage_error = 0.0f;
    float total_efficiency = 0.0f;
    float max_error = 0.0f;
    float min_error = 1000.0f;
    int total_moves = 0;
    int excellent_tests = 0, good_tests = 0, fair_tests = 0, poor_tests = 0;
    
    printk("Individual Test Results:\n");
    printk("Pattern       | Moves | Avg Dev | Max Dev | Error%% | Efficiency | Rating\n");
    printk("------------- | ----- | ------- | ------- | ------ | ---------- | -------\n");
    
    for (int i = 0; i < test_result_count; i++) {
        TestResult& r = test_results[i];
        
        // Determine rating
        const char* rating;
        if (r.percentage_error < 1.0f) {
            rating = "EXCELLENT";
            excellent_tests++;
        } else if (r.percentage_error < 5.0f) {
            rating = "GOOD";
            good_tests++;
        } else if (r.percentage_error < 10.0f) {
            rating = "FAIR";
            fair_tests++;
        } else {
            rating = "POOR";
            poor_tests++;
        }
        
        // Print with integer arithmetic to avoid float formatting
        int avg_int = (int)(r.avg_deviation * 1000);
        int max_int = (int)(r.max_deviation * 1000);
        int err_int = (int)(r.percentage_error * 100);
        int eff_int = (int)(r.efficiency * 100);
        
        printk("%-12s |  %3d  | %3d.%03d | %3d.%03d | %2d.%02d | %3d.%02d%%   | %s\n",
               r.test_name, r.move_count,
               avg_int/1000, avg_int%1000,
               max_int/1000, max_int%1000,
               err_int/100, err_int%100,
               eff_int/100, eff_int%100,
               rating);
        
        total_percentage_error += r.percentage_error;
        total_efficiency += r.efficiency;
        total_moves += r.move_count;
        
        if (r.percentage_error > max_error) max_error = r.percentage_error;
        if (r.percentage_error < min_error) min_error = r.percentage_error;
    }
    
    printk("\n");
    printk("Overall Statistics:\n");
    printk("  Total Tests:        %d\n", test_result_count);
    printk("  Total Movements:    %d\n", total_moves);
    
    float avg_error = total_percentage_error / test_result_count;
    float avg_efficiency = total_efficiency / test_result_count;
    
    int avg_err_int = (int)(avg_error * 100);
    int max_err_int = (int)(max_error * 100);
    int min_err_int = (int)(min_error * 100);
    int avg_eff_int = (int)(avg_efficiency * 100);
    
    printk("  Average Error:      %d.%02d%%\n", avg_err_int/100, avg_err_int%100);
    printk("  Error Range:        %d.%02d%% - %d.%02d%%\n", 
           min_err_int/100, min_err_int%100, max_err_int/100, max_err_int%100);
    printk("  Average Efficiency: %d.%02d%%\n", avg_eff_int/100, avg_eff_int%100);
    
    printk("\n");
    printk("Test Distribution:\n");
    printk("  EXCELLENT (<1%%):   %d tests\n", excellent_tests);
    printk("  GOOD (1-5%%):       %d tests\n", good_tests);
    printk("  FAIR (5-10%%):      %d tests\n", fair_tests);
    printk("  POOR (>10%%):       %d tests\n", poor_tests);
    
    // Overall system rating
    const char* system_rating;
    if (avg_error < 1.0f) system_rating = "EXCELLENT";
    else if (avg_error < 5.0f) system_rating = "GOOD";
    else if (avg_error < 10.0f) system_rating = "FAIR";
    else system_rating = "POOR";
    
    printk("\n");
    printk("OVERALL SYSTEM RATING: %s\n", system_rating);
    printk("======================================================================\n");
    
    // Print final summary - the most important metric
    printk("\n*** FINAL RESULT: AVERAGE ACCURACY ERROR = %d.%02d%% ***\n\n", 
           avg_err_int/100, avg_err_int%100);
}

// Helper function to create G-code command
InstructionParser::GCodeCmd create_gcode_cmd(char code, int number, 
                                           float x = NAN, float y = NAN, float s = NAN) {
    InstructionParser::GCodeCmd cmd = {};
    cmd.code = code;
    cmd.number = number;
    cmd.argc = 0;
    
    if (!isnan(x)) {
        cmd.args[cmd.argc].letter = 'X';
        cmd.args[cmd.argc].value = x;
        cmd.argc++;
    }
    if (!isnan(y)) {
        cmd.args[cmd.argc].letter = 'Y';
        cmd.args[cmd.argc].value = y;
        cmd.argc++;
    }
    if (!isnan(s)) {
        cmd.args[cmd.argc].letter = 'S';
        cmd.args[cmd.argc].value = s;
        cmd.argc++;
    }
    
    return cmd;
}

ZTEST(navigation, test_simple_square_pattern) {
    Navigation nav;
    movement_count = 0;
    
    printk("Testing simple square pattern movement\n");
    
    // Set to relative mode
    auto cmd_g91 = create_gcode_cmd('G', 91);
    auto delta = nav.processGCodeCommand(cmd_g91);
    log_movement(0, nav, delta, "G91");
    nav.updatePosition(delta);
    
    // Draw a 50x50 square
    struct {
        float x, y;
        const char* desc;
    } square_moves[] = {
        {50.0f, 0.0f, "G1 X50"},      // Right
        {0.0f, 50.0f, "G1 Y50"},      // Up  
        {-50.0f, 0.0f, "G1 X-50"},    // Left
        {0.0f, -50.0f, "G1 Y-50"}     // Down
    };
    
    // Pen down
    auto pen_down_cmd = create_gcode_cmd('M', 280, NAN, NAN, 90.0f);
    delta = nav.processGCodeCommand(pen_down_cmd);
    log_movement(1, nav, delta, "M280 S90");
    nav.updatePosition(delta);
    
    // Execute square moves
    for (int i = 0; i < 4; i++) {
        auto move_cmd = create_gcode_cmd('G', 1, square_moves[i].x, square_moves[i].y);
        delta = nav.processGCodeCommand(move_cmd);
        log_movement(i + 2, nav, delta, square_moves[i].desc);
        nav.updatePosition(delta);
    }
    
    // Pen up
    auto pen_up_cmd = create_gcode_cmd('M', 280, NAN, NAN, 0.0f);
    delta = nav.processGCodeCommand(pen_up_cmd);
    log_movement(6, nav, delta, "M280 S0");
    nav.updatePosition(delta);
    
    // Analyze the deviation from expected path
    analyze_path_deviation("square");
    
    // Verify final position is back at origin
    zassert_within(nav.getCurrentPosition().x, 0.0f, 0.001f, "Should return to X=0");
    zassert_within(nav.getCurrentPosition().y, 0.0f, 0.001f, "Should return to Y=0");
}

ZTEST(navigation, test_circle_pattern) {
    Navigation nav;
    movement_count = 0;
    
    printk("Testing circular pattern movement\n");
    
    // Set to absolute mode
    auto cmd_g90 = create_gcode_cmd('G', 90);
    auto delta = nav.processGCodeCommand(cmd_g90);
    log_movement(0, nav, delta, "G90");
    nav.updatePosition(delta);
    
    // Pen down
    auto pen_down_cmd = create_gcode_cmd('M', 280, NAN, NAN, 90.0f);
    delta = nav.processGCodeCommand(pen_down_cmd);
    log_movement(1, nav, delta, "M280 S90");
    nav.updatePosition(delta);
    
    // Draw circle with 8 points (approximate circle)
    float radius = 30.0f;
    int num_points = 8;
    
    for (int i = 0; i <= num_points; i++) {
        float angle = (2.0f * M_PI * i) / num_points;
        float x = radius * cos(angle);
        float y = radius * sin(angle);
        
        char desc[32];
        snprintf(desc, sizeof(desc), "G1 X%.1f Y%.1f", x, y);
        
        auto move_cmd = create_gcode_cmd('G', 1, x, y);
        delta = nav.processGCodeCommand(move_cmd);
        log_movement(i + 2, nav, delta, desc);
        nav.updatePosition(delta);
    }
    
    // Pen up
    auto pen_up_cmd = create_gcode_cmd('M', 280, NAN, NAN, 0.0f);
    delta = nav.processGCodeCommand(pen_up_cmd);
    log_movement(num_points + 3, nav, delta, "M280 S0");
    nav.updatePosition(delta);
    
    analyze_path_deviation("circle");
    
    // Verify we ended up back near the start of the circle
    zassert_within(nav.getCurrentPosition().x, radius, 0.1f, "Should end near circle start X");
    zassert_within(nav.getCurrentPosition().y, 0.0f, 0.1f, "Should end near circle start Y");
}

ZTEST(navigation, test_star_pattern) {
    Navigation nav;
    movement_count = 0;
    
    printk("Testing star pattern movement\n");
    
    // Set to absolute mode
    auto cmd_g90 = create_gcode_cmd('G', 90);
    auto delta = nav.processGCodeCommand(cmd_g90);
    log_movement(0, nav, delta, "G90");
    nav.updatePosition(delta);
    
    // Pen down
    auto pen_down_cmd = create_gcode_cmd('M', 280, NAN, NAN, 90.0f);
    delta = nav.processGCodeCommand(pen_down_cmd);
    log_movement(1, nav, delta, "M280 S90");
    nav.updatePosition(delta);
    
    // Draw 5-pointed star
    float outer_radius = 40.0f;
    float inner_radius = 16.0f;
    int points = 5;
    
    for (int i = 0; i <= points * 2; i++) {
        float angle = (2.0f * M_PI * i) / (points * 2);
        float radius = (i % 2 == 0) ? outer_radius : inner_radius;
        float x = radius * cos(angle - M_PI/2); // Start at top
        float y = radius * sin(angle - M_PI/2);
        
        char desc[32];
        snprintf(desc, sizeof(desc), "G1 X%.1f Y%.1f", x, y);
        
        auto move_cmd = create_gcode_cmd('G', 1, x, y);
        delta = nav.processGCodeCommand(move_cmd);
        log_movement(i + 2, nav, delta, desc);
        nav.updatePosition(delta);
    }
    
    // Pen up
    auto pen_up_cmd = create_gcode_cmd('M', 280, NAN, NAN, 0.0f);
    delta = nav.processGCodeCommand(pen_up_cmd);
    log_movement(points * 2 + 3, nav, delta, "M280 S0");
    nav.updatePosition(delta);
    
    analyze_path_deviation("star");
}

ZTEST(navigation, test_mixed_coordinate_modes) {
    Navigation nav;
    movement_count = 0;
    
    printk("Testing mixed absolute and relative movements\n");
    
    // Start in absolute mode, go to (20, 20)
    auto cmd_g90 = create_gcode_cmd('G', 90);
    auto delta = nav.processGCodeCommand(cmd_g90);
    log_movement(0, nav, delta, "G90");
    nav.updatePosition(delta);
    
    auto move_abs = create_gcode_cmd('G', 0, 20.0f, 20.0f);
    delta = nav.processGCodeCommand(move_abs);
    log_movement(1, nav, delta, "G0 X20 Y20");
    nav.updatePosition(delta);
    
    // Switch to relative mode, make some relative moves
    auto cmd_g91 = create_gcode_cmd('G', 91);
    delta = nav.processGCodeCommand(cmd_g91);
    log_movement(2, nav, delta, "G91");
    nav.updatePosition(delta);
    
    // Relative moves
    struct {
        float x, y;
        const char* desc;
    } rel_moves[] = {
        {10.0f, 0.0f, "G1 X10"},
        {0.0f, 10.0f, "G1 Y10"},
        {-5.0f, -5.0f, "G1 X-5 Y-5"}
    };
    
    for (int i = 0; i < 3; i++) {
        auto move_cmd = create_gcode_cmd('G', 1, rel_moves[i].x, rel_moves[i].y);
        delta = nav.processGCodeCommand(move_cmd);
        log_movement(i + 3, nav, delta, rel_moves[i].desc);
        nav.updatePosition(delta);
    }
    
    // Back to absolute mode, go to origin
    auto cmd_g90_2 = create_gcode_cmd('G', 90);
    delta = nav.processGCodeCommand(cmd_g90_2);
    log_movement(6, nav, delta, "G90");
    nav.updatePosition(delta);
    
    auto move_home = create_gcode_cmd('G', 0, 0.0f, 0.0f);
    delta = nav.processGCodeCommand(move_home);
    log_movement(7, nav, delta, "G0 X0 Y0");
    nav.updatePosition(delta);
    
    analyze_path_deviation("mixed_modes");
    
    // Verify final position
    zassert_within(nav.getCurrentPosition().x, 0.0f, 0.001f, "Should be at X=0");
    zassert_within(nav.getCurrentPosition().y, 0.0f, 0.001f, "Should be at Y=0");
}

ZTEST(navigation, test_z_final_summary) {
    printk("Generating overall navigation accuracy summary...\n");
    print_overall_accuracy_summary();
}

ZTEST_SUITE(navigation, NULL, NULL, NULL, NULL, NULL);