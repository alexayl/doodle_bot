#include <zephyr/ztest.h>
#include "navigation.h"
#include "instruction_parser.h"

/* QUEUE MANAGEMENT */

#define MESSAGES_PER_QUEUE 100

K_MSGQ_DEFINE(gcode_cmd_msgq, sizeof(InstructionParser::GCodeCmd), MESSAGES_PER_QUEUE, alignof(InstructionParser::GCodeCmd));
K_MSGQ_DEFINE(nav_cmd_msgq, sizeof(NavCommand), MESSAGES_PER_QUEUE, alignof(NavCommand));
K_MSGQ_DEFINE(step_cmd_msgq, sizeof(StepCommand), MESSAGES_PER_QUEUE, alignof(StepCommand));

/* THREAD DEFINITION AND MANAGEMENT */

#define STACK_SIZE      2048

#define COMMS_PRIORITY  1
#define NAV_PRIORITY    3
#define STATE_PRIORITY  1

K_THREAD_STACK_DEFINE(comms_stack, STACK_SIZE);
K_THREAD_STACK_DEFINE(nav_stack, STACK_SIZE);
K_THREAD_STACK_DEFINE(state_stack, STACK_SIZE);

static struct k_thread comms_thread_data;
static struct k_thread nav_thread_data;
static struct k_thread state_thread_data;

K_TIMER_DEFINE(motor_control_timer, MotionPlanner::motor_control_handler, NULL);

/* HELPER FUNCTIONS */

#define MAX_STEP_RECORDS 100

struct StepCommandRecord {
    int16_t left_velocity;   // signed velocity
    int16_t right_velocity;  // signed velocity
    uint32_t timestamp_ms;
};

static void collect_step_commands(StepCommandRecord records[], size_t* count, size_t max_count) {
    StepCommand step_cmd;
    uint32_t start_time = k_uptime_get_32();
    *count = 0;
    
    while (k_msgq_get(&step_cmd_msgq, &step_cmd, K_NO_WAIT) == 0 && *count < max_count) {
        records[*count].left_velocity = step_cmd.left_velocity;
        records[*count].right_velocity = step_cmd.right_velocity;
        records[*count].timestamp_ms = k_uptime_get_32() - start_time;
        (*count)++;
    }
}

static void print_step_commands_csv(const StepCommandRecord records[], size_t count, const char* test_name) {
    printk("\n=== CSV OUTPUT for %s ===\n", test_name);
    printk("timestamp_ms,left_velocity,right_velocity\n");
    for (size_t i = 0; i < count; i++) {
        printk("%u,%d,%d\n", records[i].timestamp_ms, records[i].left_velocity, records[i].right_velocity);
    }
    printk("=== END CSV ===\n\n");
}

static void send_gcode_and_wait(InstructionParser::GCodeCmd& cmd, int wait_ms = 100) {
    k_msgq_put(&gcode_cmd_msgq, &cmd, K_NO_WAIT);
    k_sleep(K_MSEC(wait_ms));
}

/* TEST CASES */

ZTEST(navigation_thread, test_straight_line_movements) {
    StepCommandRecord records[MAX_STEP_RECORDS];
    size_t count = 0;
    
    printk("Test: Straight line movements (positive and negative)\n");
    
    // Move right (positive X)
    InstructionParser::GCodeCmd cmd1 = { .code = 'G', .number = 1, .args = { {'X', 10.0f}, {'Y', 0.0f} }, .argc = 2 };
    send_gcode_and_wait(cmd1);
    
    // Move up (positive Y)
    InstructionParser::GCodeCmd cmd2 = { .code = 'G', .number = 1, .args = { {'X', 0.0f}, {'Y', 10.0f} }, .argc = 2 };
    send_gcode_and_wait(cmd2);
    
    // Move left (negative X)
    InstructionParser::GCodeCmd cmd3 = { .code = 'G', .number = 1, .args = { {'X', -10.0f}, {'Y', 0.0f} }, .argc = 2 };
    send_gcode_and_wait(cmd3);
    
    // Move down (negative Y)
    InstructionParser::GCodeCmd cmd4 = { .code = 'G', .number = 1, .args = { {'X', 0.0f}, {'Y', -10.0f} }, .argc = 2 };
    send_gcode_and_wait(cmd4);
    
    collect_step_commands(records, &count, MAX_STEP_RECORDS);
    print_step_commands_csv(records, count, "straight_line_movements");
    
    zassert_true(count > 0, "Should have generated step commands");
}

ZTEST(navigation_thread, test_diagonal_movements) {
    StepCommandRecord records[MAX_STEP_RECORDS];
    size_t count = 0;
    
    printk("Test: Diagonal movements\n");
    
    // Move diagonally (45 degrees)
    InstructionParser::GCodeCmd cmd1 = { .code = 'G', .number = 1, .args = { {'X', 10.0f}, {'Y', 10.0f} }, .argc = 2 };
    send_gcode_and_wait(cmd1);
    
    // Move diagonally (-45 degrees)
    InstructionParser::GCodeCmd cmd2 = { .code = 'G', .number = 1, .args = { {'X', 10.0f}, {'Y', -10.0f} }, .argc = 2 };
    send_gcode_and_wait(cmd2);
    
    // Move diagonally (135 degrees)
    InstructionParser::GCodeCmd cmd3 = { .code = 'G', .number = 1, .args = { {'X', -10.0f}, {'Y', 10.0f} }, .argc = 2 };
    send_gcode_and_wait(cmd3);
    
    collect_step_commands(records, &count, MAX_STEP_RECORDS);
    print_step_commands_csv(records, count, "diagonal_movements");
    
    zassert_true(count > 0, "Should have generated step commands");
}

ZTEST(navigation_thread, test_square_path) {
    StepCommandRecord records[MAX_STEP_RECORDS];
    size_t count = 0;
    
    printk("Test: Square path (4 corners)\n");
    
    // Draw a square
    InstructionParser::GCodeCmd cmd1 = { .code = 'G', .number = 1, .args = { {'X', 20.0f}, {'Y', 0.0f} }, .argc = 2 };
    InstructionParser::GCodeCmd cmd2 = { .code = 'G', .number = 1, .args = { {'X', 0.0f}, {'Y', 20.0f} }, .argc = 2 };
    InstructionParser::GCodeCmd cmd3 = { .code = 'G', .number = 1, .args = { {'X', -20.0f}, {'Y', 0.0f} }, .argc = 2 };
    InstructionParser::GCodeCmd cmd4 = { .code = 'G', .number = 1, .args = { {'X', 0.0f}, {'Y', -20.0f} }, .argc = 2 };
    
    send_gcode_and_wait(cmd1);
    send_gcode_and_wait(cmd2);
    send_gcode_and_wait(cmd3);
    send_gcode_and_wait(cmd4);
    
    collect_step_commands(records, &count, MAX_STEP_RECORDS);
    print_step_commands_csv(records, count, "square_path");
    
    // Note: Due to timing or queue state, we may get commands from state changes
    zassert_true(count >= 4, "Should have at least 4 step commands for square");
}

ZTEST(navigation_thread, test_small_movements) {
    StepCommandRecord records[MAX_STEP_RECORDS];
    size_t count = 0;
    
    printk("Test: Small movements (precision test)\n");
    
    // Very small movements
    InstructionParser::GCodeCmd cmd1 = { .code = 'G', .number = 1, .args = { {'X', 1.0f}, {'Y', 0.0f} }, .argc = 2 };
    InstructionParser::GCodeCmd cmd2 = { .code = 'G', .number = 1, .args = { {'X', 0.0f}, {'Y', 1.0f} }, .argc = 2 };
    InstructionParser::GCodeCmd cmd3 = { .code = 'G', .number = 1, .args = { {'X', 0.5f}, {'Y', 0.5f} }, .argc = 2 };
    
    send_gcode_and_wait(cmd1);
    send_gcode_and_wait(cmd2);
    send_gcode_and_wait(cmd3);
    
    collect_step_commands(records, &count, MAX_STEP_RECORDS);
    print_step_commands_csv(records, count, "small_movements");
    
    zassert_true(count > 0, "Should have generated step commands");
}

ZTEST(navigation_thread, test_large_movements) {
    StepCommandRecord records[MAX_STEP_RECORDS];
    size_t count = 0;
    
    printk("Test: Large movements\n");
    
    // Large movements
    InstructionParser::GCodeCmd cmd1 = { .code = 'G', .number = 1, .args = { {'X', 100.0f}, {'Y', 0.0f} }, .argc = 2 };
    InstructionParser::GCodeCmd cmd2 = { .code = 'G', .number = 1, .args = { {'X', 0.0f}, {'Y', 100.0f} }, .argc = 2 };
    InstructionParser::GCodeCmd cmd3 = { .code = 'G', .number = 1, .args = { {'X', 100.0f}, {'Y', 100.0f} }, .argc = 2 };
    
    send_gcode_and_wait(cmd1);
    send_gcode_and_wait(cmd2);
    send_gcode_and_wait(cmd3);
    
    collect_step_commands(records, &count, MAX_STEP_RECORDS);
    print_step_commands_csv(records, count, "large_movements");
    
    zassert_true(count > 0, "Should have generated step commands");
}

/* SETUP AND TEARDOWN */

static void *navigation_setup(void) {
    // Start the navigation thread once for all tests
    k_thread_create(&nav_thread_data, nav_stack, STACK_SIZE,
                    nav_thread, &gcode_cmd_msgq, &nav_cmd_msgq, &step_cmd_msgq,
                    NAV_PRIORITY, 0, K_NO_WAIT);
    return NULL;
}

static void navigation_before(void *fixture) {
    // Clear queues before each test
    k_msgq_purge(&gcode_cmd_msgq);
    k_msgq_purge(&nav_cmd_msgq);
    k_msgq_purge(&step_cmd_msgq);
    
    // Reset robot state for fresh test
    // Note: We'd need to expose theta_current or add a reset command
    // For now, tests should account for accumulated state
}

ZTEST_SUITE(navigation_thread, NULL, navigation_setup, navigation_before, NULL, NULL);