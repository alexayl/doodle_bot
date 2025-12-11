#include <zephyr/kernel.h>
#include <stdlib.h>
#include <zephyr/sys/printk.h>

#include "instruction_parser.h"
#include "comms_thread.h"
#include "motion_plan.h"
#include "motion_execute.h"
#include "state_task.h"
#include "config.h"
#include "pwm_buzzer.h"


/* QUEUE MANAGEMENT */
K_MSGQ_DEFINE(gcode_cmd_msgq, sizeof(InstructionParser::GCodeCmd), MESSAGES_PER_QUEUE, alignof(InstructionParser::GCodeCmd));
// Execute queue only needs to buffer between threads, not hold entire motion plan output
#define EXECUTE_QUEUE_SIZE 128
K_MSGQ_DEFINE(execute_cmd_msgq, sizeof(ExecuteCommand), EXECUTE_QUEUE_SIZE, alignof(ExecuteCommand));


/* THREAD DEFINITION AND MANAGEMENT */

#define STACK_SIZE 1024*8

#define MOTION_EXECUTE_PRIORITY  0  // Highest priority for real-time motion
#define STATE_PRIORITY           1
#define MOTION_PLAN_PRIORITY     2
#define COMMS_PRIORITY           3

K_THREAD_STACK_DEFINE(comms_thread_stack, STACK_SIZE);
K_THREAD_STACK_DEFINE(motion_plan_thread_stack, STACK_SIZE);
K_THREAD_STACK_DEFINE(motion_execute_thread_stack, STACK_SIZE);
K_THREAD_STACK_DEFINE(state_thread_stack, STACK_SIZE);

static struct k_thread comms_thread_data;
static struct k_thread motion_plan_thread_data;
static struct k_thread motion_execute_thread_data;
static struct k_thread state_thread_data;

    
/* HARDWARE INITIALIZATION */

static int hardware_init() {
    // Stepper motors are auto-initialized via Zephyr's device model (devicetree)
    // Servo, LED initialization is handled by their respective wrappers
    
    // Initialize buzzer early so it's ready for BLE connect/disconnect events
    int ret = pwm_buzzer_init();
    if (ret < 0) {
        printk("WARNING: Buzzer initialization failed: %d\n", ret);
    }
    
    printk("Hardware initialized successfully\n");
    return 0;
}

/* MAIN FUNCTION */

int main(void) {

    int ret = hardware_init();
    if (ret < 0)
    {
        printk("ERROR: Hardware initialization failed\n");
    }
    
    
    /* Start threads */
    k_thread_create(&comms_thread_data, comms_thread_stack, STACK_SIZE,
                    comms_thread, &gcode_cmd_msgq, NULL, NULL,
                    COMMS_PRIORITY, 0, K_NO_WAIT);

    k_thread_create(&motion_plan_thread_data, motion_plan_thread_stack, STACK_SIZE,
                    motion_plan_thread, &gcode_cmd_msgq, &execute_cmd_msgq, NULL,
                    MOTION_PLAN_PRIORITY, 0, K_NO_WAIT);
    
    k_thread_create(&motion_execute_thread_data, motion_execute_thread_stack, STACK_SIZE,
                    motion_execute_thread, &execute_cmd_msgq, NULL, NULL,
                    MOTION_EXECUTE_PRIORITY, 0, K_NO_WAIT);


    k_thread_create(&state_thread_data, state_thread_stack, STACK_SIZE,
                    state_thread, NULL, NULL, NULL,
                    STATE_PRIORITY, 0, K_NO_WAIT);

    return EXIT_SUCCESS;
}