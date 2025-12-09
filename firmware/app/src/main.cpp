#include <zephyr/kernel.h>
#include <stdlib.h>
#include <zephyr/sys/printk.h>

#include "instruction_parser.h"
#include "comms_thread.h"
#include "motion_plan.h"
#include "motion_execute.h"
#include "state_task.h"
#include "config.h"


/* QUEUE MANAGEMENT */
K_MSGQ_DEFINE(gcode_cmd_msgq, sizeof(InstructionParser::GCodeCmd), MESSAGES_PER_QUEUE, alignof(InstructionParser::GCodeCmd));
K_MSGQ_DEFINE(execute_cmd_msgq, sizeof(ExecuteCommand), MESSAGES_PER_QUEUE, alignof(ExecuteCommand));


/* THREAD DEFINITION AND MANAGEMENT */

#define STACK_SIZE      1024*8

#define COMMS_PRIORITY  2
#define NAV_PRIORITY    1
#define STATE_PRIORITY  0

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
    // Servo, LED, Buzzer initialization is handled by their respective wrappers
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
                    NAV_PRIORITY, 0, K_NO_WAIT);
    
    k_thread_create(&motion_execute_thread_data, motion_execute_thread_stack, STACK_SIZE,
                    motion_execute_thread, &execute_cmd_msgq, NULL, NULL,
                    NAV_PRIORITY, 0, K_NO_WAIT);


    k_thread_create(&state_thread_data, state_thread_stack, STACK_SIZE,
                    state_thread, NULL, NULL, NULL,
                    STATE_PRIORITY, 0, K_NO_WAIT);

    return EXIT_SUCCESS;
}