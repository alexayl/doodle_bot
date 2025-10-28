#include <zephyr/kernel.h>
#include <stdlib.h>
#include <zephyr/sys/printk.h>

#include "instruction_parser.h"
#include "comms_thread.h"
#include "navigation.h"
#include "state_task.h"

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

    
/* HARDWARE INITIALIZATION */

static int hardware_init() {
   
    // TODO: init hardware peripherals as they are added
    
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
    k_thread_create(&comms_thread_data, comms_stack, STACK_SIZE,
                    comms_thread, &gcode_cmd_msgq, NULL, NULL,
                    COMMS_PRIORITY, 0, K_NO_WAIT);

    k_thread_create(&nav_thread_data, nav_stack, STACK_SIZE,
                    nav_thread, &gcode_cmd_msgq, &nav_cmd_msgq, &step_cmd_msgq,
                    NAV_PRIORITY, 0, K_NO_WAIT);

    k_thread_create(&state_thread_data, state_stack, STACK_SIZE,
                    state_thread, NULL, NULL, NULL,
                    STATE_PRIORITY, 0, K_NO_WAIT);

    return EXIT_SUCCESS;
}