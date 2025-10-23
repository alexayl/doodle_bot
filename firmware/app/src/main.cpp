#include <zephyr/kernel.h>
#include <stdlib.h>
#include <zephyr/sys/printk.h>

#include "instruction_parser.h"
#include "comms_thread.h"
#include "navigation.h"
#include "state_task.h"

/* QUEUE MANAGEMENT */

#define MESSAGES_PER_QUEUE 5

K_MSGQ_DEFINE(nav_instr_queue, sizeof(InstructionParser::GCodeCmd), MESSAGES_PER_QUEUE, alignof(InstructionParser::GCodeCmd));

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

int main(void) {
    
    /* Start threads */
    k_thread_create(&comms_thread_data, comms_stack, STACK_SIZE,
                    comms_thread, &nav_instr_queue, NULL, NULL,
                    COMMS_PRIORITY, 0, K_NO_WAIT);

    k_thread_create(&nav_thread_data, nav_stack, STACK_SIZE,
                    nav_thread, &nav_instr_queue, NULL, NULL,
                    NAV_PRIORITY, 0, K_NO_WAIT);

    k_thread_create(&state_thread_data, state_stack, STACK_SIZE,
                    state_thread, NULL, NULL, NULL,
                    STATE_PRIORITY, 0, K_NO_WAIT);


    return EXIT_SUCCESS;
}