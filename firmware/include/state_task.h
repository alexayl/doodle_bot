#pragma once

#include <zephyr/kernel.h>
#include "state_machine.h"

/**
 * @brief Hardware initialization work item (defined in main.cpp)
 */
extern struct k_work hardware_init_work;

/**
 * @brief Post an event to the state machine thread.
 * 
 * This function is thread-safe and can be called from any context.
 * 
 * @param event Event to post to the state machine.
 */
void post_state_event(Event event);

/**
 * @brief State machine thread entry point.
 * 
 * This thread waits for events on a message queue and processes them
 * through the state machine, then dispatches the resulting commands.
 */
void state_thread(void *arg1, void *arg2, void *arg3);
