#include <zephyr/kernel.h>
#include "state_task.h"
#include "state_machine.h"

// Event queue configuration
#define EVENT_QUEUE_SIZE 16
#define EVENT_QUEUE_ALIGN 4

// State names for logging
static const char* state_names[] = {
    "Init",
    "Actuate", 
    "Idle",
    "Error"
};

// Global event message queue
K_MSGQ_DEFINE(event_msgq, sizeof(Event), EVENT_QUEUE_SIZE, EVENT_QUEUE_ALIGN);

// Global state machine instance
static DoodleBotState state_machine;

void post_state_event(Event event) {
    int ret = k_msgq_put(&event_msgq, &event, K_NO_WAIT);
    if (ret != 0) {
        printk("Failed to post event %d to queue (ret: %d)\n", (int)event, ret);
    } else {
        printk("Posted event %d to state machine\n", (int)event);
    }
}

static void execute_command(Command cmd) {
    switch (cmd) {
        case Command::Initialize:
            printk("Executing: Initialize system\n");
            k_work_submit(&hardware_init_work);
            break;
            
        case Command::ProcessCommand:
            printk("Executing: Process command\n");
            // TODO: Add command processing logic here
            break;
            
        case Command::Sleep:
            printk("Executing: Enter sleep mode\n");
            // TODO: Add sleep/idle logic here
            break;
            
        case Command::HandleError:
            printk("Executing: Handle error condition\n");
            // TODO: Add error handling logic here
            break;
            
        case Command::None:
            printk("No command to execute\n");
            break;
            
        default:
            printk("Unknown command: %d\n", (int)cmd);
            break;
    }
}

void state_thread(void *arg1, void *arg2, void *arg3) {
    Event event;
    Command cmd;
        
    while (1) {
        // wait for event
        int ret = k_msgq_get(&event_msgq, &event, K_FOREVER);
        
        if (ret != 0) {
            printk("Failed to get event from queue (ret: %d)\n", ret);
            continue;
        }

        printk("Received event %d in state %d\n",
            (int)event, (int)state_machine.getCurrentState());
        
        cmd = state_machine.processEvent(event);
        
        printk("Processed event %d, new state: %d, command: %d\n",
            (int)event, (int)state_machine.getCurrentState(), (int)cmd);
        
        execute_command(cmd);
            
    
    }
}