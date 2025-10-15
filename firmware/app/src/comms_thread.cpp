#include <zephyr/kernel.h>
#include "comms_thread.h"
#include "instruction_parser.h"
#include "ble_service.h"
#include "navigation.h"
#include <stdbool.h>
#include <stdio.h>

//------------------
// THREAD
// -----------------

/* THREAD FUNCTIONS */

// Handler that processes G-code and queues navigation instructions
void gcode_to_nav_handler(const void* data, uint16_t len, k_msgq *q) {

    printk("G-code handler processing %d bytes\n", len);
    
    // Parse received G-code
    InstructionParser parser;
    InstructionParser::GCodeCmd cmd;
    
    if (!parser.parseLine((const char*)data, cmd)) {
        printk("Handler: Failed to parse G-code\n");
        return;
    }
    
    if (!InstructionParser::isSupported(cmd)) {
        printk("Handler: Unsupported G-code command\n");
        return;
    }

    printk("Handler: Processing G-code %c%d\n", cmd.code, cmd.number);

    // Queue parsed instruction for navigation
    if (k_msgq_put(q, &cmd, K_NO_WAIT) != 0) {
        printk("Queue full, dropping message\n");
    }

}

void comms_thread(void *nav_instr_queue, void *arg2, void *arg3) {

    k_msgq *q = static_cast<k_msgq *>(nav_instr_queue);

    // Create BLE service with queue and handler
    BleService bleService(q, gcode_to_nav_handler);
    bleService.init();

    while(true) {
        const char *hello_world = "Hello World!\n";

        k_sleep(K_SECONDS(3));
        // bleService.send(hello_world, strlen(hello_world));

    }


    return;
}