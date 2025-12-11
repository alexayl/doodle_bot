#include <zephyr/kernel.h>
#include <zephyr/sys/errno_private.h>
#include "comms_thread.h"
#include "instruction_parser.h"
#include "ble_service.h"
#include "motion_plan.h"
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

// Global BLE service pointer
BleService* g_bleService = nullptr;

// Static parser instance (file-level so it can be reset)
static InstructionParser g_parser;

void comms_reset() {
    g_parser.reset();
    printk("COMMS::INFO: Parser state reset (packet ID = 0)\n");
}

//------------------
// THREAD
// -----------------

/* THREAD FUNCTIONS */

// Handler that processes G-code and queues navigation instructions
void gcode_to_nav_handler(const void* data, uint16_t len, k_msgq *q) {

    int ret = 0;

    #ifdef DEBUG_BLE
    printk("COMMS_HANDLER: Received data of length %d: ", len);
    for (int i = 0; i < len; i++) {
        printk("%02X ", ((uint8_t*)data)[i]);
    }
    printk("\n");
    printk("COMMS_HANDLER: As string: %.*s\n", len, (const char*)data);
    #endif

    InstructionParser::GCodeCmd cmd;

    uint8_t expected_packet_id = g_parser.expected_packet_id;

    // Let the parser handle packet ID validation as it was designed
    ret = g_parser.parseLine((const char*)data, cmd);

    // handle failed parse
    if (ret < 0) {
        printk("COMMS_HANDLER::ERROR: failed to parse G-code\n");

        // Send NACK
        char nack[sizeof("afail\n")] = "afail\n";
        nack[0] = expected_packet_id;
        #ifdef DEBUG_BLE
            printk("Sending NACK: %s (id:%d)\n", nack, (uint8_t)nack[0]);
        #endif
        g_bleService->send(nack, sizeof(nack));
        return;
    }

    // Queue parsed instruction for navigation
    if (k_msgq_put(q, &cmd, K_NO_WAIT) != 0) {
        printk("Queue full, dropping message\n");
        return;
    }

    // NOTE: ACK is sent by motion_executor when command execution completes,
    // not here. This prevents duplicate ACKs that can overwhelm the Python client.
}

void comms_thread(void *gcode_cmd_msgq_void, void *arg2, void *arg3) {

    k_msgq *q = static_cast<k_msgq *>(gcode_cmd_msgq_void);

    BleService bleService(q, gcode_to_nav_handler);
    g_bleService = &bleService; // TODO: add ownership to this pointer
    
    bleService.init();

    while(true) {
        k_sleep(K_FOREVER);
    }

}