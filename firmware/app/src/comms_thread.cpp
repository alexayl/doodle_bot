#include <zephyr/kernel.h>
#include <zephyr/sys/errno_private.h>
#include "comms_thread.h"
#include "instruction_parser.h"
#include "ble_service.h"
#include "navigation.h"
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

//------------------
// THREAD
// -----------------

static BleService* g_bleService = nullptr;

/* THREAD FUNCTIONS */

// Handler that processes G-code and queues navigation instructions
void gcode_to_nav_handler(const void* data, uint16_t len, k_msgq *q) {

    int ret = 0;

    static InstructionParser parser;
    InstructionParser::GCodeCmd cmd;

    uint8_t expected_packet_id = parser.expected_packet_id;

    // parse the incoming G-code string into the GCodeCmd structure
    ret = parser.parseLine((const char*)data, cmd);

    // handle failed parse
    if (ret < 0) {
        printk("Handler: Failed to parse G-code\n");

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
    }

    // Send ACK with packet ID as first byte
    if (g_bleService) {
        char ack[sizeof("aok\n")] = "aok\n";
        ack[0] = expected_packet_id;
        #ifdef DEBUG_BLE
            printk("Sending ACK: %s (id:%d)\n", ack, (uint8_t)ack[0]);
        #endif
        g_bleService->send(ack, sizeof(ack));
    }

}

void comms_thread(void *nav_instr_queue_void, void *arg2, void *arg3) {

    k_msgq *q = static_cast<k_msgq *>(nav_instr_queue_void);

    BleService bleService(q, gcode_to_nav_handler);
    g_bleService = &bleService; // TODO: add ownership to this pointer
    
    bleService.init();

    while(true) {
        k_sleep(K_FOREVER);
    }

}