#include <zephyr/kernel.h>
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

/* GLOBAL BLE SERVICE INSTANCE */
static BleService* g_bleService = nullptr;

/* THREAD FUNCTIONS */

// Handler that processes G-code and queues navigation instructions
void gcode_to_nav_handler(const void* data, uint16_t len, k_msgq *q) {

    printk("G-code handler processing %d bytes\n", len);

    bool failed = false;
    uint8_t received_packet_id = 0;
    
    // Parse received G-code - use static parser to maintain packet ID state
    static InstructionParser parser;
    InstructionParser::GCodeCmd cmd;
    
    // Store the packet ID that was expected before parsing
    uint8_t expected_packet_id = parser.packet_id;
    
    if (!parser.parseLine((const char*)data, cmd)) {
        printk("Handler: Failed to parse G-code\n");
        failed = true;
        received_packet_id = expected_packet_id; // Use expected since parsing failed
    } else {
        // Parsing succeeded, so packet ID was incremented. The received ID was (current - 1)
        received_packet_id = parser.packet_id - 1;
    }
    
    if (!failed && !InstructionParser::isSupported(cmd)) {
        printk("Handler: Unsupported G-code command\n");
        failed = true;
    }

    if (failed) {
        // Send NACK with packet ID as first byte
        if (g_bleService) {
            char nack[10];
            nack[0] = received_packet_id;  // Packet ID as first byte
            strcpy(&nack[1], "error\n");
            g_bleService->send(nack, strlen(&nack[1]) + 1);
        }
        return;
    }

    // Queue parsed instruction for navigation
    if (k_msgq_put(q, &cmd, K_NO_WAIT) != 0) {
        printk("Queue full, dropping message\n");
    }

    // Send ACK with packet ID as first byte
    if (g_bleService) {
        char ack[10];
        ack[0] = received_packet_id;  // Packet ID as first byte
        strcpy(&ack[1], "ok\n");
        g_bleService->send(ack, strlen(&ack[1]) + 1);
    }

}

void comms_thread(void *nav_instr_queue, void *arg2, void *arg3) {

    k_msgq *q = static_cast<k_msgq *>(nav_instr_queue);

    // Create BLE service with queue and handler
    BleService bleService(q, gcode_to_nav_handler);
    
    // Set global pointer for handler access
    g_bleService = &bleService;
    
    bleService.init();

    while(true) {
        const char *hello_world = "Hello World!\n";

        k_sleep(K_SECONDS(3));
        // bleService.send(hello_world, strlen(hello_world));

    }

    // Clear global pointer before exiting
    g_bleService = nullptr;
    return;
}