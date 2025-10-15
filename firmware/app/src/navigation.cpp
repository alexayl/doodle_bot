#include <zephyr/kernel.h>
#include "comms.h"
#include "navigation.h"
#include <stdbool.h>
#include <stdio.h> // Use printf instead of iostream

void gcode_to_nav(const nav_instr_t &gcode_instr) {
    // TODO: Implement G-code to navigation instruction conversion
}

void nav_thread(void *nav_instr_queue, void *arg2, void *arg3) {

    auto *q = static_cast<k_msgq *>(nav_instr_queue);

    while(1) {
        nav_instr_t current_instruction;
        k_msgq_get(q, &current_instruction, K_NO_WAIT);
        
        // Use printf instead of iostream
        // printf("Nav thread received: x_delta=%d, y_delta=%d\n", 
        //        static_cast<int>(current_instruction.x_delta),
        //        static_cast<int>(current_instruction.y_delta));
        
        gcode_to_nav(current_instruction);

        print_nav_instr(current_instruction);

        k_msleep(1000);
    }

    return;
}

void send_nav_instr() {
    // TODO: Implement navigation instruction sending
}