#include <zephyr/kernel.h>
#include "comms.h"
#include "navigation.h"
#include <stdbool.h>
#include <iostream>

gcode_to_nav(const nav_instr_t &gcode_instr) {
    
}

void nav_thread(void *nav_instr_queue, void *arg2, void *arg3) {

    auto *q = static_cast<k_msgq *>(nav_instr_queue);

    while(1) {
        nav_instr_t current_instruction;
        k_msgq_get(q, &current_instruction, K_NO_WAIT);
        std::cout << "Nav thread received: " << current_instruction << std::endl;
        
        gcode_to_nav(current_instruction);

        send_nav_instr();

         

        k_msleep(1000);
    }

    return;
}