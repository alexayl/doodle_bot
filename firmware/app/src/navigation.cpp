#include <zephyr/kernel.h>
#include "navigation.h"
#include "instruction_parser.h"

int Navigator::consumeInstruction() {
    // TODO: handle new nav instruction

    return 0;
}

int PeripheralMover::consumeInstruction() {
    // TODO: handler new peripheral instruction

    return 0;
}

void nav_thread(void *nav_instr_queue, void *arg2, void *arg3) {

    auto *instruction_queue = static_cast<k_msgq *>(nav_instr_queue);
    InstructionParser::GCodeCmd current_instruction;

    while(1) {
        // Block until message arrives
        k_msgq_get(instruction_queue, &current_instruction, K_FOREVER);
        
        #ifdef DEBUG_NAV
        printk("Nav thread received %c%d command\n", current_instruction.code, current_instruction.number);
        #endif

        char code = current_instruction.code;
        int num = current_instruction.number;

        Navigator navigator;
        PeripheralMover marker;
        PeripheralMover eraser;

        // command router
        if(code == 'G' && num == '1') {
            navigator.consumeInstruction();

        } else if (code == 'M' && num == '280') {
            if (current_instruction.args[0].letter == 'P' && current_instruction.args[0].value == 0) {
                marker.consumeInstruction();
                
            } else if (current_instruction.args[0].letter == 'P' && current_instruction.args[0].value == 1) {
                eraser.consumeInstruction();

            } else {
                printk("Unhandled servo command: %c %d\n", current_instruction.args[0].letter, current_instruction.args[0].value);
            }

        } else {
            printk("Unhandled command: %c %d\n", code, num);
        }
        
    }

    return;
}
