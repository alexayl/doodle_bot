#include <zephyr/kernel.h>
#include "navigation.h"
#include "instruction_parser.h"
#include "stepper.h"
#include "buzzer.h"
#include <simple_led.h>
#include <stdbool.h>
#include <stdio.h> // Use printf instead of iostream

void gcode_to_nav(const InstructionParser::GCodeCmd &gcode_instr) {
    // TODO: Implement G-code to navigation instruction conversion
}

int hardware_init() {
    // steppers
    if (!stepper_init()) {
        return 1;
    }

    // servo
    // TODO: get init from jain
    const struct device *servo_eraser = servo_init_by_alias

    // leds
    simple_led_init();
    // led_driver_set(led_command_t);

    // buzzer
    buzzer_init();
    

}

void nav_thread(void *nav_instr_queue, void *arg2, void *arg3) {

    auto *q = static_cast<k_msgq *>(nav_instr_queue);

    hardware_init();

    while(1) {
        InstructionParser::GCodeCmd current_instruction;

        // Block until message arrives
        k_msgq_get(q, &current_instruction, K_FOREVER);

        // sanity check
        switch(current_instruction.number) {
            case InstructionParser::G91:
                printf("Nav thread received G91 command\n");
                // TODO: Add support for G90 and G91
                break;
            case InstructionParser::G0:
            case InstructionParser::G1:
                printf("Nav thread received G0 command\n");
                // process stepper movement
                move_stepper()
                // process servo movement
                move_servo(current_instrution.);
                break;
            case InstructionParser::M280:
                printf("Nav thread received M280 command\n");
                break;
            default:
                printf("Nav thread received unknown command %c%d\n", current_instruction.code, current_instruction.number);
                break;
        }
        
        // gcode_to_nav(current_instruction);

        // print_nav_instr(current_instruction);

    }

    return;

}
