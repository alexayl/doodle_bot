#include <zephyr/kernel.h>
#include "navigation.h"
#include "instruction_parser.h"
#include "stepper.h"
#include "buzzer.h"
#include <simple_led.h>
#include <stdbool.h>
#include <stdio.h>
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

/* Navigation Class Implementation */

MovementDelta Navigation::processGCodeCommand(const InstructionParser::GCodeCmd& cmd) {
    MovementDelta delta;
    
    switch (cmd.code) {
        case 'G':
            switch (cmd.number) {
                case 0: // Rapid move (pen up)
                case 1: { // Linear move (pen down)
                    delta.pen_down = (cmd.number == 1);
                    
                    float target_x = current_position.x;
                    float target_y = current_position.y;
                    
                    // Parse X and Y coordinates from arguments
                    for (int i = 0; i < cmd.argc; i++) {
                        switch (cmd.args[i].letter) {
                            case 'X':
                                if (coordinate_mode == CoordinateMode::Absolute) {
                                    target_x = cmd.args[i].value;
                                } else {
                                    target_x = current_position.x + cmd.args[i].value;
                                }
                                break;
                            case 'Y':
                                if (coordinate_mode == CoordinateMode::Absolute) {
                                    target_y = cmd.args[i].value;
                                } else {
                                    target_y = current_position.y + cmd.args[i].value;
                                }
                                break;
                        }
                    }
                    
                    // Calculate movement delta
                    delta.dx = target_x - current_position.x;
                    delta.dy = target_y - current_position.y;
                    
                    // Update target position
                    target_position.x = target_x;
                    target_position.y = target_y;
                    
                    printk("Navigation: Move from (%.2f,%.2f) to (%.2f,%.2f), delta=(%.2f,%.2f)\n",
                           (double)current_position.x, (double)current_position.y,
                           (double)target_x, (double)target_y, (double)delta.dx, (double)delta.dy);
                    break;
                }
                case 90: // Absolute coordinates
                    setCoordinateMode(CoordinateMode::Absolute);
                    printk("Navigation: Switched to absolute coordinates\n");
                    break;
                case 91: // Relative coordinates  
                    setCoordinateMode(CoordinateMode::Relative);
                    printk("Navigation: Switched to relative coordinates\n");
                    break;
            }
            break;
            
        case 'M':
            switch (cmd.number) {
                case 280: { // Servo control (pen up/down)
                    for (int i = 0; i < cmd.argc; i++) {
                        if (cmd.args[i].letter == 'S') {
                            // Assuming S90 = pen down, S0 = pen up
                            bool new_pen_state = (cmd.args[i].value > 45.0f);
                            setPenDown(new_pen_state);
                            delta.pen_down = new_pen_state;
                            printk("Navigation: Pen %s (S%.0f)\n", 
                                   new_pen_state ? "DOWN" : "UP", (double)cmd.args[i].value);
                        }
                    }
                    break;
                }
            }
            break;
    }
    
    return delta;
}

void Navigation::updatePosition(const MovementDelta& delta) {
    // Update current position
    current_position.x += delta.dx;
    current_position.y += delta.dy;
    
    // Update pen state
    pen_is_down = delta.pen_down;
    
    // If there was movement, update heading to face movement direction
    if (delta.dx != 0.0f || delta.dy != 0.0f) {
        float movement_heading = atan2f(delta.dx, delta.dy) * 180.0f / M_PI;
        current_position.heading_degrees = normalize_angle(movement_heading);
    }
    
    printk("Navigation: Updated position to (%.2f,%.2f) @ %.1f°, pen %s\n",
           (double)current_position.x, (double)current_position.y, (double)current_position.heading_degrees,
           pen_is_down ? "DOWN" : "UP");
}

int hardware_init() {
    // steppers
    if (!stepper_init()) {
        return 1;
    }

    // servo
    // TODO: get init from jain
    // const struct device *servo_eraser = servo_init_by_alias

    // leds
    simple_led_init();
    // led_driver_set(led_command_t);

    // buzzer
    buzzer_init();
    
    return 0;
}

void nav_thread(void *nav_instr_queue, void *arg2, void *arg3) {

    auto *q = static_cast<k_msgq *>(nav_instr_queue);

    hardware_init();
    
    // Create navigation system
    Navigation navigation;
    
    printf("Navigation thread started\n");
    navigation.printStatus();

    while(1) {
        InstructionParser::GCodeCmd current_instruction;

        // Block until message arrives
        k_msgq_get(q, &current_instruction, K_FOREVER);

        printf("Nav thread received %c%d command\n", 
               current_instruction.code, current_instruction.number);

        // Process the G-code command through navigation system
        MovementDelta delta = navigation.processGCodeCommand(current_instruction);
        
        // Execute the movement (this is where you'd interface with hardware)
        if (delta.dx != 0.0f || delta.dy != 0.0f) {
            printf("Executing movement: dx=%.2f, dy=%.2f, pen=%s\n",
                   (double)delta.dx, (double)delta.dy, delta.pen_down ? "DOWN" : "UP");
                   
            // TODO: Interface with stepper motors
            // stepper_move_xy(delta.dx, delta.dy);
            
            // TODO: Control pen servo
            // if (delta.pen_down != navigation.isPenDown()) {
            //     servo_set_pen(delta.pen_down);
            // }
            
            // Simulate movement completion and update position
            navigation.updatePosition(delta);
        }
        
        // Handle coordinate mode changes
        switch(current_instruction.number) {
            case 90: // G90 - Absolute mode
            case 91: // G91 - Relative mode
                // Already handled in processGCodeCommand
                break;
        }
        
        // Print navigation status periodically
        navigation.printStatus();
    }

    return;
}
