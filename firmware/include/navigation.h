#pragma once

#include <stdint.h>
#include <zephyr/sys/printk.h>


/* Data types */

enum class PeripheralPosition : uint8_t {Down, Up};

enum class StepperDirection : uint8_t {Clockwise, CounterClockwise};

struct nav_instr_t {
    uint8_t x_delta, y_delta;
    PeripheralPosition eraser_position, marker_position; 
};

struct stepper_instr_t {
    uint8_t steps, velocity;
    StepperDirection stepper_direction; 
};

inline void print_nav_instr(const nav_instr_t& instr) {
    printk("x_delta: %d, y_delta: %d, eraser_position: %s, marker_position: %s\n",
           static_cast<int>(instr.x_delta),
           static_cast<int>(instr.y_delta),
           (instr.eraser_position == PeripheralPosition::Up ? "Up" : "Down"),
           (instr.marker_position == PeripheralPosition::Up ? "Up" : "Down"));
}

/* Functions */

void nav_thread(void *, void *, void *);
