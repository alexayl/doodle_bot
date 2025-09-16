#pragma once

#include <stdint.h>
#include <ostream>


/* Data types */

enum class PeripheralPosition : bool {Down, Up};

struct nav_instr_t {
    uint8_t x_delta, y_delta;
    PeripheralPosition eraser_position, marker_position; 
};

inline std::ostream& operator<<(std::ostream& os, const nav_instr_t& instr) {
    os << "x_delta: " << static_cast<int>(instr.x_delta)
       << ", y_delta: " << static_cast<int>(instr.y_delta)
       << ", eraser_position: " << (instr.eraser_position == PeripheralPosition::Up ? "Up" : "Down")
       << ", marker_position: " << (instr.marker_position == PeripheralPosition::Up ? "Up" : "Down");
    return os;
}

/* Functions */

void nav_thread(void *, void *, void *);
