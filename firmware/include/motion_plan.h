#pragma once

#include "instruction_parser.h"
#include "peripheral_wrappers.h"
#include "motion_execute.h"
#include <zephyr/kernel.h>
#include "config.h"


#define PI                      (3.14159265359f)

// MAX_LINEAR_VELOCITY is defined in config.h

#define DEG_TO_RAD(deg)         ((deg) * PI / 180.0f)
#define RAD_TO_DEG(rad)         ((rad) * 180.0f / PI)

struct NavCommand {
    float r;          // distance to travel
    float theta;      // angle to change
};


/* Functions */

class MotionPlanner {
public:
    /* TYPES */
    struct Output {
        ExecuteCommand cmds[MOTION_PLAN_OUTPUT_SIZE];
        size_t count;
        
        Output() : count(0) {}
    };

    /* METHODS */
    MotionPlanner() 
        : current_gcode_cmd_{}
        , current_packet_id_(0)
        , theta_current_(0.0f)
        , output_{}
    {
        k_msgq_init(&nav_queue_, nav_queue_buffer_, sizeof(NavCommand), MESSAGES_PER_QUEUE);
    }

    Output consumeGcode(const InstructionParser::GCodeCmd &gcode_cmd);

    // Accessor for output commands
    const Output& output() const { return output_; }

    // Reset planner state
    void reset();

private:
    // message queues
    char nav_queue_buffer_[MESSAGES_PER_QUEUE * sizeof(NavCommand)] __aligned(4);
    struct k_msgq nav_queue_;

    // consume
    InstructionParser::GCodeCmd current_gcode_cmd_;
    uint8_t current_packet_id_;

    void consumeLocomotion();
    void consumeMarker();
    void consumeEraser();
    void consumeConfig();

    // interpolation
    float theta_current_;
    Output output_;

    void clearOutput() { output_.count = 0; }
    void addToOutput(const ExecuteCommand& cmd) { 
        if (output_.count < MOTION_PLAN_OUTPUT_SIZE) {
            output_.cmds[output_.count++] = cmd; 
        }
    }

    int interpolate(int x_delta, int y_delta);      // G-code -> (r, theta) - reads from current_instruction_, writes to nav_queue
    int discretize(const NavCommand& nav_command);  // (r, theta) -> (vL, vR) - reads from nav_queue, writes to output_
};

void motion_plan_thread(void *, void *, void *);
