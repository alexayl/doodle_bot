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

// Interpolation modes
enum InterpolationMode {
    INTERP_LINEAR = 0,        // Original point-to-point linear interpolation
    INTERP_CUBIC_HERMITE = 1  // Smooth cubic Hermite spline interpolation
};

// Waypoint for path interpolation
struct Waypoint {
    float x;
    float y;
};

// Maximum waypoints for look-ahead buffering (for Hermite interpolation)
#define MAX_WAYPOINTS 4

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
        , interp_mode_(INTERP_LINEAR)
        , pos_x_(0.0f)
        , pos_y_(0.0f)
        , waypoint_count_(0)
    {
        k_msgq_init(&nav_queue_, nav_queue_buffer_, sizeof(NavCommand), MESSAGES_PER_QUEUE);
    }

    Output consumeGcode(const InstructionParser::GCodeCmd &gcode_cmd);

    // Accessor for output commands
    const Output& output() const { return output_; }

    // Reset planner state
    void reset();

    // Set interpolation mode
    void setInterpolationMode(InterpolationMode mode) { interp_mode_ = mode; }
    InterpolationMode getInterpolationMode() const { return interp_mode_; }

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

    // interpolation state
    float theta_current_;       // current heading angle
    Output output_;
    InterpolationMode interp_mode_;
    
    // Position tracking for Hermite interpolation
    float pos_x_;               // current absolute X position
    float pos_y_;               // current absolute Y position
    
    // Waypoint buffer for look-ahead (needed for tangent calculation)
    Waypoint waypoint_buffer_[MAX_WAYPOINTS];
    int waypoint_count_;

    void clearOutput() { output_.count = 0; }
    void addToOutput(const ExecuteCommand& cmd) { 
        if (output_.count < MOTION_PLAN_OUTPUT_SIZE) {
            output_.cmds[output_.count++] = cmd; 
        }
    }

    // Linear interpolation (original)
    int interpolateLinear(int x_delta, int y_delta);
    
    // Cubic Hermite interpolation
    int interpolateHermite(int x_delta, int y_delta);
    
    // Main interpolation dispatcher
    int interpolate(int x_delta, int y_delta);      // G-code -> (r, theta) - reads from current_instruction_, writes to nav_queue
    int discretize(const NavCommand& nav_command);  // (r, theta) -> (vL, vR) - reads from nav_queue, writes to output_
};

void motion_plan_thread(void *, void *, void *);
