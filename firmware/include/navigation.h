#pragma once

#include "instruction_parser.h"
#include "peripheral_wrappers.h"

#define DEBUG_NAV

#define PI 3.14159265359
#define WHEEL_RADIUS            (10) // radius of the wheels on each stepper motor
#define DOODLEBOT_RADIUS        (20) // distance from center of doodlebot to wheel
#define STEPPER_CTRL_FREQ       (20) // occurrences / SEC
#define STEPPER_CTRL_PERIOD     (1 / STEPPER_CTRL_FREQ) // seconds per occurrence

extern k_timer motor_control_timer;


struct NavCommand {
    float r;          // distance to travel
    float theta;      // angle to change
};

struct StepCommand {
    int16_t left_velocity;   // signed velocity: positive = forward, negative = backward
    int16_t right_velocity;  // signed velocity: positive = forward, negative = backward
};

/* Functions */

class InstructionHandler {
public:
    virtual int consumeInstruction(const InstructionParser::GCodeCmd &) = 0;

protected:
    static InstructionParser::GCodeCmd current_instruction_;
};

class MotionPlanner : public InstructionHandler {
public:
    /* TYPES */

    /* METHODS */
    MotionPlanner(k_msgq *nav_queue, k_msgq *step_queue) {
        nav_queue_ = nav_queue;
        step_queue_ = step_queue;
    };
    int consumeInstruction(const InstructionParser::GCodeCmd &) override;

    static void motor_control_handler(k_timer *timer);
    static void reset_state();  // Reset robot state (heading, position, etc.)

private:
    static k_msgq *nav_queue_;   // r, theta
    static k_msgq *step_queue_;  // vL, vR

    // interpolation
    static float theta_current;

    int interpolate();          // G-code -> (r, theta)
    int discretize();           // (r, theta) -> (vL, vR)
};

class ServoMover : public InstructionHandler {
public:
    int consumeInstruction(const InstructionParser::GCodeCmd &) override;

protected:
    Servo servo_;
};

void nav_thread(void *, void *, void *);
