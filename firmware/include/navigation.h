#pragma once

#include "instruction_parser.h"
#include "peripheral_wrappers.h"

#define DEBUG_NAV

#define PI 3.14159265359
#define WHEEL_RADIUS            (-1)
#define DOODLEBOT_RADIUS        (-1)
#define STEPPER_CTRL_FREQ       (20) // occurrences / SEC
#define STEPPER_CTRL_PERIOD     (1 / STEPPER_CTRL_FREQ)  

extern k_timer motor_control_timer;


struct NavCommand {
    float r;          // distance to travel
    float theta;      // angle to change
};

struct StepCommand {
    uint8_t left_velocity;
    uint8_t right_velocity;
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
