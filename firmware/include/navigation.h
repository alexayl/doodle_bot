#pragma once

#include "instruction_parser.h"
#include "peripheral_wrappers.h"
#include <zephyr/kernel.h>

// #define DEBUG_NAV

#define PI                      (3.14159265359)
#define WHEEL_RADIUS            (10.0) // radius of the wheels on each stepper motor
#define DOODLEBOT_RADIUS        (50.0) // distance from center of doodlebot to wheel
#define STEPPER_CTRL_FREQ       (5.0) // occurrences / SEC
#define STEPPER_CTRL_PERIOD     (1.0 / STEPPER_CTRL_FREQ) // seconds per occurrence

#define STEPPER_MAX_VELOCITY   (150.0f) // degrees per second

struct NavCommand {
    float r;          // distance to travel
    float theta;      // angle to change
};

struct StepCommand {
    int16_t left_velocity;   // signed velocity: positive = forward, negative = backward
    int16_t right_velocity;  // signed velocity: positive = forward, negative = backward

    void print() const {
        printk("StepCommand: left_velocity=%d, right_velocity=%d\n", left_velocity, right_velocity);
    }
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
        // Initialize steppers only once
        static bool initialized = false;
        if (!initialized) {
            stepper_left_.initialize();
            stepper_right_.initialize();
            initialized = true;
        }
    };
    int consumeInstruction(const InstructionParser::GCodeCmd &) override;

    static void motor_control_handler(k_timer *timer);
    static void reset_state();  // Reset robot state (heading, position, etc.)

    // Public static members for work handler access
    static k_msgq *nav_queue_;   // r, theta
    static k_msgq *step_queue_;  // vL, vR
    static Stepper stepper_left_;
    static Stepper stepper_right_;

private:

    // interpolation
    static float theta_current;

    int interpolate();          // G-code -> (r, theta)
    int discretize();           // (r, theta) -> (vL, vR)
};

// External timer declaration
extern struct k_timer motor_control_timer;

class ServoMover : public InstructionHandler {
public:
    ServoMover(const char* servo_alias) : servo_(servo_alias) {}
    int consumeInstruction(const InstructionParser::GCodeCmd &) override;

protected:
    Servo servo_;
};

void nav_thread(void *, void *, void *);
