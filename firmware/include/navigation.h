#pragma once

#include "instruction_parser.h"
#include "peripheral_wrappers.h"
#include <zephyr/kernel.h>

#define DEBUG_NAV


#define PI                      (3.14159265359f)

#define WHEEL_RADIUS            (48.0f)    // radius of the wheels on each stepper motor
#define DOODLEBOT_RADIUS        (165.0f)   // distance from center of doodlebot to wheel

#define STEPPER_CTRL_FREQ       (5.0f)     // control frequency (Hz)
#define STEPPER_CTRL_PERIOD     (1.0f / STEPPER_CTRL_FREQ) // seconds per occurrence

#define MAX_LINEAR_VELOCITY     (200.0f) // maximum linear velocity (mm/s)

#define DEG_TO_RAD(deg)         ((deg) * PI / 180.0f)
#define RAD_TO_DEG(rad)         ((rad) * 180.0f / PI)

struct NavCommand {
    float r;          // distance to travel
    float theta;      // angle to change
};

// signed velocity command for each stepper motor deg/s
struct StepCommand {
    int16_t left_velocity;
    int16_t right_velocity;

    void print() const {
        printk("StepCommand: left_velocity=%d, right_velocity=%d\n", left_velocity, right_velocity);
    }
};

/* Functions */

class InstructionHandler {
public:
    virtual int consumeInstruction(const InstructionParser::GCodeCmd &) = 0;
    
    // For testing purposes
    static void set_current_instruction(const InstructionParser::GCodeCmd &cmd) {
        current_instruction_ = cmd;
    }

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
    static float theta_current_;

public: // Made public for testing
    int interpolate();          // G-code -> (r, theta)
    int discretize();           // (r, theta) -> (vL, vR)
private:
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
