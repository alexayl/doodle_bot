#pragma once

#include "instruction_parser.h"
#include "peripheral_wrappers.h"
#include <zephyr/kernel.h>
#include "config.h"


#define PI                      (3.14159265359f)

#define STEPPER_CTRL_PERIOD     (1.0f / STEPPER_CTRL_FREQ) // seconds per occurrence

// MAX_LINEAR_VELOCITY is defined in config.h

#define DEG_TO_RAD(deg)         ((deg) * PI / 180.0f)
#define RAD_TO_DEG(rad)         ((rad) * 180.0f / PI)

struct NavCommand {
    float r;          // distance to travel
    float theta;      // angle to change
};

// signed velocity command for each stepper motor deg/s
struct StepCommand {
    uint8_t packet_id;
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
    int interpolate();          // G-code -> (r, theta) - reads from current_instruction_, writes to nav_queue
    int discretize();           // (r, theta) -> (vL, vR) - reads from nav_queue, writes to step_queue
    
    // Pure computation functions for testing
    static int interpolate(int x_delta, int y_delta,
                          float& theta_current,
                          void (*output_fn)(const NavCommand&));
    static int discretize(const NavCommand& nav_command,
                         uint8_t packet_id,
                         void (*output_fn)(const StepCommand&));
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
