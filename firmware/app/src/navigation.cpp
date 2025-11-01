#include <zephyr/kernel.h>
#include <math.h>
#include "navigation.h"
#include "instruction_parser.h"

/* STATIC MEMBERS */
InstructionParser::GCodeCmd InstructionHandler::current_instruction_;
float MotionPlanner::theta_current_ = 0.0f;

k_msgq* MotionPlanner::nav_queue_ = nullptr;
k_msgq* MotionPlanner::step_queue_ = nullptr;

Stepper MotionPlanner::stepper_left_(STEPPER_LEFT);
Stepper MotionPlanner::stepper_right_(STEPPER_RIGHT);

static void motor_control_work_handler(struct k_work *work);
K_WORK_DEFINE(motor_control_work, motor_control_work_handler);

void motor_control_timer_handler(k_timer *timer) {
    k_work_submit(&motor_control_work);
}
K_TIMER_DEFINE(motor_control_timer, motor_control_timer_handler, NULL);

/* MOTION PLANNER */

int MotionPlanner::interpolate() {

    // See docs/firmware/differential_drive_kinematics.md for derivation

    int x_delta = current_instruction_.args[0].value;
    int y_delta = current_instruction_.args[1].value;
    
    float r_delta = sqrtf((float)(x_delta * x_delta + y_delta * y_delta));

    float theta_prime = atan2f((float)y_delta, (float)x_delta);
    float theta_delta = theta_prime - theta_current_;

    // normalize angle to (-pi, pi) range
    while (theta_delta > PI) {
        theta_delta -= 2.0f * PI;
    }
    while (theta_delta < -PI) {
        theta_delta += 2.0f * PI;
    }
    
    // update current heading state
    theta_current_ = theta_prime;
    
    // turn
    if (fabsf(theta_delta) > 0.01f) {
        NavCommand turn_command = {0.0f, theta_delta};
        k_msgq_put(nav_queue_, &turn_command, K_FOREVER);
    }
    
    // forward movement
    if (r_delta > 0.01f) { 
        NavCommand move_command = {r_delta, 0.0f};
        k_msgq_put(nav_queue_, &move_command, K_FOREVER);
    }

    return 0;
}

int MotionPlanner::discretize() {

    NavCommand nav_command;
    k_msgq_get(nav_queue_, &nav_command, K_FOREVER);

    // See docs/firmware/differential_drive_kinematics.md for derivation

    // step 2
    float d_left = nav_command.r - (nav_command.theta * DOODLEBOT_RADIUS);
    float d_right = nav_command.r + (nav_command.theta * DOODLEBOT_RADIUS);

    // step 3
    float v_left = d_left * STEPPER_CTRL_FREQ;   // mm/s
    float v_right = d_right * STEPPER_CTRL_FREQ; // mm/s
    
    StepCommand step_command;

    // rate limiting subsequent steps to prevent high velocity movement
    do {
        float current_v_left, current_v_right;
        
        if (fabsf(v_left) > MAX_LINEAR_VELOCITY) {
            if (v_left > 0) {
                v_left -= MAX_LINEAR_VELOCITY;
                current_v_left = MAX_LINEAR_VELOCITY;
            } else {
                v_left += MAX_LINEAR_VELOCITY;
                current_v_left = -MAX_LINEAR_VELOCITY;
            }
        } else {
            current_v_left = v_left;
            v_left = 0.0f;
        }

        if (fabsf(v_right) > MAX_LINEAR_VELOCITY) {
            if (v_right > 0) {
                v_right -= MAX_LINEAR_VELOCITY;
                current_v_right = MAX_LINEAR_VELOCITY;
            } else {
                v_right += MAX_LINEAR_VELOCITY;
                current_v_right = -MAX_LINEAR_VELOCITY;
            }
        } else {
            current_v_right = v_right;
            v_right = 0.0f;
        }
        
        // step 4
        float omega_left_rad = current_v_left / WHEEL_RADIUS;
        float omega_right_rad = current_v_right / WHEEL_RADIUS;
        
        // step 5
        step_command.left_velocity = (int16_t)RAD_TO_DEG(omega_left_rad);
        step_command.right_velocity = (int16_t)RAD_TO_DEG(omega_right_rad);
        
        k_msgq_put(step_queue_, &step_command, K_FOREVER);

    } while (fabsf(v_left) > MAX_LINEAR_VELOCITY || fabsf(v_right) > MAX_LINEAR_VELOCITY);

    return 0;
}

int MotionPlanner::consumeInstruction(const InstructionParser::GCodeCmd &current_instruction) {
    int ret;
    current_instruction_ = current_instruction;

    // turn gcode into motion commands
    ret = interpolate();
    if (ret < 0) {
        printk("ERROR: Interpolation not successful.\n");
    }

    // turn motion commands into wheel velocities
    while(k_msgq_num_used_get(nav_queue_) > 0) {
        ret = discretize();

        if (ret < 0) {
            printk("ERROR: Discretization not successful.\n");
        }
    }

    // wheel velocities are consumed by motor_control_handler()
    if (!k_timer_remaining_ticks(&motor_control_timer)) {
        #ifdef DEBUG_NAV
        printk("Starting motor control timer\n");
        #endif
        k_timer_start(&motor_control_timer, K_NO_WAIT, K_MSEC(STEPPER_CTRL_PERIOD * 1000));
    }

    return 0;
}

static void motor_control_work_handler(struct k_work *work) {
    #ifdef DEBUG_NAV
    printk("Motor control work handler triggered\n");
    #endif

    // get step command
    StepCommand step_command;
    if (k_msgq_get(MotionPlanner::step_queue_, &step_command, K_NO_WAIT)) {
        #ifdef DEBUG_NAV
        printk("No more step commands, stopping steppers\n");
        #endif
        MotionPlanner::stepper_left_.stop();
        MotionPlanner::stepper_right_.stop();
        k_timer_stop(&motor_control_timer);
        return;
    }

    // call stepper driver at specified velocity
    #ifdef DEBUG_NAV
    printk("Setting velocity\n");
    step_command.print();
    #endif

    MotionPlanner::stepper_left_.setVelocity(step_command.left_velocity);
    MotionPlanner::stepper_right_.setVelocity(step_command.right_velocity);
}
void MotionPlanner::reset_state() {
    theta_current_ = 0.0f;
    #ifdef DEBUG_NAV
    printk("MotionPlanner: State reset\n");
    #endif
}


/* SERVO */

int ServoMover::consumeInstruction(const InstructionParser::GCodeCmd &gCodeCmd) {

    
    return 0;
}



/* THREAD */

void nav_thread(void *gcode_msgq_void, void *nav_cmd_msgq_void, void *step_cmd_msgq_void) {

    auto *gcode_msgq = (k_msgq *)(gcode_msgq_void);
    k_msgq *nav_cmd_msgq = (k_msgq *)(nav_cmd_msgq_void);
    k_msgq *step_cmd_msgq = (k_msgq *)(step_cmd_msgq_void);
    InstructionParser::GCodeCmd current_instruction;

    // Create handler objects once, before the loop
    MotionPlanner motionPlanner(nav_cmd_msgq, step_cmd_msgq);
    ServoMover marker("servom");  // servo marker
    ServoMover eraser("servoe");  // servo eraser


    while(1) {
        // Block until message arrives
        k_msgq_get(gcode_msgq, &current_instruction, K_FOREVER);
        
        #ifdef DEBUG_NAV
        printk("Nav thread received %c%d command\n", current_instruction.code, current_instruction.number);
        #endif

        char code = current_instruction.code;
        int num = current_instruction.number;

        // command router
        if(code == 'G' && num == 1) {
            #ifdef DEBUG_NAV
            printk("Nav thread routing G1 command\n");
            #endif
            motionPlanner.consumeInstruction(current_instruction);

        } else if (code == 'M' && num == 280) {
            if (current_instruction.args[0].letter == 'P' && current_instruction.args[0].value == 0) {
                marker.consumeInstruction(current_instruction);
                
            } else if (current_instruction.args[0].letter == 'P' && current_instruction.args[0].value == 1) {
                eraser.consumeInstruction(current_instruction);

            } else {
                printk("Unhandled servo command: %c %f\n", current_instruction.args[0].letter, (double)current_instruction.args[0].value);
            }

        } else {
            printk("Unhandled command: %c %d\n", code, num);
        }
        
    }

    return;
}
