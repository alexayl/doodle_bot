#include <zephyr/kernel.h>
#include <stdlib.h>
#include <math.h>

#include "comms_thread.h"
#include "config.h"
#include "instruction_parser.h"
#include "motion_plan.h"
#include "motion_execute.h"


// -------------------
// MotionPlanner Class
// -------------------

int MotionPlanner::interpolate(int x_delta, int y_delta) {

    // See docs/firmware/differential_drive_kinematics.md for derivation

    float r_delta = sqrtf((float)((x_delta) * (x_delta) + (y_delta) * (y_delta)));

    float theta_prime = atan2f((float)y_delta, (float)x_delta);
    float theta_delta = theta_prime - theta_current_;

    #ifdef DEBUG_INTERPOLATE
    printk("Moving theta_delta: %d, theta_prime %d, theta_prev %d, distance %d (x=%d, y=%d)\n", 
           (int)RAD_TO_DEG(theta_delta), (int)RAD_TO_DEG(theta_prime), (int)RAD_TO_DEG(theta_current_), (int)r_delta, x_delta, y_delta);
    #endif /* DEBUG_INTERPOLATE */

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
        k_msgq_put(&nav_queue_, &turn_command, K_FOREVER);
    }
    
    // forward movement
    if (r_delta > 0.01f) { 
        NavCommand forward_command = {r_delta, 0.0f};
        k_msgq_put(&nav_queue_, &forward_command, K_FOREVER);
    }

    return 0;
}


int MotionPlanner::discretize(const NavCommand& nav_command) {

    // See docs/firmware/differential_drive_kinematics.md for derivation

    // step 2
    float d_left = nav_command.r - (nav_command.theta * DOODLEBOT_RADIUS);
    float d_right = nav_command.r + (nav_command.theta * DOODLEBOT_RADIUS);

    #ifdef DEBUG_INTERPOLATE
    printk("Discretize: d_left=%.2f mm, d_right=%.2f mm\n", (double)d_left, (double)d_right);
    #endif /* DEBUG_INTERPOLATE */

    // step 3
    float v_left = d_left * STEPPER_CTRL_FREQ;   // mm/s
    float v_right = d_right * STEPPER_CTRL_FREQ; // mm/s

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
        
        // step 4: convert linear velocity to angular velocity
        float omega_left_rad = current_v_left / WHEEL_RADIUS;
        float omega_right_rad = current_v_right / WHEEL_RADIUS;
        
        // step 5: create stepper command
        ExecuteCommand::StepperData stepper_data;
        stepper_data.left_velocity = (int16_t)RAD_TO_DEG(omega_left_rad);
        stepper_data.right_velocity = (int16_t)RAD_TO_DEG(omega_right_rad);
        
        ExecuteCommand execute_cmd;
        execute_cmd.set(Steppers, &stepper_data, current_packet_id_);
        addToOutput(execute_cmd);

    } while (fabsf(v_left) > 0.01f || fabsf(v_right) > 0.01f);

    return 0;
}


void MotionPlanner::consumeLocomotion() {
    
    int ret;

    // turn gcode cmds into wheel distances
    int x_delta = (int)current_gcode_cmd_.args[0].value;
    int y_delta = (int)current_gcode_cmd_.args[1].value;

    ret = interpolate(x_delta, y_delta);

    if (ret < 0) {
        printk("ERROR: Interpolation not successful.\n");
    }

    // turn wheel distances into wheel velocities
    while (k_msgq_num_used_get(&nav_queue_) > 0) {

        NavCommand nav_command;
        k_msgq_get(&nav_queue_, &nav_command, K_FOREVER);

        ret = discretize(nav_command);

        if (ret < 0) {
            printk("ERROR: Discretization not successful.\n");
        }
    }
}


void MotionPlanner::consumeMarker() {

    uint8_t angle = (uint8_t)current_gcode_cmd_.args[1].value;  // S value = angle

    // Create servo command
    ExecuteCommand::ServoData servo_data;
    servo_data.servo_id = 0;  // Marker servo
    servo_data.angle = angle;

    // Queue it up
    ExecuteCommand execute_cmd;
    execute_cmd.set(MarkerServo, &servo_data, current_packet_id_);
    addToOutput(execute_cmd);
}


void MotionPlanner::consumeEraser() {

    uint8_t angle = (uint8_t)current_gcode_cmd_.args[1].value;  // S value = angle

    // Create servo command
    ExecuteCommand::ServoData servo_data;
    servo_data.servo_id = 1;  // Eraser servo
    servo_data.angle = angle;

    // Queue it up
    ExecuteCommand execute_cmd;
    execute_cmd.set(EraserServo, &servo_data, current_packet_id_);
    addToOutput(execute_cmd);
}


MotionPlanner::Output MotionPlanner::consumeGcode(const InstructionParser::GCodeCmd &gcode_cmd) {

    // Store current command and packet ID
    current_gcode_cmd_ = gcode_cmd;
    current_packet_id_ = gcode_cmd.packet_id;

    // Clear previous output
    clearOutput();

    // Command router
    if (current_gcode_cmd_.code == 'G' && current_gcode_cmd_.number == 1) {
        #ifdef DEBUG_NAV
        printk("MotionPlanner::consumeGcode - routing G1 command\n");
        #endif
        consumeLocomotion();

    } else if (current_gcode_cmd_.code == 'M' && current_gcode_cmd_.number == 280) {
        // M280 P<servo_id> S<angle>
        if (current_gcode_cmd_.args[0].letter == 'P' && (int)current_gcode_cmd_.args[0].value == 0) {
            consumeMarker();
            
        } else if (current_gcode_cmd_.args[0].letter == 'P' && (int)current_gcode_cmd_.args[0].value == 1) {
            consumeEraser();

        } else {
            printk("Unhandled servo command: P%d\n", (int)current_gcode_cmd_.args[0].value);
        }

    } else {
        printk("Unhandled command: %c%d\n", current_gcode_cmd_.code, current_gcode_cmd_.number);
    }

    return output_;
}


void MotionPlanner::reset() {
    theta_current_ = 0.0f;
    clearOutput();
    k_msgq_purge(&nav_queue_);
    
    #ifdef DEBUG_NAV
    printk("MotionPlanner: State reset\n");
    #endif
}


// ---------------------
// Motion Planner Thread
// ---------------------

void motion_plan_thread(void *gcode_cmd_msgq_void, void *execute_cmd_msgq_void, void *arg3) {

    auto *gcode_cmd_msgq = static_cast<struct k_msgq *>(gcode_cmd_msgq_void);
    auto *execute_cmd_msgq = static_cast<struct k_msgq *>(execute_cmd_msgq_void);
    ARG_UNUSED(arg3);

    InstructionParser::GCodeCmd current_instruction;
    MotionPlanner motionPlanner;

    #ifdef DEBUG_NAV
    printk("Motion plan thread started\n");
    #endif

    while (1) {
        // Block until message arrives
        k_msgq_get(gcode_cmd_msgq, &current_instruction, K_FOREVER);
        
        #ifdef DEBUG_NAV
        printk("Nav thread received %c%d command\n", current_instruction.code, current_instruction.number);
        #endif

        // Preprocess the gcode command
        MotionPlanner::Output execution_cmds = motionPlanner.consumeGcode(current_instruction);

        // Send outputs to execution thread
        for (size_t i = 0; i < execution_cmds.count; i++) {
            k_msgq_put(execute_cmd_msgq, &execution_cmds.cmds[i], K_FOREVER);
        }
    }
}
