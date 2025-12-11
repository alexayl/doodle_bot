#include <zephyr/kernel.h>
#include <stdlib.h>
#include <math.h>

#include "comms_thread.h"
#include "config.h"
#include "runtime_config.h"
#include "instruction_parser.h"
#include "motion_plan.h"
#include "motion_execute.h"

// Define the global RuntimeConfig instance
RuntimeConfig RuntimeConfig::instance_;


// -------------------
// S-Curve Helpers
// -------------------

/**
 * @brief S-curve velocity profile using half-cosine
 * 
 * Provides smooth acceleration with zero jerk at start/end.
 * Input: t in [0, 1], Output: scale in [0, 1]
 * 
 * Profile shape: starts slow, accelerates in middle, ends slow
 * Integral equals 0.5 (same as linear ramp), so timing math unchanged.
 */
static inline float s_curve(float t) {
    return (1.0f - cosf(PI * t)) * 0.5f;
}


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

    // turn (threshold: ~0.06 degrees)
    if (fabsf(theta_delta) > 0.001f) {
        NavCommand turn_command = {0.0f, theta_delta};
        k_msgq_put(&nav_queue_, &turn_command, K_FOREVER);
    }
    
    // forward movement (threshold: 0.1mm)
    if (r_delta > 0.1f) { 
        NavCommand forward_command = {r_delta, 0.0f};
        k_msgq_put(&nav_queue_, &forward_command, K_FOREVER);
    }

    return 0;
}


int MotionPlanner::discretize(const NavCommand& nav_command) {

    // See docs/firmware/differential_drive_kinematics.md for derivation

    // step 2 - use runtime config for wheelbase
    float doodlebot_radius = RUNTIME_DOODLEBOT_RADIUS;
    float d_left = nav_command.r - (nav_command.theta * doodlebot_radius);
    float d_right = nav_command.r + (nav_command.theta * doodlebot_radius);

    #ifdef DEBUG_INTERPOLATE
    printk("Discretize: d_left=%.2f mm, d_right=%.2f mm\n", (double)d_left, (double)d_right);
    #endif /* DEBUG_INTERPOLATE */

    // step 3
    float v_left = d_left * STEPPER_CTRL_FREQ;   // mm/s
    float v_right = d_right * STEPPER_CTRL_FREQ; // mm/s

    // Calculate number of steps needed based on the wheel with the higher velocity
    float max_v = fmaxf(fabsf(v_left), fabsf(v_right));
    if (max_v < 0.01f) {
        return 0;  // No motion needed
    }

    // Velocity smoothing: ramp up over RAMP_CYCLES, cruise, then ramp down over RAMP_CYCLES
    const int RAMP_CYCLES = 6;
    
    // Calculate equivalent cruise steps from ramp phases
    // Ramp up: scales [1/RAMP, 2/RAMP, ..., RAMP/RAMP] = sum of 1 to RAMP divided by RAMP
    // Ramp down: scales [(RAMP-1)/RAMP, ..., 1/RAMP] = sum of 1 to RAMP-1 divided by RAMP
    float ramp_up_equivalent = (float)(RAMP_CYCLES + 1) / 2.0f;      // (1+2+..+6)/6 = 3.5
    float ramp_down_equivalent = (float)(RAMP_CYCLES - 1) / 2.0f;    // (5+4+..+1)/6 = 2.5
    float total_ramp_equivalent = ramp_up_equivalent + ramp_down_equivalent;  // 3.0 cruise-equivalent steps
    
    // Total equivalent cruise steps needed
    float total_equivalent_steps = max_v / MAX_LINEAR_VELOCITY;
    
    // Calculate cruise steps (can be 0 or negative if motion is too short for full ramps)
    int cruise_steps = (int)ceilf(total_equivalent_steps - total_ramp_equivalent);
    if (cruise_steps < 0) {
        cruise_steps = 0;
    }
    
    // Total actual steps
    int total_steps = RAMP_CYCLES + cruise_steps + (RAMP_CYCLES - 1);  // ramp up + cruise + ramp down
    
    // For very short motions, use triangular profile (just ramp up then down)
    if (total_equivalent_steps < total_ramp_equivalent) {
        // Scale down the velocity to fit the motion
        float scale_factor = total_equivalent_steps / total_ramp_equivalent;
        total_steps = RAMP_CYCLES + (RAMP_CYCLES - 1);  // No cruise phase
        
        // Recalculate velocities scaled down
        float max_step_v_left = (v_left / total_equivalent_steps) * scale_factor;
        float max_step_v_right = (v_right / total_equivalent_steps) * scale_factor;
        
        for (int i = 0; i < total_steps; i++) {
            float velocity_scale;
            if (i < RAMP_CYCLES) {
                // S-curve ramp up: smooth acceleration
                float t = (float)(i + 1) / (float)RAMP_CYCLES;
                velocity_scale = s_curve(t);
            } else {
                // S-curve ramp down: smooth deceleration
                int ramp_down_idx = i - RAMP_CYCLES;
                float t = (float)(RAMP_CYCLES - 1 - ramp_down_idx) / (float)RAMP_CYCLES;
                velocity_scale = s_curve(t);
            }
            
            float current_v_left = max_step_v_left * velocity_scale;
            float current_v_right = max_step_v_right * velocity_scale;
            
            // step 4: convert linear velocity to angular velocity (use runtime config)
            float wheel_radius = RUNTIME_WHEEL_RADIUS;
            float omega_left_rad = current_v_left / wheel_radius;
            float omega_right_rad = current_v_right / wheel_radius;
            
            // step 5: create stepper command
            ExecuteCommand::StepperData stepper_data;
            stepper_data.left_velocity = RAD_TO_DEG(omega_left_rad);
            stepper_data.right_velocity = RAD_TO_DEG(omega_right_rad);
            
            ExecuteCommand execute_cmd;
            execute_cmd.set(Steppers, &stepper_data, current_packet_id_);
            addToOutput(execute_cmd);
        }
    } else {
        // Normal trapezoidal profile with cruise phase
        float max_step_v_left = v_left / (total_ramp_equivalent + (float)cruise_steps);
        float max_step_v_right = v_right / (total_ramp_equivalent + (float)cruise_steps);
        
        for (int i = 0; i < total_steps; i++) {
            float velocity_scale;
            if (i < RAMP_CYCLES) {
                // S-curve ramp up: smooth acceleration
                float t = (float)(i + 1) / (float)RAMP_CYCLES;
                velocity_scale = s_curve(t);
            } else if (i < RAMP_CYCLES + cruise_steps) {
                // Cruise: full speed
                velocity_scale = 1.0f;
            } else {
                // S-curve ramp down: smooth deceleration
                int ramp_down_idx = i - RAMP_CYCLES - cruise_steps;
                float t = (float)(RAMP_CYCLES - 1 - ramp_down_idx) / (float)RAMP_CYCLES;
                velocity_scale = s_curve(t);
            }
            
            float current_v_left = max_step_v_left * velocity_scale;
            float current_v_right = max_step_v_right * velocity_scale;
            
            // step 4: convert linear velocity to angular velocity (use runtime config)
            float wheel_radius = RUNTIME_WHEEL_RADIUS;
            float omega_left_rad = current_v_left / wheel_radius;
            float omega_right_rad = current_v_right / wheel_radius;
            
            // step 5: create stepper command
            ExecuteCommand::StepperData stepper_data;
            stepper_data.left_velocity = RAD_TO_DEG(omega_left_rad);
            stepper_data.right_velocity = RAD_TO_DEG(omega_right_rad);
            
            ExecuteCommand execute_cmd;
            execute_cmd.set(Steppers, &stepper_data, current_packet_id_);
            addToOutput(execute_cmd);
        }
    }

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

    // LED on before stepper motion
    ExecuteCommand::LedData led_on = {true};
    ExecuteCommand led_on_cmd;
    led_on_cmd.set(Device::StatusLed, &led_on, current_packet_id_);
    addToOutput(led_on_cmd);

    // turn wheel distances into wheel velocities
    while (k_msgq_num_used_get(&nav_queue_) > 0) {

        NavCommand nav_command;
        k_msgq_get(&nav_queue_, &nav_command, K_FOREVER);

        ret = discretize(nav_command);

        if (ret < 0) {
            printk("ERROR: Discretization not successful.\n");
        }
    }

    // LED off after stepper motion
    ExecuteCommand::LedData led_off = {false};
    ExecuteCommand led_off_cmd;
    led_off_cmd.set(Device::StatusLed, &led_off, current_packet_id_);
    addToOutput(led_off_cmd);
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


void MotionPlanner::consumeConfig() {
    // M503 - Report current configuration
    // M504 W<wheelbase_mm> R<wheel_radius_mm> - Set configuration
    
    RuntimeConfig& config = RuntimeConfig::instance();
    
    if (current_gcode_cmd_.number == 503) {
        // Report current config
        config.printConfig();
    } else if (current_gcode_cmd_.number == 504) {
        // Set config - parse W (wheelbase) and R (radius) arguments
        for (uint8_t i = 0; i < current_gcode_cmd_.argc; i++) {
            char letter = current_gcode_cmd_.args[i].letter;
            float value = current_gcode_cmd_.args[i].value;
            
            if (letter == 'W') {
                config.setWheelbase(value);
            } else if (letter == 'R') {
                config.setWheelRadius(value);
            } else if (letter == 'D') {
                // Convenience: D sets wheel diameter (converts to radius)
                config.setWheelRadius(value / 2.0f);
            }
        }
        // Print updated config
        config.printConfig();
    }
    
    // Config commands complete immediately - create a no-op execute command
    // to trigger the ACK response
    ExecuteCommand execute_cmd;
    execute_cmd.set(ConfigAck, nullptr, current_packet_id_);
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
        consumeLocomotion();

    } else if (current_gcode_cmd_.code == 'G' && 
               (current_gcode_cmd_.number == 90 || current_gcode_cmd_.number == 91)) {
        // G90/G91 - Positioning mode commands (absolute/relative)
        // These are no-ops since firmware uses relative coordinates internally
        // Just need to ACK
        ExecuteCommand execute_cmd;
        execute_cmd.set(ConfigAck, nullptr, current_packet_id_);
        addToOutput(execute_cmd);

    } else if (current_gcode_cmd_.code == 'M' && current_gcode_cmd_.number == 280) {
        // M280 P<servo_id> S<angle>
        if (current_gcode_cmd_.args[0].letter == 'P' && (int)current_gcode_cmd_.args[0].value == 0) {
            consumeMarker();
            
        } else if (current_gcode_cmd_.args[0].letter == 'P' && (int)current_gcode_cmd_.args[0].value == 1) {
            consumeEraser();

        } else {
            printk("Unhandled servo command: P%d\n", (int)current_gcode_cmd_.args[0].value);
        }

    } else if (current_gcode_cmd_.code == 'M' && 
               (current_gcode_cmd_.number == 503 || current_gcode_cmd_.number == 504)) {
        // M503 - Report config, M504 - Set config
        consumeConfig();

    } else {
        printk("Unhandled command: %c%d\n", current_gcode_cmd_.code, current_gcode_cmd_.number);
    }

    return output_;
}


void MotionPlanner::reset() {
    theta_current_ = 0.0f;
    clearOutput();
    k_msgq_purge(&nav_queue_);
}


// ---------------------
// Motion Planner Thread
// ---------------------

// File-scope static to avoid stack overflow with large MOTION_PLAN_OUTPUT_SIZE
// (and avoids C++ guard functions not available in minimal Zephyr runtime)
static MotionPlanner motionPlanner;

void motion_plan_thread(void *gcode_cmd_msgq_void, void *execute_cmd_msgq_void, void *arg3) {

    auto *gcode_cmd_msgq = static_cast<struct k_msgq *>(gcode_cmd_msgq_void);
    auto *execute_cmd_msgq = static_cast<struct k_msgq *>(execute_cmd_msgq_void);
    ARG_UNUSED(arg3);

    InstructionParser::GCodeCmd current_instruction;

    while (1) {
        // block until message arrives
        k_msgq_get(gcode_cmd_msgq, &current_instruction, K_FOREVER);

        // preprocess the gcode command
        MotionPlanner::Output execution_cmds = motionPlanner.consumeGcode(current_instruction);

        // send outputs to execution thread
        for (size_t i = 0; i < execution_cmds.count; i++) {
            k_msgq_put(execute_cmd_msgq, &execution_cmds.cmds[i], K_FOREVER);
        }
    }
}
