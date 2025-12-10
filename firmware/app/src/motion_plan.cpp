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


// -----------------------------------
// Cubic Hermite Interpolation Helpers
// -----------------------------------

/**
 * @brief 2D point/vector structure for path interpolation
 */
struct Vec2 {
    float x;
    float y;
    
    Vec2() : x(0.0f), y(0.0f) {}
    Vec2(float x_, float y_) : x(x_), y(y_) {}
    
    Vec2 operator+(const Vec2& other) const { return Vec2(x + other.x, y + other.y); }
    Vec2 operator-(const Vec2& other) const { return Vec2(x - other.x, y - other.y); }
    Vec2 operator*(float s) const { return Vec2(x * s, y * s); }
    
    float length() const { return sqrtf(x * x + y * y); }
    Vec2 normalized() const {
        float len = length();
        return (len > 0.0001f) ? Vec2(x / len, y / len) : Vec2(0, 0);
    }
};

/**
 * @brief Hermite basis functions for cubic interpolation
 * 
 * H00(t) = 2t³ - 3t² + 1    (position at P0)
 * H10(t) = t³ - 2t² + t     (tangent at P0)
 * H01(t) = -2t³ + 3t²       (position at P1)
 * H11(t) = t³ - t²          (tangent at P1)
 */
static inline float hermite_h00(float t) {
    return 2.0f * t * t * t - 3.0f * t * t + 1.0f;
}

static inline float hermite_h10(float t) {
    return t * t * t - 2.0f * t * t + t;
}

static inline float hermite_h01(float t) {
    return -2.0f * t * t * t + 3.0f * t * t;
}

static inline float hermite_h11(float t) {
    return t * t * t - t * t;
}

/**
 * @brief Evaluate cubic Hermite spline at parameter t
 * 
 * @param p0 Start point
 * @param p1 End point
 * @param t0 Tangent at start (scaled by segment length for Catmull-Rom style)
 * @param t1 Tangent at end
 * @param t Parameter in [0, 1]
 * @return Interpolated point
 */
static Vec2 hermite_interpolate(const Vec2& p0, const Vec2& p1, 
                                 const Vec2& t0, const Vec2& t1, float t) {
    float h00 = hermite_h00(t);
    float h10 = hermite_h10(t);
    float h01 = hermite_h01(t);
    float h11 = hermite_h11(t);
    
    return p0 * h00 + t0 * h10 + p1 * h01 + t1 * h11;
}

/**
 * @brief Calculate tangent using Catmull-Rom style (average of neighboring segments)
 * 
 * For interior points: T_i = 0.5 * (P_{i+1} - P_{i-1})
 * For endpoints: use the single available direction
 * 
 * @param p_prev Previous point (or current if at start)
 * @param p_curr Current point
 * @param p_next Next point (or current if at end)
 * @param tension Tension parameter (0.5 = Catmull-Rom, 0 = linear, 1 = tight)
 */
static Vec2 catmull_rom_tangent(const Vec2& p_prev, const Vec2& p_curr, 
                                 const Vec2& p_next, float tension = 0.5f) {
    return (p_next - p_prev) * tension;
}


// -------------------
// MotionPlanner Class
// -------------------

// -----------------------------------
// Linear Interpolation (Original)
// -----------------------------------

int MotionPlanner::interpolateLinear(int x_delta, int y_delta) {

    // See docs/firmware/differential_drive_kinematics.md for derivation

    float r_delta = sqrtf((float)((x_delta) * (x_delta) + (y_delta) * (y_delta)));

    float theta_prime = atan2f((float)y_delta, (float)x_delta);
    float theta_delta = theta_prime - theta_current_;

    #ifdef DEBUG_INTERPOLATE
    printk("[Linear] Moving theta_delta: %d, theta_prime %d, theta_prev %d, distance %d (x=%d, y=%d)\n", 
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

    // Update position tracking
    pos_x_ += (float)x_delta;
    pos_y_ += (float)y_delta;

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


// -----------------------------------
// Cubic Hermite Interpolation
// -----------------------------------

int MotionPlanner::interpolateHermite(int x_delta, int y_delta) {

    // Calculate target position
    float target_x = pos_x_ + (float)x_delta;
    float target_y = pos_y_ + (float)y_delta;
    
    // Add new waypoint to buffer
    if (waypoint_count_ < MAX_WAYPOINTS) {
        waypoint_buffer_[waypoint_count_].x = target_x;
        waypoint_buffer_[waypoint_count_].y = target_y;
        waypoint_count_++;
    } else {
        // Shift buffer left and add new waypoint at end
        for (int i = 0; i < MAX_WAYPOINTS - 1; i++) {
            waypoint_buffer_[i] = waypoint_buffer_[i + 1];
        }
        waypoint_buffer_[MAX_WAYPOINTS - 1].x = target_x;
        waypoint_buffer_[MAX_WAYPOINTS - 1].y = target_y;
    }
    
    // Need at least 2 waypoints to interpolate
    if (waypoint_count_ < 2) {
        // First waypoint - just store it, no motion yet
        // (Or fall back to linear for the very first move)
        pos_x_ = target_x;
        pos_y_ = target_y;
        return 0;
    }
    
    // Determine which segment to interpolate
    // We interpolate from waypoint[idx-1] to waypoint[idx]
    int idx = (waypoint_count_ >= 3) ? 1 : waypoint_count_ - 1;
    
    // Get the four points for tangent calculation (Catmull-Rom style)
    Vec2 p0, p1, p_prev, p_next;
    
    // Current position is always the start
    p0 = Vec2(pos_x_, pos_y_);
    p1 = Vec2(waypoint_buffer_[idx].x, waypoint_buffer_[idx].y);
    
    // For tangent at p0: use previous waypoint or current position
    if (idx >= 2) {
        p_prev = Vec2(waypoint_buffer_[idx - 2].x, waypoint_buffer_[idx - 2].y);
    } else {
        // No previous point - use current direction as tangent hint
        p_prev = p0;  // Will result in tangent pointing toward p1
    }
    
    // For tangent at p1: use next waypoint if available
    if (idx + 1 < waypoint_count_) {
        p_next = Vec2(waypoint_buffer_[idx + 1].x, waypoint_buffer_[idx + 1].y);
    } else {
        // No next point - use direction from p0 to p1
        p_next = p1;
    }
    
    // Calculate Catmull-Rom tangents
    Vec2 t0 = catmull_rom_tangent(p_prev, p0, p1, 0.5f);
    Vec2 t1 = catmull_rom_tangent(p0, p1, p_next, 0.5f);
    
    // Handle edge case: if p_prev == p0, use direction to p1 as tangent
    if (t0.length() < 0.001f) {
        t0 = (p1 - p0) * 0.5f;
    }
    // If p_next == p1, use direction from p0 as tangent
    if (t1.length() < 0.001f) {
        t1 = (p1 - p0) * 0.5f;
    }
    
    // Calculate segment length for determining number of samples
    float segment_length = (p1 - p0).length();
    
    // Sample the curve - more samples for longer segments
    // Minimum 2mm per segment for smooth motion
    const float SAMPLE_RESOLUTION = 2.0f;  // mm per sample
    int num_samples = (int)(segment_length / SAMPLE_RESOLUTION);
    if (num_samples < 2) num_samples = 2;
    if (num_samples > 20) num_samples = 20;  // Cap for memory/performance
    
    #ifdef DEBUG_INTERPOLATE
    printk("[Hermite] Segment from (%.1f,%.1f) to (%.1f,%.1f), %d samples\n",
           (double)p0.x, (double)p0.y, (double)p1.x, (double)p1.y, num_samples);
    #endif
    
    Vec2 prev_pos = p0;
    
    for (int i = 1; i <= num_samples; i++) {
        float t = (float)i / (float)num_samples;
        
        // Evaluate Hermite spline at t
        Vec2 curr_pos = hermite_interpolate(p0, p1, t0, t1, t);
        
        // Calculate delta from previous sample point
        Vec2 delta = curr_pos - prev_pos;
        float r_delta = delta.length();
        
        if (r_delta > 0.1f) {  // Threshold: 0.1mm
            // Calculate heading for this micro-segment
            float theta_prime = atan2f(delta.y, delta.x);
            float theta_delta = theta_prime - theta_current_;
            
            // Normalize angle to (-pi, pi)
            while (theta_delta > PI) theta_delta -= 2.0f * PI;
            while (theta_delta < -PI) theta_delta += 2.0f * PI;
            
            theta_current_ = theta_prime;
            
            // Generate turn command if needed (threshold: ~0.06 degrees)
            if (fabsf(theta_delta) > 0.001f) {
                NavCommand turn_command = {0.0f, theta_delta};
                k_msgq_put(&nav_queue_, &turn_command, K_FOREVER);
            }
            
            // Generate forward command
            NavCommand forward_command = {r_delta, 0.0f};
            k_msgq_put(&nav_queue_, &forward_command, K_FOREVER);
        }
        
        prev_pos = curr_pos;
    }
    
    // Update current position to end of segment
    pos_x_ = p1.x;
    pos_y_ = p1.y;
    
    // Shift waypoint buffer - remove the waypoint we just processed
    for (int i = 0; i < waypoint_count_ - 1; i++) {
        waypoint_buffer_[i] = waypoint_buffer_[i + 1];
    }
    if (waypoint_count_ > 0) {
        waypoint_count_--;
    }
    
    return 0;
}


// -----------------------------------
// Interpolation Dispatcher
// -----------------------------------

int MotionPlanner::interpolate(int x_delta, int y_delta) {
    switch (interp_mode_) {
        case INTERP_CUBIC_HERMITE:
            return interpolateHermite(x_delta, y_delta);
        case INTERP_LINEAR:
        default:
            return interpolateLinear(x_delta, y_delta);
    }
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


void MotionPlanner::consumeConfig() {
    // M503 - Report current configuration
    // M504 W<wheelbase_mm> R<wheel_radius_mm> - Set configuration
    // M505 I<mode> - Set interpolation mode (0=linear, 1=cubic hermite)
    
    RuntimeConfig& config = RuntimeConfig::instance();
    
    if (current_gcode_cmd_.number == 503) {
        // Report current config
        config.printConfig();
        printk("Interpolation mode: %s\n", 
               interp_mode_ == INTERP_LINEAR ? "LINEAR" : "CUBIC_HERMITE");
               
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
        
    } else if (current_gcode_cmd_.number == 505) {
        // Set/get interpolation mode
        bool mode_set = false;
        for (uint8_t i = 0; i < current_gcode_cmd_.argc; i++) {
            char letter = current_gcode_cmd_.args[i].letter;
            int value = (int)current_gcode_cmd_.args[i].value;
            
            if (letter == 'I') {
                if (value == 0) {
                    interp_mode_ = INTERP_LINEAR;
                    printk("Interpolation mode: LINEAR\n");
                } else if (value == 1) {
                    interp_mode_ = INTERP_CUBIC_HERMITE;
                    printk("Interpolation mode: CUBIC_HERMITE\n");
                } else {
                    printk("Unknown interpolation mode: %d (use 0=linear, 1=hermite)\n", value);
                }
                mode_set = true;
            }
        }
        if (!mode_set) {
            // No argument - just report current mode
            printk("Interpolation mode: %s (use M505 I0 for linear, M505 I1 for hermite)\n",
                   interp_mode_ == INTERP_LINEAR ? "LINEAR" : "CUBIC_HERMITE");
        }
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
               (current_gcode_cmd_.number == 503 || current_gcode_cmd_.number == 504 ||
                current_gcode_cmd_.number == 505)) {
        // M503 - Report config, M504 - Set config, M505 - Set interpolation mode
        consumeConfig();

    } else {
        printk("Unhandled command: %c%d\n", current_gcode_cmd_.code, current_gcode_cmd_.number);
    }

    return output_;
}


void MotionPlanner::reset() {
    theta_current_ = 0.0f;
    pos_x_ = 0.0f;
    pos_y_ = 0.0f;
    waypoint_count_ = 0;
    clearOutput();
    k_msgq_purge(&nav_queue_);
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
