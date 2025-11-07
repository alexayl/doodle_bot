#include <zephyr/kernel.h>
#include <math.h>
#include "navigation.h"
#include "instruction_parser.h"
#include "comms_thread.h"

/* STATIC MEMBERS */
InstructionParser::GCodeCmd InstructionHandler::current_instruction_;
float MotionPlanner::theta_current = 0.0f;

k_msgq* MotionPlanner::nav_queue_ = nullptr;
k_msgq* MotionPlanner::step_queue_ = nullptr;

Stepper MotionPlanner::stepper_left_(STEPPER_LEFT);
Stepper MotionPlanner::stepper_right_(STEPPER_RIGHT);

// Track which packet IDs have been acknowledged
static uint8_t last_acked_packet_id = 255;  // Initialize to invalid packet ID
static uint8_t current_packet_id = 0;       // Track the currently processing packet ID

// Work queue for motor control (to avoid ISR context issues)
static void motor_control_work_handler(struct k_work *work);
K_WORK_DEFINE(motor_control_work, motor_control_work_handler);

// Timer definition - now just triggers work instead of doing heavy lifting
void motor_control_timer_handler(k_timer *timer) {
    k_work_submit(&motor_control_work);
}
K_TIMER_DEFINE(motor_control_timer, motor_control_timer_handler, NULL);

/* MOTION PLANNER */

int MotionPlanner::interpolate() {

    // Using linear interpolation
    int x_delta = current_instruction_.args[0].value;
    int y_delta = current_instruction_.args[1].value;
    
    // calculate r
    float r_delta = (float)sqrt(pow(x_delta, 2) + pow(y_delta, 2));

    // calculate theta
    float theta_desired = (float)atan2(y_delta, x_delta);

    // calculate change in delta
    float theta_delta = theta_desired - theta_current;

    // normalize theta (-PI, PI)
    while (theta_delta > PI) {
        theta_delta -= 2 * PI;
    }
    while (theta_delta < -PI) {
        theta_delta += 2 * PI;
    }
    
    // Update current heading after turn
    theta_current = theta_desired;
    
    // Generate TWO commands: first turn in place, then move forward
    // Command 1: Turn in place (r=0, theta=theta_delta)
    if (fabs(theta_delta) > 0.01) {  // Only turn if angle is significant
        NavCommand turn_command = {0.0f, theta_delta};
        k_msgq_put(nav_queue_, &turn_command, K_FOREVER);
    }
    
    // Command 2: Move forward (r=r_delta, theta=0)
    if (r_delta > 0.01) {  // Only move if distance is significant
        NavCommand move_command = {r_delta, 0.0f};
        k_msgq_put(nav_queue_, &move_command, K_FOREVER);
    }

    return 0;
}

int MotionPlanner::discretize() {

    // take navigation commands from nav_queue_
    NavCommand nav_command;
    k_msgq_get(nav_queue_, &nav_command, K_FOREVER);

    // Differential drive: arc length for each wheel
    // l_dist = distance left wheel travels = R_left * theta
    // r_dist = distance right wheel travels = R_right * theta
    // Where R_left/right are the radii from the ICC (instantaneous center of curvature)
    
    // For a turn by angle theta_delta and forward movement r:
    // Left wheel:  arc = r - (theta_delta * DOODLEBOT_RADIUS)
    // Right wheel: arc = r + (theta_delta * DOODLEBOT_RADIUS)
    
    float l_dist = nav_command.r - (nav_command.theta * DOODLEBOT_RADIUS);
    float r_dist = nav_command.r + (nav_command.theta * DOODLEBOT_RADIUS);
    
    // Convert to velocities
    int left_vel = (int)(l_dist * STEPPER_CTRL_FREQ);
    int right_vel = (int)(r_dist * STEPPER_CTRL_FREQ);
    
    StepCommand step_command;
    
    // If velocity larger than max velocity, split into multiple commands
    do {
        step_command.packet_id = current_instruction_.packet_id;
        if (fabs(left_vel) > STEPPER_MAX_VELOCITY) {
            if (left_vel > 0) {
                left_vel -= STEPPER_MAX_VELOCITY;
                step_command.left_velocity = (int16_t)STEPPER_MAX_VELOCITY;
            } else {
                left_vel += STEPPER_MAX_VELOCITY;  // Add for negative values
                step_command.left_velocity = (int16_t)(-STEPPER_MAX_VELOCITY);
            }
        } else {
            step_command.left_velocity = (int16_t)left_vel;
            left_vel = 0;
        }
        if (fabs(right_vel) > STEPPER_MAX_VELOCITY) {
            if (right_vel > 0) {
                right_vel -= STEPPER_MAX_VELOCITY;
                step_command.right_velocity = (int16_t)STEPPER_MAX_VELOCITY;
            } else {
                right_vel += STEPPER_MAX_VELOCITY;  // Add for negative values
                step_command.right_velocity = (int16_t)(-STEPPER_MAX_VELOCITY);
            }
        } else {
            step_command.right_velocity = (int16_t)right_vel;
            right_vel = 0;
        }

        // push wheel velocities to step_queue_
        k_msgq_put(step_queue_, &step_command, K_FOREVER);

    } while (fabs(left_vel) > STEPPER_MAX_VELOCITY || fabs(right_vel) > STEPPER_MAX_VELOCITY);

    return 0;
}

int MotionPlanner::consumeInstruction(const InstructionParser::GCodeCmd &current_instruction) {
    int ret;
    current_instruction_ = current_instruction;
    current_packet_id = current_instruction.packet_id;  // Track the packet ID being processed

    // turn gcode into motion commands
    ret = interpolate();
    if (ret < 0) {
        printk("ERROR: Interpolation not successful.\n");
    }

    
    // turn motion commands into wheel velocities
    // Process all nav commands in the queue
    while(k_msgq_num_used_get(nav_queue_) > 0) {
        ret = discretize();

        if (ret < 0) {
            printk("ERROR: Discretization not successful.\n");
        }
    }

    // wheel velocities are consumed by motor_control_handler()
    // start the timer that runs it if it is not currently running
    if (!k_timer_remaining_ticks(&motor_control_timer)) {
        #ifdef DEBUG_NAV
        printk("Starting motor control timer\n");
        #endif
        k_timer_start(&motor_control_timer, K_NO_WAIT, K_MSEC(STEPPER_CTRL_PERIOD * 1000));
    }
    
    return 0;
}


// Work handler - runs in thread context, not ISR context
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
        
        // Send final ACK if we haven't already for the last instruction
        // This handles the case where the last step command completes
        if (g_bleService && (current_packet_id != last_acked_packet_id)) {
            last_acked_packet_id = current_packet_id;
            char ack[sizeof("aok\n")] = "aok\n";
            ack[0] = current_packet_id;
            #ifdef DEBUG_NAV
            printk("MOTOR_CTRL: Sending FINAL ACK for packet ID %d\n", current_packet_id);
            #endif
            g_bleService->send(ack, sizeof(ack));
        } else if (g_bleService) {
            #ifdef DEBUG_NAV
            printk("MOTOR_CTRL: Skipping duplicate FINAL ACK for packet ID %d (last_acked=%d)\n", 
                   current_packet_id, last_acked_packet_id);
            #endif
        }
        return;
    }

    // call stepper driver at specified velocity
    #ifdef DEBUG_NAV
    printk("Setting velocity\n");
    step_command.print();
    #endif

    MotionPlanner::stepper_left_.setVelocity(step_command.left_velocity);
    MotionPlanner::stepper_right_.setVelocity(step_command.right_velocity);

    // Send ACK with packet ID as first byte, but only once per packet ID
    if (g_bleService && (step_command.packet_id != last_acked_packet_id)) {
        last_acked_packet_id = step_command.packet_id;
        char ack[sizeof("aok\n")] = "aok\n";
        ack[0] = step_command.packet_id;
        #ifdef DEBUG_NAV
        printk("MOTOR_CTRL: Sending ACK for packet ID %d\n", step_command.packet_id);
        #endif
        g_bleService->send(ack, sizeof(ack));
    } else if (g_bleService) {
        #ifdef DEBUG_NAV
        printk("MOTOR_CTRL: Skipping duplicate ACK for packet ID %d (last_acked=%d)\n", 
               step_command.packet_id, last_acked_packet_id);
        #endif
    }
}
void MotionPlanner::reset_state() {
    theta_current = 0.0f;
    #ifdef DEBUG_NAV
    printk("MotionPlanner: State reset\n");
    #endif
}


/* SERVO */

int ServoMover::consumeInstruction(const InstructionParser::GCodeCmd &gCodeCmd) {

    // consume servo instruction
    
    // send ack command back
    if (g_bleService) {
        char ack[sizeof("aok\n")] = "aok\n";
        ack[0] = gCodeCmd.packet_id;
        #ifdef DEBUG_NAV
        printk("SERVO: Sending ACK for packet ID %d\n", gCodeCmd.packet_id);
        #endif
        g_bleService->send(ack, sizeof(ack));
    }
    
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
