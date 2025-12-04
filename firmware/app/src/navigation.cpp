#include <zephyr/kernel.h>
#include <math.h>
#include "navigation.h"
#include "instruction_parser.h"
#include "comms_thread.h"

/* STATIC MEMBERS */
InstructionParser::GCodeCmd InstructionHandler::current_instruction_;
float MotionPlanner::theta_current_ = 0.0f;

k_msgq* MotionPlanner::nav_queue_ = nullptr;
k_msgq* MotionPlanner::step_queue_ = nullptr;

Stepper MotionPlanner::stepper_left_(STEPPER_LEFT);
Stepper MotionPlanner::stepper_right_(STEPPER_RIGHT);

// Static servo instances for use by motor control handler
static Servo g_servo_marker("servom");
static Servo g_servo_eraser("servoe");

// Track which packet IDs have been acknowledged
static uint8_t last_acked_packet_id = 255;  // Initialize to invalid packet ID
static uint8_t current_packet_id = 0;       // Track the currently processing packet ID

// Work queue for motor control (to avoid ISR context issues)
static void motor_control_work_handler(struct k_work *work);
K_WORK_DEFINE(motor_control_work, motor_control_work_handler);

void motor_control_timer_handler(k_timer *timer) {
    k_work_submit(&motor_control_work);
}
K_TIMER_DEFINE(motor_control_timer, motor_control_timer_handler, NULL);

/* MOTION PLANNER */

int MotionPlanner::interpolate(int x_delta, int y_delta,
                                float& theta_current,
                                void (*output_fn)(const NavCommand&)) {

    // See docs/firmware/differential_drive_kinematics.md for derivation

    // TODO: going to have to be reset soon 
    static int x_prev, y_prev;

    float r_delta = sqrtf((float)((x_delta) * (x_delta) + (y_delta) * (y_delta)));

    float theta_prime = atan2f((float)y_delta, (float)x_delta);
    float theta_delta = theta_prime - theta_current;

    #ifdef DEBUG_INTERPOLATE
    printk("Moving theta_delta: %d, theta_prime %d, theta_prev %d, distance %d (x=%d, y=%d)\n", 
           (int)RAD_TO_DEG(theta_delta), (int)RAD_TO_DEG(theta_prime), (int)RAD_TO_DEG(theta_current), (int)r_delta, x_delta, y_delta);
    #endif /* DEBUG_INTERPOLATE */

    // normalize angle to (-pi, pi) range
    while (theta_delta > PI) {
        theta_delta -= 2.0f * PI;
    }
    while (theta_delta < -PI) {
        theta_delta += 2.0f * PI;
    }
    
    // update current heading state
    theta_current = theta_prime;
    x_prev = x_delta;
    y_prev = y_delta;

    // turn
    if (fabsf(theta_delta) > 0.01f) {
        NavCommand turn_command = {0.0f, theta_delta};
        output_fn(turn_command);
    }
    
    // forward movement
    if (r_delta > 0.01f) { 
        NavCommand move_command = {r_delta, 0.0f};
        output_fn(move_command);
    }

    return 0;
}

// Queue-based wrapper for production use
int MotionPlanner::interpolate() {

    // See docs/firmware/differential_drive_kinematics.md for derivation

    int x_delta = current_instruction_.args[0].value;
    int y_delta = current_instruction_.args[1].value;

    return interpolate(x_delta, y_delta, theta_current_,
                      [](const NavCommand& cmd) {
                          k_msgq_put(nav_queue_, &cmd, K_FOREVER);
                      });
}

int MotionPlanner::discretize(const NavCommand& nav_command,
                               uint8_t packet_id,
                               void (*output_fn)(const StepCommand&)) {

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
        
        // step 4
        float omega_left_rad = current_v_left / WHEEL_RADIUS;
        float omega_right_rad = current_v_right / WHEEL_RADIUS;
        
        // step 5
        StepCommand step_command;
        step_command.packet_id = packet_id;
        step_command.type = StepCommandType::STEPPER;
        step_command.stepper.left_velocity = (int16_t)RAD_TO_DEG(omega_left_rad);
        step_command.stepper.right_velocity = (int16_t)RAD_TO_DEG(omega_right_rad);
        
        output_fn(step_command);

    } while (fabsf(v_left) > 0 || fabsf(v_right) > 0);

    return 0;
}

int MotionPlanner::discretize() {

    NavCommand nav_command;
    k_msgq_get(nav_queue_, &nav_command, K_FOREVER);

    return discretize(nav_command, current_instruction_.packet_id, 
                     [](const StepCommand& cmd) {
                         k_msgq_put(step_queue_, &cmd, K_FOREVER);
                     });
}

int MotionPlanner::consumeInstruction(const InstructionParser::GCodeCmd &current_instruction) {
    int ret;
    current_instruction_ = current_instruction;
    current_packet_id = current_instruction.packet_id;

    // turn gcode into motion commands
    ret = interpolate();
    if (ret < 0) {
        printk("ERROR: Interpolation not successful.\n");
    }

    // turn motion commands into wheel velocities
    // process all nav commands in the queue
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
        #ifdef DEBUG_MOTION
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

    #ifdef DEBUG_MOTION
    step_command.print();
    #endif

    // Handle command based on type
    if (step_command.type == StepCommandType::SERVO) {
        // Execute servo command
        printk("MOTOR_CTRL: Executing SERVO command - servo_id=%d, angle=%d\n", 
               step_command.servo.servo_id, step_command.servo.angle);
        if (step_command.servo.servo_id == 0) {
            g_servo_marker.setAngle(step_command.servo.angle);
        } else {
            g_servo_eraser.setAngle(step_command.servo.angle);
        }
    } else {
        // Execute stepper command
        MotionPlanner::stepper_left_.setVelocity(step_command.stepper.left_velocity);
        MotionPlanner::stepper_right_.setVelocity(step_command.stepper.right_velocity);
    }

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
    theta_current_ = 0.0f;
    #ifdef DEBUG_NAV
    printk("MotionPlanner: State reset\n");
    #endif
}


/* SERVO */

int ServoMover::consumeInstruction(const InstructionParser::GCodeCmd &gCodeCmd) {
    // Queue a servo command to be executed in sequence with stepper commands
    if (gCodeCmd.args[0].letter != 'P') {
        printk("ServoMover: Unrecognized argument letter %c\n", gCodeCmd.args[0].letter);
        return -EINVAL;
    }

    uint8_t servo_id = (uint8_t)gCodeCmd.args[0].value;  // P0 = marker, P1 = eraser
    uint8_t angle = (uint8_t)gCodeCmd.args[1].value;     // S value = angle

    printk("ServoMover: Queueing servo %d to angle %d (packet_id=%d)\n", servo_id, angle, gCodeCmd.packet_id);

    // Create servo step command
    StepCommand step_command;
    step_command.packet_id = gCodeCmd.packet_id;
    step_command.type = StepCommandType::SERVO;
    step_command.servo.servo_id = servo_id;
    step_command.servo.angle = angle;

    // Queue it
    int ret = k_msgq_put(MotionPlanner::step_queue_, &step_command, K_FOREVER);
    printk("ServoMover: Queue put returned %d, queue has %d items\n", ret, k_msgq_num_used_get(MotionPlanner::step_queue_));

    // Start the motor control timer if not already running
    if (!k_timer_remaining_ticks(&motor_control_timer)) {
        printk("ServoMover: Starting motor control timer\n");
        k_timer_start(&motor_control_timer, K_NO_WAIT, K_MSEC(STEPPER_CTRL_PERIOD * 1000));
    } else {
        printk("ServoMover: Timer already running\n");
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
    // marker.servo_.initialize();
    // eraser.servo_.initialize();


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
