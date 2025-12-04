#include <zephyr/kernel.h>
#include <stdlib.h>

#include "motion_plan.h"
#include "motion_execute.h"
#include "ble_service.h"
#include "config.h"

// Static stepper instances
Stepper MotionExecutor::stepper_left_(STEPPER_LEFT);
Stepper MotionExecutor::stepper_right_(STEPPER_RIGHT);

// Global executor instance
MotionExecutor g_motionExecutor;

// ----------------------------
// Timer and Work Queue Setup
// ----------------------------

static void motor_control_work_handler(struct k_work *work);
K_WORK_DEFINE(motor_control_work, motor_control_work_handler);

static void motor_control_timer_handler(struct k_timer *timer) {
    k_work_submit(&motor_control_work);
}
K_TIMER_DEFINE(motor_control_timer, motor_control_timer_handler, NULL);

/**
 * @brief Work handler for motor control
 * 
 * Deferred from timer ISR context to allow safe hardware access.
 * Processes stepper commands from the step queue at the control frequency.
 */
static void motor_control_work_handler(struct k_work *work) {
    ARG_UNUSED(work);
    g_motionExecutor.timerHandler();
}


// ----------------------------
// MotionExecutor Implementation
// ----------------------------

MotionExecutor::MotionExecutor() 
    : servo_marker_("servom")
    , servo_eraser_("servoe")
    , execute_cmd_queue_(nullptr)
    , last_acked_packet_id_(0xFF)  // Invalid initial value
    , current_packet_id_(0)
{
    k_msgq_init(&step_queue_, step_queue_buffer_, sizeof(ExecuteCommand), MESSAGES_PER_QUEUE);
}


void MotionExecutor::init(struct k_msgq *execute_cmd_queue) {
    execute_cmd_queue_ = execute_cmd_queue;
    
    // Initialize hardware
    stepper_left_.initialize();
    stepper_right_.initialize();
    servo_marker_.initialize();
    servo_eraser_.initialize();

    #ifdef DEBUG_MOTION
    printk("MotionExecutor: Initialized\n");
    #endif
}


void MotionExecutor::processCommands() {
    ExecuteCommand cmd;

    // Block until a command is available
    if (k_msgq_get(execute_cmd_queue_, &cmd, K_FOREVER) != 0) {
        return;
    }

    #ifdef DEBUG_MOTION
    printk("MotionExecutor: Received command\n");
    cmd.print();
    #endif

    uint8_t prev_packet_id = current_packet_id_;
    current_packet_id_ = cmd.packet_id();

    switch (cmd.device()) {
        case Device::Steppers:
            #ifdef DEBUG_NAV
            if (current_packet_id_ != prev_packet_id) {
                printk("Executing move (packet %d)\n", current_packet_id_);
            }
            #endif
            executeStepperCommand(cmd);
            break;

        case Device::MarkerServo:
        case Device::EraserServo:
            executeServoCommand(cmd);
            break;

        default:
            printk("MotionExecutor: Unknown device type %d\n", cmd.device());
            break;
    }
}


void MotionExecutor::executeStepperCommand(const ExecuteCommand& cmd) {
    // Queue the command for timer-based execution
    if (k_msgq_put(&step_queue_, &cmd, K_NO_WAIT) != 0) {
        printk("MotionExecutor: Step queue full, dropping command\n");
        return;
    }

    // Start the motor control timer if not already running
    startTimer();
}


void MotionExecutor::executeServoCommand(const ExecuteCommand& cmd) {
    const ExecuteCommand::ServoData& servo_data = cmd.servo();

    #ifdef DEBUG_MOTION
    printk("MotionExecutor: Executing servo command - id=%d, angle=%d\n",
           servo_data.servo_id, servo_data.angle);
    #endif

    if (servo_data.servo_id == 0) {
        servo_marker_.setAngle(servo_data.angle);
    } else {
        servo_eraser_.setAngle(servo_data.angle);
    }

    // Send ACK immediately for servo commands
    sendAck(cmd.packet_id());

    // Start timer to ensure servo has time to move before next command
    startTimer();
}


void MotionExecutor::timerHandler() {
    #ifdef DEBUG_MOTION
    printk("MotionExecutor: Timer handler triggered\n");
    #endif

    ExecuteCommand cmd;
    
    // Get next step command from queue
    if (k_msgq_get(&step_queue_, &cmd, K_NO_WAIT) != 0) {
        // No more commands - stop steppers and timer
        #ifdef DEBUG_MOTION
        printk("MotionExecutor: No more step commands, stopping\n");
        #endif

        stepper_left_.stop();
        stepper_right_.stop();
        stopTimer();

        // Send final ACK for the last processed packet
        sendFinalAck();
        return;
    }

    #ifdef DEBUG_MOTION
    cmd.print();
    #endif

    // Execute the command based on type
    if (cmd.device() == Device::Steppers) {
        const ExecuteCommand::StepperData& stepper_data = cmd.steppers();
        stepper_left_.setVelocity(stepper_data.left_velocity);
        stepper_right_.setVelocity(stepper_data.right_velocity);
    } else {
        // Servo command in step queue (shouldn't happen normally)
        const ExecuteCommand::ServoData& servo_data = cmd.servo();
        if (servo_data.servo_id == 0) {
            servo_marker_.setAngle(servo_data.angle);
        } else {
            servo_eraser_.setAngle(servo_data.angle);
        }
    }

    // Send ACK for this command (only once per packet ID)
    sendAck(cmd.packet_id());
}


void MotionExecutor::sendAck(uint8_t packet_id) {
    if (g_bleService == nullptr) {
        return;
    }

    // Only send ACK if we haven't already ACKed this packet
    if (packet_id != last_acked_packet_id_) {
        last_acked_packet_id_ = packet_id;
        
        char ack[sizeof("aok\n")] = "aok\n";
        ack[0] = packet_id;

        g_bleService->send(ack, sizeof(ack));
    }
}


void MotionExecutor::sendFinalAck() {
    if (g_bleService == nullptr) {
        return;
    }

    // Send final ACK if we haven't already for the current packet
    if (current_packet_id_ != last_acked_packet_id_) {
        last_acked_packet_id_ = current_packet_id_;

        char ack[sizeof("aok\n")] = "aok\n";
        ack[0] = current_packet_id_;

        g_bleService->send(ack, sizeof(ack));
    }
}


void MotionExecutor::startTimer() {
    if (!k_timer_remaining_ticks(&motor_control_timer)) {
        #ifdef DEBUG_MOTION
        printk("MotionExecutor: Starting motor control timer\n");
        #endif
        k_timer_start(&motor_control_timer, K_NO_WAIT, K_MSEC((int)(STEPPER_CTRL_PERIOD * 1000)));
    }
}


void MotionExecutor::stopTimer() {
    k_timer_stop(&motor_control_timer);
    
    #ifdef DEBUG_MOTION
    printk("MotionExecutor: Stopped motor control timer\n");
    #endif
}


void MotionExecutor::reset() {
    // Stop all motion
    stepper_left_.stop();
    stepper_right_.stop();
    stopTimer();

    // Clear step queue
    k_msgq_purge(&step_queue_);

    // Reset ACK tracking
    last_acked_packet_id_ = 0xFF;
    current_packet_id_ = 0;

    #ifdef DEBUG_MOTION
    printk("MotionExecutor: State reset\n");
    #endif
}


// ----------------------------
// Motion Execute Thread
// ----------------------------

void motion_execute_thread(void *execute_cmd_msgq_void, void *arg2, void *arg3) {
    auto *execute_cmd_msgq = static_cast<struct k_msgq *>(execute_cmd_msgq_void);
    ARG_UNUSED(arg2);
    ARG_UNUSED(arg3);

    // Initialize the global executor
    g_motionExecutor.init(execute_cmd_msgq);

    #ifdef DEBUG_MOTION
    printk("Motion execute thread started\n");
    #endif

    while (1) {
        g_motionExecutor.processCommands();
    }
}
