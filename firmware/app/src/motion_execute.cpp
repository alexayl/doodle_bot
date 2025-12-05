#include <zephyr/kernel.h>
#include <stdlib.h>

#include "motion_plan.h"
#include "motion_execute.h"
#include "ble_service.h"
#include "config.h"

// Static stepper instances
Stepper MotionExecutor::stepper_left_(STEPPER_LEFT);
Stepper MotionExecutor::stepper_right_(STEPPER_RIGHT);


// ----------------------------
// MotionExecutor Implementation
// ----------------------------

MotionExecutor::MotionExecutor() : servo_marker_("servom"), servo_eraser_("servoe") {
    
    // init the hardware
    stepper_left_.initialize();
    stepper_right_.initialize();
    servo_marker_.initialize();
    servo_eraser_.initialize();

    #ifdef DEBUG_MOTION_EXECUTION
    printk("MotionExecutor: Initialized\n");
    #endif
}


void MotionExecutor::consumeCommands(const ExecuteCommand& cmd) {

    switch (cmd.device()) {
        case Device::Steppers:
            executeStepperCommand(cmd);
            break;

        case Device::MarkerServo:
        case Device::EraserServo:
            executeServoCommand(cmd);
            break;

        default:
            printk("MotionExecutor: Unknown device type %d\n", static_cast<int>(cmd.device()));
            break;
    }
}


void MotionExecutor::executeStepperCommand(const ExecuteCommand& cmd)
{
    #ifdef DEBUG_MOTION_EXECUTION
    printk("MotionExecutor: Executing stepper command - left=%d, right=%d\n",
           cmd.steppers().left_velocity, cmd.steppers().right_velocity);
    #endif

    // start motors
    stepper_left_.setVelocity(cmd.steppers().left_velocity);
    stepper_right_.setVelocity(cmd.steppers().right_velocity);

    // block for duration of command (convert seconds to milliseconds)
    k_sleep(K_MSEC((int)(STEPPER_CTRL_PERIOD * 1000)));

    // stop motors
    stepper_left_.setVelocity(0);
    stepper_right_.setVelocity(0);
}


void MotionExecutor::executeServoCommand(const ExecuteCommand& cmd) {
    const ExecuteCommand::ServoData& servo_data = cmd.servo();

    #ifdef DEBUG_MOTION_EXECUTION
    printk("MotionExecutor: Executing servo command - id=%d, angle=%d\n",
           servo_data.servo_id, servo_data.angle);
    #endif

    if (servo_data.servo_id == 0) {
        servo_marker_.setAngle(servo_data.angle);
    } else {
        servo_eraser_.setAngle(servo_data.angle);
    }
}


void MotionExecutor::reset() {
    stepper_left_.stop();
    stepper_right_.stop();

    #ifdef DEBUG_MOTION_EXECUTION
    printk("MotionExecutor: State reset\n");
    #endif
}


// ----------------------------
// Motion Execute Thread
// ----------------------------

// Timeout for detecting end of transmission (ACK last packet after this)
#define LAST_PACKET_ACK_TIMEOUT_MS 500

void motion_execute_thread(void *execute_cmd_msgq_void, void *arg2, void *arg3) {
    auto *execute_cmd_msgq = static_cast<struct k_msgq *>(execute_cmd_msgq_void);
    ARG_UNUSED(arg2);
    ARG_UNUSED(arg3);

    // Initialize the BLE ACK queue
    BleService::initAckQueue();

    // Initialize the executor
    MotionExecutor motionExecutor;

    #ifdef DEBUG_MOTION_EXECUTION
    printk("Motion execute thread started\n");
    #endif

    ExecuteCommand cmd;
    uint8_t last_packet_id = 0xFF;  // 0xFF = no previous packet
    bool last_packet_acked = true;

    while (1) {
        // Wait for message with timeout (to detect end of transmission)
        int ret = k_msgq_get(execute_cmd_msgq, &cmd, K_MSEC(LAST_PACKET_ACK_TIMEOUT_MS));
        
        if (ret == -EAGAIN) {
            // Timeout - no new commands, ACK the last packet if not already done
            if (!last_packet_acked && last_packet_id != 0xFF && g_bleService != nullptr) {
                #ifdef DEBUG_BLE
                printk("TIMEOUT: ACKing last packet pid=%d\n", last_packet_id);
                #endif
                g_bleService->queueAck(last_packet_id);
                last_packet_acked = true;
            }
            continue;
        }

        // Got a new command - if packet ID changed, ACK the previous packet
        if (last_packet_id != 0xFF && cmd.packet_id() != last_packet_id && g_bleService != nullptr) {
            g_bleService->queueAck(last_packet_id);
        }
        last_packet_id = cmd.packet_id();
        last_packet_acked = false;

        // Execute the command 
        motionExecutor.consumeCommands(cmd);
    }
}
