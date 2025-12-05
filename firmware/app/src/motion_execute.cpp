#include <zephyr/kernel.h>
#include "motion_execute.h"
#include "ble_service.h"
#include "config.h"

// Static stepper instances
Stepper MotionExecutor::stepper_left_(STEPPER_LEFT);
Stepper MotionExecutor::stepper_right_(STEPPER_RIGHT);

MotionExecutor::MotionExecutor() : servo_marker_("servom"), servo_eraser_("servoe") {
    stepper_left_.initialize();
    stepper_right_.initialize();
    servo_marker_.initialize();
    servo_eraser_.initialize();
}

void MotionExecutor::consumeCommands(const ExecuteCommand& cmd) {
    switch (cmd.device()) {
        case Device::Steppers:
            #ifdef DEBUG_MOTION_EXECUTION
            printk("[EXEC] G1 - Begin stepper move (L:%d R:%d deg/s)\n", cmd.steppers().left_velocity, cmd.steppers().right_velocity);
            #endif
            executeStepperCommand(cmd);
            break;
        case Device::MarkerServo:
            #ifdef DEBUG_MOTION_EXECUTION
            printk("[EXEC] M280 P0 S%d - Begin marker servo move\n", cmd.servo().angle);
            #endif
            executeServoCommand(cmd);
            break;
        case Device::EraserServo:
            #ifdef DEBUG_MOTION_EXECUTION
            printk("[EXEC] M280 P1 S%d - Begin eraser servo move\n", cmd.servo().angle);
            #endif
            executeServoCommand(cmd);
            break;
        default:
            printk("MotionExecutor: Unknown device %d\n", static_cast<int>(cmd.device()));
            break;
    }
}

void MotionExecutor::executeStepperCommand(const ExecuteCommand& cmd) {
    stepper_left_.setVelocity(cmd.steppers().left_velocity);
    stepper_right_.setVelocity(cmd.steppers().right_velocity);
    k_sleep(K_MSEC((int)(STEPPER_CTRL_PERIOD * 1000)));
    stepper_left_.setVelocity(0);
    stepper_right_.setVelocity(0);
}

// Time for servo to reach position (ms) - typical hobby servo is ~200ms for full sweep
#define SERVO_SETTLE_TIME_MS 200

void MotionExecutor::executeServoCommand(const ExecuteCommand& cmd) {
    const auto& servo = cmd.servo();

    if (servo.servo_id == 0) {
        servo_marker_.setAngle(servo.angle);
    } else {
        servo_eraser_.setAngle(servo.angle);
    }
    
    // Wait for servo to physically reach position
    k_sleep(K_MSEC(SERVO_SETTLE_TIME_MS));
}

void MotionExecutor::reset() {
    stepper_left_.stop();
    stepper_right_.stop();
}

// ---------------------
// Motion Execute Thread
// ---------------------

#define PACKET_ACK_TIMEOUT_MS 500

void motion_execute_thread(void *msgq, void *, void *) {
    auto *queue = static_cast<struct k_msgq *>(msgq);

    BleService::initAckQueue();
    MotionExecutor executor;
    PacketAckTracker ackTracker;

    ExecuteCommand cmd;
    while (1) {
        if (k_msgq_get(queue, &cmd, K_MSEC(PACKET_ACK_TIMEOUT_MS)) == -EAGAIN) {
            ackTracker.onTimeout();
            continue;
        }
        ackTracker.onCommand(cmd.packet_id());
        executor.consumeCommands(cmd);
    }
}
