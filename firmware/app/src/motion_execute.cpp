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
            executeStepperCommand(cmd);
            break;
        case Device::MarkerServo:
        case Device::EraserServo:
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

void MotionExecutor::executeServoCommand(const ExecuteCommand& cmd) {
    const auto& servo = cmd.servo();
    if (servo.servo_id == 0) {
        servo_marker_.setAngle(servo.angle);
    } else {
        servo_eraser_.setAngle(servo.angle);
    }
}

void MotionExecutor::reset() {
    stepper_left_.stop();
    stepper_right_.stop();
}

// --- Thread ---

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
