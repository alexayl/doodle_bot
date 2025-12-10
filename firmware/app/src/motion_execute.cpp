#include <zephyr/kernel.h>
#include "motion_execute.h"
#include "ble_service.h"
#include "config.h"

// Static stepper instances
Stepper MotionExecutor::stepper_left_(STEPPER_LEFT);
Stepper MotionExecutor::stepper_right_(STEPPER_RIGHT);

// Stepper motion timer - stops motors after STEPPER_CTRL_PERIOD
K_SEM_DEFINE(stepper_motion_complete, 0, 1);

static void stepper_timer_expiry(struct k_timer *timer)
{
    ARG_UNUSED(timer);
    MotionExecutor::stepperLeft().setVelocity(0);
    MotionExecutor::stepperRight().setVelocity(0);
    k_sem_give(&stepper_motion_complete);
}

K_TIMER_DEFINE(stepper_motion_timer, stepper_timer_expiry, NULL);

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
            printk("[EXEC] G1 - Begin stepper move (L:%.2f R:%.2f deg/s)\n", 
                   (double)cmd.steppers().left_velocity, (double)cmd.steppers().right_velocity);
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
    
    // Start timer to stop motors after control period
    k_timer_start(&stepper_motion_timer, K_MSEC((int)(STEPPER_CTRL_PERIOD * 1000)), K_NO_WAIT);
    
    // Wait for timer callback to complete motion
    k_sem_take(&stepper_motion_complete, K_FOREVER);
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
