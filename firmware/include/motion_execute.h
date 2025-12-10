#pragma once

#include <zephyr/kernel.h>
#include <stdint.h>
#include "peripheral_wrappers.h"
#include "config.h"

// Forward declaration
class BleService;
extern BleService* g_bleService;

/**
 * @brief Device types for execution commands
 */
enum Device : uint8_t {
    Steppers,
    MarkerServo,
    EraserServo
};

/**
 * @brief Command structure for motion execution
 * 
 * Encapsulates either a stepper velocity command or a servo angle command,
 * along with packet ID for ACK tracking.
 */
class ExecuteCommand {
public:
    // Data structures
    struct StepperData {
        float left_velocity;   // degrees/second
        float right_velocity;  // degrees/second
    };

    struct ServoData {
        uint8_t servo_id;
        uint8_t angle;
    };

    // Default constructor
    ExecuteCommand() : device_(Steppers), packet_id_(0), steppers_{0.0f, 0.0f}, servo_{0, 0} {}

    /**
     * @brief Set the command data
     * @param device_type The target device type
     * @param cmd Pointer to command data (StepperData* or ServoData*)
     * @param packet_id Packet ID for ACK tracking
     */
    void set(Device device_type, const void *cmd, uint8_t packet_id) {
        device_ = device_type;
        packet_id_ = packet_id;

        switch (device_type) {
            case Device::Steppers:
                steppers_ = *static_cast<const StepperData*>(cmd);
                break;

            case Device::MarkerServo:
                servo_ = *static_cast<const ServoData*>(cmd);
                servo_.servo_id = 0;
                break;

            case Device::EraserServo:
                servo_ = *static_cast<const ServoData*>(cmd);
                servo_.servo_id = 1;
                break;

            default:
                break;
        }
    }

    // Accessors
    Device device() const { return device_; }
    uint8_t packet_id() const { return packet_id_; }
    const StepperData& steppers() const { return steppers_; }
    const ServoData& servo() const { return servo_; }

    void print() const {
        if (device_ == Steppers) {
            printk("ExecuteCommand[STEPPER]: packet=%d, left=%.2f, right=%.2f\n",
                   packet_id_, (double)steppers_.left_velocity, (double)steppers_.right_velocity);
        } else {
            printk("ExecuteCommand[SERVO]: packet=%d, servo_id=%d, angle=%d\n",
                   packet_id_, servo_.servo_id, servo_.angle);
        }
    }

private:
    Device device_;
    uint8_t packet_id_;
    StepperData steppers_;
    ServoData servo_;
};


/**
 * @brief Motion Executor class
 * 
 * Handles execution of motion commands: stepper velocity control
 * and servo angle control. ACKs are managed by the motion execute thread
 * via BleService::queueAck().
 */
class MotionExecutor {
public:
    MotionExecutor();

    /**
     * @brief Process incoming commands from the queue
     * Called by the motion execute thread
     */
    void consumeCommands(const ExecuteCommand& cmd);

    /**
     * @brief Reset executor state (stop all motors)
     */
    void reset();

    /**
     * @brief Stop stepper motors (called when no more stepper commands pending)
     */
    void stopSteppers();

    // Static accessors for hardware
    static Stepper& stepperLeft() { return stepper_left_; }
    static Stepper& stepperRight() { return stepper_right_; }

private:
    // Hardware peripherals
    static Stepper stepper_left_;
    static Stepper stepper_right_;
    Servo servo_marker_;
    Servo servo_eraser_;

    // Helper methods
    void executeStepperCommand(const ExecuteCommand& cmd);
    void executeServoCommand(const ExecuteCommand& cmd);
};


/**
 * @brief Motion execute thread entry point
 * @param execute_cmd_msgq_void Pointer to ExecuteCommand message queue
 * @param arg2 Unused
 * @param arg3 Unused
 */
void motion_execute_thread(void *execute_cmd_msgq_void, void *arg2, void *arg3);
