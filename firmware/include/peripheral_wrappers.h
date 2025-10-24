#pragma once
#include "stepper.h"

// TODO: Complete
class Peripheral {
public:
    virtual void initialize() = 0;
    virtual ~Peripheral() = default;
};

// TODO: Complete
class Stepper: public Peripheral {
public:
    Stepper(stepper_motor motor) : motor_(motor) {}
    void initialize() override {
        stepper_init();
    }

    void stop() {
        stepper_stop(motor_);
    }

    void setVelocity(float velocity) {
        stepper_set_velocity(motor_, velocity);
    }

private:
    stepper_motor motor_;
};

// TODO: Complete
class Servo {
};

// TODO: Complete
class Led: public Peripheral {
};

// TODO: Complete
class Buzzer: public Peripheral {
};