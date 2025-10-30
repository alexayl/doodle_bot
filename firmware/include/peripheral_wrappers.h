#pragma once
#include "stepper.h"
#include "servo.h"
#include "simple_led.h"
#include "buzzer.h"

class Peripheral {
public:
    virtual void initialize() = 0;
    virtual ~Peripheral() = default;
};

class Stepper: public Peripheral {
public:
    Stepper(stepper_motor motor) : motor_(motor) {}
    void initialize() override { stepper_init(); stepper_enable(motor_); }
    void stop() { stepper_stop(motor_); }
    void setVelocity(float velocity) { stepper_set_velocity(motor_, velocity); }

private:
    stepper_motor motor_;
};

class Servo : public Peripheral {
public:
    Servo(const char* alias) : alias_(alias), dev_(nullptr) {}
    
    void initialize() override { 
        dev_ = servo_init_by_alias(alias_); 
    }
    
    int setAngle(uint8_t angle) { 
        return servo_set_angle(dev_, angle); 
    }

private:
    const char* alias_;
    const struct device* dev_;
};

class Led: public Peripheral {
public:
    void initialize() override { 
        simple_led_init(); 
    }
    
    int turnOn() { 
        return simple_led_on(); 
    }
    
    int turnOff() { 
        return simple_led_off(); 
    }
    
    int set(int state) {
        return led_driver_set(state);
    }

private:
};

class Buzzer: public Peripheral {
public:
    void initialize() override { 
        buzzer_init(); 
    }
    
    void buzzOn() { 
        buzzer_on(); 
    }
    
    void buzzOff() { 
        buzzer_off(); 
    }
    
    void toggle() {
        buzzer_toggle();
    }
    
    bool isReady() {
        return buzzer_is_ready();
    }

private:
};