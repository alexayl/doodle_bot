#pragma once
#include <zephyr/drivers/stepper.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <math.h>
#include "servo.h"
#include "simple_led.h"
#include "pwm_buzzer.h"

class Peripheral {
public:
    virtual void initialize() = 0;
    virtual ~Peripheral() = default;
};

/* Stepper motor wrapper using Zephyr's stepper API */
class Stepper: public Peripheral {
public:
    Stepper(const struct device* dev) : dev_(dev) {}
    
    void initialize() override {
        if (dev_ && device_is_ready(dev_)) stepper_enable(dev_);
    }
    
    void stop() { if (dev_) stepper_stop(dev_); }
    
    void setVelocity(float velocity_deg_s) {
        if (!dev_) return;
        velocity_deg_s = -velocity_deg_s;  // Match robot's forward direction
        
        if (fabsf(velocity_deg_s) < 0.01f) {
            stepper_stop(dev_);
            return;
        }
        
        // Convert deg/s to step interval: 200 steps/rev * 16 microsteps / 360 deg
        float steps_per_second = fabsf(velocity_deg_s) * (200.0f * 16) / 360.0f;
        stepper_set_microstep_interval(dev_, (uint64_t)(1e9f / steps_per_second));
        stepper_run(dev_, velocity_deg_s > 0 ? STEPPER_DIRECTION_POSITIVE : STEPPER_DIRECTION_NEGATIVE);
    }
    
    void enable()  { if (dev_) stepper_enable(dev_); }
    void disable() { if (dev_) stepper_disable(dev_); }

private:
    const struct device* dev_;
};

#define STEPPER_LEFT  DEVICE_DT_GET_OR_NULL(DT_NODELABEL(stepper_right))
#define STEPPER_RIGHT DEVICE_DT_GET_OR_NULL(DT_NODELABEL(stepper_left))

class Servo : public Peripheral {
public:
    Servo(const char* alias) : alias_(alias) { dev_ = servo_init_by_alias(alias_); }
    void initialize() override { dev_ = servo_init_by_alias(alias_); }
    int setAngle(uint8_t angle) { return servo_set_angle(dev_, angle); }

private:
    const char* alias_;
    const struct device* dev_;
};

class Led: public Peripheral {
public:
    void initialize() override { simple_led_init(); }
    int turnOn()  { return simple_led_on(); }
    int turnOff() { return simple_led_off(); }
    int set(int state) { return led_driver_set(state); }
};

class Buzzer: public Peripheral {
public:
    void initialize() override { pwm_buzzer_init(); }
    void buzzOn()  { pwm_buzzer_on(); }
    void buzzOff() { pwm_buzzer_off(); }
    void toggle()  { pwm_buzzer_toggle(); }
    bool isReady() { return pwm_buzzer_is_ready(); }
    void beep(uint16_t freq_hz, uint8_t volume_pct, uint32_t duration_ms) {
        pwm_buzzer_beep(freq_hz, volume_pct, duration_ms);
    }
};
