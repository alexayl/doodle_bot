# Doodlebot Firmware Drivers

## Overview

This directory contains custom Zephyr RTOS drivers for the Doodlebot drawing robot. The drivers provide a clean abstraction layer between high-level control algorithms and low-level hardware interfaces, enabling intuitive control of motors, actuators, and feedback devices.

## Architecture

The driver architecture follows a layered approach:

```
┌─────────────────────────────────────────┐
│          Application Layer              │
│     (Path Planning, Drawing Logic)      │
└─────────────────────────────────────────┘
                    │
┌─────────────────────────────────────────┐
│        Control Thread (50 Hz)          │
│    (Motor Control, State Management)    │
└─────────────────────────────────────────┘
                    │
┌─────────────────────────────────────────┐
│          Driver API Layer              │
│     (stepper.h, buzzer.h, servo.h)     │
└─────────────────────────────────────────┘
                    │
┌─────────────────────────────────────────┐
│        Hardware Abstraction            │
│     (GPIO, Timers, Device Tree)        │
└─────────────────────────────────────────┘
                    │
┌─────────────────────────────────────────┐
│          Physical Hardware             │
│  (A4988 Steppers, Servos, Buzzer)     │
└─────────────────────────────────────────┘
```

## Features

### Stepper Motor Driver
- **Input Interface**: Angular velocity in degrees/second
- **Hardware Support**: A4988/DRV8825 stepper driver modules
- **Microstepping**: 16x microstepping for smooth motion
- **Real-time Control**: Optimized for 50 Hz velocity updates
- **Dual Motor Support**: Independent left/right wheel control

### Servo Motor Driver  
- **Input Interface**: Angle in degrees (0-180°)
- **Hardware Support**: Standard PWM servo motors
- **Dual Servo Control**: Eraser and marker pen control
- **Smooth Movement**: PWM-based positioning

### LED Driver
- **Input Interface**: Simple on/off control
- **Hardware Support**: GPIO-controlled LEDs
- **Status Indication**: Visual feedback for system states
- **Multi-LED Support**: Independent control of multiple LEDs

### Buzzer Driver
- **Input Interface**: Simple on/off and beep patterns
- **Hardware Support**: GPIO-controlled piezo buzzer
- **Feedback**: Audio feedback for user interactions

## Driver Details

### Stepper Motor Driver (`stepper/`)

**Purpose**: Controls dual stepper motors for robot movement

**Key Specifications**:
- **Motor Type**: 1.8° stepper motors (200 steps/revolution)
- **Driver Modules**: A4988 or DRV8825 with 16x microstepping
- **Resolution**: 3200 microsteps per revolution
- **Conversion**: `f_step = velocity_deg_s × (3200/360) = velocity_deg_s × 8.89`

**API Functions**:
```c
int stepper_init(void);
int stepper_enable(enum stepper_motor motor);
int stepper_set_velocity(enum stepper_motor motor, float velocity_deg_s);
int stepper_stop(enum stepper_motor motor);
float stepper_get_velocity(enum stepper_motor motor);
```

**Device Tree Binding**: `stepper-driver.yaml`
```yaml
stepper_left: stepper-left {
    compatible = "stepper-driver";
    step-gpio = <&gpio0 12 GPIO_ACTIVE_HIGH>;
    direction-gpio = <&gpio0 13 GPIO_ACTIVE_HIGH>;
    enable-gpio = <&gpio0 14 GPIO_ACTIVE_LOW>;
    steps-per-revolution = <200>;
    microsteps = <16>;
};
```

**Hardware Interface**:
- **STEP**: GPIO pulse train at calculated frequency
- **DIR**: GPIO level for rotation direction
- **EN**: GPIO level for driver enable/disable

**Real-time Operation**:
- Designed for 50 Hz velocity updates from control thread
- Uses Zephyr timers for precise step pulse generation
- Optimizes GPIO writes by only changing direction when needed

**Pseudocode Implementation**:

```pseudocode
// ========== CORE CONCEPT ==========
// We convert degrees/second to timer interrupts that toggle GPIO pins
// 1.8° motor + 16x microstepping = 3200 steps per full rotation
// So: 1 deg/s = 8.89 steps/s

// ========== MAIN VELOCITY FUNCTION ==========
FUNCTION stepper_set_velocity(motor, velocity_deg_s):
    // Step 1: Convert degrees/s to step frequency
    steps_per_second = abs(velocity_deg_s) * 8.89
    
    // Step 2: Determine direction (positive = clockwise)
    clockwise = (velocity_deg_s >= 0)
    
    // Step 3: Set direction pin (only if direction changed)
    IF direction_changed:
        gpio_set(direction_pin, clockwise)
    
    // Step 4: Start timer to generate step pulses
    IF steps_per_second > 0 AND motor_enabled:
        timer_period = 500000 / steps_per_second  // microseconds
        start_timer(timer_period)
    ELSE:
        stop_timer()

// ========== TIMER INTERRUPT ==========
TIMER_INTERRUPT step_pulse_generator():
    // This runs every timer_period microseconds
    toggle_gpio(step_pin)  // Creates the step pulse for A4988

// ========== MOTOR CONTROL ==========
FUNCTION stepper_enable(motor):
    gpio_set(enable_pin, LOW)    // A4988 enable is active LOW
    motor_enabled = TRUE

FUNCTION stepper_disable(motor):
    stop_timer()                 // Stop stepping first
    gpio_set(enable_pin, HIGH)   // Disable A4988
    motor_enabled = FALSE

// ========== SIMPLE EXAMPLE ==========
// stepper_set_velocity(LEFT_MOTOR, 180.0)
// 
// 1. steps_per_second = 180 * 8.89 = 1600 Hz
// 2. timer_period = 500000 / 1600 = 312.5 μs
// 3. Timer toggles GPIO every 312.5 μs
// 4. Result: 1600 step pulses per second = 0.5 revolutions/s = 180°/s
```

### Example Calculations

| Input Velocity | Step Frequency | Timer Period | Physical Result |
|----------------|----------------|---------------|-----------------|
| 360 deg/s | 3200 steps/s | 156.25 μs | 1.0 rev/s |
| 180 deg/s | 1600 steps/s | 312.5 μs | 0.5 rev/s |
| 90 deg/s | 800 steps/s | 625 μs | 0.25 rev/s |

### Servo Motor Driver (`servo/`)

**Purpose**: Controls servo motors for drawing tool positioning

**Hardware Support**: Standard PWM servo motors (SG90, MG996R, etc.)

**API Functions**:
```c
int servo_init(void);
int servo_set_angle(enum servo_motor servo, float angle_degrees);
float servo_get_angle(enum servo_motor servo);
```

**Pseudocode Implementation**:
```pseudocode
FUNCTION servo_set_angle(servo, angle_degrees):
    // Convert angle to PWM duty cycle
    // Standard servo: 1ms (0°) to 2ms (180°) pulse width
    pulse_width_us = 1000 + (angle_degrees / 180.0) * 1000
    duty_cycle = pulse_width_us / 20000  // 20ms period
    
    // Update PWM channel
    pwm_set_duty_cycle(servo_pwm_channel, duty_cycle)
    
    // Store current angle
    servo_state[servo].current_angle = angle_degrees
```

### LED Driver (`led/`)

**Purpose**: Provides visual status indication and user feedback

**Hardware Support**: GPIO-controlled LEDs (power, status, drawing indicators)

**API Functions**:
```c
int led_init(void);
int led_on(enum led_type led);
int led_off(enum led_type led);
int led_toggle(enum led_type led);
int led_blink(enum led_type led, uint32_t period_ms);
```

**Device Tree Binding**: `gpio-leds.yaml`
```yaml
leds {
    compatible = "gpio-leds";
    led_power: led-power {
        gpios = <&gpio0 15 GPIO_ACTIVE_HIGH>;
        label = "Power LED";
    };
    led_drawing: led-drawing {
        gpios = <&gpio0 16 GPIO_ACTIVE_HIGH>;
        label = "Drawing Status LED";
    };
};
```

**Pseudocode Implementation**:
```pseudocode
FUNCTION led_on(led_type):
    gpio_set(led_gpio[led_type], HIGH)
    led_state[led_type] = ON

FUNCTION led_blink(led_type, period_ms):
    led_state[led_type] = BLINKING
    start_timer(period_ms/2, led_blink_callback)

TIMER_INTERRUPT led_blink_callback():
    toggle_gpio(led_gpio[led_type])
```

### Buzzer Driver (`buzzer/`)

**Purpose**: Provides audio feedback for user interactions

**Hardware Support**: GPIO-controlled piezo buzzer

**API Functions**:
```c
int buzzer_init(void);
int buzzer_on(void);
int buzzer_off(void);
int buzzer_beep(uint32_t duration_ms);
```

**Pseudocode Implementation**:
```pseudocode
FUNCTION buzzer_beep(duration_ms):
    gpio_set(buzzer_gpio, HIGH)
    start_timer(duration_ms, buzzer_off_callback)

TIMER_INTERRUPT buzzer_off_callback():
    gpio_set(buzzer_gpio, LOW)

FUNCTION buzzer_on():
    gpio_set(buzzer_gpio, HIGH)
    buzzer_state = ON

FUNCTION buzzer_off():
    gpio_set(buzzer_gpio, LOW)
    buzzer_state = OFF
```

## Testing

Each driver includes comprehensive tests in the `tests/` directory:

- **`tests/stepper/`**: Revolution tests and velocity validation
- **`tests/servo/`**: Angle positioning tests  
- **`tests/led/`**: LED pattern and timing tests
- **`tests/buzzer/`**: Audio pattern tests

Run tests with:
```bash
just test-stepper
just test-servo  
just test-led
just test-buzzer
```

## Hardware Configuration

### Platform Support
- **Primary**: ESP32-S3 DevKitC
- **Secondary**: Nordic nRF52840 DK

### Pin Assignments
See device tree overlay files:
- `app/boards/esp32s3_devkitc.overlay`
- `app/boards/nucleo_f302r8.overlay`

## Development Notes

### Performance Considerations
- Stepper driver optimized for high-frequency operation (>3 kHz step rates)
- Direction changes minimized to reduce GPIO overhead
- Timer-based pulse generation for precise timing

### Error Handling
- Graceful degradation on hardware initialization failures
- Parameter validation for velocity and angle limits
- Logging integration for debugging and monitoring

### Future Enhancements
- Acceleration/deceleration profiles for stepper motors
- Closed-loop position feedback
- Dynamic microstepping adjustment
- Power management features

