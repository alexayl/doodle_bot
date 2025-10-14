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

**Device Tree Binding**: `dts/bindings/stepper/stepper-driver.yaml`
```yaml
stepper_left: stepper_left {
    compatible = "doodle,a4988-stepper";
    micro-step-res = <16>;
    step-gpios = <&gpio0 5 GPIO_ACTIVE_HIGH>;      /* ESP32: GPIO5, nRF52: P0.02 */
    dir-gpios = <&gpio0 6 GPIO_ACTIVE_HIGH>;       /* ESP32: GPIO6, nRF52: P0.03 */
    en-gpios = <&gpio0 7 GPIO_ACTIVE_LOW>;         /* ESP32: GPIO7, nRF52: P0.04 */
    label = "LEFT_STEPPER";
};
```

**Hardware Interface**:
- **STEP**: GPIO pulse train at calculated frequency
- **DIR**: GPIO level for rotation direction (HIGH = clockwise)
- **EN**: GPIO level for driver enable/disable (Active LOW)

**A4988/DRV8825 Driver Connections**:
```
ESP32-S3/nRF52840 → A4988/DRV8825 Driver
STEP pin          → STEP
DIR pin           → DIR
EN pin            → EN (Enable)
3.3V              → VDD (logic power)
3.3V              → RESET (tie high)
3.3V              → SLEEP (tie high)
3.3V              → MS1, MS2, MS3 (for 16x microstepping)
GND               → GND
                    VMOT ← 12V Motor Power Supply
                    1A, 1B, 2A, 2B → Stepper Motor Phases
```

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

### Servo Motor Driver (`servo/servo.c`)

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

### LED Driver (`led/led_gpio.c`)

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

Each driver includes comprehensive tests in the `../tests/` directory (relative to drivers/):

### Stepper Motor Tests (`../tests/stepper/stepper_control.cpp`)
**Purpose**: Validates stepper motor velocity control and direction changes
**Test Pattern**:
```cpp
// Continuous test cycle:
1. Forward rotation: 360°/s (1 revolution/second) for 2 seconds
2. Stop for 1 second  
3. Reverse rotation: -180°/s (0.5 revolution/second) for 2 seconds
4. Stop for 2 seconds and repeat
```
**Expected Behavior**: Left stepper motor should make 2 full forward rotations, pause, then 1 reverse rotation, creating a distinctive back-and-forth pattern
**Hardware Validation**: Visually confirm motor rotation direction and speed match the programmed values

### Servo Motor Tests (`../tests/servo/servo_control.cpp`)
**Purpose**: Tests dual servo positioning for eraser and marker control
**Test Pattern**:
```cpp
// Synchronized servo movement with offset pattern:
Eraser positions: 0° → 90° → 180° → 90° → repeat
Marker positions: 180° → 90° → 0° → 90° → repeat (offset by 2 steps)
```
**Expected Output**:
```
Dual Servo Demo - Eraser: servo_eraser, Marker: servo_marker
Eraser: 0°, Marker: 180°
Eraser: 90°, Marker: 90° 
Eraser: 180°, Marker: 0°
Eraser: 90°, Marker: 90°
```
**Hardware Validation**: Both servo horns should move in coordinated but offset patterns

### LED Driver Tests (`../tests/led/led_control.cpp`)
**Purpose**: Tests LED abstraction layer and GPIO control
**Test Pattern**:
```cpp
// Simple ON/OFF toggle with driver abstraction:
500ms ON → 500ms OFF → repeat
Uses led_command_t enum (LED_ON/LED_OFF) sent to led_driver_set()
```
**Expected Output**:
```
LED Driver Test starting...
LED device initialized successfully
GPIO11 LED driver ready
Testing both ACTIVE_HIGH and ACTIVE_LOW configurations...
LED ON
LED ON
LED ON
...
```
**Hardware Validation**: Status LED should blink at 1Hz (500ms on, 500ms off)

### Buzzer Driver Tests (`../tests/buzzer/buzzer_test.cpp`)
**Purpose**: Tests buzzer GPIO control and audio feedback
**Test Pattern**:
```cpp
// Simple buzzer toggle:
500ms ON → 500ms OFF → repeat
Uses buzzer_toggle() function for state changes
```
**Expected Output**:
```
Buzzer Driver Test - 20ms Toggle
Buzzer initialized successfully - starting toggle test
```
**Hardware Validation**: Should hear intermittent buzzer sound at 1Hz

**Run tests with justfile commands**:
```bash
# Stepper motor tests
just test-nrf-stepper    # nRF52840DK: Pins P0.02, P0.03, P0.04
just test-esp-stepper    # ESP32-S3: Pins GPIO5, GPIO6, GPIO7

# Servo motor tests  
just test-nrf-servo      # nRF52840DK: PWM pins P0.13, P0.03
just test-esp-servo      # ESP32-S3: PWM pins GPIO2, GPIO3

# LED tests
just test-nrf-led        # nRF52840DK: GPIO P0.14
just test-esp-led        # ESP32-S3: GPIO11

# Buzzer tests
just test-nrf-buzzer     # nRF52840DK: GPIO P0.15
just test-esp-buzzer     # ESP32-S3: GPIO12
```

**Test Build System Integration**:
Tests are integrated into the CMake build system via `CONFIG_*_TEST` flags:
- `CONFIG_STEPPER_TEST=y` → Builds `stepper_control.cpp` + `stepper.c`
- `CONFIG_SERVO_TEST=y` → Builds `servo_control.cpp` + `servo.c`
- `CONFIG_LED_TEST=y` → Builds `led_control.cpp` + `led_gpio.c`
- `CONFIG_BUZZER_TEST=y` → Builds `buzzer_test.cpp` + `buzzer.c`

## Hardware Configuration

### Platform Support
- **Primary**: ESP32-S3 DevKitC
- **Secondary**: Nordic nRF52840 DK

### Pin Assignments
See device tree overlay files for complete pin mappings:
- `app/boards/esp32s3_devkitc_procpu.overlay` (ESP32-S3 DevKitC)
- `app/boards/nrf52840dk_nrf52840.overlay` (Nordic nRF52840 DK)

**ESP32-S3 DevKitC Pinout**:
```
Left Stepper:  GPIO5 (STEP), GPIO6 (DIR), GPIO7 (EN)
Right Stepper: GPIO8 (STEP), GPIO9 (DIR), GPIO10 (EN) [disabled]
Servo Eraser:  GPIO2 (PWM)
Servo Marker:  GPIO3 (PWM)
Status LED:    GPIO11
Buzzer:        GPIO12
```

**nRF52840 DK Pinout**:
```
Left Stepper:  P0.02 (STEP), P0.03 (DIR), P0.04 (EN)
Right Stepper: P0.05 (STEP), P0.06 (DIR), P0.07 (EN)
Servo Eraser:  P0.13 (PWM)
Servo Marker:  P0.03 (PWM) [shared with stepper DIR]
Status LED:    P0.14
Buzzer:        P0.15
```

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
- **Stepper Improvements**: Acceleration/deceleration profiles for smoother motion
- **Position Feedback**: Closed-loop position control with encoders
- **Dynamic Microstepping**: Adaptive microstepping based on speed requirements
- **Power Management**: Sleep modes and current reduction during idle
- **Dual Motor Coordination**: Synchronized control for differential drive
- **Safety Features**: Thermal protection and stall detection
- **Integration**: Integrate with Core Firmware

