#pragma once

#include <stdint.h>
#include <zephyr/sys/printk.h>
#include <math.h>
#include "instruction_parser.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

/* Data types */

enum class PeripheralPosition : uint8_t {Down, Up};

enum class StepperDirection : uint8_t {Clockwise, CounterClockwise};

enum class CoordinateMode : uint8_t {Absolute, Relative};

struct Position {
    float x;
    float y;
    float heading_degrees; // 0 = North, 90 = East, 180 = South, 270 = West
    
    Position() : x(0.0f), y(0.0f), heading_degrees(0.0f) {}
    Position(float x, float y, float heading = 0.0f) : x(x), y(y), heading_degrees(heading) {}
};

struct MovementDelta {
    float dx;
    float dy;
    bool pen_down;
    
    MovementDelta() : dx(0.0f), dy(0.0f), pen_down(false) {}
    MovementDelta(float dx, float dy, bool pen = false) : dx(dx), dy(dy), pen_down(pen) {}
};

struct DifferentialDriveParams {
    float wheel_base_mm;        // Distance between left and right wheels
    float wheel_diameter_mm;    // Diameter of drive wheels
    int steps_per_revolution;   // Stepper motor steps per wheel revolution
    
    DifferentialDriveParams() : wheel_base_mm(100.0f), wheel_diameter_mm(50.0f), steps_per_revolution(200) {}
};

struct WheelCommands {
    int left_steps;     // Positive = forward, negative = backward
    int right_steps;    // Positive = forward, negative = backward
    float left_speed;   // Steps per second
    float right_speed;  // Steps per second
    
    WheelCommands() : left_steps(0), right_steps(0), left_speed(0.0f), right_speed(0.0f) {}
};

struct nav_instr_t {
    uint8_t x_delta, y_delta;
    PeripheralPosition eraser_position, marker_position; 
};

struct stepper_instr_t {
    uint8_t steps, velocity;
    StepperDirection stepper_direction; 
};

/* Navigation Class */

class Navigation {
private:
    Position current_position;
    Position target_position;
    CoordinateMode coordinate_mode;
    bool pen_is_down;
    DifferentialDriveParams drive_params;
    
    // Convert heading to radians
    float heading_to_radians() const {
        return current_position.heading_degrees * M_PI / 180.0f;
    }
    
    // Normalize angle to 0-360 degrees
    float normalize_angle(float degrees) const {
        while (degrees < 0) degrees += 360.0f;
        while (degrees >= 360.0f) degrees -= 360.0f;
        return degrees;
    }

public:
    Navigation() : coordinate_mode(CoordinateMode::Absolute), pen_is_down(false) {
        current_position = Position(0.0f, 0.0f, 0.0f);
        target_position = current_position;
    }
    
    // Configure differential drive parameters
    void setDriveParams(float wheel_base_mm, float wheel_diameter_mm, int steps_per_rev) {
        drive_params.wheel_base_mm = wheel_base_mm;
        drive_params.wheel_diameter_mm = wheel_diameter_mm;
        drive_params.steps_per_revolution = steps_per_rev;
    }
    
    // Get current position
    const Position& getCurrentPosition() const { return current_position; }
    
    // Get target position 
    const Position& getTargetPosition() const { return target_position; }
    
    // Set coordinate mode (G90 = Absolute, G91 = Relative)
    void setCoordinateMode(CoordinateMode mode) { coordinate_mode = mode; }
    CoordinateMode getCoordinateMode() const { return coordinate_mode; }
    
    // Set pen state
    void setPenDown(bool down) { pen_is_down = down; }
    bool isPenDown() const { return pen_is_down; }
    
    // Process G-code command and calculate movement delta
    MovementDelta processGCodeCommand(const InstructionParser::GCodeCmd& cmd);
    
    // Update current position after movement is complete
    void updatePosition(const MovementDelta& delta);
    
    // Set absolute position (for homing, calibration)
    void setPosition(float x, float y, float heading = -1.0f) {
        current_position.x = x;
        current_position.y = y;
        if (heading >= 0) {
            current_position.heading_degrees = normalize_angle(heading);
        }
        target_position = current_position;
    }
    
    // Rotate to face a target direction
    float calculateHeadingChange(float target_heading_degrees) const {
        float current = current_position.heading_degrees;
        float target = normalize_angle(target_heading_degrees);
        
        float delta = target - current;
        
        // Choose shortest rotation path
        if (delta > 180.0f) delta -= 360.0f;
        if (delta < -180.0f) delta += 360.0f;
        
        return delta;
    }
    
    // Calculate distance to target
    float calculateDistance(float target_x, float target_y) const {
        float dx = target_x - current_position.x;
        float dy = target_y - current_position.y;
        return sqrtf(dx * dx + dy * dy);
    }
    
    // Calculate required heading to reach target
    float calculateRequiredHeading(float target_x, float target_y) const {
        float dx = target_x - current_position.x;
        float dy = target_y - current_position.y;
        
        if (dx == 0.0f && dy == 0.0f) return current_position.heading_degrees;
        
        // Calculate angle in radians, convert to degrees
        float angle_rad = atan2f(dx, dy); // atan2(x, y) for North=0° convention
        float angle_deg = angle_rad * 180.0f / M_PI;
        
        return normalize_angle(angle_deg);
    }
    
    void printStatus() const {
        printk("Navigation Status:\n");
        printk("  Current: (%.2f, %.2f) @ %.1f°\n", 
               (double)current_position.x, (double)current_position.y, (double)current_position.heading_degrees);
        printk("  Target:  (%.2f, %.2f)\n", (double)target_position.x, (double)target_position.y);
        printk("  Mode: %s, Pen: %s\n",
               coordinate_mode == CoordinateMode::Absolute ? "Absolute" : "Relative",
               pen_is_down ? "Down" : "Up");
    }
    
    // Differential Drive Kinematics
    
    // Convert dx/dy movement to left/right wheel commands
    WheelCommands calculateWheelCommands(float dx, float dy, float movement_speed_mm_per_sec = 50.0f) const {
        WheelCommands cmd;
        
        // Calculate linear distance for this movement
        float distance_mm = sqrtf(dx * dx + dy * dy);
        if (distance_mm < 0.001f) return cmd; // No movement
        
        // For now, assume straight-line movement (no rotation during translation)
        // More advanced: could calculate arc movement for smoother paths
        
        // Calculate wheel circumference
        float wheel_circumference = M_PI * drive_params.wheel_diameter_mm;
        
        // Calculate total wheel rotations needed
        float wheel_rotations = distance_mm / wheel_circumference;
        
        // Convert to stepper steps
        int total_steps = (int)(wheel_rotations * drive_params.steps_per_revolution);
        
        // For straight movement, both wheels move the same amount
        cmd.left_steps = total_steps;
        cmd.right_steps = total_steps;
        
        // Calculate speed in steps per second
        float time_for_movement = distance_mm / movement_speed_mm_per_sec;
        if (time_for_movement > 0) {
            cmd.left_speed = cmd.right_speed = total_steps / time_for_movement;
        }
        
        return cmd;
    }
    
    // Calculate wheel commands for rotation (in-place turn)
    WheelCommands calculateRotationCommands(float rotation_degrees, float rotation_speed_deg_per_sec = 90.0f) const {
        WheelCommands cmd;
        
        if (fabsf(rotation_degrees) < 0.1f) return cmd; // No rotation
        
        // Calculate arc length each wheel travels during rotation
        // For in-place rotation, each wheel travels: (wheel_base / 2) * angle_radians
        float angle_radians = fabsf(rotation_degrees) * M_PI / 180.0f;
        float arc_distance_mm = (drive_params.wheel_base_mm / 2.0f) * angle_radians;
        
        // Calculate wheel circumference and steps
        float wheel_circumference = M_PI * drive_params.wheel_diameter_mm;
        float wheel_rotations = arc_distance_mm / wheel_circumference;
        int steps = (int)(wheel_rotations * drive_params.steps_per_revolution);
        
        // For clockwise rotation (positive degrees), left wheel forward, right wheel backward
        if (rotation_degrees > 0) {
            cmd.left_steps = steps;
            cmd.right_steps = -steps;
        } else {
            cmd.left_steps = -steps;
            cmd.right_steps = steps;
        }
        
        // Calculate speed
        float time_for_rotation = fabsf(rotation_degrees) / rotation_speed_deg_per_sec;
        if (time_for_rotation > 0) {
            cmd.left_speed = cmd.right_speed = steps / time_for_rotation;
        }
        
        return cmd;
    }
    
    // Combined movement: translate to target with optional heading adjustment
    WheelCommands calculateMovementToTarget(float target_x, float target_y, bool adjust_heading = false) const {
        float dx = target_x - current_position.x;
        float dy = target_y - current_position.y;
        
        if (adjust_heading) {
            // Calculate required heading to face target
            float required_heading = calculateRequiredHeading(target_x, target_y);
            float heading_error = calculateHeadingChange(required_heading);
            
            // If significant heading error, return rotation command first
            if (fabsf(heading_error) > 5.0f) {
                return calculateRotationCommands(heading_error);
            }
        }
        
        // Calculate direct movement
        return calculateWheelCommands(dx, dy);
    }
    
    // Forward kinematics: given wheel movements, calculate robot displacement
    MovementDelta calculateRobotMovement(int left_steps, int right_steps) const {
        MovementDelta delta;
        
        // Convert steps to wheel distances
        float wheel_circumference = M_PI * drive_params.wheel_diameter_mm;
        float left_distance = (float)left_steps * wheel_circumference / drive_params.steps_per_revolution;
        float right_distance = (float)right_steps * wheel_circumference / drive_params.steps_per_revolution;
        
        if (left_steps == right_steps) {
            // Straight movement
            float distance = (left_distance + right_distance) / 2.0f;
            float heading_rad = heading_to_radians();
            delta.dx = distance * sinf(heading_rad); // sin for x in North=0° convention
            delta.dy = distance * cosf(heading_rad); // cos for y in North=0° convention
        } else {
            // Differential movement (rotation + possible translation)
            float average_distance = (left_distance + right_distance) / 2.0f;
            float rotation_distance = (right_distance - left_distance) / 2.0f;
            float rotation_radians = rotation_distance / (drive_params.wheel_base_mm / 2.0f);
            
            // For small rotations, approximate as straight + rotation
            float heading_rad = heading_to_radians();
            delta.dx = average_distance * sinf(heading_rad);
            delta.dy = average_distance * cosf(heading_rad);
            
            // Note: This is a simplified model. More accurate would use arc calculations
        }
        
        return delta;
    }
};

inline void print_nav_instr(const nav_instr_t& instr) {
    printk("x_delta: %d, y_delta: %d, eraser_position: %s, marker_position: %s\n",
           static_cast<int>(instr.x_delta),
           static_cast<int>(instr.y_delta),
           (instr.eraser_position == PeripheralPosition::Up ? "Up" : "Down"),
           (instr.marker_position == PeripheralPosition::Up ? "Up" : "Down"));
}

/* Functions */

void nav_thread(void *, void *, void *);
