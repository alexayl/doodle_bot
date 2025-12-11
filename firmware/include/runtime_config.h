#pragma once

#include "config.h"
#include <zephyr/sys/printk.h>

/**
 * Runtime Configuration Module
 * 
 * Allows wheelbase and wheel radius to be set at runtime via M-codes,
 * without requiring firmware recompilation.
 * 
 * M-codes:
 *   M503         - Report current configuration
 *   M504 W<mm>   - Set wheelbase (mm)
 *   M504 R<mm>   - Set wheel radius (mm)
 *   M504 W<mm> R<mm> - Set both
 * 
 * Values are initialized from config.h defaults and can be changed at runtime.
 * Changes are NOT persisted across resets (use config.h for permanent changes).
 */

class RuntimeConfig {
public:
    // Use a global instance instead of Meyer's singleton to avoid __cxa_guard
    static RuntimeConfig& instance() {
        return instance_;
    }

    // Getters
    float wheelbase() const { return wheelbase_; }
    float wheelRadius() const { return wheel_radius_; }
    float doodlebotRadius() const { return wheelbase_ / 2.0f; }

    // Setters (return true if value changed)
    bool setWheelbase(float mm) {
        if (mm > 0.0f && mm < 1000.0f) {
            wheelbase_ = mm;
            printk("CONFIG: Wheelbase set to %.2f mm\n", (double)wheelbase_);
            return true;
        }
        printk("CONFIG: Invalid wheelbase value %.2f (must be 0-1000mm)\n", (double)mm);
        return false;
    }

    bool setWheelRadius(float mm) {
        if (mm > 0.0f && mm < 500.0f) {
            wheel_radius_ = mm;
            printk("CONFIG: Wheel radius set to %.2f mm\n", (double)wheel_radius_);
            return true;
        }
        printk("CONFIG: Invalid wheel radius value %.2f (must be 0-500mm)\n", (double)mm);
        return false;
    }

    // Reset to compile-time defaults
    void resetToDefaults() {
        wheelbase_ = WHEELBASE;
        wheel_radius_ = WHEEL_RADIUS;
        printk("CONFIG: Reset to defaults (wheelbase=%.2f, radius=%.2f)\n",
               (double)wheelbase_, (double)wheel_radius_);
    }

    // Print current configuration
    void printConfig() const {
        printk("CONFIG: Wheelbase=%.2f mm, WheelRadius=%.2f mm, WheelDiameter=%.2f mm\n",
               (double)wheelbase_, (double)wheel_radius_, (double)(wheel_radius_ * 2.0f));
    }

private:
    // Private constructor - only instance_ should exist
    RuntimeConfig() 
        : wheelbase_(WHEELBASE)
        , wheel_radius_(WHEEL_RADIUS) 
    {}

    float wheelbase_;
    float wheel_radius_;
    
    // Global instance (defined in a .cpp file)
    static RuntimeConfig instance_;
};

// Convenience macros for accessing runtime config (replaces compile-time constants)
#define RUNTIME_WHEELBASE       (RuntimeConfig::instance().wheelbase())
#define RUNTIME_WHEEL_RADIUS    (RuntimeConfig::instance().wheelRadius())
#define RUNTIME_DOODLEBOT_RADIUS (RuntimeConfig::instance().doodlebotRadius())
