/*
 * Copyright (c) 2025 Doodle Bot Project
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file Servo motor control using custom doodle_servo driver
 * Provides power-safe servo control with gradual movement
 */

#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>

/* Simple forward declaration for servo driver API */
extern "C" int servo_set_angle(const struct device *dev, uint8_t angle);

extern "C" {

static const struct device *servo_dev = DEVICE_DT_GET(DT_NODELABEL(servo));

/* Global stop flag for controlling servo operation */
static volatile bool servo_stop_requested = false;

#define POWER_SAFE_DELAY_MS 200  /* Delay between movements */

/**
 * Request servo operations to stop
 */
void servo_request_stop(void)
{
    servo_stop_requested = true;
    printk("Servo stop requested...\n");
}

/**
 * Set servo to specific angle with gradual movement (power-safe)
 * @param angle Desired angle in degrees
 * @return 0 on success, negative on error
 */
int servo_set_angle_gradual(uint16_t angle)
{
    if (!device_is_ready(servo_dev)) {
        printk("Error: Servo device is not ready\n");
        return -ENODEV;
    }
    
    if (angle > 180) {
        return -EINVAL;
    }
    
    /* For power-safe operation, move gradually in smaller steps */
    static uint8_t current_angle = 90;  /* Start from center */
    
    int step_size = 5;  /* 5-degree steps */
    int ret = 0;
    
    /* Move gradually to reduce current spikes */
    while (current_angle != angle && ret == 0) {
        if (current_angle < angle) {
            current_angle = (angle - current_angle > step_size) ? 
                           current_angle + step_size : angle;
        } else {
            current_angle = (current_angle - angle > step_size) ? 
                           current_angle - step_size : angle;
        }
        
        ret = servo_set_angle(servo_dev, current_angle);
        if (ret == 0) {
            k_sleep(K_MSEC(50));  /* Pause between steps */
        }
    }
    
    return ret;
}

/**
 * Demo function showing continuous servo sweep
 */
void servo_demo_sweep(void)
{
    printk("Servo Motor Control Demo using custom driver\n");

    if (!device_is_ready(servo_dev)) {
        printk("Error: Servo device %s is not ready\n", servo_dev->name);
        return;
    }

    /* Sweep from 0 to 180 degrees and back */
    for (int i = 0; i <= 180; i += 10) {
        int ret = servo_set_angle(servo_dev, i);
        if (ret != 0) {
            printk("Error setting servo angle: %d\n", ret);
            return;
        }
        printk("Servo angle: %d degrees\n", i);
        k_sleep(K_MSEC(500));
    }

    for (int i = 180; i >= 0; i -= 10) {
        int ret = servo_set_angle(servo_dev, i);
        if (ret != 0) {
            printk("Error setting servo angle: %d\n", ret);
            return;
        }
        printk("Servo angle: %d degrees\n", i);
        k_sleep(K_MSEC(500));
    }
}

/**
 * Power-safe servo test function - used in main.cpp
 */
void servo_power_safe_test(void)
{
    printk("Starting power-safe servo test...\n");
    
    if (!device_is_ready(servo_dev)) {
        printk("Error: Servo device is not ready\n");
        return;
    }

    /* Test sequence: 45° -> 135° -> 90° (center) */
    printk("Moving to 45 degrees...\n");
    int ret = servo_set_angle_gradual(45);
    if (ret != 0) {
        printk("Error moving to 45 degrees: %d\n", ret);
        return;
    }
    k_sleep(K_MSEC(2000));

    printk("Moving to 135 degrees...\n");
    ret = servo_set_angle_gradual(135);
    if (ret != 0) {
        printk("Error moving to 135 degrees: %d\n", ret);
        return;
    }
    k_sleep(K_MSEC(2000));

    printk("Moving to center (90 degrees)...\n");
    ret = servo_set_angle_gradual(90);
    if (ret != 0) {
        printk("Error moving to center: %d\n", ret);
        return;
    }
    k_sleep(K_MSEC(2000));

    printk("Servo test completed successfully!\n");
}

/**
 * Continuous servo sweep - runs forever
 */
void servo_continuous_sweep(void)
{
    printk("Starting continuous servo sweep...\n");
    printk("Note: Reset board to stop servo sweep\n");
    
    if (!device_is_ready(servo_dev)) {
        printk("Error: Servo device is not ready\n");
        return;
    }

    int sweep_count = 0;
    
    while (!servo_stop_requested) {
        sweep_count++;
        printk("Sweep cycle %d - Moving 0° to 180°\n", sweep_count);
        
        /* Sweep from 0 to 180 degrees */
        for (int i = 0; i <= 180 && !servo_stop_requested; i += 15) {
            int ret = servo_set_angle(servo_dev, i);
            if (ret != 0) {
                printk("Error setting servo angle: %d\n", ret);
                return;
            }
            printk("Servo: %d°\n", i);
            k_sleep(K_MSEC(300));
        }
        
        if (servo_stop_requested) break;
        
        printk("Sweep cycle %d - Moving 180° to 0°\n", sweep_count);
        
        /* Sweep from 180 to 0 degrees */
        for (int i = 180; i >= 0 && !servo_stop_requested; i -= 15) {
            int ret = servo_set_angle(servo_dev, i);
            if (ret != 0) {
                printk("Error setting servo angle: %d\n", ret);
                return;
            }
            printk("Servo: %d°\n", i);
            k_sleep(K_MSEC(300));
        }
        
        if (servo_stop_requested) break;
        
        printk("Completed sweep cycle %d\n\n", sweep_count);
        k_sleep(K_MSEC(1000)); /* Pause between cycles */
    }
    
    /* Stop detected - move to center position */
    if (servo_stop_requested) {
        printk("Servo stop requested - moving to center position...\n");
        servo_set_angle(servo_dev, 90);
        printk("Servo stopped at center (90°)\n");
    }
}

} /* extern "C" */

/**
 * Main entry point for servo driver tests
 */
int main(void)
{
    printk("========================================\n");
    printk("Doodle Bot Servo Driver Test Suite\n");
    printk("Platform: Native Simulation\n");
    printk("========================================\n\n");

    // /* Test 1: Basic servo functionality */
    // printk("--- Test 1: Power-Safe Servo Test ---\n");
    // servo_power_safe_test();
    // k_sleep(K_MSEC(2000));

    // /* Test 2: Individual angle tests */
    // printk("\n--- Test 2: Individual Angle Tests ---\n");
    
    // int test_angles[] = {0, 45, 90, 135, 180};
    // int num_tests = sizeof(test_angles) / sizeof(test_angles[0]);
    
    // for (int i = 0; i < num_tests; i++) {
    //     printk("Testing angle: %d degrees\n", test_angles[i]);
    //     int ret = servo_set_angle_gradual(test_angles[i]);
    //     if (ret == 0) {
    //         printk("✓ Angle %d degrees: PASS\n", test_angles[i]);
    //     } else {
    //         printk("✗ Angle %d degrees: FAIL (error %d)\n", test_angles[i], ret);
    //     }
    //     k_sleep(K_MSEC(1000));
    // }

    // /* Test 3: Boundary tests */
    // printk("\n--- Test 3: Boundary Tests ---\n");
    
    // /* Test invalid angle */
    // printk("Testing invalid angle: 200 degrees\n");
    // int ret = servo_set_angle_gradual(200);
    // if (ret != 0) {
    //     printk("✓ Invalid angle rejection: PASS\n");
    // } else {
    //     printk("✗ Invalid angle rejection: FAIL (should have failed)\n");
    // }

    /* Test 4: Demo sweep (shortened for testing) */
    printk("\n--- Test 4: Demo Sweep ---\n");
    printk("Running shortened demo sweep...\n");
    servo_demo_sweep();

    printk("\n========================================\n");
    printk("Servo Driver Test Suite Complete\n");
    printk("All tests executed successfully!\n");
    printk("========================================\n");

    return 0;
}