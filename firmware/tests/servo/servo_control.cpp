#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <servo.h>

#define SERVO_ERASER_NODE DT_ALIAS(servo0)
#define SERVO_MARKER_NODE DT_ALIAS(servo1)
#define SERVO_ERASER_DEVICE DEVICE_DT_GET(SERVO_ERASER_NODE)
#define SERVO_MARKER_DEVICE DEVICE_DT_GET(SERVO_MARKER_NODE)

int main()
{
    const struct device *servo_eraser = SERVO_ERASER_DEVICE;
    const struct device *servo_marker = SERVO_MARKER_DEVICE;
    
    if (!device_is_ready(servo_eraser)) {
        printk("Eraser servo device not ready\n");
        return -1;
    }

    if (!device_is_ready(servo_marker)) {
        printk("Marker servo device not ready\n");
        return -1;
    }

    printk("Dual Servo Demo - Eraser: %s, Marker: %s\n", 
           servo_eraser->name, servo_marker->name);

    const int positions[] = {0, 90, 180, 90};
    const int num_positions = sizeof(positions) / sizeof(positions[0]);
    int step = 0;

    while (1) {
        int eraser_angle = positions[step % num_positions];
        int marker_angle = positions[(step + 2) % num_positions];  // Offset pattern
        
        int ret1 = servo_set_angle(servo_eraser, eraser_angle);
        int ret2 = servo_set_angle(servo_marker, marker_angle);
        
        if (ret1 == 0 && ret2 == 0) {
            printk("Eraser: %d°, Marker: %d°\n", eraser_angle, marker_angle);
        }

        step++;
        k_sleep(K_MSEC(1000));
    }

    return 0;
}