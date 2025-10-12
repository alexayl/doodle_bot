#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <servo.h>

#define SERVO_NODE DT_ALIAS(servo0)
#define SERVO_DEVICE DEVICE_DT_GET(SERVO_NODE)

int main()
{
    const struct device *servo_dev = SERVO_DEVICE;
    
    if (!device_is_ready(servo_dev)) {
        printk("Servo device not ready\n");
        return -1;
    }

    printk("Servo Demo - %s\n", servo_dev->name);

    const int positions[] = {0, 90, 180, 90};
    const int num_positions = sizeof(positions) / sizeof(positions[0]);
    int step = 0;

    while (1) {
        int angle = positions[step % num_positions];
        
        int ret = servo_set_angle(servo_dev, angle);
        if (ret == 0) {
            printk("Angle: %d°\n", angle);
        }

        step++;
        k_sleep(K_MSEC(1000));
    }

    return 0;
}