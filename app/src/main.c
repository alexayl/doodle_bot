#include <zephyr/kernel.h>

int main(void)
{
    while (1) {
        printk("Hello World! QEMU\n");
        k_msleep(1000);
    }
}
