#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>

int main(void)
{
    printk("Hello from NRF52840!\n");
    printk("Simple test starting...\n");
    
    int counter = 0;
    while (1) {
        printk("Counter: %d\n", counter++);
        k_sleep(K_SECONDS(1));
    }
    
    return 0;
}