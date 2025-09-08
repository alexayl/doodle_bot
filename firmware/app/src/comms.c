#include <zephyr/kernel.h>
#include <comms.h>
#include <stdio.h>

K_SEM_DEFINE(comms_sem, 0, 1);

void comms_thread(void *arg1, void *arg2, void *arg3) {

    printk("comms thread started\n");

    while(1) {
        k_sem_take(&comms_sem, K_FOREVER);
        // Update comms here
        printk("comms thread running\n");

        k_sleep(K_MSEC(10000));
    }

    return;
}