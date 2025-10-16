#include <zephyr/kernel.h>
#include "state_task.h"

// put this in a different thing later

void state_thread(void *arg1, void *arg2, void *arg3) {

    printk("State thread started\n");

    while(1) {
        // Update state here
        // printk("State thread running\n");

        k_sleep(K_MSEC(10000));
    }

    return;
}