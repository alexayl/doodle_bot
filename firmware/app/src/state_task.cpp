#include <zephyr/kernel.h>
#include "state_task.h"

void state_thread(void *arg1, void *arg2, void *arg3) {


    while(1) {
        k_sleep(K_FOREVER);
    }

    return;
}