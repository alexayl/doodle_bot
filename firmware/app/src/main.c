#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <stdlib.h>

#include "comms.h"
// #include "navigation.h"
#include "state.h"
// #include "ui.h"


#define STACK_SIZE      2048


#define COMMS_PRIORITY   1
// #define IMU_PRIORITY    2
// #define NAV_PRIORITY    3
#define STATE_PRIORITY  1

K_THREAD_STACK_DEFINE(comms_stack, STACK_SIZE);
// K_THREAD_STACK_DEFINE(imu_stack, STACK_SIZE);
// K_THREAD_STACK_DEFINE(nav_stack, STACK_SIZE);
K_THREAD_STACK_DEFINE(state_stack, STACK_SIZE);

static struct k_thread comms_thread_data;
// static struct k_thread imu_thread_data;
// static struct k_thread nav_thread_data;
static struct k_thread state_thread_data;

typedef bool (*init_func_t)(void);

int main(void) {

    // Initialize hardware

    k_thread_create(&comms_thread_data, comms_stack, STACK_SIZE,
                    comms_thread, NULL, NULL, NULL,
                    COMMS_PRIORITY, 0, K_NO_WAIT);

    // k_thread_create(&imu_thread_data, imu_stack, STACK_SIZE,
    //                 imu_thread, NULL, NULL, NULL,
    //                 IMU_PRIORITY, 0, K_NO_WAIT);

    // k_thread_create(&nav_thread_data, nav_stack, STACK_SIZE,
    //                 nav_thread, NULL, NULL, NULL,
    //                 NAV_PRIORITY, 0, K_NO_WAIT);

    k_thread_create(&state_thread_data, state_stack, STACK_SIZE,
                    state_thread, NULL, NULL, NULL,
                    STATE_PRIORITY, 0, K_NO_WAIT);

    return EXIT_SUCCESS;
}