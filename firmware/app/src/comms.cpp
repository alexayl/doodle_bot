#include <zephyr/kernel.h>
#include "comms.h"
#include "navigation.h"
#include <stdbool.h>
#include <stdio.h> // Use printf instead of iostream

#include <zephyr/kernel.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/services/nus.h>

// #define TEST_CODE

K_SEM_DEFINE(comms_sem, 0, 1);

nav_instr_t dummy_instr = { 10, 20, PeripheralPosition::Up, PeripheralPosition::Down};


/* CLASS METHODS */

void ble_service::init() {
    // Initialize BLE service
    int err;

	printk("Sample - Bluetooth Peripheral NUS\n");

	err = bt_nus_cb_register(&nus_listener, NULL);
	if (err) {
		printk("Failed to register NUS callback: %d\n", err);
		return err;
	}

	err = bt_enable(NULL);
	if (err) {
		printk("Failed to enable bluetooth: %d\n", err);
		return err;
	}

	err = bt_le_adv_start(BT_LE_ADV_CONN_FAST_1, ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));
	if (err) {
		printk("Failed to start advertising: %d\n", err);
		return err;
	}

	printk("Initialization complete\n");
}

void ble_service::send(const char *data, size_t len) {
    // Send data over BLE
    int err = bt_nus_send(NULL, data, len);
    if (err < 0 && (err != -EAGAIN) && (err != -ENOTCONN)) {
        std::cerr << "Failed to send data over BLE: " << err << std::endl;
    }
}

bool ble_service::is_connected() {
    // Check if BLE is connected
    return true;
}

/* THREAD FUNCTIONS */

void *pull_next_instr() {
    // TODO: implement this, just mocking out for now

    // get from gatt
        // store in var
    
    // send ack to gat for next instr to post
    
    return &dummy_instr;
}

void comms_thread(void *nav_instr_queue, void *arg2, void *arg3) {

    k_msgq *q = static_cast<k_msgq *>(nav_instr_queue);

    

    #ifdef TEST_CODE
    printk("comms thread started\n");

    while(1) {
        k_sem_take(&comms_sem, K_FOREVER);
        // Update comms here
        printk("comms thread running\n");

        k_sleep(K_MSEC(10000));
    }
    #endif

    while(1) {

        printf("Sanity check comms thread!\n");
        static uint8_t count;



        // TODO: Create semaphore to wake when available data
        // based on BLE isr

        // Check if room in instruction queue
        if (k_msgq_num_free_get(q)){
            // get data
            // pull from gat
            count += 1;
            printf("%d messages sent.\n", count);
            void *next_instr = pull_next_instr();
            k_msgq_put(q, next_instr, K_FOREVER);
        } else {
            // sleep
            k_msleep(1000); // sleep 100 ms
        }
    
    }

    return;
}