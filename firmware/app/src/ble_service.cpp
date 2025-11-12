#include <zephyr/kernel.h>
#include "ble_service.h"
#include <stdbool.h>
#include <stdio.h>


/* STATIC MEMBER DEFINITIONS */

// Static instance pointer for callbacks
BleService* BleService::instance = nullptr;

const struct bt_data BleService::ad[] = {
    BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
    BT_DATA(BT_DATA_NAME_COMPLETE, DEVICE_NAME, DEVICE_NAME_LEN),
};

const struct bt_data BleService::sd[] = {
    BT_DATA_BYTES(BT_DATA_UUID128_ALL, BT_UUID_NUS_SRV_VAL),
};

struct bt_nus_cb BleService::nus_listener = {
    .notif_enabled = BleService::notif_enabled,
    .received = BleService::received,
};

// Static callback implementations
void BleService::notif_enabled(bool enabled, void *ctx) {
    ARG_UNUSED(ctx);
    #ifdef DEBUG_BLE
    printk("%s() - %s\n", __func__, (enabled ? "Enabled" : "Disabled"));
    #endif
}

void BleService::received(struct bt_conn *conn, const void *data, uint16_t len, void *ctx) {
    ARG_UNUSED(conn);
    ARG_UNUSED(ctx);
    
    // Forward to instance method if available
    if (BleService::instance) {
        BleService::instance->receive(data, len);
    }
}


/* INSTANCE METHODS */

void BleService::init() {

    // Set static instance pointer for callbacks
    BleService::instance = this;

    int err;

	err = bt_nus_cb_register(&nus_listener, NULL);
	if (err) {
		printk("BLE_INIT::ERROR: Failed to register NUS callback: %d\n", err);
		return;
	}

	err = bt_enable(NULL);
	if (err) {
		printk("BLE_INIT::ERROR: Failed to enable bluetooth: %d\n", err);
		return;
	}

	err = bt_le_adv_start(BT_LE_ADV_CONN_FAST_1, ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));
	if (err) {
		printk("BLE_INIT::ERROR: Failed to start advertising: %d\n", err);
		return;
	}
    #ifdef DEBUG_BLE
	printk("BLE_INIT::SUCCESS: Initialization complete\n");
    #endif
}

void BleService::send(const char *data, size_t len) {

    // Acquire mutex to prevent race conditions between different threads sending BLE data
    k_mutex_lock(&send_mutex, K_FOREVER);
    
    int err = bt_nus_send(NULL, data, len);
    
    if (err < 0 && (err != -EAGAIN) && (err != -ENOTCONN)) {
        printk("BLE_SEND::ERROR: Failed to send data over BLE: %d\n", err);
    } else {
        #ifdef DEBUG_BLE
        printk("BLE_SEND::SUCCESS: pid: %hhu msg: %.*s\n", *(uint8_t*)data, len-1, (const char *)(data + 1));
        #endif
    }
    
    k_mutex_unlock(&send_mutex);
}

void BleService::receive(const void *data, uint16_t len) {
    
    if (receiveHandler) {
        #ifdef DEBUG_BLE
        printk("BLE_RECEIVE::SUCCESS: received pid=%hhu cmd=%.*s\n", *(uint8_t*)data, len, (const char *)data+1);
        #endif

        receiveHandler(data, len, navigationQueue);
        return;
    } else {
        #ifdef DEBUG_BLE
        printk("BLE_RECEIVE::ERROR: No receive handler specified\n");
        #endif
    }
}