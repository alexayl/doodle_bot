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
    printk("%s() - %s\n", __func__, (enabled ? "Enabled" : "Disabled"));
}

void BleService::received(struct bt_conn *conn, const void *data, uint16_t len, void *ctx) {
    ARG_UNUSED(conn);
    ARG_UNUSED(ctx);
    printk("%s() - Len: %d, Message: %.*s\n", __func__, len, len, (char *)data);
    
    // Forward to instance method if available
    if (BleService::instance) {
        BleService::instance->receive(data, len);
    }
}


/* INSTANCE METHODS */

void BleService::init() {
    // Set static instance pointer for callbacks
    BleService::instance = this;
    
    // Initialize BLE service
    int err;

	printk("Sample - Bluetooth Peripheral NUS\n");

	err = bt_nus_cb_register(&nus_listener, NULL);
	if (err) {
		printk("ERROR: Failed to register NUS callback: %d\n", err);
		return;
	}

	err = bt_enable(NULL);
	if (err) {
		printk("ERROR: Failed to enable bluetooth: %d\n", err);
		return;
	}

	err = bt_le_adv_start(BT_LE_ADV_CONN_FAST_1, ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));
	if (err) {
		printk("ERROR: Failed to start advertising: %d\n", err);
		return;
	}

	printk("Initialization complete\n");
}

void BleService::send(const char *data, size_t len) {
    // Send data over BLE
    int err = bt_nus_send(NULL, data, len);
    printk("Data send - Result: %d\n", err);
    if (err < 0 && (err != -EAGAIN) && (err != -ENOTCONN)) {
        printk("ERROR: Failed to send data over BLE: %d\n", err);
    }
}

void BleService::receive(const void *data, uint16_t len) {
    
    // If custom handler is provided, use it exclusively
    if (receiveHandler) {
        receiveHandler(data, len, navigationQueue);
        return;
    }
    
    // Default behavior: just print
    printk("No BLE handler specified. Received %d bytes: %.*s\n", len, len, (char*)data);
}