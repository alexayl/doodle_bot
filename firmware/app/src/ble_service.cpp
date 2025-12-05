#include <zephyr/kernel.h>
#include "ble_service.h"
#include "comms_thread.h"
#include <stdbool.h>
#include <stdio.h>


/* ACK QUEUE CONFIGURATION */

#define ACK_QUEUE_SIZE 16
#define ACK_DELAY_MS 100  // Minimum delay between ACKs (100ms for reliable BLE)

// Queue to hold pending ACK packet IDs
K_MSGQ_DEFINE(ack_queue, sizeof(uint8_t), ACK_QUEUE_SIZE, 1);

// Delayed work for sending ACKs
static struct k_work_delayable ack_work;

static void ack_work_handler(struct k_work *work) {
    ARG_UNUSED(work);
    uint8_t pid;
    
    // Get next ACK from queue
    if (k_msgq_get(&ack_queue, &pid, K_NO_WAIT) == 0) {
        if (BleService::instance != nullptr) {
            char ack[sizeof("aok\n")] = "aok\n";
            ack[0] = pid;
            
            #ifdef DEBUG_BLE
            printk("BLE_ACK: Sending ACK for pid=%d\n", pid);
            #endif
            
            BleService::instance->send(ack, sizeof(ack));
        }
        
        // If more ACKs pending, schedule next one with delay
        if (k_msgq_num_used_get(&ack_queue) > 0) {
            k_work_schedule(&ack_work, K_MSEC(ACK_DELAY_MS));
        }
    }
}


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

static struct k_work_delayable adv_restart_work;

static void adv_restart_work_handler(struct k_work *work) {
    ARG_UNUSED(work);
    if (BleService::instance) {
        BleService::instance->startAdvertising();
    }
}

BT_CONN_CB_DEFINE(conn_callbacks) = {
    .connected = BleService::connected,
    .disconnected = BleService::disconnected,
};

// Static callback implementations for NUS
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

void BleService::connected(struct bt_conn *conn, uint8_t err) {
    if (err) {
        printk("BLE_CONN::ERROR: Connection failed (err %u)\n", err);
        k_work_schedule(&adv_restart_work, K_MSEC(100));
        return;
    }
    printk("BLE_CONN::SUCCESS: Device connected\n");
}

void BleService::disconnected(struct bt_conn *conn, uint8_t reason) {
    printk("BLE_CONN::INFO: Device disconnected (reason %u)\n", reason);
    
    // Reset comms state (packet ID counter, etc.)
    comms_reset();
    
    // Schedule advertising restart via work queue (deferred to system workqueue context)
    k_work_schedule(&adv_restart_work, K_MSEC(100));
}


/* INSTANCE METHODS */

void BleService::init() {

    // Set static instance pointer for callbacks
    BleService::instance = this;

    k_work_init_delayable(&adv_restart_work, adv_restart_work_handler);

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

	startAdvertising();
    #ifdef DEBUG_BLE
	printk("BLE_INIT::SUCCESS: Initialization complete\n");
    #endif
}

void BleService::startAdvertising() {
	bt_le_adv_stop();
	
	int err = bt_le_adv_start(BT_LE_ADV_CONN_FAST_1, ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));
	if (err) {
		printk("BLE_ADV::ERROR: Failed to start advertising: %d\n", err);
		return;
	}
	printk("BLE_ADV::SUCCESS: Advertising started, waiting for connection...\n");
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

void BleService::initAckQueue() {
    k_work_init_delayable(&ack_work, ack_work_handler);
}

void BleService::queueAck(uint8_t packet_id) {
    #ifdef DEBUG_BLE
    printk("BLE_ACK: Queuing ACK for pid=%d\n", packet_id);
    #endif
    
    // Add to queue (non-blocking)
    if (k_msgq_put(&ack_queue, &packet_id, K_NO_WAIT) != 0) {
        printk("BLE_ACK: Queue full, dropping ACK for pid=%d\n", packet_id);
        return;
    }
    
    // Schedule work if not already scheduled
    k_work_schedule(&ack_work, K_MSEC(ACK_DELAY_MS));
}