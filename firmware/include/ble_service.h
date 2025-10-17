#pragma once

#include <stddef.h>
#include <stdint.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/services/nus.h>
#include <zephyr/sys/printk.h>

//------------------
// BLE_SERVICE
// -----------------

// Function pointer type for receive handler
typedef void (*ReceiveHandler)(const void* data, uint16_t len, k_msgq *q);

class BleService {
public:
	/**
	 * Constructor with both queue and handler injection
	 * @param nav_queue Message queue for navigation instructions
	 * @param handler Function pointer to handle received data
	 */
	BleService(k_msgq* nav_queue, ReceiveHandler handler = nullptr) 
		: navigationQueue(nav_queue), receiveHandler(handler) {}

	/**
	 * init - performs hardware initialization and links handlers to service.
	 */
	void init();

	/**
	 * send - advertises inputted message (tx characteristic)
	 */
    void send(const char *data, size_t len);

	/**
	 * receieve - called when new message received (rx characteristic)
	 */
    void receive(const void *data, uint16_t len);

private:
	// Instance members for dependency injection
	k_msgq* navigationQueue;      // Queue for navigation instructions
	ReceiveHandler receiveHandler; // Custom handler function

	// Static members for BLE service
    static constexpr const char* DEVICE_NAME = "DOODLEBOT";
    static constexpr size_t DEVICE_NAME_LEN = sizeof(DEVICE_NAME) - 1;

    static const struct bt_data ad[];
    static const struct bt_data sd[];
    static struct bt_nus_cb nus_listener;
    
    // Static instance pointer for callbacks
    static BleService* instance;

    static void notif_enabled(bool enabled, void *ctx);
    static void received(struct bt_conn *conn, const void *data, uint16_t len, void *ctx);
};