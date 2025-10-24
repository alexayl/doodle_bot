#pragma once

#include <stddef.h>
#include <stdint.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/services/nus.h>
#include <zephyr/sys/printk.h>

// #define	 DEBUG_BLE

// Function pointer type for receive handler
typedef void (*ReceiveHandler)(const void* data, uint16_t len, k_msgq *q);

class BleService {
public:
	/**
	 * @brief Constructor
	 * @param nav_queue Message queue for navigation instructions.
	 * @param handler Function pointer to handle received data.
	 */
	BleService(k_msgq* nav_queue, ReceiveHandler handler) 
		: navigationQueue(nav_queue), receiveHandler(handler) {}

	/**
	 * @brief Initializes the NUS server and BLE service.
	 */
	void init();

	/**
	 * @brief Sends data over the BLE connection (tx characteristic)
	 * @param data Pointer to the data to send.
	 * @param len Length of the data to send.
	 * 
	 * TODO: Could be extended to support std::string
	 */
    void send(const char *data, size_t len); 
	
	/**
	 * @brief Called when new message received (rx characteristic) and maps it
	 * to the receive handler.
	 * 
	 * @param data Pointer to the received data.
	 * @param len Length of the received data.
	 */
    void receive(const void *data, uint16_t len);

private:
	// Instance members for handling message I/O
	k_msgq* navigationQueue;		///< Message queue for passing navigation instructions
	ReceiveHandler receiveHandler;	///< Custom handler function to process received BLE data

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