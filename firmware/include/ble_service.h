#pragma once

#include <stddef.h>
#include <stdint.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/services/nus.h>
#include <zephyr/sys/printk.h>
#include <zephyr/kernel.h>
#include "config.h"

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
		: navigationQueue(nav_queue), receiveHandler(handler) {
		k_mutex_init(&send_mutex);
	}

	/**
	 * @brief Initializes the NUS server and BLE service.
	 */
	void init();

	/**
	 * @brief Sends data over the BLE connection (tx characteristic)
	 * @param data Pointer to the data to send.
	 * @param len Length of the data to send.
	 */
    void send(const char *data, size_t len);

	/**
	 * @brief Queue an ACK to be sent with rate limiting.
	 * 
	 * ACKs are queued and sent with delays to prevent overwhelming
	 * the receiver's single-threaded ACK handler.
	 * 
	 * @param packet_id The packet ID to acknowledge.
	 */
	void queueAck(uint8_t packet_id); 
	
	/**
	 * @brief Called when new message received (rx characteristic) and maps it
	 * to the receive handler.
	 * 
	 * @param data Pointer to the received data.
	 * @param len Length of the received data.
	 */
    void receive(const void *data, uint16_t len);

	/**
	 * @brief Starts BLE advertising to allow new connections.
	 */
	void startAdvertising();

    // Static callbacks for connection events (public for BT_CONN_CB_DEFINE macro)
    static void connected(struct bt_conn *conn, uint8_t err);
    static void disconnected(struct bt_conn *conn, uint8_t reason);
    static BleService* instance;

	/**
	 * @brief Initialize the ACK work queue (call once at startup).
	 */
	static void initAckQueue();

private:
	// Instance members for handling message I/O
	k_msgq* navigationQueue;		///< Message queue for passing navigation instructions
	ReceiveHandler receiveHandler;	///< Custom handler function to process received BLE data
	
	// Mutex to protect BLE send operations from race conditions
	struct k_mutex send_mutex;

	// Static members for BLE service
    static constexpr const char* DEVICE_NAME = "BOO";
    static constexpr size_t DEVICE_NAME_LEN = sizeof(DEVICE_NAME) - 1;

    static const struct bt_data ad[];
    static const struct bt_data sd[];
    static struct bt_nus_cb nus_listener;

    // Static callbacks for NUS service
    static void notif_enabled(bool enabled, void *ctx);
    static void received(struct bt_conn *conn, const void *data, uint16_t len, void *ctx);
};


/**
 * @brief Tracks packet IDs and queues ACKs when packets complete.
 */
class PacketAckTracker {
public:
    PacketAckTracker() : last_id_(0), has_prev_(false), acked_(true) {}

    /** Called when command arrives. ACKs previous packet if ID changed. */
    void onCommand(uint8_t packet_id);

    /** Called on timeout. Flushes last packet ACK if pending. */
    void onTimeout();

private:
    uint8_t last_id_;
    bool has_prev_;
    bool acked_;
};