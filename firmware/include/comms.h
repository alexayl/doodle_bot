#pragma once

#include <stddef.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/services/nus.h>
#include <zephyr/sys/printk.h>

class ble_service {
public:

	/**
	 * init - performs hardware initialization and links handlers to service.
	 */
    static void init();

	/**
	 * send - advertises inputted message (tx characteristic)
	 */
    static void send(const char *data, size_t len);

	/**
	 * receieve - called when new message received (rx characteristic)
	 */
    static void receive(const void *data, uint16_t len);

	/**
	 * is_connected - double-checks connection
	 */
    static bool is_connected();

private:
    static constexpr const char* DEVICE_NAME = "DOODLEBOT";
    static constexpr size_t DEVICE_NAME_LEN = sizeof(DEVICE_NAME) - 1;

    static const struct bt_data ad[];
    static const struct bt_data sd[];
    static struct bt_nus_cb nus_listener;

    static void notif_enabled(bool enabled, void *ctx);
    static void received(struct bt_conn *conn, const void *data, uint16_t len, void *ctx);
};

void comms_thread(void *, void *, void *);