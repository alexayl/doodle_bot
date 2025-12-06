#pragma once

#include "ble_service.h"

extern BleService* g_bleService;
void comms_thread(void *, void *, void *);

/**
 * @brief Reset comms state (called on BLE disconnect)
 * 
 * Resets the instruction parser's expected packet ID to 0.
 */
void comms_reset();