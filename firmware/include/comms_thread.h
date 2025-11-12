#pragma once

#include "ble_service.h"

extern BleService* g_bleService;
void comms_thread(void *, void *, void *);