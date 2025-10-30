/**
 * DFPlayer Mini driver implementation
 * Communicates with DFPlayer Mini module via UART for MP3/WAV playback
 */

#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/devicetree.h>
#include <zephyr/logging/log.h>
#include <zephyr/kernel.h>
#include "dfplayer.h"

LOG_MODULE_REGISTER(dfplayer, CONFIG_LOG_DEFAULT_LEVEL);

/* DFPlayer devicetree node */
#define DFPLAYER_NODE DT_NODELABEL(dfplayer)

#if !DT_NODE_EXISTS(DFPLAYER_NODE)
#error "DFPlayer devicetree node 'dfplayer' not found"
#endif

/* UART device from devicetree */
static const struct device *uart_dev = DEVICE_DT_GET(DT_PHANDLE(DFPLAYER_NODE, uart));

/* DFPlayer command packet structure */
struct dfplayer_packet {
    uint8_t start_byte;     /* 0x7E */
    uint8_t version;        /* 0xFF */
    uint8_t length;         /* 0x06 */
    uint8_t command;        /* Command code */
    uint8_t feedback;       /* 0x00 = no feedback, 0x01 = feedback */
    uint8_t param_high;     /* Parameter high byte */
    uint8_t param_low;      /* Parameter low byte */
    uint8_t checksum_high;  /* Checksum high byte */
    uint8_t checksum_low;   /* Checksum low byte */
    uint8_t end_byte;       /* 0xEF */
} __packed;

/* Driver state */
static bool initialized = false;
static uint8_t current_volume = 15; /* Default volume */

/* Calculate checksum for DFPlayer packet */
static uint16_t dfplayer_calculate_checksum(const struct dfplayer_packet *packet)
{
    uint16_t sum = 0;
    sum = packet->version + packet->length + packet->command + 
          packet->feedback + packet->param_high + packet->param_low;
    return (uint16_t)(0 - sum); /* Two's complement */
}

/* Send command packet to DFPlayer */
static int dfplayer_send_command(uint8_t command, uint16_t parameter)
{
    struct dfplayer_packet packet = {0};
    uint16_t checksum;
    
    if (!device_is_ready(uart_dev)) {
        LOG_ERR("UART device not ready");
        return -ENODEV;
    }
    
    /* Build packet */
    packet.start_byte = 0x7E;
    packet.version = 0xFF;
    packet.length = 0x06;
    packet.command = command;
    packet.feedback = 0x00; /* No feedback for most commands */
    packet.param_high = (parameter >> 8) & 0xFF;
    packet.param_low = parameter & 0xFF;
    
    /* Calculate checksum */
    checksum = dfplayer_calculate_checksum(&packet);
    packet.checksum_high = (checksum >> 8) & 0xFF;
    packet.checksum_low = checksum & 0xFF;
    packet.end_byte = 0xEF;
    
    /* Send packet via UART */
    for (int i = 0; i < sizeof(packet); i++) {
        uart_tx_put(uart_dev, ((uint8_t*)&packet)[i]);
    }
    
    LOG_DBG("Sent DFPlayer command: 0x%02X, param: 0x%04X", command, parameter);
    
    /* Small delay for command processing */
    k_msleep(100);
    
    return 0;
}

/* Send query command and wait for response */
static int dfplayer_query_command(uint8_t command, uint16_t *response)
{
    struct dfplayer_packet packet = {0};
    uint16_t checksum;
    uint8_t rx_buffer[10];
    int bytes_received = 0;
    int timeout_ms = 1000;
    
    if (!device_is_ready(uart_dev)) {
        LOG_ERR("UART device not ready");
        return -ENODEV;
    }
    
    /* Build query packet with feedback enabled */
    packet.start_byte = 0x7E;
    packet.version = 0xFF;
    packet.length = 0x06;
    packet.command = command;
    packet.feedback = 0x01; /* Enable feedback for queries */
    packet.param_high = 0x00;
    packet.param_low = 0x00;
    
    checksum = dfplayer_calculate_checksum(&packet);
    packet.checksum_high = (checksum >> 8) & 0xFF;
    packet.checksum_low = checksum & 0xFF;
    packet.end_byte = 0xEF;
    
    /* Clear any pending RX data */
    while (uart_rx_get(uart_dev, rx_buffer, 1) == 0) {
        /* Drain buffer */
    }
    
    /* Send query packet */
    for (int i = 0; i < sizeof(packet); i++) {
        uart_tx_put(uart_dev, ((uint8_t*)&packet)[i]);
    }
    
    /* Wait for response */
    while (timeout_ms > 0 && bytes_received < 10) {
        if (uart_rx_get(uart_dev, &rx_buffer[bytes_received], 1) == 0) {
            bytes_received++;
            if (bytes_received >= 10) break;
        }
        k_msleep(1);
        timeout_ms--;
    }
    
    if (bytes_received < 10) {
        LOG_WRN("DFPlayer query timeout or incomplete response");
        return -ETIMEDOUT;
    }
    
    /* Parse response */
    if (rx_buffer[0] == 0x7E && rx_buffer[9] == 0xEF) {
        *response = (rx_buffer[5] << 8) | rx_buffer[6];
        LOG_DBG("DFPlayer query response: 0x%04X", *response);
        return 0;
    }
    
    LOG_WRN("Invalid DFPlayer response format");
    return -EINVAL;
}

int dfplayer_init(void)
{
    if (initialized) {
        return 0;
    }
    
    if (!device_is_ready(uart_dev)) {
        LOG_ERR("UART device not ready");
        return -ENODEV;
    }
    
    LOG_INF("Initializing DFPlayer Mini...");
    
    /* Reset module */
    dfplayer_send_command(DFPLAYER_CMD_RESET, 0);
    k_msleep(3000); /* Wait for reset to complete */
    
    /* Select SD card as source */
    dfplayer_send_command(DFPLAYER_CMD_SELECT_DEVICE, DFPLAYER_DEVICE_SD);
    k_msleep(200);
    
    /* Set default volume */
    dfplayer_send_command(DFPLAYER_CMD_SET_VOLUME, current_volume);
    k_msleep(200);
    
    initialized = true;
    LOG_INF("DFPlayer Mini initialized successfully");
    
    return 0;
}

int dfplayer_play_track(uint16_t track_number)
{
    if (!initialized) {
        dfplayer_init();
    }
    
    if (track_number == 0 || track_number > 3000) {
        LOG_ERR("Invalid track number: %d (must be 1-3000)", track_number);
        return -EINVAL;
    }
    
    LOG_INF("Playing track %d", track_number);
    return dfplayer_send_command(DFPLAYER_CMD_PLAY_TRACK, track_number);
}

int dfplayer_set_volume(uint8_t volume)
{
    if (!initialized) {
        dfplayer_init();
    }
    
    if (volume > 30) {
        LOG_ERR("Invalid volume: %d (must be 0-30)", volume);
        return -EINVAL;
    }
    
    current_volume = volume;
    LOG_INF("Setting volume to %d", volume);
    return dfplayer_send_command(DFPLAYER_CMD_SET_VOLUME, volume);
}

int dfplayer_next(void)
{
    if (!initialized) {
        dfplayer_init();
    }
    
    LOG_INF("Next track");
    return dfplayer_send_command(DFPLAYER_CMD_NEXT, 0);
}

int dfplayer_previous(void)
{
    if (!initialized) {
        dfplayer_init();
    }
    
    LOG_INF("Previous track");
    return dfplayer_send_command(DFPLAYER_CMD_PREV, 0);
}

int dfplayer_play(void)
{
    if (!initialized) {
        dfplayer_init();
    }
    
    LOG_INF("Resume playback");
    return dfplayer_send_command(DFPLAYER_CMD_PLAY, 0);
}

int dfplayer_pause(void)
{
    if (!initialized) {
        dfplayer_init();
    }
    
    LOG_INF("Pause playback");
    return dfplayer_send_command(DFPLAYER_CMD_PAUSE, 0);
}

int dfplayer_stop(void)
{
    if (!initialized) {
        dfplayer_init();
    }
    
    LOG_INF("Stop playback");
    return dfplayer_send_command(DFPLAYER_CMD_STOP, 0);
}

int dfplayer_set_eq(enum dfplayer_eq eq)
{
    if (!initialized) {
        dfplayer_init();
    }
    
    if (eq > DFPLAYER_EQ_BASS) {
        LOG_ERR("Invalid EQ mode: %d", eq);
        return -EINVAL;
    }
    
    LOG_INF("Setting EQ mode to %d", eq);
    return dfplayer_send_command(DFPLAYER_CMD_SET_EQ, eq);
}

int dfplayer_reset(void)
{
    LOG_INF("Resetting DFPlayer Mini");
    int ret = dfplayer_send_command(DFPLAYER_CMD_RESET, 0);
    k_msleep(3000); /* Wait for reset */
    initialized = false; /* Force re-initialization */
    return ret;
}

int dfplayer_get_volume(void)
{
    if (!initialized) {
        dfplayer_init();
    }
    
    uint16_t volume;
    int ret = dfplayer_query_command(DFPLAYER_CMD_QUERY_VOLUME, &volume);
    if (ret == 0) {
        LOG_INF("Current volume: %d", volume);
        return volume;
    }
    
    /* Fallback to cached volume if query fails */
    LOG_WRN("Volume query failed, returning cached value: %d", current_volume);
    return current_volume;
}

int dfplayer_get_file_count(void)
{
    if (!initialized) {
        dfplayer_init();
    }
    
    uint16_t count;
    int ret = dfplayer_query_command(DFPLAYER_CMD_QUERY_FILES, &count);
    if (ret == 0) {
        LOG_INF("File count: %d", count);
        return count;
    }
    
    return ret;
}

int dfplayer_get_current_track(void)
{
    if (!initialized) {
        dfplayer_init();
    }
    
    uint16_t track;
    int ret = dfplayer_query_command(DFPLAYER_CMD_QUERY_TRACK, &track);
    if (ret == 0) {
        LOG_INF("Current track: %d", track);
        return track;
    }
    
    return ret;
}