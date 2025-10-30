/*
 * Copyright (c) 2025 Doodle Bot Project
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file DFPlayer Mini driver for MP3/WAV playback from microSD
 * @brief Simple audio playback driver for DFPlayer Mini module via UART
 * 
 * This driver supports:
 * - Playing MP3/WAV files from microSD card
 * - Volume control (0-30)
 * - Track selection by number
 * - Basic playback control (play, pause, stop, next, previous)
 * - Status queries
 * 
 * Hardware requirements:
 * - DFPlayer Mini module
 * - MicroSD card with MP3/WAV files (named 0001.mp3, 0002.mp3, etc.)
 * - UART connection to ESP32
 */

#ifndef DFPLAYER_H
#define DFPLAYER_H

#include <zephyr/kernel.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief DFPlayer command codes
 */
enum dfplayer_command {
    DFPLAYER_CMD_NEXT = 0x01,           /**< Next track */
    DFPLAYER_CMD_PREV = 0x02,           /**< Previous track */
    DFPLAYER_CMD_PLAY_TRACK = 0x03,     /**< Play specific track (1-3000) */
    DFPLAYER_CMD_VOLUME_UP = 0x04,      /**< Volume up */
    DFPLAYER_CMD_VOLUME_DOWN = 0x05,    /**< Volume down */
    DFPLAYER_CMD_SET_VOLUME = 0x06,     /**< Set volume (0-30) */
    DFPLAYER_CMD_SET_EQ = 0x07,         /**< Set equalizer */
    DFPLAYER_CMD_PLAY_LOOP = 0x08,      /**< Play track in loop */
    DFPLAYER_CMD_SELECT_DEVICE = 0x09,  /**< Select storage device */
    DFPLAYER_CMD_STANDBY = 0x0A,        /**< Enter standby mode */
    DFPLAYER_CMD_NORMAL = 0x0B,         /**< Normal working mode */
    DFPLAYER_CMD_RESET = 0x0C,          /**< Reset module */
    DFPLAYER_CMD_PLAY = 0x0D,           /**< Resume playback */
    DFPLAYER_CMD_PAUSE = 0x0E,          /**< Pause playback */
    DFPLAYER_CMD_PLAY_FOLDER = 0x0F,    /**< Play track from specific folder */
    DFPLAYER_CMD_STOP = 0x16,           /**< Stop playback */
    DFPLAYER_CMD_PLAY_FOLDER_LOOP = 0x17,/**< Loop play folder */
    DFPLAYER_CMD_RANDOM_PLAY = 0x18,    /**< Random play all */
    DFPLAYER_CMD_QUERY_STATUS = 0x42,   /**< Query current status */
    DFPLAYER_CMD_QUERY_VOLUME = 0x43,   /**< Query current volume */
    DFPLAYER_CMD_QUERY_FILES = 0x48,    /**< Query number of files */
    DFPLAYER_CMD_QUERY_TRACK = 0x4C     /**< Query current track */
};

/**
 * @brief DFPlayer equalizer modes
 */
enum dfplayer_eq {
    DFPLAYER_EQ_NORMAL = 0,     /**< Normal */
    DFPLAYER_EQ_POP = 1,        /**< Pop */
    DFPLAYER_EQ_ROCK = 2,       /**< Rock */
    DFPLAYER_EQ_JAZZ = 3,       /**< Jazz */
    DFPLAYER_EQ_CLASSIC = 4,    /**< Classic */
    DFPLAYER_EQ_BASS = 5        /**< Bass */
};

/**
 * @brief DFPlayer device sources
 */
enum dfplayer_device {
    DFPLAYER_DEVICE_USB = 1,    /**< USB disk */
    DFPLAYER_DEVICE_SD = 2,     /**< SD card */
    DFPLAYER_DEVICE_AUX = 3,    /**< AUX input */
    DFPLAYER_DEVICE_SLEEP = 4,  /**< Sleep mode */
    DFPLAYER_DEVICE_FLASH = 5   /**< Flash memory */
};

/**
 * @brief Initialize DFPlayer Mini module
 * 
 * Configures UART interface and initializes the DFPlayer module.
 * Sets default volume and selects SD card as source.
 * 
 * @return 0 on success, negative errno on error
 */
int dfplayer_init(void);

/**
 * @brief Play a specific track by number
 * 
 * Plays track from microSD card. Files should be named 0001.mp3, 0002.mp3, etc.
 * 
 * @param track_number Track number to play (1-3000)
 * @return 0 on success, negative errno on error
 */
int dfplayer_play_track(uint16_t track_number);

/**
 * @brief Set playback volume
 * 
 * @param volume Volume level (0-30, where 30 is maximum)
 * @return 0 on success, negative errno on error
 */
int dfplayer_set_volume(uint8_t volume);

/**
 * @brief Play next track
 * 
 * @return 0 on success, negative errno on error
 */
int dfplayer_next(void);

/**
 * @brief Play previous track
 * 
 * @return 0 on success, negative errno on error
 */
int dfplayer_previous(void);

/**
 * @brief Resume playback
 * 
 * @return 0 on success, negative errno on error
 */
int dfplayer_play(void);

/**
 * @brief Pause playback
 * 
 * @return 0 on success, negative errno on error
 */
int dfplayer_pause(void);

/**
 * @brief Stop playback
 * 
 * @return 0 on success, negative errno on error
 */
int dfplayer_stop(void);

/**
 * @brief Set equalizer mode
 * 
 * @param eq Equalizer mode
 * @return 0 on success, negative errno on error
 */
int dfplayer_set_eq(enum dfplayer_eq eq);

/**
 * @brief Reset DFPlayer module
 * 
 * @return 0 on success, negative errno on error
 */
int dfplayer_reset(void);

/**
 * @brief Query current volume
 * 
 * @return Current volume (0-30) on success, negative errno on error
 */
int dfplayer_get_volume(void);

/**
 * @brief Query number of files on SD card
 * 
 * @return Number of files on success, negative errno on error
 */
int dfplayer_get_file_count(void);

/**
 * @brief Query current track number
 * 
 * @return Current track number on success, negative errno on error
 */
int dfplayer_get_current_track(void);

#ifdef __cplusplus
}
#endif

#endif /* DFPLAYER_H */