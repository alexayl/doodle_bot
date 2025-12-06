/*
 * DFPlayer Mini test - Audio playback from microSD
 */

#include <zephyr/kernel.h>
#include "dfplayer.h"

int main(void)
{
    printk("=== DFPLAYER MINI TEST ===\n");
    printk("Testing MP3/WAV playback from microSD card\n");
    printk("Make sure microSD has files: 0001.mp3, 0002.mp3, etc.\n\n");

    // Initialize DFPlayer
    int ret = dfplayer_init();
    if (ret != 0) {
        printk("ERROR: DFPlayer initialization failed: %d\n", ret);
        return -1;
    }
    
    printk("DFPlayer initialized successfully\n");
    k_sleep(K_SECONDS(2));

    // Query file count
    int file_count = dfplayer_get_file_count();
    if (file_count > 0) {
        printk("Found %d audio files on SD card\n", file_count);
    } else {
        printk("Warning: Could not query file count (may still work)\n");
    }

    printk("\nStarting audio test sequence...\n");
    
    while (1) {
        // Test 1: Play track 1
        printk("\n>>> Playing track 1\n");
        dfplayer_play_track(1);
        k_sleep(K_SECONDS(5));  // Play for 5 seconds
        
        // Test 2: Play track 2
        printk(">>> Playing track 2\n");
        dfplayer_play_track(2);
        k_sleep(K_SECONDS(5));
        
        // Test 3: Volume control
        printk(">>> Setting volume to maximum (30)\n");
        dfplayer_set_volume(30);
        k_sleep(K_SECONDS(2));
        
        printk(">>> Setting volume to medium (15)\n");
        dfplayer_set_volume(15);
        k_sleep(K_SECONDS(2));
        
        printk(">>> Setting volume to low (5)\n");
        dfplayer_set_volume(5);
        k_sleep(K_SECONDS(2));
        
        // Test 4: Playback controls
        printk(">>> Pausing playback\n");
        dfplayer_pause();
        k_sleep(K_SECONDS(2));
        
        printk(">>> Resuming playback\n");
        dfplayer_play();
        k_sleep(K_SECONDS(3));
        
        // Test 5: Next/Previous
        printk(">>> Next track\n");
        dfplayer_next();
        k_sleep(K_SECONDS(3));
        
        printk(">>> Previous track\n");
        dfplayer_previous();
        k_sleep(K_SECONDS(3));
        
        // Test 6: EQ modes
        printk(">>> Setting EQ to Rock\n");
        dfplayer_set_eq(DFPLAYER_EQ_ROCK);
        k_sleep(K_SECONDS(3));
        
        printk(">>> Setting EQ to Normal\n");
        dfplayer_set_eq(DFPLAYER_EQ_NORMAL);
        k_sleep(K_SECONDS(3));
        
        // Stop and wait
        printk(">>> Stopping playback\n");
        dfplayer_stop();
        
        printk("\n=== Audio test cycle complete ===\n");
        printk("Waiting 5 seconds before next cycle...\n");
        k_sleep(K_SECONDS(5));
    }

    return 0;
}