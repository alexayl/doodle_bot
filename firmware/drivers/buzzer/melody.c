/**
 * Melody Player Implementation
 * MIDI-style melody playback through PWM buzzer
 */

#include "melody.h"
#include "pwm_buzzer.h"
#include <zephyr/kernel.h>
#include <math.h>

/* ============================================================================
 * MIDI to Frequency Conversion
 * Formula: f = 440 * 2^((n-69)/12) where n is MIDI note number
 * ============================================================================ */

// Pre-calculated frequency table for MIDI notes 36-107 (C2 to B7)
// This avoids floating-point math at runtime
static const uint16_t midi_freq_table[] = {
    // Octave 2: C2-B2 (MIDI 36-47)
    65,   69,   73,   78,   82,   87,   92,   98,   104,  110,  117,  123,
    // Octave 3: C3-B3 (MIDI 48-59)
    131,  139,  147,  156,  165,  175,  185,  196,  208,  220,  233,  247,
    // Octave 4: C4-B4 (MIDI 60-71)
    262,  277,  294,  311,  330,  349,  370,  392,  415,  440,  466,  494,
    // Octave 5: C5-B5 (MIDI 72-83)
    523,  554,  587,  622,  659,  698,  740,  784,  831,  880,  932,  988,
    // Octave 6: C6-B6 (MIDI 84-95)
    1047, 1109, 1175, 1245, 1319, 1397, 1480, 1568, 1661, 1760, 1865, 1976,
    // Octave 7: C7-B7 (MIDI 96-107)
    2093, 2217, 2349, 2489, 2637, 2794, 2960, 3136, 3322, 3520, 3729, 3951
};

#define MIDI_TABLE_START 36
#define MIDI_TABLE_END   107

uint16_t midi_to_freq(uint8_t midi_note)
{
    if (midi_note == NOTE_REST) {
        return 0;
    }
    if (midi_note < MIDI_TABLE_START || midi_note > MIDI_TABLE_END) {
        return 0;  // Out of range
    }
    return midi_freq_table[midi_note - MIDI_TABLE_START];
}


/* ============================================================================
 * Playback Functions
 * ============================================================================ */

int melody_note(uint8_t midi_note, uint16_t duration_ms, uint8_t volume)
{
    if (midi_note == NOTE_REST) {
        // Rest: just wait
        k_sleep(K_MSEC(duration_ms));
        return 0;
    }
    
    uint16_t freq = midi_to_freq(midi_note);
    if (freq == 0) {
        return -1;  // Invalid note
    }
    
    return pwm_buzzer_beep(freq, volume, duration_ms);
}

int melody_play(const Note* notes, uint8_t volume)
{
    return melody_play_tempo(notes, volume, 100);
}

int melody_play_tempo(const Note* notes, uint8_t volume, uint8_t tempo_percent)
{
    if (notes == NULL || tempo_percent == 0) {
        return -1;
    }
    
    for (int i = 0; !IS_MELODY_END(notes[i]); i++) {
        // Scale duration by tempo (100% = normal, 200% = double speed/half duration)
        uint16_t duration = (notes[i].duration_ms * 100) / tempo_percent;
        
        int err = melody_note(notes[i].midi_note, duration, volume);
        if (err < 0) {
            return err;
        }
    }
    
    return 0;
}


/* ============================================================================
 * Built-in Melodies
 * ============================================================================ */

// Super Mario Bros Course Clear fanfare
const Note MELODY_MARIO_COURSE_CLEAR[] = {
    // Ascending arpeggio intro (fast)
    {NOTE_G4,  60},
    {NOTE_C5,  60},
    {NOTE_E5,  60},
    {NOTE_G5,  60},
    {NOTE_C6,  60},
    {NOTE_E6,  150},  // Top note, slight hold
    // Main phrase
    {NOTE_C6,  200},
    {NOTE_G5,  200},
    // Chromatic figure (the distinctive Mario sound)
    {NOTE_AB5, 100},
    {NOTE_BB5, 100},
    {NOTE_AB5, 100},
    // Resolution
    {NOTE_G5,  100},
    {NOTE_F5,  100},
    {NOTE_G5,  400},  // Final hold
    MELODY_END
};

// Rising arpeggio for connection
const Note MELODY_CONNECT[] = {
    {NOTE_C5, 80},
    {NOTE_E5, 80},
    {NOTE_G5, 120},
    MELODY_END
};

// Falling tone for disconnect
const Note MELODY_DISCONNECT[] = {
    {NOTE_G5, 100},
    {NOTE_E5, 100},
    {NOTE_C5, 150},
    MELODY_END
};

// Success confirmation
const Note MELODY_SUCCESS[] = {
    {NOTE_G5, 80},
    {NOTE_C6, 150},
    MELODY_END
};

// Error beep
const Note MELODY_ERROR[] = {
    {NOTE_C4, 150},
    {NOTE_REST, 50},
    {NOTE_C4, 150},
    MELODY_END
};

const Note MELODY_LEVEL_COMPLETE[] = {
    {NOTE_G3, 131},
    {NOTE_C4, 131},
    {NOTE_E3, 131},
    {NOTE_E4, 130},
    {NOTE_G3, 131},
    {NOTE_G4, 131},
    {NOTE_C4, 131},
    {NOTE_C3, 131},
    {NOTE_C5, 131},
    {NOTE_E3, 131},
    {NOTE_E4, 131},
    {NOTE_E5, 130},
    {NOTE_G3, 130},
    {NOTE_G4, 131},
    {NOTE_G5, 393},
    {NOTE_C5, 393},
    {NOTE_E4, 393},
    {NOTE_REST, 21},
    {NOTE_E5, 393},
    {NOTE_G4, 393},
    {NOTE_C4, 393},
    {NOTE_REST, 20},
    {NOTE_GS3, 131},
    {NOTE_C4, 131},
    {NOTE_DS3, 131},
    {NOTE_DS4, 130},
    {NOTE_GS3, 131},
    {NOTE_GS4, 131},
    {NOTE_C4, 131},
    {NOTE_C3, 131},
    {NOTE_C5, 131},
    {NOTE_DS3, 131},
    {NOTE_DS4, 131},
    {NOTE_DS5, 130},
    {NOTE_GS3, 130},
    {NOTE_GS4, 131},
    {NOTE_GS5, 393},
    {NOTE_C5, 393},
    {NOTE_DS4, 393},
    {NOTE_REST, 20},
    {NOTE_DS5, 393},
    {NOTE_GS4, 393},
    {NOTE_C4, 393},
    {NOTE_REST, 21},
    {NOTE_AS3, 131},
    {NOTE_D4, 131},
    {NOTE_F3, 131},
    {NOTE_F4, 130},
    {NOTE_AS3, 131},
    {NOTE_AS4, 131},
    {NOTE_D4, 131},
    {NOTE_D3, 196},
    {NOTE_D5, 131},
    {NOTE_F3, 131},
    {NOTE_F4, 131},
    {NOTE_F5, 130},
    {NOTE_AS3, 130},
    {NOTE_AS4, 131},
    {NOTE_AS5, 393},
    {NOTE_D5, 393},
    {NOTE_F4, 393},
    {NOTE_REST, 21},
    {NOTE_AS5, 131},
    {NOTE_D4, 131},
    {NOTE_D5, 393},
    {NOTE_AS5, 131},
    {NOTE_D4, 131},
    {NOTE_AS5, 130},
    {NOTE_D4, 130},
    {NOTE_E5, 393},
    {NOTE_C6, 393},
    {NOTE_C4, 393},
    MELODY_END
};

const Note MELODY_1UP[] = {
    {NOTE_E6, 68},
    {NOTE_REST, 68},
    {NOTE_G6, 68},
    {NOTE_REST, 68},
    {NOTE_E7, 68},
    {NOTE_REST, 69},
    {NOTE_C7, 68},
    {NOTE_REST, 68},
    {NOTE_D7, 68},
    {NOTE_REST, 68},
    {NOTE_G7, 681},
    MELODY_END
};

const Note MELODY_GAME_OVER[] = {
    {NOTE_C5, 199},
    {NOTE_E4, 199},
    {NOTE_G2, 199},
    {NOTE_G3, 199},
    {NOTE_REST, 200},
    {NOTE_G4, 199},
    {NOTE_C4, 199},
    {NOTE_E2, 199},
    {NOTE_E3, 199},
    {NOTE_REST, 201},
    {NOTE_E4, 300},
    {NOTE_G3, 300},
    {NOTE_C2, 300},
    {NOTE_C3, 300},
    {NOTE_A4, 229},
    {NOTE_F3, 693},
    {NOTE_F2, 693},
    {NOTE_F4, 700},
    {NOTE_B4, 229},
    {NOTE_A4, 233},
    {NOTE_GS4, 300},
    {NOTE_F4, 899},
    {NOTE_CS3, 950},
    {NOTE_CS2, 950},
    {NOTE_AS4, 300},
    {NOTE_GS4, 300},
    {NOTE_G4, 150},
    {NOTE_E4, 150},
    {NOTE_C2, 2099},
    {NOTE_C3, 2099},
    {NOTE_F4, 150},
    {NOTE_D4, 150},
    {NOTE_G4, 1799},
    {NOTE_E4, 1799},
    MELODY_END
};
