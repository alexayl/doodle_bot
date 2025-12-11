/**
 * Melody Player API
 * Play MIDI-style melodies through the PWM buzzer
 * 
 * Usage:
 *   // Define a melody using MIDI note numbers
 *   static const Note my_melody[] = {
 *       {NOTE_C5, 100},   // C5 for 100ms
 *       {NOTE_E5, 100},   // E5 for 100ms
 *       {NOTE_G5, 200},   // G5 for 200ms
 *       {NOTE_REST, 50},  // Rest for 50ms
 *       MELODY_END
 *   };
 *   
 *   melody_play(my_melody, 10);  // Play at 10% volume
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================================
 * MIDI Note Definitions (Note Number -> Frequency)
 * Standard MIDI note numbers: Middle C (C4) = 60
 * ============================================================================ */

// Rest (silence)
#define NOTE_REST   0

// Octave 2
#define NOTE_C2     36
#define NOTE_CS2    37
#define NOTE_D2     38
#define NOTE_DS2    39
#define NOTE_E2     40
#define NOTE_F2     41
#define NOTE_FS2    42
#define NOTE_G2     43
#define NOTE_GS2    44
#define NOTE_A2     45
#define NOTE_AS2    46
#define NOTE_B2     47

// Octave 3
#define NOTE_C3     48
#define NOTE_CS3    49
#define NOTE_D3     50
#define NOTE_DS3    51
#define NOTE_E3     52
#define NOTE_F3     53
#define NOTE_FS3    54
#define NOTE_G3     55
#define NOTE_GS3    56
#define NOTE_A3     57
#define NOTE_AS3    58
#define NOTE_B3     59

// Octave 4 (Middle C octave)
#define NOTE_C4     60
#define NOTE_CS4    61
#define NOTE_D4     62
#define NOTE_DS4    63
#define NOTE_E4     64
#define NOTE_F4     65
#define NOTE_FS4    66
#define NOTE_G4     67
#define NOTE_GS4    68
#define NOTE_A4     69   // A440 tuning reference
#define NOTE_AS4    70
#define NOTE_B4     71

// Octave 5
#define NOTE_C5     72
#define NOTE_CS5    73
#define NOTE_D5     74
#define NOTE_DS5    75
#define NOTE_E5     76
#define NOTE_F5     77
#define NOTE_FS5    78
#define NOTE_G5     79
#define NOTE_GS5    80
#define NOTE_A5     81
#define NOTE_AS5    82
#define NOTE_B5     83

// Octave 6
#define NOTE_C6     84
#define NOTE_CS6    85
#define NOTE_D6     86
#define NOTE_DS6    87
#define NOTE_E6     88
#define NOTE_F6     89
#define NOTE_FS6    90
#define NOTE_G6     91
#define NOTE_GS6    92
#define NOTE_A6     93
#define NOTE_AS6    94
#define NOTE_B6     95

// Octave 7
#define NOTE_C7     96
#define NOTE_CS7    97
#define NOTE_D7     98
#define NOTE_DS7    99
#define NOTE_E7     100
#define NOTE_F7     101
#define NOTE_FS7    102
#define NOTE_G7     103
#define NOTE_GS7    104
#define NOTE_A7     105
#define NOTE_AS7    106
#define NOTE_B7     107

// Flat note aliases (enharmonic equivalents)
#define NOTE_DB2    NOTE_CS2
#define NOTE_EB2    NOTE_DS2
#define NOTE_GB2    NOTE_FS2
#define NOTE_AB2    NOTE_GS2
#define NOTE_BB2    NOTE_AS2

#define NOTE_DB3    NOTE_CS3
#define NOTE_EB3    NOTE_DS3
#define NOTE_GB3    NOTE_FS3
#define NOTE_AB3    NOTE_GS3
#define NOTE_BB3    NOTE_AS3

#define NOTE_DB4    NOTE_CS4
#define NOTE_EB4    NOTE_DS4
#define NOTE_GB4    NOTE_FS4
#define NOTE_AB4    NOTE_GS4
#define NOTE_BB4    NOTE_AS4

#define NOTE_DB5    NOTE_CS5
#define NOTE_EB5    NOTE_DS5
#define NOTE_GB5    NOTE_FS5
#define NOTE_AB5    NOTE_GS5
#define NOTE_BB5    NOTE_AS5

#define NOTE_DB6    NOTE_CS6
#define NOTE_EB6    NOTE_DS6
#define NOTE_GB6    NOTE_FS6
#define NOTE_AB6    NOTE_GS6
#define NOTE_BB6    NOTE_AS6

#define NOTE_DB7    NOTE_CS7
#define NOTE_EB7    NOTE_DS7
#define NOTE_GB7    NOTE_FS7
#define NOTE_AB7    NOTE_GS7
#define NOTE_BB7    NOTE_AS7


/* ============================================================================
 * Melody Structures
 * ============================================================================ */

/**
 * Single note in a melody
 */
typedef struct {
    uint8_t  midi_note;    // MIDI note number (0 = rest, 48-107 for notes)
    uint16_t duration_ms;  // Duration in milliseconds
} Note;

/**
 * Marker for end of melody array
 */
#define MELODY_END {0, 0}

/**
 * Check if note is the end marker
 */
#define IS_MELODY_END(note) ((note).midi_note == 0 && (note).duration_ms == 0)


/* ============================================================================
 * API Functions
 * ============================================================================ */

/**
 * Convert MIDI note number to frequency in Hz
 * @param midi_note MIDI note number (0-127, where 69 = A4 = 440Hz)
 * @return Frequency in Hz (0 for rest/invalid)
 */
uint16_t midi_to_freq(uint8_t midi_note);

/**
 * Play a single note
 * @param midi_note MIDI note number (use NOTE_* defines)
 * @param duration_ms Duration in milliseconds
 * @param volume Volume percentage (0-100)
 * @return 0 on success, negative on error
 */
int melody_note(uint8_t midi_note, uint16_t duration_ms, uint8_t volume);

/**
 * Play a melody (blocking)
 * @param notes Array of Note structs, terminated with MELODY_END
 * @param volume Volume percentage (0-100)
 * @return 0 on success, negative on error
 */
int melody_play(const Note* notes, uint8_t volume);

/**
 * Play a melody with tempo scaling
 * @param notes Array of Note structs, terminated with MELODY_END
 * @param volume Volume percentage (0-100)
 * @param tempo_percent Tempo as percentage (100 = normal, 200 = double speed, 50 = half speed)
 * @return 0 on success, negative on error
 */
int melody_play_tempo(const Note* notes, uint8_t volume, uint8_t tempo_percent);


/* ============================================================================
 * Built-in Melodies
 * ============================================================================ */

/** Super Mario Bros Course Clear fanfare */
extern const Note MELODY_MARIO_COURSE_CLEAR[];

/** Super Mario Bros Level Complete (full version) */
extern const Note MELODY_LEVEL_COMPLETE[];

/** Simple rising arpeggio (connection sound) */
extern const Note MELODY_CONNECT[];

/** Simple falling tone (error/disconnect) */
extern const Note MELODY_DISCONNECT[];

/** Success/confirmation beep */
extern const Note MELODY_SUCCESS[];

/** Error beep */
extern const Note MELODY_ERROR[];

extern const Note MELODY_1UP[];

extern const Note MELODY_GAME_OVER[];

#ifdef __cplusplus
}
#endif
