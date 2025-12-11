#!/usr/bin/env python3
"""
MIDI to Melody Converter
Converts MIDI files to C code for the melody.h API

Usage:
    python midi2melody.py song.mid [--name MELODY_NAME] [--track N] [--transpose N]
    
Example:
    python midi2melody.py mario.mid --name MELODY_MARIO_THEME
    
Output can be pasted directly into your firmware code.

Requirements:
    pip install mido
"""

import argparse
import sys
try:
    import mido
except ImportError:
    print("Error: 'mido' package required. Install with: pip install mido")
    sys.exit(1)


# MIDI note number to note name mapping
NOTE_NAMES = ['C', 'CS', 'D', 'DS', 'E', 'F', 'FS', 'G', 'GS', 'A', 'AS', 'B']

def midi_note_to_name(midi_note: int) -> str:
    """Convert MIDI note number to NOTE_XX format"""
    if midi_note < 36 or midi_note > 107:
        return None  # Out of supported range
    octave = (midi_note // 12) - 1
    note_idx = midi_note % 12
    return f"NOTE_{NOTE_NAMES[note_idx]}{octave}"


def parse_midi(filepath: str, track_num: int = None, transpose: int = 0) -> list:
    """
    Parse MIDI file and extract notes with durations.
    Returns list of (midi_note, duration_ms) tuples.
    """
    mid = mido.MidiFile(filepath)
    
    # Get tempo (default 120 BPM = 500000 microseconds per beat)
    tempo = 500000
    for track in mid.tracks:
        for msg in track:
            if msg.type == 'set_tempo':
                tempo = msg.tempo
                break
    
    # Select track(s)
    if track_num is not None:
        if track_num >= len(mid.tracks):
            print(f"Warning: Track {track_num} doesn't exist. Using all tracks.")
            tracks = mid.tracks
        else:
            tracks = [mid.tracks[track_num]]
    else:
        tracks = mid.tracks
    
    # Merge all selected tracks
    merged = mido.merge_tracks(tracks) if len(tracks) > 1 else tracks[0]
    
    notes = []
    active_notes = {}  # note -> start_time
    current_time = 0  # in ticks
    
    for msg in merged:
        current_time += msg.time
        
        if msg.type == 'set_tempo':
            tempo = msg.tempo
        
        if msg.type == 'note_on' and msg.velocity > 0:
            # Note on
            active_notes[msg.note] = current_time
            
        elif msg.type == 'note_off' or (msg.type == 'note_on' and msg.velocity == 0):
            # Note off
            if msg.note in active_notes:
                start = active_notes.pop(msg.note)
                duration_ticks = current_time - start
                # Convert ticks to milliseconds
                duration_ms = int(mido.tick2second(duration_ticks, mid.ticks_per_beat, tempo) * 1000)
                
                transposed_note = msg.note + transpose
                if duration_ms > 0:
                    notes.append((transposed_note, start, duration_ms))
    
    # Sort by start time and return just note + duration
    notes.sort(key=lambda x: x[1])
    
    # Convert to sequential notes with rests
    result = []
    last_end = 0
    
    for note, start, duration in notes:
        start_ms = int(mido.tick2second(start, mid.ticks_per_beat, tempo) * 1000)
        
        # Add rest if there's a gap
        gap = start_ms - last_end
        if gap > 10:  # Ignore tiny gaps
            result.append((0, gap))  # REST
        
        result.append((note, duration))
        last_end = start_ms + duration
    
    return result


def generate_c_code(notes: list, name: str) -> str:
    """Generate C code for the melody"""
    lines = [f"const Note {name}[] = {{"]
    
    for midi_note, duration in notes:
        if midi_note == 0:
            lines.append(f"    {{NOTE_REST, {duration}}},")
        else:
            note_name = midi_note_to_name(midi_note)
            if note_name:
                lines.append(f"    {{{note_name}, {duration}}},")
            else:
                # Note out of range, use frequency comment
                lines.append(f"    // Note {midi_note} out of range (C3-B7)")
    
    lines.append("    MELODY_END")
    lines.append("};")
    
    return "\n".join(lines)


def print_midi_info(filepath: str):
    """Print information about the MIDI file"""
    mid = mido.MidiFile(filepath)
    print(f"\n--- MIDI File Info: {filepath} ---")
    print(f"Type: {mid.type}")
    print(f"Ticks per beat: {mid.ticks_per_beat}")
    print(f"Number of tracks: {len(mid.tracks)}")
    
    for i, track in enumerate(mid.tracks):
        note_count = sum(1 for msg in track if msg.type == 'note_on' and msg.velocity > 0)
        print(f"  Track {i}: '{track.name}' ({note_count} notes)")
    print()


def main():
    parser = argparse.ArgumentParser(
        description='Convert MIDI files to C code for melody.h API',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
    %(prog)s song.mid                        # Convert with auto-generated name
    %(prog)s song.mid --name MY_MELODY       # Custom melody name
    %(prog)s song.mid --track 1              # Use only track 1
    %(prog)s song.mid --transpose -12        # Transpose down one octave
    %(prog)s song.mid --info                 # Show MIDI file info
        """
    )
    parser.add_argument('midi_file', help='Input MIDI file')
    parser.add_argument('--name', '-n', default=None, 
                        help='Name for the melody constant (default: MELODY_<FILENAME>)')
    parser.add_argument('--track', '-t', type=int, default=None,
                        help='Track number to use (default: merge all tracks)')
    parser.add_argument('--transpose', type=int, default=0,
                        help='Transpose notes by N semitones (e.g., -12 for one octave down)')
    parser.add_argument('--info', '-i', action='store_true',
                        help='Show MIDI file information and exit')
    
    args = parser.parse_args()
    
    if args.info:
        print_midi_info(args.midi_file)
        return
    
    # Generate melody name from filename if not provided
    if args.name is None:
        import os
        basename = os.path.splitext(os.path.basename(args.midi_file))[0]
        args.name = f"MELODY_{basename.upper().replace(' ', '_').replace('-', '_')}"
    
    print(f"// Generated from: {args.midi_file}")
    if args.track is not None:
        print(f"// Track: {args.track}")
    if args.transpose != 0:
        print(f"// Transposed: {args.transpose:+d} semitones")
    print()
    
    notes = parse_midi(args.midi_file, args.track, args.transpose)
    
    if not notes:
        print("// Warning: No notes found in MIDI file!")
        return
    
    print(f"// Total notes: {len(notes)}")
    total_duration = sum(d for _, d in notes)
    print(f"// Duration: {total_duration}ms ({total_duration/1000:.1f}s)")
    print()
    
    print(generate_c_code(notes, args.name))


if __name__ == '__main__':
    main()
