# Concept: CA Music (Cellular Automaton → Audio Mapping)

## Pass 1 — Understanding

### Core Idea
Map the state of a cellular automaton (typically 1D Wolfram or 2D Life) to musical notes or rhythmic patterns. Each live cell in a row triggers a note; the column position determines pitch. The evolving CA generates algorithmic music.

### Mental Model
Imagine a piano roll where columns are notes (C, D, E...) and rows are time steps. At each CA generation, every live cell in the current row "plays" its corresponding note. The CA evolution generates the melody. Simple rules create surprisingly musical patterns.

### Key Mapping Strategies

**1D CA (Wolfram rule) → Piano roll**:
- Each row = one time step
- Column position → pitch (map 0..W to musical scale)
- Cell alive → note on; dead → rest
- Scroll through CA rows as time progresses

**2D Life → Percussion grid**:
- Divide columns into instrument zones (kick, snare, hihat, etc.)
- Each row pass triggers instruments wherever cells are alive
- Life evolution creates evolving rhythmic patterns

**Pitch mapping options**:
- Linear: `freq = BASE_FREQ + col * SEMITONE_STEP`
- Diatonic scale: map column to nearest scale degree (C major = C D E F G A B)
- Pentatonic: map to 5-note scale (more harmonious, fewer clashes)

### Data Structures
- CA state (1D or 2D)
- `note_on[W]`: bool — which columns are active this step
- Pitch table: `freq[W]` precomputed frequencies
- MIDI or terminal beep output (or just visual piano roll)
- Playhead position (current CA row)

### Non-Obvious Decisions
- **Pentatonic scale avoids dissonance**: Any combination of pentatonic notes sounds acceptable. Chromatic mapping produces frequent dissonant clusters.
- **Rhythm quantization**: CA runs continuously but notes must align to a beat grid. Trigger notes at discrete beat intervals.
- **Terminal-only audio**: Use `\a` bell character or ANSI sound escape. For real audio, write to /dev/audio or use a library (miniaudio, alsa).
- **Visual piano roll**: Even without actual audio, display the piano roll as a scrolling ASCII visualization. Live cells = bright characters at pitch positions.
- **Rule selection**: Rule 30 (Wolfram) → chaotic, surprising melody. Rule 90 → symmetric, repetitive. Rule 110 → complex, semi-structured.

### Key Constants
| Name | Role |
|------|------|
| RULE | Wolfram rule number (0–255) |
| BASE_NOTE | lowest MIDI note |
| SCALE | scale mapping (pentatonic, major, chromatic) |
| BPM | beats per minute (CA steps per second) |
| N_COLS | number of columns = number of pitches |

### Open Questions
- Which Wolfram rule produces the most musical output?
- Can you detect repeated patterns in the CA and use them as motifs?
- Add velocity mapping: neighbor count → note volume

---

## Pass 2 — Implementation

### Pseudocode
```
pentatonic_freq(col, n_cols, base_midi):
    scale = [0, 2, 4, 7, 9]   # pentatonic intervals in semitones
    degree = col * len(scale) / n_cols
    octave = degree // len(scale)
    note   = scale[degree % len(scale)]
    midi = base_midi + octave*12 + note
    return 440 * 2^((midi-69)/12)

step_ca():
    next_row = apply_rule(current_row, RULE)
    for col in 0..W:
        if next_row[col]: play_note(col, pentatonic_freq(col,...))
    scroll_display(next_row)
    current_row = next_row

draw_piano_roll(history[H]):
    for row in 0..H:
        for col in 0..W:
            if history[row][col]:
                color = hue_from_pitch(col)
                mvaddch(row, col, '█' | COLOR_PAIR(color))
            else:
                mvaddch(row, col, '·')
    # highlight playhead
    draw_hline(playhead_row)
```

### Module Map
```
§1 config    — RULE, BASE_NOTE, SCALE, BPM
§2 ca        — apply_rule_1d(), wolfram_lookup()
§3 pitch     — build_pitch_table(), pentatonic_map()
§4 audio     — play_note() (beep/MIDI/alsa), schedule_beat()
§5 draw      — piano_roll display, scrolling history
§6 app       — main loop (beat timer), keys (rule, scale, tempo, mute)
```

### Data Flow
```
CA row → apply rule → new row → trigger notes for live cells
→ scroll piano roll history → draw → audio output
```
