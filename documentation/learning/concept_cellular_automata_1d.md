# Pass 1 — artistic/cellular_automata_1d.c: 1-D Wolfram Cellular Automata

## Core Idea

A single row of cells, each either alive (1) or dead (0). Each generation, every cell looks at its left neighbour, itself, and its right neighbour — three cells form a 3-bit pattern (0–7), and the rule number encodes what the center cell becomes for each pattern. Starting from a single live cell in the center, the pattern builds downward one row at a time, eventually filling the screen with whatever structure the rule produces: static blocks, repeating stripes, chaotic noise, triangular fractals, or complex self-similar structures.

## The Mental Model

Think of it like a 1-row game where each cell is a light switch. Every second, each switch looks at itself and its two neighbours — that combination of three switches appears in one of 8 possible patterns. The rule number (0–255) is like a lookup table: for each of the 8 patterns, does the center switch turn on or off in the next generation? Rule 30 looks chaotic; rule 90 makes perfect Sierpinski triangles; rule 110 can compute anything a computer can compute.

## Rule Encoding

The 3 neighbours form a 3-bit index: `idx = (l << 2) | (m << 1) | r`. The rule bit for that index is:

    next_cell = (rule >> idx) & 1

For example, rule 30 = 0b00011110:
| Left | Center | Right | Index | New value |
|---|---|---|---|---|
| 1 | 1 | 1 | 7 | 0 |
| 1 | 1 | 0 | 6 | 0 |
| 1 | 0 | 1 | 5 | 0 |
| 1 | 0 | 0 | 4 | 1 |
| 0 | 1 | 1 | 3 | 1 |
| 0 | 1 | 0 | 2 | 1 |
| 0 | 0 | 1 | 1 | 1 |
| 0 | 0 | 0 | 0 | 0 |

## Build-Down Animation

Rather than showing the complete pattern all at once, the pattern builds from row 0 downward. The state machine:

```
ST_BUILD: add one row per g_delay ticks (default 3 ticks = ~0.1 s per row)
     ↓ (g_gen reaches g_ca_rows — screen full)
ST_PAUSE: hold complete pattern for PAUSE_TICKS = 90 ticks (~3 s)
     ↓
next preset or reset → ST_BUILD
```

This lets the user see the pattern growing — which is often more visually interesting than the final static result.

## Wolfram Classification

Stephen Wolfram classified all 256 rules into 4 behavioral classes:

| Class | Behavior | Color | Example rules |
|---|---|---|---|
| Fixed | All cells die or all cells live | Grey | 0, 255 |
| Periodic | Repeating stripes or checkerboards | Cyan | 4, 8, 12 |
| Chaotic | Seemingly random noise | Orange | 30, 45, 22 |
| Complex | Localised structures, computation-universal | Green | 110, 54, 105 |
| Fractal | Self-similar triangular patterns | Yellow | 90, 126, 57 |

The title bar displays the class name in its class color, so the user immediately knows what type of pattern to expect.

## Non-Obvious Decisions

### Why build-down instead of scroll?
Scrolling means old rows disappear off the top — the user never sees the full pattern at once. Build-down lets the user watch the pattern develop and then see the complete result. The PAUSE state gives time to appreciate the finished pattern.

### Why the A_REVERSE title bar?
The class color is the key piece of information (tells you what kind of pattern to expect). `A_REVERSE` swaps foreground and background, making the entire title bar row a solid color block — highly visible without any background color that might not be supported.

### Why 17 specific presets rather than all 256?
Many rules produce boring results (all dead, all alive, simple stripes). The 17 presets cover the range of interesting behavior — one or two examples from each class — so cycling through them shows the diversity without the tedium of all 256.

### Why live digit input for custom rules?
The presets are a starting point, but part of the joy of cellular automata is exploring. Allowing any rule number 0–255 to be entered directly invites experimentation.

## From the Source

**Algorithm:** Wolfram Elementary Cellular Automaton (ECA), introduced by Stephen Wolfram (1983). A 1-D binary row evolves by applying an 8-bit lookup table (the rule): each cell's new state depends on itself and its two neighbours — 2³=8 neighbourhood configurations → 256 possible rules.

**Math:** Rule encoding: the 3-bit neighbourhood (left, center, right) is treated as a binary number 0–7; the rule's n-th bit gives the new center state for neighbourhood n. Example: Rule 110 = 0b01101110 → bit[n] = (110>>n)&1.

**Physics:** Rule 110 is Wolfram Class 4 and is Turing-complete (Cook, 2004).

**Performance:** Each row update is O(cols) — single pass with bitwise neighbourhood extraction. No double buffer needed since rows are computed top-to-bottom from the previous row only.

## Key Constants

| Constant | Effect |
|---|---|
| g_delay | Ticks per row during ST_BUILD; lower → faster build |
| PAUSE_TICKS | How long the complete pattern is held before advancing |
| g_ca_rows | Number of grid rows = screen rows − 2 (title bar + HUD) |
| DELAY_DEF | Default delay; adjustable with +/- |
