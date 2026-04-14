# Pass 1 — mandelbulb_raymarcher.c: Hue-Depth Fractal (No Lighting)

## Core Idea

A stripped-down Mandelbulb renderer with **zero lighting computation** — no normals, no shadows, no AO. The sole visual signal is **hue**: the smooth escape value `mu` maps to a position on a 30-step rainbow wheel, cycling through `COLOR_BANDS` full hue rotations across the depth shells of the fractal. Color alone encodes everything about the surface.

This is both simpler and philosophically distinct from mandelbulb_explorer.c: it explores what information is already in the fractal's geometry without adding any shading model on top.

---

## 30-Step Hue Wheel

```c
static const int k_hue256[30] = {
    196, 202, 208, 214, 220, 226,   /* red → orange → yellow      */
    190, 154, 118,  82,  46,        /* yellow-green → green        */
     47,  48,  49,  50,  51,        /* spring-green → cyan         */
     45,  39,  33,  27,  21,        /* azure → sky-blue → blue     */
     57,  93, 129, 165, 201,        /* indigo → violet → magenta   */
    200, 199, 198, 197              /* rose → back toward red      */
};
```

Each entry is an xterm-256 index in the color cube. 30 steps covers all major hue families. The last 4 entries (rose) smoothly bridge magenta back to red so the cycle has no visible seam.

### Hue Index Calculation

```c
static int hue_color_pair(float smooth, int bands) {
    float f   = smooth * HUE_N * bands;   /* multiply by bands for cycling */
    int   idx = (int)f % HUE_N;           /* position on wheel */
    idx = (idx + g_hue_offset + HUE_N*16) % HUE_N;  /* apply shift key */
    return idx + 1;                        /* pair 1..30 */
}
```

`COLOR_BANDS=3` means `mu` from 0→1 cycles through the full rainbow 3 times. The fractal surface has multiple nested shells at different escape times — bands=3 applies a different color to each shell crossing, making the layered structure visible.

The `HUE_N*16` addition prevents modular underflow when `g_hue_offset` is negative.

---

## Why '.' for All Hit Pixels

The char density ramp (`luma_to_cell`) was removed. All hit pixels show `'.'`. This forces the viewer to read depth purely from color — there is no brightness gradient to fall back on. The uniform character also makes the fractal surface look like a continuous colored solid rather than an ASCII dithered image.

For miss pixels (no hit): nothing is drawn (space character, default background). The fractal floats in blackness.

---

## Comparing to mandelbulb_explorer.c

| Aspect | explorer.c | raymarcher.c |
|---|---|---|
| Lighting | Full Phong + AO + shadows | None |
| Color signal | 8-entry palette + luma | 30-step hue from escape time |
| Characters | Bourke density ramp | All '.' |
| Themes | 8 multi-hue themes | Key shifts hue offset |
| Computation | Heavy (normals, shadows) | Light (DE only) |
| Visual style | 3D shaded sculpture | Psychedelic color field |

The no-lighting approach reveals that the fractal's coloring is an intrinsic property of the escape time distribution, not a product of the lighting model.

---

## Non-Obvious Decisions

### Why 30 hue steps not 256?

30 steps covers all hue families with distinct color separation. More steps would make adjacent colors too similar to distinguish at terminal resolution. Fewer steps (e.g., 7) would show hard color jumps rather than smooth cycling.

### Why cycle with COLOR_BANDS instead of using raw mu?

With bands=1, the entire depth range maps once through the rainbow — the fractal shows one full hue cycle. With bands=3, a single outer "shell" of the fractal might span only a third of the color range, giving tighter, more visible banding. High bands = thin color rings; low bands = broad color regions. The key `c` cycles 1→2→3→4→5 for quick comparison.

### Near-miss glow without lighting

`glow_str = (1 - min_d / GLOW_RANGE)^3` tracks how close the march got without hitting. For near-misses (ray grazes the fractal), glow_str > 0 and a dim halo character is placed. This preserves the edge glow effect even without a lighting model — it comes from the march geometry alone.
