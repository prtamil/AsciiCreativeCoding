# Pass 1 — sdf_gallery.c: SDF Sculpture Gallery

## Core Idea

Five interactive raymarched scenes, each demonstrating one SDF composition technique:

| Key | Preset | Technique |
|-----|--------|-----------|
| `1` | Blend   | `smin` smooth union with animated blend radius `k` and sphere separation |
| `2` | Boolean | `min` union / `max` intersection / `max(a,-b)` subtraction side-by-side |
| `3` | Twist   | Pre-warp `p.xz = rot2(p.y·k)·p.xz` before box SDF |
| `4` | Repeat  | Domain repetition `p -= cell·round(p/cell)` in xz |
| `5` | Sculpt  | Organic figure assembled from `smin`-blended primitives |

All five share: sphere marching, central-difference normals, 3-point lighting, soft shadows, AO,
progressive row-by-row rendering, gamma-encoded character selection, and 5 bright hue-varying themes.

**Interactive controls:**

| Key | Action |
|-----|--------|
| `1-5` | Switch preset scene |
| `l` | Cycle lighting mode: N·V → Phong → Flat |
| `t` | Cycle colour theme (5 themes) |
| `s` | Toggle soft shadows |
| `o` | Toggle ambient occlusion |
| `+/-` | Orbit speed |
| `p` | Pause/resume |
| `r` | Reset camera |
| `q/ESC` | Quit |

---

## SDF Primitives

Each primitive is a closed-form expression returning signed distance from point `p`:

```c
/* Sphere: centered at origin */
float sdSphere(Vec3 p, float r)  { return length(p) - r; }

/* Capsule: line segment a→b */
float sdCapsule(Vec3 p, Vec3 a, Vec3 b, float r) {
    Vec3 ab = b - a;
    float t = clamp(dot(p-a, ab) / dot(ab,ab), 0, 1);
    return length(p - (a + t*ab)) - r;
}

/* Torus in xz plane: major radius R, tube radius r */
float sdTorus(Vec3 p, float R, float r) {
    float qx = length(vec2(p.x, p.z)) - R;
    return length(vec2(qx, p.y)) - r;
}

/* Rounded box: half-extents b, corner radius r */
float sdRoundBox(Vec3 p, Vec3 b, float r) {
    Vec3 q = abs(p) - b;
    return length(max(q,0)) + min(max(q.x,max(q.y,q.z)),0) - r;
}
```

Primitives compose freely using Boolean operators.

---

## Boolean Operations

```
union        min(a, b)       — nearest surface; merges two objects
intersection max(a, b)       — only the overlap region
subtraction  max(a, -b)      — A with B-shaped holes drilled in
```

These are exact: no approximation, no mesh bookkeeping. Any SDF expression
can replace `a` or `b`, giving arbitrarily deep Boolean trees.

Scene 2 puts all three side-by-side with slow Y rotation so all faces are visible,
with distinct `col` values (0.15, 0.50, 0.85) so each operation gets a different theme color.

---

## Smooth Union — smin

`min(a, b)` creates a sharp crease where surfaces meet. The polynomial smooth minimum:

```c
float smin(float a, float b, float k) {
    float h = max(k - abs(a - b), 0.0) / k;
    return min(a, b) - h*h*k*0.25;
}
```

`h` is zero far from the blend zone and rises to 1 at the exact merge boundary.
The `h²·k/4` subtraction pulls both surfaces toward each other, rounding the crease
into a smooth neck.

Parameter `k` controls blend width:
- `k = 0` → hard `min` (sharp crease)
- `k = 0.1` → subtle smooth join
- `k = 0.5` → wide bulgy merge

Scene 1 animates both `k` and sphere separation to show the blend transition live.
Scene 5 (sculpt) uses `k=0.10` to weld seven primitives into seamless organic skin.

---

## Twist Deformation

```c
Vec3 twist(Vec3 p, float k) {
    float angle = p.y * k;
    float c = cos(angle), s = sin(angle);
    return vec3(c*p.x - s*p.z, p.y, s*p.x + c*p.z);
}

/* Usage: */
float d = sdRoundBox(twist(p, twist_k), half_extents, r);
```

The key insight: **deform the query point, not the primitive**. The box SDF remains
unchanged; by rotating the xz-plane by an angle proportional to height before querying,
the returned distance sees a twisted version of the geometry.

### Why the march step must be reduced

The twist stretches the local metric. A step of size `d` in the original space covers
more than `d` in the twisted space. The normal `MARCH_STEP = 0.85` can leap past thin
geometry. Scene 3 uses `MARCH_TW = 0.60`.

---

## Domain Repetition

```c
Vec3 domain_rep_xz(Vec3 p, float cell) {
    p.x -= cell * roundf(p.x / cell);
    p.z -= cell * roundf(p.z / cell);
    return p;
}
```

`round(p/cell)` selects which tile `p` is in; subtracting `cell × tile_index` maps `p`
back into `[-cell/2, cell/2]`. The single primitive SDF evaluated on this wrapped `p`
tiles infinitely at no extra cost.

### What must stay in original space

- **Light direction**: computed from the original hit position. Wrapping it would give
  each repeated sphere a different shadow direction.
- **Shadow ray origin**: must be the unrepeated surface point so rays escape correctly.
- **Color signal**: scene 4 uses the original xz angle (before repetition) as `col`
  so adjacent tiles have distinct hues across the grid.

---

## Three Lighting Modes (key `l`)

```
light_mode = 0  N·V    brightness = dot(N, camera_dir)
light_mode = 1  Phong  3-point key + fill + rim + shadow + AO
light_mode = 2  Flat   luma = 1.0 always, no computation
```

### N·V mode (default)

```c
float ndv  = max(0, dot(N, V));
float luma = KA + KD * ndv * ao;
```

No fixed light direction means no global brightness gradient. Every surface patch is
equally legible regardless of orientation. The theme color (driven by `col`) becomes the
primary visual signal. Best for understanding the shape of each preset.

### Phong mode

```c
float ndl  = max(0, dot(N, key_L));
float spec = pow(max(0, dot(reflect(-key_L,N), V)), SHIN);
float luma = KA + (KD*ndl + KS*spec)*shadow*ao
                + FILL_STR*KD*ndl_fill*ao
                + RIM_STR*KD*ndl_rim;
```

Three independent lights: orbiting key (strong, casts shadows), fixed fill (prevents
pitch-black shadow side), rim from below (separates silhouette from background).
`KA = 0.10` is kept low to maximise the dark-to-bright contrast range.

### Flat mode

Returns `1.0` immediately before any computation. Every hit pixel shows the densest
Bourke character. Only the theme color varies. Fastest render; useful for inspecting
the `col` signal distribution across the scene.

---

## 3-Point Lighting Setup

```
key_L  = normalize(cos(t·0.4)·0.7,  0.65, sin(t·0.4)·0.7)  orbits XZ
fill_L = normalize(-1.5, 0.5, -1.2)                          fixed, opposite side
rim_L  = normalize(0.0, -0.4, -1.0)                          back-edge, from below
```

Key orbits over time so shadows change as the scene animates. Fill and rim are fixed.
`KA = 0.10` (down from a naive 0.18) gives sharper contrast: edge-on faces read as
dark `:` characters while face-on reads as bright `#@`.

---

## Gamma Encoding (T.15)

```c
/* In pixel_to_cell(): */
float lg = powf(luma, 0.45f);
int ri = (int)(lg * (float)(BOURKE_LEN - 1) + 0.5f);
```

Phong/N·V luma is computed in **linear light space**. The Bourke ramp is drawn for
**perceptual uniformity** — each character step looks equally spaced to the eye.
Without gamma, linear values cluster in the top 2–3 chars (#, @) and most of the ramp
sits empty. `luma^0.45` redistributes values so the full 8-step ramp is used.

Example for N·V mode (KA=0.10, KD=0.72):

| Surface angle | linear luma | gamma luma | char |
|---|---|---|---|
| edge-on (ndv=0) | 0.10 | 0.35 | `.` |
| 45° (ndv=0.5) | 0.46 | 0.69 | `+` |
| face-on (ndv=1) | 0.82 | 0.91 | `#` |

The full ramp `.` → `:` → `=` → `+` → `*` → `#` → `@` is accessible.

---

## 8-Char Bourke Ramp

```c
#define BOURKE_LEN  8
static const char k_bourke[BOURKE_LEN + 1] = " .:=+*#@";
```

The old 10-char ramp `" .,:;!|%#@"` had ambiguous pairs: `,` and `;` look identical in
most terminal fonts, as do `!` and `|`. Wasted levels reduce the visible contrast range.
The 8-char set matches `mandelbulb_explorer.c` — every adjacent step is perceptually
distinct at arm's length.

Index 0 is space (no-hit background). Hit pixels are clamped to `[1, 7]` so every hit
pixel shows at least `.` and never collapses to the background.

---

## Bright Hue-varying Palette Design

```c
static const short k_palette[N_THEMES][GRAD_N] = {
    /* 0  Studio:  gold → yellow → warm white  */
    {214, 220, 221, 226, 227, 228, 230, 231},
    /* 1  Ember:   red → orange → yellow fire  */
    {196, 202, 203, 208, 209, 214, 220, 226},
    /* 2  Arctic:  cyan → sky → lavender → white */
    { 51,  87, 123, 159, 153, 189, 225, 231},
    /* 3  Toxic:   bright green → lime → yellow-green */
    { 46,  82, 118, 119, 154, 155, 190, 226},
    /* 4  Neon:    magenta → violet → pink → white */
    {201, 165, 171, 207, 177, 213, 219, 231},
};
```

**Old design flaw**: each theme ramped from near-black (e.g. index `17` = dark navy) to
bright. When `col` mapped to the low gradient steps the color was invisible against the
dark terminal background.

**New design rule**: every entry has at least one RGB channel at 4 or 5 in the xterm-256
cube (`color = 16 + 36r + 6g + b`). The gradient varies **hue** within the theme's
family rather than **brightness**. The darkest step in any theme is still a vivid
saturated color.

xterm verification (all entries are bright):
- `46` = (r=0,g=5,b=0) = saturated green ✓
- `51` = (r=0,g=5,b=5) = saturated cyan ✓
- `196` = (r=5,g=0,b=0) = saturated red ✓
- `201` = (r=5,g=0,b=5) = saturated magenta ✓
- `214` = (r=5,g=3,b=0) = bright orange ✓

---

## March Quality Constants

```c
#define MARCH_MAX   120    /* steps per ray: enough for complex Boolean scenes */
#define MARCH_EPS  0.0015f /* hit threshold: 2× tighter than 0.003 → crisper edges */
#define MARCH_STEP  0.85f  /* conservative: prevents over-stepping              */
#define MARCH_TW    0.60f  /* twist scene: approximate DE, step smaller         */
#define NORM_H     0.007f  /* central-diff step: tighter = sharper normals      */
```

Compared to the first version: EPS halved (0.003 → 0.0015) and steps increased
(80 → 120). Together these make edges sharper and eliminate surface bleed-through on
complex Boolean subtraction geometry. Camera moved from `3.8` to `2.8` units so objects
fill roughly 36% more screen area.

---

## Camera Snapshot (T.11)

A full frame takes `ch / ROWS_PER_TICK ≈ 14` render ticks. Auto-orbit advances
`cam_phi` every tick. Without a snapshot the camera moves mid-frame, shearing geometry.

```c
if (g_render_row == 0)
    camera_basis(cam_dist, cam_theta, cam_phi,
                 &g_snap_cam, &g_snap_fwd, &g_snap_right, &g_snap_up);
```

The basis is frozen only when `g_render_row == 0` (start of new pass). All rows in the
same pass share one frozen camera. `camera_basis` is NOT called in the main loop every
tick — only at frame start from within `canvas_render_rows`.

---

## Stable Buffer (T.10)

`g_fbuf` fills row-by-row. Reading unfinished rows directly shows a hard black scanline.
`g_stable` holds the last complete frame:

```c
Pixel *row = (vy < g_render_row) ? g_fbuf[vy] : g_stable[vy];
```

Fresh rows come from `g_fbuf`; pending rows come from `g_stable`. The frame always looks
complete; the scan frontier is invisible.

---

## Scene 5 — Sculpt Assembly

Seven primitives connected with `smin`:

```
body   = sdRoundBox at (0,-0.35,0)    — torso
head   = sdSphere   at (0, 0.82,0)    — head
neck   = sdCapsule  (0,0.2,0)→(0,0.58,0) — join head to torso
belt   = sdTorus    at (0,-0.62,0)    — waist ring
armL/R = sdCapsule  lateral           — left/right arms
armF/B = sdCapsule  anterior/posterior — front/back arms
```

```c
d = smin(body, head,  k=0.10)
d = smin(d,    neck,  k=0.10)
d = smin(d,    belt,  k=0.06)   /* tighter blend: keep ring detail */
d = smin(d,    armL,  k=0.10)
d = smin(d,    armR,  k=0.10)
d = smin(d,    armF,  k=0.10)
d = smin(d,    armB,  k=0.10)
```

Each `smin` call blends one part into the growing composite. The result is a single
continuous SDF — normals, AO, and shadows work identically to a carved solid.
No mesh, no UV, no topology management.

`col` = height-based gradient so head and arms read different theme colors from the body.
The figure rotates at `t * 0.38` rad/s so all four arms are visible over one orbit.

---

## From the Source

**Algorithm:** SDF composition gallery — showcasing five distinct SDF operations in five named scenes.

1. **Smooth union:** `smin(d1, d2, k)` — blends two SDFs into a single connected surface with a controllable blend radius k.
2. **Boolean CSG:** union = `min(d1, d2)`, intersection = `max(d1, d2)`, subtraction = `max(d1, −d2)`. Exact set operations on SDFs — union/intersection require no parameter k.
3. **Twist deformation:** rotate `p.xz` by angle θ ∝ p.y (twist around the Y axis). Domain deformation violates the Lipschitz condition so the SDF is no longer a true signed distance — the march step is reduced to `MARCH_TW = 0.60` (vs the standard `MARCH_STEP = 0.85`) to prevent overshoot.
4. **Domain repetition:** `p = mod(p, cell_size) − cell_size/2`. Evaluating SDF at the remapped p produces an infinite periodic grid from a single primitive's SDF.
5. **smin-based organic sculpting:** composing many simple primitives with smooth-min produces blob-like organic shapes.

**Rendering:** The gallery uses a **stable double-buffer** scheme: `g_fbuf` fills row-by-row while `g_stable` holds the last complete frame. Unfinished rows always show from `g_stable` so the scan frontier is invisible to the user.

# Structure

| Symbol | Type | Size | Role |
|--------|------|------|------|
| `g_fbuf[CANVAS_MAX_H][CANVAS_MAX_W]` | `Pixel[55][220]` | ~48 KB | Working framebuffer filled row-by-row each progressive render pass |
| `g_stable[CANVAS_MAX_H][CANVAS_MAX_W]` | `Pixel[55][220]` | ~48 KB | Last fully-rendered frame shown while the next pass is in progress |
| `k_palette[N_THEMES][GRAD_N]` | `short[5][8]` | 80 B | 256-color indices for the 5 colour themes × 8 gradient steps |
