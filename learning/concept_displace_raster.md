# Pass 1 — displace_raster: Vertex Displacement Rasterizer

## Core Idea

The same UV sphere from sphere_raster.c, but the vertex shader displaces each vertex along its surface normal by a time-varying scalar function. The mesh breathes, ripples, pulses, and spikes. The critical challenge: after displacement, surface normals are wrong. The vertex shader recomputes them using the central difference method — sampling the displacement function at nearby tangent points to estimate the actual deformed surface gradient.

---

## The Mental Model

Take the sphere mesh. In the vertex shader, instead of just applying MVP and passing through the normal, we:
1. Evaluate a displacement function at this vertex's surface position
2. Push the vertex outward along its normal by that amount
3. Recompute the actual normal of the deformed surface (since the sphere normal no longer points correctly after deformation)

The key insight: the displacement function is **pure** (same input = same output, no state). This is required because the central difference normal estimation calls the displacement function at nearby positions — the deltas must be genuine derivatives of the function.

---

## The Four Displacement Modes

| Mode | Formula | Effect |
|---|---|---|
| RIPPLE | `sin(time*2.5 + r*freq) * amp * taper` where r = distance from Y axis | Concentric rings radiating from equator |
| WAVE | `sin(time*2 + x*f + y*f*0.8 + z*f*0.5) * amp` | Diagonal travelling wave |
| PULSE | `(sin(t*1.5)*0.85 + sin(t*4.5)*0.15) * amp * exp(-r*f*0.4)` | Whole sphere breathes, equator-concentrated |
| SPIKY | `|sin(x*f)*sin(y*f)*sin(z*f)|^0.6 * amp` | Spiky ball with animated spikes |

---

## Central Difference Normal Recomputation

This is the most important algorithmic addition beyond sphere_raster.c:

**Problem**: After displacement, the surface normal changes. The original sphere normal only works for the undisplaced sphere.

**Solution**: For each vertex:
1. Build an orthogonal tangent basis (Tu, Tv) perpendicular to the sphere normal N
2. Sample the displacement function at `pos ± CD_EPS*Tu` and `pos ± CD_EPS*Tv`
3. The displaced tangent vectors are:
   - `T'_u = Tu*(2*CD_EPS) + N*(displace(pos+eps*Tu) - displace(pos-eps*Tu))`
   - `T'_v = Tv*(2*CD_EPS) + N*(displace(pos+eps*Tv) - displace(pos-eps*Tv))`
4. New normal = `normalize(cross(T'_u, T'_v))`

The CD_EPS must be right: too small → floating point noise, too large → normals lag the curvature.

---

## Data Structures

Same as sphere_raster.c, with additions:

### Displacement uniforms (added to Uniforms)
- `time` — seconds since start
- `amp` — displacement amplitude (from mode-specific params)
- `freq` — spatial frequency
- `disp_mode` — which of 4 functions to call

### `vert_displace` (new vertex shader)
Instead of vert_default, this shader:
1. Evaluates the displacement at the vertex position
2. Offsets position along normal
3. Recomputes normal via central difference

---

## Non-Obvious Decisions

### Displacement functions must be pure
For central differences to work correctly, `displace(pos)` must be a pure function. If it read from a changing array or had side effects, the (pos+eps) vs (pos-eps) samples would not give a genuine derivative.

### RIPPLE tapers at poles
`taper = 1 - |y| * 0.6` — the ripple is multiplied by this tapering factor so the poles stay nearly spherical. Without this, the poles develop large displacements that look visually broken because the ring-center distance `r` is zero at the poles.

### PULSE has two frequencies
`sin(t*1.5)*0.85 + sin(t*4.5)*0.15` — the 3x-frequency secondary oscillation gives the breath a slight "catch" so it doesn't feel perfectly mechanical. The falloff `exp(-r*freq*0.4)` concentrates the effect at the equator.

### TESS_U=48, TESS_V=32 — higher than sphere_raster
Higher tessellation shows smoother displacement detail. The wave mode in particular needs enough triangles for the travelling wave to look smooth rather than faceted.

---

## State Machine

No state machine. Continuous animation. One boolean: `paused`.

---

## Key Constants

| Constant | Default | Effect |
|---|---|---|
| `TESS_U` | 48 | Longitude resolution (more = smoother waves) |
| `TESS_V` | 32 | Latitude resolution |
| `CD_EPS` | 0.03 * SPHERE_R | Central difference step for normal recomputation |
| `ROT_Y` | 0.30 | Slow rotation (displacement detail needs to be visible) |
| `ROT_X` | 0.12 | |

---

## Open Questions for Pass 3

1. Disable normal recomputation (use original sphere normal). How wrong does the lighting look for each mode?
2. Try CD_EPS = 0.001 and CD_EPS = 0.5 — what are the artifacts at each extreme?
3. Add a 5th displacement mode: lattice pattern `|sin(x*f)|^10 + |sin(y*f)|^10 + |sin(z*f)|^10`. What shape does it produce?
4. What happens if amplitude exceeds 1.0? (vertices invert the sphere - can this cause z-fighting?)
5. Can you combine two displacement modes by adding their outputs? What does RIPPLE + SPIKY look like?
