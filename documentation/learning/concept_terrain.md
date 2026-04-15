# Concept: Terrain (Diamond-Square Algorithm)

## Pass 1 — Understanding

### Core Idea
Generate a fractal heightmap using the diamond-square algorithm. Start with four corner heights, recursively subdivide: the diamond step averages four corners and adds random displacement, the square step averages four diamonds. At each level, reduce the displacement range by a roughness factor.

### Mental Model
Start with a blank grid (2ⁿ+1 × 2ⁿ+1). Set corners randomly. Now find the center of each square: average the four corners, add random noise. Find the center of each diamond: average the four diamond points, add noise. Repeat at half the scale. The noise shrinks with scale, so large features dominate small ones — exactly like real terrain.

### Key Equations
```
Diamond step: center = avg(4 corners) + random(-h, +h)
Square step:  edge   = avg(4 diamonds) + random(-h, +h)
h *= roughness^(1/2)    ← scale per level
```

Roughness H (Hurst exponent) controls terrain type:
- H=0.5: rough, rocky
- H=0.9: smooth, rolling hills
- H=0.3: very jagged

### Data Structures
- `heightmap[N+1][N+1]`: float array, N=2^k
- Values in [0,1] after normalization
- Render: map height to terrain char/color

### Non-Obvious Decisions
- **Must be 2^n + 1 size**: The algorithm halves the step size each iteration. Grid must be exactly this size for the steps to align.
- **Toroidal edge handling**: When computing square step at the boundary, wrap-around gives seamless tileable terrain. Non-wrapped gives cliffs at edges.
- **Roughness vs. random scale**: Decrease `h` by multiplying by `roughness^(1/2)` at each level (not `roughness`), because you're halving the step.
- **Hydraulic erosion**: Add erosion pass after generation: water flows downhill carrying sediment, deposits at flat areas. Softens the terrain realistically.
- **Color by height bands**: snow (peaks), rock (mid-high), grass (mid), beach (low), water (below sea level).

## From the Source

**Algorithm:** Diamond-Square algorithm combined with thermal weathering erosion — a 2D analogue of midpoint displacement. Diamond step: centre of each square = mean of 4 corners + random(amplitude). Square step: edge midpoints = mean of 2 opposing corners + mean of 2 adjacent diamonds + random(amplitude). Amplitude halved each iteration → fractal terrain.

**Math:** Diamond-Square produces 1/f^(2H) power spectrum where H is the Hurst exponent. With ROUGHNESS=0.60 → H≈0.5 (standard Brownian surface, "white" terrain). Thermal erosion rule: if slope > TALUS (0.022), move `EROSION_RATE × (slope − TALUS)` material downhill per pass. This rounds peaks and fills valleys, mimicking real geological weathering. Grid size: GRID_N=6 → 65×65 (2^6+1) heightmap. Contour lines use marching squares on the interior.

**Performance:** EROSION_RATE=0.0012, ERODE_PASSES=2 per tick at 60 fps gives steady, visible erosion over minutes of runtime.

### Key Constants
| Name | Role |
|------|------|
| N | grid size = 2^k (e.g. k=8 → 257×257) |
| ROUGHNESS | 0.4–0.9, controls H exponent |
| SEA_LEVEL | height threshold for water |
| EROSION_ITER | hydraulic erosion passes |

### Open Questions
- How does the Hurst exponent relate to the fractal dimension of the terrain?
- Add thermal erosion: material slides from steep to flat. How does it change the look?
- Fly over the terrain using a camera moving across the heightmap (parallax scrolling).

---

## Pass 2 — Implementation

### Pseudocode
```
diamond_square(grid, N):
    h = 1.0   # initial random range
    step = N
    while step > 1:
        half = step/2

        # Diamond step: center of each square
        for x in range(0,N,step):
            for y in range(0,N,step):
                avg = (grid[x][y] + grid[x+step][y]
                     + grid[x][y+step] + grid[x+step][y+step]) / 4
                grid[x+half][y+half] = avg + random(-h,h)

        # Square step: midpoint of each edge
        for x in range(0,N+1,half):
            for y in range((x/half+1)%2*half, N+1, step):
                sum, count = 0, 0
                for (dx,dy) in [(-half,0),(half,0),(0,-half),(0,half)]:
                    nx,ny = x+dx, y+dy
                    if 0<=nx<=N and 0<=ny<=N:
                        sum += grid[nx][ny]; count++
                grid[x][y] = sum/count + random(-h,h)

        h *= ROUGHNESS     # reduce displacement
        step = half
```

### Module Map
```
§1 config    — N (power of 2), ROUGHNESS, SEA_LEVEL
§2 generate  — diamond_square(), normalize to [0,1]
§3 erosion   — hydraulic_erode() (optional)
§4 draw      — height → terrain char + color band
§5 app       — main, keys (seed, roughness, sea level, regenerate)
```

### Data Flow
```
seed corners → diamond_square → heightmap[N+1][N+1]
→ normalize → (optional erosion) → height bands → ASCII render
```

