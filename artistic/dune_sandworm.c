/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/* dune_sandworm.c — Arrakis sandworm: swim underground, breach, open mouth, dive
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra artistic/dune_sandworm.c -o dune_sandworm -lncurses -lm
 *
 * Keys: q quit | p pause | r reset | +/- speed | Space trigger breach
 */

/* ── CONCEPTS ─────────────────────────────────────────────────────────── *
 *
 * Algorithm      : Segmented worm body using a chain-of-circles (Conga)
 *                  follower model.  Each segment follows the previous:
 *                    if dist(seg[i], seg[i-1]) > SEG_LEN, move seg[i]
 *                    toward seg[i-1] until separation == SEG_LEN.
 *                  The head drives; the body follows with O(N_SEGS) updates.
 *
 * Math           : Underground path: sinusoidal oscillation
 *                    y = surface_row + SWIM_DEPTH + SWIM_AMP·sin(SWIM_FREQ·x)
 *                  Breach arc: parabolic trajectory above the surface —
 *                    apex at BREACH_HEIGHT rows, spanning BREACH_SPAN cols.
 *                  Terrain surface: procedural height map built from a sum
 *                  of low-frequency cosine waves to mimic sand dunes.
 *
 * Rendering      : Body segments drawn with direction-dependent ASCII chars
 *                  (─ │ ╱ ╲ for horizontal/vertical/diagonal).  Head has
 *                  a multi-row open-mouth sprite during breach.  Sand ripples
 *                  expand radially at RIPPLE_SPEED cols/s after each breach.
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── §1 config ───────────────────────────────────────────────────────────── */
#define _POSIX_C_SOURCE 200809L
#include <ncurses.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <signal.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define TICK_NS        33333333L   /* 30 Hz */

/* body chain */
#define N_SEGS         50          /* number of body segments */
#define SEG_LEN        1.8f        /* cells between consecutive segments */

/* worms */
#define MAX_WORMS      8          /* hard upper limit */
#define DEFAULT_WORMS  2

/* swimming */
#define SWIM_DEPTH     9.f         /* rows below terrain surface */
#define SWIM_AMP       2.8f        /* sinusoidal vertical swing */
#define SWIM_FREQ      0.06f       /* radians per col advanced */
#define WORM_SPEED0    14.f        /* default cols/sec */
#define SWIM_TIME_MIN  3.5f        /* seconds between breaches */
#define SWIM_TIME_MAX  7.f

/* breach arc */
#define BREACH_HEIGHT  13.f        /* rows above surface at apex */
#define BREACH_SPAN    58.f        /* horizontal cols for one full arc */

/* ripples */
#define MAX_RIPPLES    8
#define RIPPLE_SPEED   9.f         /* cols/sec */
#define RIPPLE_LIFE    1.8f

/* sand spray */
#define MAX_SPRAY      36

#define MAX_COLS 512

/* ── §2 clock ────────────────────────────────────────────────────────────── */
static long long clock_ns(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (long long)ts.tv_sec * 1000000000LL + ts.tv_nsec;
}
static void clock_sleep_ns(long long ns) {
    if (ns <= 0) return;
    struct timespec ts = { ns / 1000000000LL, ns % 1000000000LL };
    nanosleep(&ts, NULL);
}

/* ── §3 color & themes ───────────────────────────────────────────────────── */
enum {
    CP_STAR = 1,
    CP_GROUND, CP_SAND,
    CP_WORM_TOP, CP_WORM_SUB, CP_WORM_HEAD, CP_MOUTH,
    CP_RIPPLE, CP_SPRAY,
    CP_HUD
};

typedef struct {
    const char *name;
    /* 256-color foreground indices for each pair (background = -1) */
    int star, ground, sand, worm_top, worm_sub, worm_head, mouth, ripple, spray;
} Theme;

#define N_THEMES 10
static const Theme k_themes[N_THEMES] = {
/*          name        star  gnd  sand  top  sub  head  mth  rpl  spr */
/* 0 */ { "Desert",    250, 136,  94,  214, 130,  226, 196, 220, 230 },
/* 1 */ { "Midnight",  147,  67,  17,   51,  23,  123, 201, 111, 159 },
/* 2 */ { "Crimson",   253, 160,  88,  202, 124,  220, 231, 167, 210 },
/* 3 */ { "Arctic",    255, 195, 153,   51,  23,  123, 200, 159, 231 },
/* 4 */ { "Toxic",     248, 148,  22,  118,  28,  154, 196,  82, 155 },
/* 5 */ { "Volcanic",  240,  88,  52,  202, 124,  226, 231, 166, 208 },
/* 6 */ { "Cosmic",    141,  93,  54,  177,  55,  207, 196, 141, 183 },
/* 7 */ { "Sunset",    223, 209, 130,  208,  94,  226, 160, 215, 229 },
/* 8 */ { "Neon",      201,  46,  22,  201,  53,  226, 196,  51, 231 },
/* 9 */ { "Ghost",     252, 240, 235,  255, 244,  231, 238, 248, 253 },
};

static int g_theme = 0;

static void theme_apply(int t) {
    const Theme *th = &k_themes[t];
    init_pair(CP_STAR,      th->star,     -1);
    init_pair(CP_GROUND,    th->ground,   -1);
    init_pair(CP_SAND,      th->sand,     -1);
    init_pair(CP_WORM_TOP,  th->worm_top, -1);
    init_pair(CP_WORM_SUB,  th->worm_sub, -1);
    init_pair(CP_WORM_HEAD, th->worm_head,-1);
    init_pair(CP_MOUTH,     th->mouth,    -1);
    init_pair(CP_RIPPLE,    th->ripple,   -1);
    init_pair(CP_SPRAY,     th->spray,    -1);
}

static void color_init(void) {
    start_color();
    use_default_colors();
    if (COLORS >= 256) {
        init_pair(CP_HUD, 232, 250);  /* dark text on silver bar */
        theme_apply(g_theme);
    } else {
        init_pair(CP_HUD, COLOR_BLACK, COLOR_WHITE);
        init_pair(CP_STAR,      COLOR_WHITE,  COLOR_BLACK);
        init_pair(CP_GROUND,    COLOR_YELLOW, COLOR_BLACK);
        init_pair(CP_SAND,      COLOR_YELLOW, COLOR_BLACK);
        init_pair(CP_WORM_TOP,  COLOR_YELLOW, COLOR_BLACK);
        init_pair(CP_WORM_SUB,  COLOR_RED,    COLOR_BLACK);
        init_pair(CP_WORM_HEAD, COLOR_WHITE,  COLOR_BLACK);
        init_pair(CP_MOUTH,     COLOR_RED,    COLOR_BLACK);
        init_pair(CP_RIPPLE,    COLOR_YELLOW, COLOR_BLACK);
        init_pair(CP_SPRAY,     COLOR_WHITE,  COLOR_BLACK);
    }
}

/* ── §4 terrain ──────────────────────────────────────────────────────────── */
static int g_terrain[MAX_COLS];
static int g_surface_row;          /* approximate center surface row */

static void terrain_init(int cols, int rows) {
    /* surface sits at mid-screen */
    g_surface_row = rows / 2;
    for (int c = 0; c < cols; c++) {
        float x = (float)c / (float)(cols > 1 ? cols - 1 : 1);
        float h = sinf(x * (float)M_PI * 2.1f)          * 1.5f
                + sinf(x * (float)M_PI * 5.3f  + 0.9f)  * 0.8f
                + sinf(x * (float)M_PI * 11.7f + 0.4f)  * 0.4f;
        int gr = g_surface_row + (int)(h + 0.5f);
        if (gr < g_surface_row - 3) gr = g_surface_row - 3;
        if (gr > g_surface_row + 3) gr = g_surface_row + 3;
        g_terrain[c] = gr;
    }
}

static int terrain_at(float x, int cols) {
    int c = (int)(x + 0.5f);
    if (c < 0)    c = 0;
    if (c >= cols) c = cols - 1;
    return g_terrain[c];
}

/* ── §5 ripples ──────────────────────────────────────────────────────────── */
typedef struct {
    float ox, radius, life;
    int   active;
} Ripple;

static Ripple g_ripples[MAX_RIPPLES];

static void ripple_spawn(float ox) {
    for (int i = 0; i < MAX_RIPPLES; i++) {
        if (g_ripples[i].active) continue;
        g_ripples[i] = (Ripple){ ox, 0.f, RIPPLE_LIFE, 1 };
        return;
    }
}

static void ripple_update(float dt) {
    for (int i = 0; i < MAX_RIPPLES; i++) {
        if (!g_ripples[i].active) continue;
        g_ripples[i].life   -= dt;
        g_ripples[i].radius += RIPPLE_SPEED * dt;
        if (g_ripples[i].life <= 0.f) g_ripples[i].active = 0;
    }
}

/* ── §6 spray ────────────────────────────────────────────────────────────── */
typedef struct {
    float x, y, vx, vy, life, max_life;
    int   active;
} Spray;

static Spray g_spray[MAX_SPRAY];

static void spray_one(float x, float y) {
    for (int i = 0; i < MAX_SPRAY; i++) {
        if (g_spray[i].active) continue;
        /* upward hemisphere: angle 0..π → left/up/right burst */
        float ang = (float)(rand() % 180) * (float)M_PI / 180.f;
        float spd = 3.f + (float)(rand() % 700) * 0.01f;
        g_spray[i].active   = 1;
        g_spray[i].x        = x + (float)(rand() % 7 - 3) * 0.5f;
        g_spray[i].y        = y;
        g_spray[i].vx       =  cosf(ang) * spd;
        g_spray[i].vy       = -sinf(ang) * spd;   /* negative = upward */
        g_spray[i].life     = 0.4f + (float)(rand() % 60) * 0.01f;
        g_spray[i].max_life = g_spray[i].life;
        return;
    }
}

static void spray_burst(float x, float y, int n) {
    for (int i = 0; i < n; i++) spray_one(x, y);
}

static void spray_update(float dt) {
    for (int i = 0; i < MAX_SPRAY; i++) {
        if (!g_spray[i].active) continue;
        g_spray[i].life -= dt;
        if (g_spray[i].life <= 0.f) { g_spray[i].active = 0; continue; }
        g_spray[i].vy += 9.f * dt;
        g_spray[i].x  += g_spray[i].vx * dt;
        g_spray[i].y  += g_spray[i].vy * dt;
    }
}

/* ── §7 worm ─────────────────────────────────────────────────────────────── */
typedef enum { WS_SWIM, WS_BREACH } WState;

typedef struct { float x, y; } Seg;

typedef struct {
    Seg    segs[N_SEGS];
    float  hx, hy;          /* head position (float cells) */
    float  dir;             /* +1 right / -1 left */
    float  speed;
    WState state;
    float  swim_timer;      /* seconds until next breach */
    float  swim_phase;      /* sinusoidal phase accumulator */
    float  breach_x0;       /* hx at start of breach */
    float  mouth_open;      /* 0=closed 1=fully open */
    float  ripple_timer;
    float  respawn_timer;   /* counts up when inactive */
    int    sprayed;         /* sand spray fired this breach? */
    int    active;
} Worm;

static Worm  g_worms[MAX_WORMS];
static float g_speed = WORM_SPEED0;

static void worm_reset(Worm *w, int cols, int rows, float dir) {
    float respawn = w->respawn_timer;  /* preserve across reset */
    memset(w, 0, sizeof *w);
    w->respawn_timer = respawn;
    w->active      = 1;
    w->dir         = dir;
    w->speed       = g_speed * (0.85f + (float)(rand() % 30) * 0.01f);
    w->state       = WS_SWIM;
    w->swim_timer  = SWIM_TIME_MIN
                   + (float)(rand() % (int)((SWIM_TIME_MAX - SWIM_TIME_MIN) * 100)) * 0.01f;
    w->swim_phase  = (float)(rand() % 628) * 0.01f;
    w->ripple_timer = 0.6f;

    /* start off-screen on the side matching dir */
    float start_x = (dir > 0.f)
                  ? -(float)(N_SEGS) * SEG_LEN
                  : (float)cols + (float)(N_SEGS) * SEG_LEN;
    float start_y = (float)(g_surface_row + (int)SWIM_DEPTH);

    for (int i = 0; i < N_SEGS; i++) {
        w->segs[i].x = start_x - dir * (float)i * SEG_LEN;
        w->segs[i].y = start_y;
    }
    w->hx = start_x;
    w->hy = start_y;
    (void)rows;
}

static void worm_update(Worm *w, float dt, int cols, int rows) {
    if (!w->active) return;

    /* advance head horizontally at constant speed */
    w->hx += w->dir * w->speed * dt;

    if (w->state == WS_SWIM) {
        float surf     = (float)terrain_at(w->hx, cols);
        float target_y = surf + SWIM_DEPTH + sinf(w->swim_phase) * SWIM_AMP;
        w->swim_phase += SWIM_FREQ * w->speed * dt;
        w->hy          = target_y;
        w->mouth_open  = 0.f;

        /* spawn ripples when near surface */
        float depth = w->hy - surf;
        if (depth < SWIM_DEPTH * 0.65f) {
            w->ripple_timer -= dt;
            if (w->ripple_timer <= 0.f) {
                ripple_spawn(w->hx);
                w->ripple_timer = 0.45f + (float)(rand() % 40) * 0.01f;
            }
        }

        /* countdown to breach */
        w->swim_timer -= dt;
        if (w->swim_timer <= 0.f) {
            w->state     = WS_BREACH;
            w->breach_x0 = w->hx;
            w->sprayed   = 0;
            /* big ripple cluster at the breach point */
            ripple_spawn(w->hx);
            ripple_spawn(w->hx);
            ripple_spawn(w->hx + w->dir * 3.f);
        }

    } else { /* WS_BREACH */
        /* parametric position along breach parabola: t = 0..1 */
        float t = (w->hx - w->breach_x0) / (w->dir * BREACH_SPAN);
        if (t < 0.f) t = 0.f;

        if (t >= 1.f) {
            /* completed arc — resume swimming */
            w->state      = WS_SWIM;
            w->swim_timer = SWIM_TIME_MIN
                          + (float)(rand() % (int)((SWIM_TIME_MAX-SWIM_TIME_MIN)*100)) * 0.01f;
            w->mouth_open = 0.f;
        } else {
            float surf  = (float)terrain_at(w->hx, cols);
            float above = BREACH_HEIGHT * 4.f * t * (1.f - t);  /* parabola peak at t=0.5 */
            w->hy       = surf - above;

            /* mouth opens near apex (t≈0.5) and closes away from it */
            float d_apex = fabsf(t - 0.5f);
            w->mouth_open = (d_apex < 0.22f) ? 1.f - d_apex / 0.22f : 0.f;

            /* one-time sand spray as head breaks the surface (t~0.05) */
            if (!w->sprayed && t > 0.02f && t < 0.13f) {
                spray_burst(w->hx, surf, 24);
                w->sprayed = 1;
            }
        }
    }

    /* retire worm once fully off the far side */
    float margin = (float)(N_SEGS) * SEG_LEN + 10.f;
    if ((w->dir > 0.f && w->hx > (float)cols + margin) ||
        (w->dir < 0.f && w->hx < -margin)) {
        w->active = 0;
        return;
    }

    /* chain constraint: each segment is pulled toward the segment ahead */
    w->segs[0].x = w->hx;
    w->segs[0].y = w->hy;
    for (int i = 1; i < N_SEGS; i++) {
        float dx   = w->segs[i-1].x - w->segs[i].x;
        float dy   = w->segs[i-1].y - w->segs[i].y;
        float dist = sqrtf(dx * dx + dy * dy);
        if (dist > SEG_LEN && dist > 0.001f) {
            float s = (dist - SEG_LEN) / dist;
            w->segs[i].x += dx * s;
            w->segs[i].y += dy * s;
        }
    }
    (void)rows;
}

/* ── §8 draw ─────────────────────────────────────────────────────────────── */
static void draw_stars(int cols, int rows) {
    (void)rows;
    int sky_bottom = g_surface_row - 4;
    attron(COLOR_PAIR(CP_STAR) | A_DIM);
    for (int r = 0; r < sky_bottom; r++) {
        for (int c = 0; c < cols; c++) {
            unsigned int h = (unsigned int)(c * 1234597u ^ r * 987659u);
            h ^= h >> 13; h *= 0x5bd1e995u; h ^= h >> 15;
            if ((h & 0xFF) < 6)
                mvaddch(r, c, (h >> 9 & 1) ? '*' : '.');
        }
    }
    attroff(COLOR_PAIR(CP_STAR) | A_DIM);
}

static void draw_terrain(int cols, int rows) {
    for (int c = 0; c < cols; c++) {
        int gr = g_terrain[c];
        if (gr < 0 || gr >= rows) continue;

        int   prev_gr = (c > 0)        ? g_terrain[c-1] : gr;
        int   next_gr = (c < cols - 1) ? g_terrain[c+1] : gr;
        int   slope   = next_gr - prev_gr;
        char  sch     = (slope < -1) ? '/' : (slope > 1) ? '\\' : '_';

        attron(COLOR_PAIR(CP_GROUND) | A_BOLD);
        mvaddch(gr, c, (chtype)(unsigned char)sch);
        attroff(COLOR_PAIR(CP_GROUND) | A_BOLD);

        attron(COLOR_PAIR(CP_SAND));
        for (int r = gr + 1; r < rows - 1; r++) {
            char fc = ((c * 3 + r * 7) % 13 < 2) ? '.' : ' ';
            mvaddch(r, c, (chtype)(unsigned char)fc);
        }
        attroff(COLOR_PAIR(CP_SAND));
    }
}

/* segment character: direction toward head gives slope → ring char */
static char seg_char(float dx, float dy, int idx) {
    float ax = fabsf(dx), ay = fabsf(dy);
    if (ay > ax * 1.73f)
        return (idx % 2 == 0) ? '|' : '!';
    if (ax > ay * 1.73f)
        return (idx % 3 == 0) ? 'O' : (idx % 3 == 1) ? 'o' : '0';
    return (dx * dy > 0.f) ? '\\' : '/';
}

static void draw_worm(Worm *w, int cols, int rows) {
    if (!w->active) return;

    /* body: draw tail-to-head so head writes last (on top) */
    for (int i = N_SEGS - 1; i >= 1; i--) {
        int sr = (int)(w->segs[i].y + 0.5f);
        int sc = (int)(w->segs[i].x + 0.5f);
        if (sr < 0 || sr >= rows || sc < 0 || sc >= cols) continue;

        /* direction of this segment toward head */
        float dx = w->segs[i-1].x - w->segs[i].x;
        float dy = w->segs[i-1].y - w->segs[i].y;
        char  ch = seg_char(dx, dy, i);

        /* last 8 segments taper as the tail */
        int is_tail = (i > N_SEGS - 8);

        int surf        = terrain_at(w->segs[i].x, cols);
        int above_ground = (sr < surf);

        if (above_ground) {
            attr_t at = is_tail ? A_DIM : A_BOLD;
            attron(COLOR_PAIR(CP_WORM_TOP) | at);
            mvaddch(sr, sc, (chtype)(unsigned char)ch);
            /* thicken: draw second row when segment is mostly horizontal */
            if (!is_tail && fabsf(dx) > fabsf(dy) * 1.3f && sr + 1 < surf && sr + 1 < rows)
                mvaddch(sr + 1, sc, (chtype)(unsigned char)ch);
            attroff(COLOR_PAIR(CP_WORM_TOP) | at);
        } else {
            /* underground: visible but dim through the sand */
            if (!is_tail) {
                attron(COLOR_PAIR(CP_WORM_SUB) | A_DIM);
                mvaddch(sr, sc, (chtype)(unsigned char)(i % 3 == 0 ? 'o' : '.'));
                attroff(COLOR_PAIR(CP_WORM_SUB) | A_DIM);
            }
        }
    }

    /* ── head ── */
    int hr = (int)(w->hy + 0.5f);
    int hc = (int)(w->hx + 0.5f);
    if (hc < 0 || hc >= cols || hr < 0 || hr >= rows) return;

    int  surf        = terrain_at(w->hx, cols);
    int  above_ground = (hr < surf);

    if (above_ground) {
        /* open_w: how many chars the mouth extends each side */
        int open_w = (int)(w->mouth_open * 3.5f);

        attron(COLOR_PAIR(CP_WORM_HEAD) | A_BOLD);

        /* top lip */
        if (w->mouth_open > 0.08f && hr - 1 >= 0) {
            for (int j = -open_w; j <= open_w; j++) {
                int mc = hc + j;
                if (mc < 0 || mc >= cols) continue;
                mvaddch(hr - 1, mc, (j == -open_w || j == open_w) ? '|' : '_');
            }
        }

        /* centre row — mouth cavity */
        if (open_w == 0) {
            mvaddch(hr, hc, '@');
        } else {
            int lo = hc - open_w - 1, hi = hc + open_w + 1;
            if (lo >= 0)    mvaddch(hr, lo, '(');
            if (hi < cols)  mvaddch(hr, hi, ')');
            attron(COLOR_PAIR(CP_MOUTH) | A_BOLD);
            for (int j = -open_w; j <= open_w; j++) {
                int mc = hc + j;
                if (mc < 0 || mc >= cols) continue;
                mvaddch(hr, mc, j == 0 ? '@' : ' ');
            }
            attroff(COLOR_PAIR(CP_MOUTH) | A_BOLD);
            attron(COLOR_PAIR(CP_WORM_HEAD) | A_BOLD);
        }

        /* bottom lip */
        if (w->mouth_open > 0.08f && hr + 1 < surf && hr + 1 < rows) {
            for (int j = -open_w; j <= open_w; j++) {
                int mc = hc + j;
                if (mc < 0 || mc >= cols) continue;
                mvaddch(hr + 1, mc, (j == -open_w || j == open_w) ? '|' : '-');
            }
        }

        attroff(COLOR_PAIR(CP_WORM_HEAD) | A_BOLD);

    } else {
        /* head underground: bold dot */
        attron(COLOR_PAIR(CP_WORM_SUB) | A_BOLD);
        mvaddch(hr, hc, 'O');
        attroff(COLOR_PAIR(CP_WORM_SUB) | A_BOLD);
    }
}

static void draw_ripples(int cols, int rows) {
    for (int i = 0; i < MAX_RIPPLES; i++) {
        Ripple *rp = &g_ripples[i];
        if (!rp->active) continue;
        float frac = rp->life / RIPPLE_LIFE;
        int   r    = (int)(rp->radius + 0.5f);

        for (int side = -1; side <= 1; side += 2) {
            /* outer ring */
            int c = (int)(rp->ox + (float)(side * r) + 0.5f);
            if (c >= 0 && c < cols) {
                int row = terrain_at((float)c, cols) - 1;
                if (row >= 0 && row < rows) {
                    char rch = (frac > 0.6f) ? '~' : (frac > 0.3f) ? '.' : ' ';
                    attron(COLOR_PAIR(CP_RIPPLE) | A_BOLD);
                    mvaddch(row, c, (chtype)(unsigned char)rch);
                    attroff(COLOR_PAIR(CP_RIPPLE) | A_BOLD);
                }
            }
            /* inner half-radius ring */
            int c2 = (int)(rp->ox + (float)(side * (r / 2.0f)) + 0.5f);
            if (c2 >= 0 && c2 < cols) {
                int row = terrain_at((float)c2, cols) - 1;
                if (row >= 0 && row < rows) {
                    attron(COLOR_PAIR(CP_RIPPLE));
                    mvaddch(row, c2, '.');
                    attroff(COLOR_PAIR(CP_RIPPLE));
                }
            }
        }
    }
}

static void draw_spray(int cols, int rows) {
    for (int i = 0; i < MAX_SPRAY; i++) {
        if (!g_spray[i].active) continue;
        int sr = (int)(g_spray[i].y + 0.5f);
        int sc = (int)(g_spray[i].x + 0.5f);
        if (sr < 0 || sr >= rows || sc < 0 || sc >= cols) continue;
        float frac = g_spray[i].life / g_spray[i].max_life;
        char  ch   = (frac > 0.6f) ? '*' : (frac > 0.3f) ? '+' : '.';
        int   pair = (frac > 0.6f) ? CP_SPRAY : CP_RIPPLE;
        attron(COLOR_PAIR(pair));
        mvaddch(sr, sc, (chtype)(unsigned char)ch);
        attroff(COLOR_PAIR(pair));
    }
}

static void draw_hud(int n_active, int n_total, int rows, int cols) {
    attron(COLOR_PAIR(CP_HUD));
    for (int c = 0; c < cols; c++) mvaddch(rows - 1, c, ' ');
    mvprintw(rows - 1, 1,
             "q:quit  p:pause  r:reset  +/-:speed(%.0f)  Space:breach  w/W:worms(%d/%d)  t:theme[%s]",
             g_speed, n_active, n_total, k_themes[g_theme].name);
    attroff(COLOR_PAIR(CP_HUD));
}

/* ── §9 scene ────────────────────────────────────────────────────────────── */
static volatile sig_atomic_t g_running     = 1;
static volatile sig_atomic_t g_need_resize = 0;
static void on_sigint(int s)   { (void)s; g_running = 0; }
static void on_sigwinch(int s) { (void)s; g_need_resize = 1; }

static int g_paused  = 0;
static int g_n_worms = DEFAULT_WORMS;  /* how many worms are managed */

static int count_active(void) {
    int n = 0;
    for (int i = 0; i < g_n_worms; i++) if (g_worms[i].active) n++;
    return n;
}

static void scene_reset(int cols, int rows) {
    memset(g_worms,   0, sizeof g_worms);
    memset(g_ripples, 0, sizeof g_ripples);
    memset(g_spray,   0, sizeof g_spray);
    terrain_init(cols, rows);
    /* spawn worms alternating sides, stagger their breach timers */
    for (int i = 0; i < g_n_worms; i++) {
        float dir = (i % 2 == 0) ? 1.f : -1.f;
        worm_reset(&g_worms[i], cols, rows, dir);
        g_worms[i].swim_timer += (float)i * 1.8f;   /* stagger entries */
    }
}

static void scene_tick(float dt, int cols, int rows) {
    for (int i = 0; i < g_n_worms; i++) {
        Worm *w = &g_worms[i];
        if (!w->active) {
            /* wait 2 s then respawn from a random side */
            w->respawn_timer += dt;
            if (w->respawn_timer >= 2.f) {
                float dir = (rand() % 2 == 0) ? 1.f : -1.f;
                worm_reset(w, cols, rows, dir);
            }
            continue;
        }
        worm_update(w, dt, cols, rows);
    }
    ripple_update(dt);
    spray_update(dt);
}

/* ── §10 main ────────────────────────────────────────────────────────────── */
int main(void) {
    srand((unsigned)time(NULL));
    signal(SIGINT,   on_sigint);
    signal(SIGTERM,  on_sigint);
    signal(SIGWINCH, on_sigwinch);

    initscr();
    noecho();
    cbreak();
    curs_set(0);
    keypad(stdscr, TRUE);
    nodelay(stdscr, TRUE);
    typeahead(-1);
    color_init();

    int rows = LINES, cols = COLS;
    scene_reset(cols, rows);

    long long prev = clock_ns();

    while (g_running) {
        int ch;
        while ((ch = getch()) != ERR) {
            switch (ch) {
            case 'q': case 'Q': case 27: g_running = 0; break;
            case 'p': case 'P': g_paused = !g_paused; break;
            case 'r': case 'R': scene_reset(cols, rows); break;
            case ' ':
                /* force the next swimming worm to breach now */
                for (int i = 0; i < MAX_WORMS; i++) {
                    Worm *w = &g_worms[i];
                    if (w->active && w->state == WS_SWIM) {
                        w->swim_timer = 0.f;
                        break;
                    }
                }
                break;
            case '+': case '=':
                g_speed = fminf(g_speed + 2.f, 40.f);
                for (int i = 0; i < MAX_WORMS; i++)
                    if (g_worms[i].active) g_worms[i].speed = g_speed;
                break;
            case '-':
                g_speed = fmaxf(g_speed - 2.f, 4.f);
                for (int i = 0; i < MAX_WORMS; i++)
                    if (g_worms[i].active) g_worms[i].speed = g_speed;
                break;
            case 'w': case 'W':
                /* w = add worm, W = remove worm */
                if (ch == 'w' || ch == 'W') {
                    if (ch == 'w' && g_n_worms < MAX_WORMS) {
                        int idx = g_n_worms++;
                        float dir = (rand() % 2 == 0) ? 1.f : -1.f;
                        memset(&g_worms[idx], 0, sizeof g_worms[idx]);
                        worm_reset(&g_worms[idx], cols, rows, dir);
                    } else if (ch == 'W' && g_n_worms > 1) {
                        g_n_worms--;
                        g_worms[g_n_worms].active = 0;
                    }
                }
                break;
            case 't': case 'T':
                if (COLORS >= 256) {
                    g_theme = (g_theme + 1) % N_THEMES;
                    theme_apply(g_theme);
                }
                break;
            case KEY_RESIZE: g_need_resize = 1; break;
            }
        }

        if (g_need_resize) {
            g_need_resize = 0;
            endwin(); refresh();
            rows = LINES; cols = COLS;
            scene_reset(cols, rows);
        }

        long long now = clock_ns();
        float dt = (float)(now - prev) * 1e-9f;
        if (dt > 0.1f) dt = 0.1f;
        prev = now;

        if (!g_paused) scene_tick(dt, cols, rows);

        erase();
        draw_stars(cols, rows);
        draw_terrain(cols, rows);
        draw_ripples(cols, rows);
        draw_spray(cols, rows);
        for (int i = 0; i < MAX_WORMS; i++) draw_worm(&g_worms[i], cols, rows);
        draw_hud(count_active(), g_n_worms, rows, cols);
        wnoutrefresh(stdscr);
        doupdate();

        clock_sleep_ns(TICK_NS - (clock_ns() - now));
    }

    endwin();
    return 0;
}
