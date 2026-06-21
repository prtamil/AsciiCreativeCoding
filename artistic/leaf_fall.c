/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/* leaf_fall.c — an ASCII tree that grows itself, then drops its leaves as
 * Matrix-style digital rain, then regrows a slightly different tree, forever.
 *
 * Tree growth follows the procedural-plant idea (Prusinkiewicz & Lindenmayer,
 * "The Algorithmic Beauty of Plants", 1990); branches are drawn with Bresenham's
 * line algorithm; the falling leaves reuse the Matrix digital-rain look. */

#define _POSIX_C_SOURCE 200809L
#include <ncurses.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <signal.h>
#include <time.h>
#include <stdbool.h>
#include <stdint.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* ── §1 config — timings, sizes, and tunable knobs ── */
#define RENDER_FPS      30
#define RENDER_NS       (1000000000LL / RENDER_FPS)
#define FALL_NS         55000000LL    /* 55 ms per fall step (~18 fps) */
#define DISPLAY_NS      2500000000LL  /* 2.5 s static display */
#define RESET_NS        700000000LL   /* 0.7 s blank between trees */
#define DT_CAP_NS       100000000LL   /* clamp frame dt to 100 ms (spiral guard) */
#define TRAIL_LEN       7             /* length of green trail behind white head */
#define MAX_LEAVES      4096
#define GRID_ROWS       128
#define GRID_COLS       320
#define MAX_DEPTH       7
#define BSTACK_MAX      512
#define MAX_START_DELAY 80            /* ticks — stagger leaf fall start */

static const float TRUNK_H_MIN  = 0.45f;
static const float TRUNK_H_MAX  = 0.65f;
static const float BRANCH_SPREAD = 0.50f;

/* Foliage scatter (place_foliage): aspect-correct ellipse + edge-sparse density */
#define FOLIAGE_ASPECT          0.25f  /* dc² weight = (1/2)² → round on screen */
#define FOLIAGE_DENSITY_BASE    0.62f  /* keep-probability floor at patch centre */
#define FOLIAGE_DENSITY_FALLOFF 0.33f  /* extra sparseness toward the edge       */

/* Colour pair IDs */
#define CP_TRUNK  1
#define CP_BRANCH 2
#define CP_LEAF   3
#define CP_FALL_H 4   /* white head */
#define CP_FALL_G 5   /* green trail body */
#define CP_HUD    6   /* top: data readout */
#define CP_HINT   7   /* bottom: action keys */

/* ── §2 performance — read and sleep on the monotonic clock ── */
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

/* ── §3 simulation state — the data the simulation owns ── */

/* TreeGrid — the whole tree painted once into a fixed grid of cells, then held
 * still on screen.  We draw the branching tree ONE time into this grid and then
 * just copy cells to the screen each frame, so the costly growth happens once
 * rather than every frame.  Two parallel arrays (one char, one small int per
 * cell) keep memory tight with no padding.
 *
 *   ch[r][c]  the glyph drawn at that cell; 0 means the cell is empty.
 *   cp[r][c]  which colour pair to use AND what kind of cell it is
 *             (trunk, branch, or leaf).  The colour code doubles as a type tag:
 *             leaves are animated when they fall, trunk/branch cells never move,
 *             and a leaf is never allowed to overwrite a trunk or branch cell. */
typedef struct {
    char   ch[GRID_ROWS][GRID_COLS];
    int8_t cp[GRID_ROWS][GRID_COLS];
} TreeGrid;
static TreeGrid g_tree;

/* Leaf — one leaf that turns into a falling Matrix-rain streak: a bright white
 * head leaving a fading green tail behind it as it drops.
 *
 * We store almost nothing about the trail.  Only the head's current row is kept;
 * the tail is just redrawn each frame as the few rows directly above the head.
 * That keeps each leaf tiny, so the whole canopy fits in one fixed array with no
 * memory allocation.  Each leaf gets a random start delay and speed so the
 * canopy drops in scattered waves instead of all at once. */
typedef struct {
    int16_t orig_row, orig_col; /* where the leaf was born; also where its rain starts */
    char    ch;                 /* the glyph this leaf is drawn with                   */
    int16_t head_row;           /* the row the falling head is on right now            */
    bool    started;            /* true once start_delay is up and the leaf has begun falling */
    bool    done;               /* true once the head and its tail are fully off-screen */
    int16_t start_delay;        /* ticks to wait before this leaf starts falling       */
    int16_t fall_period;        /* ticks between downward steps: 1 fast, 2 medium, 3 slow */
    int16_t fall_sub;           /* counts up toward fall_period, then steps down a row  */
} Leaf;

/* Leaf pool + active screen extent + PRNG state. */
static Leaf  g_leaves[MAX_LEAVES];
static int   g_leaf_count;
static int   g_rows, g_cols;
static unsigned g_seed;

/* Fall sub-tick counter (advanced by fall_tick). */
static int g_fall_tick;

/* Phase — which part of the endless cycle we're in: show the finished tree,
 * drop its leaves, go blank briefly, then grow a fresh tree and repeat.  Each
 * phase ends on its own condition (a timer, or "all leaves gone"), which keeps
 * the main loop simple.  Transitions are decided in main():
 *   DISPLAY → FALLING  once the display timer runs out
 *   FALLING → RESET    once every leaf has fallen off-screen
 *   RESET   → DISPLAY  after the blank timer, growing a new tree. */
typedef enum { PHASE_DISPLAY, PHASE_FALLING, PHASE_RESET } Phase;
static Phase     g_phase;     /* which phase we're in right now             */
static long long g_phase_t;   /* clock timestamp when this phase started    */
static unsigned  g_cycle;     /* how many trees shown so far (also seeds variety) */

/* ── §4 logic — yes/no questions about the state, with no side effects ── */

static bool in_grid(int r, int c) {
    return r >= 0 && r < g_rows && r < GRID_ROWS &&
           c >= 0 && c < g_cols && c < GRID_COLS;
}

static bool all_done(void) {
    if (g_leaf_count == 0) return true;
    for (int i = 0; i < g_leaf_count; i++)
        if (!g_leaves[i].done) return false;
    return true;
}

/* ── §5 simulation — grow the tree, then advance the falling leaves ── */

/* Cheap repeatable random numbers: same seed always gives the same tree. */
static unsigned lcg(void)  { g_seed = g_seed * 1664525u + 1013904223u; return g_seed; }
static float    lcgf(void) { return (float)(lcg() & 0x7fffffffu) / (float)0x7fffffffu; }

static void grid_clear(void) {
    memset(g_tree.ch, 0, sizeof(g_tree.ch));
    memset(g_tree.cp,  0, sizeof(g_tree.cp));
    g_leaf_count = 0;
}

static void set_cell(int r, int c, char ch, int8_t cp) {
    if (in_grid(r, c)) { g_tree.ch[r][c] = ch; g_tree.cp[r][c] = cp; }
}

static void add_leaf(int r, int c, char ch) {
    if (!in_grid(r, c))                                          return;
    if (g_tree.cp[r][c] == CP_TRUNK || g_tree.cp[r][c] == CP_BRANCH)   return;
    if (g_tree.cp[r][c] == CP_LEAF)                                  return;
    if (g_leaf_count >= MAX_LEAVES)                              return;
    set_cell(r, c, ch, CP_LEAF);
    g_leaves[g_leaf_count++] = (Leaf){
        .orig_row    = (int16_t)r,
        .orig_col    = (int16_t)c,
        .ch          = ch,
        .head_row    = (int16_t)r,
        .started     = false,
        .done        = false,
        .start_delay = (int16_t)(lcg() % MAX_START_DELAY),
        .fall_period = (int16_t)(1 + (int)(lcg() % 3)),  /* random speed: 1 fast, 2 medium, 3 slow */
        .fall_sub    = 0,
    };
}

static const char k_lch[] = "*@&#%o~";

/* Scatter a clump of leaves around (r,c).  Terminal cells are taller than they
 * are wide, so the clump is stretched twice as wide as tall to look round. */
static void place_foliage(int r, int c, int rad) {
    if (rad < 1) rad = 1;
    int nlch = (int)(sizeof(k_lch) - 1);
    for (int dr = -rad; dr <= rad; dr++) {
        for (int dc = -(rad * 2); dc <= (rad * 2); dc++) {
            /* skip cells outside the round patch */
            float d = (float)(dr * dr) + FOLIAGE_ASPECT * (float)(dc * dc);
            if (d > (float)(rad * rad)) continue;
            /* thinner toward the edge, so the clump fades out instead of ending hard */
            float thresh = FOLIAGE_DENSITY_BASE
                         + FOLIAGE_DENSITY_FALLOFF * d / (float)(rad * rad + 1);
            if (lcgf() > thresh) {
                char ch = k_lch[lcg() % (unsigned)nlch];
                add_leaf(r + dr, c + dc, ch);
            }
        }
    }
}

/* Pick the glyph for a branch line based on which way it runs. */
static char branch_glyph(int dr, int dc, int sr, int sc) {
    return (dc == 0) ? '|' :
           (dr == 0) ? '-' :
           (sr * sc > 0) ? '\\' : '/';
}

/* Draw a straight branch from one cell to another using Bresenham's line. */
static void draw_branch_line(int r0, int c0, int r1, int c1, int8_t cp) {
    int dr = abs(r1 - r0), dc = abs(c1 - c0);
    int sr = (r1 > r0) ? 1 : -1, sc = (c1 > c0) ? 1 : -1;
    int err = dc - dr;
    int r = r0, c = c0;
    char ch = branch_glyph(dr, dc, sr, sc);
    for (;;) {
        /* leave the thick trunk alone; thin branches shouldn't paint over it */
        if (in_grid(r, c) && g_tree.cp[r][c] != CP_TRUNK) {
            g_tree.ch[r][c] = ch;
            g_tree.cp[r][c]  = cp;
        }
        if (r == r1 && c == c1) break;
        int e2 = 2 * err;
        if (e2 > -dr) { err -= dr; c += sc; }
        if (e2 <  dc) { err += dc; r += sr; }
    }
}

/* Branch — one branch segment still waiting to be drawn.  Instead of calling
 * itself recursively (which could overflow on a deep tree), the grower keeps a
 * to-do list of branches in a fixed array and works through it in a loop.  When
 * the list is full it simply stops, so it can never run out of memory.
 *
 *   r, c    the cell where this branch segment starts.
 *   angle   the direction it points, in radians (straight up is M_PI/2, because
 *           screen rows count downward).
 *   len     how long the segment is, in cells.
 *   depth   how deep into the tree we are: low depth means thick trunk, deeper
 *           means thin branch, and the deepest branches turn into leaf clumps. */
typedef struct { int r, c; float angle; int len; int depth; } Branch;
static Branch g_bstack[BSTACK_MAX];

/* Draw the trunk: a two-cell column, a wider flare near the base, and little roots. */
static void draw_trunk(int base_row, int base_col, int trunk_h, int trunk_top) {
    for (int row = base_row; row >= trunk_top; row--) {
        set_cell(row, base_col,     '|', CP_TRUNK);
        set_cell(row, base_col + 1, '|', CP_TRUNK);
    }
    /* widen the bottom quarter of the trunk so the base looks heavier */
    int flare = trunk_h / 4;
    for (int row = base_row; row >= base_row - flare; row--) {
        set_cell(row, base_col - 1, '|', CP_TRUNK);
        set_cell(row, base_col + 2, '|', CP_TRUNK);
    }
    /* roots splaying out at the very bottom */
    set_cell(base_row, base_col - 2, '/', CP_TRUNK);
    set_cell(base_row, base_col + 3, '\\', CP_TRUNK);
}

/* Add the first big branches: 3-5 left/right pairs going up the trunk, longest
 * around the middle.  Returns the updated stack position. */
static int push_trunk_branches(int base_row, int base_col, int trunk_h, int bsp) {
    int n_pts = 3 + (int)(lcgf() * 3.0f);   /* between 3 and 5 */
    for (int i = 0; i < n_pts && bsp < BSTACK_MAX - 2; i++) {
        float hf     = (float)(i + 1) / (float)(n_pts + 1);
        int   br     = base_row - (int)(hf * (float)trunk_h * 0.95f);
        float spread = BRANCH_SPREAD + lcgf() * 0.35f;
        /* longest in the middle of the trunk, shorter near top and bottom */
        float len_f  = 0.25f + 0.35f * (1.0f - fabsf(hf - 0.5f) * 2.0f) + lcgf() * 0.10f;
        int   blen   = (int)(len_f * (float)trunk_h);
        if (blen < 3) blen = 3;
        g_bstack[bsp++] = (Branch){ br, base_col,     (float)M_PI/2.0f + spread, blen, 1 };
        g_bstack[bsp++] = (Branch){ br, base_col + 1, (float)M_PI/2.0f - spread, blen, 1 };
    }
    return bsp;
}

/* Add the crown at the top: two branches spreading out plus one shooting straight up. */
static int push_crown_branches(int base_col, int trunk_top, int trunk_h, int bsp) {
    if (bsp < BSTACK_MAX - 3) {
        int   top_len = trunk_h / 3;
        if (top_len < 3) top_len = 3;
        float ts = 0.55f + lcgf() * 0.30f;
        g_bstack[bsp++] = (Branch){ trunk_top, base_col,     (float)M_PI/2.0f + ts,   top_len,          1 };
        g_bstack[bsp++] = (Branch){ trunk_top, base_col + 1, (float)M_PI/2.0f - ts,   top_len,          1 };
        g_bstack[bsp++] = (Branch){ trunk_top, base_col,     (float)M_PI/2.0f,         (int)(top_len*0.8f), 1 };
    }
    return bsp;
}

/* Where a branch ends, given its start, direction, and length. */
static void branch_endpoint(const Branch *t, int *er, int *ec) {
    *er = t->r - (int)(sinf(t->angle) * (float)t->len);
    *ec = t->c + (int)(cosf(t->angle) * (float)t->len);
}

/* Handle one branch: if it's a tip, top it with leaves; otherwise draw it and
 * add a couple of smaller child branches to the to-do list. */
static void grow_branch(Branch t, int *bsp) {
    if (t.len <= 1 || t.depth > MAX_DEPTH) {
        int frad = 2 + (MAX_DEPTH - t.depth) / 2;
        if (frad < 1) frad = 1;
        if (frad > 5) frad = 5;
        place_foliage(t.r, t.c, frad);
        return;
    }

    int er, ec;
    branch_endpoint(&t, &er, &ec);

    int8_t cp = (t.depth <= 2) ? CP_TRUNK : CP_BRANCH;
    draw_branch_line(t.r, t.c, er, ec, cp);

    /* sprinkle a few leaves along the deeper branches, not just at the tips */
    if (t.depth >= 4 && lcgf() < 0.40f)
        place_foliage(er, ec, 1);

    if (t.depth >= MAX_DEPTH) {
        place_foliage(er, ec, 3);
        return;
    }

    float spread = 0.30f + lcgf() * 0.30f;
    int   slen   = (int)((float)t.len * (0.50f + lcgf() * 0.20f));
    if (slen < 2) slen = 2;

    if (*bsp < BSTACK_MAX - 3) {
        g_bstack[(*bsp)++] = (Branch){ er, ec, t.angle + spread, slen, t.depth + 1 };
        g_bstack[(*bsp)++] = (Branch){ er, ec, t.angle - spread, slen, t.depth + 1 };
        /* now and then add a third branch carrying roughly straight on */
        if (lcgf() < 0.35f && *bsp < BSTACK_MAX - 1) {
            float jitter = (lcgf() - 0.5f) * 0.20f;
            g_bstack[(*bsp)++] = (Branch){ er, ec, t.angle + jitter,
                                            (int)((float)slen * 0.80f), t.depth + 1 };
        }
    }
}

/* Grow one whole tree into the grid: draw the trunk, then work through every
 * branch on the to-do list until none are left. */
static void grow_tree(int base_row, int base_col) {
    /* random trunk height, kept inside the screen */
    int trunk_h = (int)((TRUNK_H_MIN + lcgf() * (TRUNK_H_MAX - TRUNK_H_MIN)) * (float)g_rows);
    if (trunk_h < 8) trunk_h = 8;
    int trunk_top = base_row - trunk_h;
    if (trunk_top < 2) { trunk_top = 2; trunk_h = base_row - trunk_top; }

    draw_trunk(base_row, base_col, trunk_h, trunk_top);

    int bsp = 0;
    bsp = push_trunk_branches(base_row, base_col, trunk_h, bsp);
    bsp = push_crown_branches(base_col, trunk_top, trunk_h, bsp);

    while (bsp > 0) {
        Branch t = g_bstack[--bsp];
        grow_branch(t, &bsp);
    }
}

/* Advance one leaf by a tick: wait out its delay, then drop one row every few
 * ticks, and mark it done once it (and its tail) have left the screen. */
static void leaf_step(Leaf *lf) {
    if (lf->done) return;
    if (!lf->started) {
        if (g_fall_tick >= lf->start_delay) lf->started = true;
        else return;
    }
    if (++lf->fall_sub >= lf->fall_period) {
        lf->fall_sub = 0;
        lf->head_row++;
        if (lf->head_row > (int16_t)(g_rows + TRAIL_LEN)) lf->done = true;
    }
}

static void fall_tick(void) {
    g_fall_tick++;
    for (int i = 0; i < g_leaf_count; i++)
        leaf_step(&g_leaves[i]);
}

static void scene_new_tree(void) {
    /* fresh seed each cycle so every tree comes out a little different */
    g_seed = (unsigned)clock_ns() ^ (g_cycle * 0x9e3779b9u);
    g_cycle++;
    grid_clear();
    g_fall_tick = 0;
    grow_tree(g_rows - 2, g_cols / 2 - 1);
    g_phase   = PHASE_DISPLAY;
    g_phase_t = clock_ns();
}

static void scene_resize(int rows, int cols) {
    g_rows = (rows > 4 && rows < GRID_ROWS) ? rows : GRID_ROWS - 1;
    g_cols = (cols > 8 && cols < GRID_COLS) ? cols : GRID_COLS - 1;
    scene_new_tree();
}

/* ── §6 render — draw the current state to the terminal ── */
static void color_init(void) {
    start_color();
    use_default_colors();
    if (COLORS >= 256) {
        init_pair(CP_TRUNK,  130, -1);  /* brown */
        init_pair(CP_BRANCH, 94,  -1);  /* dark brown */
        init_pair(CP_LEAF,   82,  -1);  /* bright green */
        init_pair(CP_FALL_H, 231, -1);  /* white */
        init_pair(CP_FALL_G, 46,  -1);  /* vivid green */
        init_pair(CP_HUD,    226, -1);  /* bright yellow */
        init_pair(CP_HINT,    51, -1);  /* bright cyan */
    } else {
        init_pair(CP_TRUNK,  COLOR_YELLOW, -1);
        init_pair(CP_BRANCH, COLOR_YELLOW, -1);
        init_pair(CP_LEAF,   COLOR_GREEN,  -1);
        init_pair(CP_FALL_H, COLOR_WHITE,  -1);
        init_pair(CP_FALL_G, COLOR_GREEN,  -1);
        init_pair(CP_HUD,    COLOR_YELLOW, -1);
        init_pair(CP_HINT,   COLOR_CYAN,   -1);
    }
}

/* Copy the still trunk and branch cells to the screen; leaves are drawn later. */
static void draw_tree_cells(void) {
    for (int r = 0; r < g_rows && r < GRID_ROWS; r++) {
        for (int c = 0; c < g_cols && c < GRID_COLS; c++) {
            char   ch = g_tree.ch[r][c];
            int8_t cp = g_tree.cp[r][c];
            if (!ch || cp == CP_LEAF) continue;   /* skip empty cells and leaves */
            attron(COLOR_PAIR(cp) | A_BOLD);
            mvaddch(r, c, (chtype)(unsigned char)ch);
            attroff(COLOR_PAIR(cp) | A_BOLD);
        }
    }
}

/* Draw one leaf: a still green glyph when at rest, or a falling streak with a
 * white head and a green tail when it's dropping. */
static void draw_leaf(const Leaf *lf, bool falling) {
    if (!falling || !lf->started) {
        /* still green leaf, drawn where it grew */
        attron(COLOR_PAIR(CP_LEAF) | A_BOLD);
        mvaddch(lf->orig_row, lf->orig_col, (chtype)(unsigned char)lf->ch);
        attroff(COLOR_PAIR(CP_LEAF) | A_BOLD);
        return;
    }
    if (lf->done) return;   /* already off-screen; erase() handled the cleanup */

    /* draw the falling streak from tail to head; stop the tail at the leaf's
     * birthplace so the rain seems to start there, not in midair above it */
    for (int j = TRAIL_LEN; j >= 0; j--) {
        int r = lf->head_row - j;
        if (r < lf->orig_row || r < 0 || r >= g_rows) continue;
        if (j == 0) {
            attron(COLOR_PAIR(CP_FALL_H) | A_BOLD);
            mvaddch(r, lf->orig_col, (chtype)(unsigned char)lf->ch);
            attroff(COLOR_PAIR(CP_FALL_H) | A_BOLD);
        } else {
            attron(COLOR_PAIR(CP_FALL_G));
            mvaddch(r, lf->orig_col, (chtype)(unsigned char)lf->ch);
            attroff(COLOR_PAIR(CP_FALL_G));
        }
    }
}

static void scene_draw(void) {
    bool falling = (g_phase == PHASE_FALLING);
    draw_tree_cells();
    for (int i = 0; i < g_leaf_count; i++)
        draw_leaf(&g_leaves[i], falling);
}

static void screen_init(void) {
    initscr();
    cbreak(); noecho();
    keypad(stdscr, TRUE);
    nodelay(stdscr, TRUE);
    curs_set(0);
    typeahead(-1);
    color_init();
}
static void screen_present(void) { wnoutrefresh(stdscr); doupdate(); }

/* On-screen status: phase and counters top-right, key hints bottom-left. */
static void draw_hud(int rows, int cols) {
    const char *phase_name = (g_phase == PHASE_DISPLAY) ? "display" :
                             (g_phase == PHASE_FALLING) ? "falling" : "reset";
    char hud[80];
    snprintf(hud, sizeof hud, " %s  cycle:%u  leaves:%d ",
             phase_name, g_cycle, g_leaf_count);
    int hud_x = cols - (int)strlen(hud);
    if (hud_x < 0) hud_x = 0;
    attron(COLOR_PAIR(CP_HUD) | A_BOLD);
    mvprintw(0, hud_x, "%s", hud);
    attroff(COLOR_PAIR(CP_HUD) | A_BOLD);

    attron(COLOR_PAIR(CP_HINT) | A_BOLD);
    mvprintw(rows - 1, 0, " q:quit  r:new tree  spc:skip ");
    attroff(COLOR_PAIR(CP_HINT) | A_BOLD);
}

/* ── §7 app — setup, input, the main loop, and shutdown ── */
static volatile sig_atomic_t g_quit   = 0;
static volatile sig_atomic_t g_resize = 0;

static void sig_h(int s) {
    if (s == SIGINT  || s == SIGTERM) g_quit   = 1;
    if (s == SIGWINCH)                g_resize = 1;
}
static void cleanup(void) { endwin(); }

int main(void) {
    atexit(cleanup);
    signal(SIGINT,   sig_h);
    signal(SIGTERM,  sig_h);
    signal(SIGWINCH, sig_h);

    screen_init();

    int rows, cols;
    getmaxyx(stdscr, rows, cols);
    scene_resize(rows, cols);

    long long fall_acc = 0;
    long long last_ns  = clock_ns();

    while (!g_quit) {
        /* terminal was resized: grow a fresh tree to fit the new size */
        if (g_resize) {
            g_resize = 0;
            getmaxyx(stdscr, rows, cols);
            scene_resize(rows, cols);
            last_ns  = clock_ns();
            fall_acc = 0;
            continue;
        }

        /* keys: quit, grow a new tree, or skip ahead to the next phase */
        int ch = getch();
        if (ch == 'q' || ch == 27) break;
        if (ch == 'r') {
            scene_new_tree(); last_ns = clock_ns(); fall_acc = 0;
        }
        if (ch == ' ') {
            if (g_phase == PHASE_DISPLAY) {
                g_phase = PHASE_FALLING; g_phase_t = clock_ns();
            } else if (g_phase == PHASE_FALLING) {
                scene_new_tree(); last_ns = clock_ns(); fall_acc = 0;
            }
        }

        /* time since last frame; capped so a long stall can't make the sim lurch */
        long long now_ns = clock_ns();
        long long dt     = now_ns - last_ns;
        last_ns = now_ns;
        if (dt > DT_CAP_NS) dt = DT_CAP_NS;

        long long elapsed = now_ns - g_phase_t;

        /* move between phases when their timer runs out or all leaves have fallen */
        if (g_phase == PHASE_DISPLAY && elapsed >= DISPLAY_NS) {
            g_phase = PHASE_FALLING; g_phase_t = now_ns;
        }
        if (g_phase == PHASE_FALLING && all_done()) {
            g_phase = PHASE_RESET; g_phase_t = now_ns;
        }
        if (g_phase == PHASE_RESET && elapsed >= RESET_NS) {
            scene_new_tree(); last_ns = clock_ns(); fall_acc = 0;
            continue;
        }

        /* step the falling leaves at their own steady rate, separate from the frame rate */
        if (g_phase == PHASE_FALLING) {
            fall_acc += dt;
            while (fall_acc >= FALL_NS) { fall_tick(); fall_acc -= FALL_NS; }
        }

        /* draw the scene, then the status text on top */
        erase();
        if (g_phase != PHASE_RESET) scene_draw();
        draw_hud(rows, cols);
        screen_present();

        /* sleep off the rest of the frame to hold a steady frame rate */
        clock_sleep_ns(RENDER_NS - (clock_ns() - now_ns));
    }

    return 0;
}
