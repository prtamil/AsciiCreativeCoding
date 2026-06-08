/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * dna.c — 10 DNA/RNA structure visualisations in 2D ASCII art
 *
 * DNA TYPES (n/p to cycle):
 *   0  B-DNA Double Helix   — classic Watson-Crick, right-handed
 *   1  A-DNA Double Helix   — compressed, wide-grooved form
 *   2  Z-DNA                — left-handed zigzag
 *   3  Triple Helix         — Hoogsteen third strand
 *   4  G-Quadruplex         — four-strand G4 telomere structure
 *   5  RNA Hairpin          — stem-loop fold
 *   6  Replication Fork     — Y-shape parent→daughter unwinding
 *   7  Cruciform            — cross-shaped inverted-repeat
 *   8  Plasmid              — closed circular bacterial DNA
 *   9  DNA Ladder           — educational unrolled base-pair ladder
 *
 * COLOR THEMES (t):
 *   0 BioLab  1 Neon  2 Ocean  3 Fire  4 Cosmic  5 Mono
 *   Each theme supplies 5 complementary colors:
 *   strand1, strand2, strand3, bond, label
 *
 * Keys:
 *   q / ESC   quit
 *   space     pause / resume
 *   n         next DNA type
 *   p         previous DNA type
 *   t         next color theme
 *   ] / [     fps up / down
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra artistic/dna.c -o dna -lncurses -lm
 *
 * Sections (single-concern layers — see ARCHITECTURE below):
 *   §1 config   §2 clock=PERFORMANCE   §3 color=RENDER-setup   §4 draw=RENDER
 *   §5 scene=STATE+events   §6 simulation   §7 render   §8 app=events+PERFORMANCE
 */

/* ── CONCEPTS ─────────────────────────────────────────────────────────── *
 *
 * Helix rendering:
 *   Backbone positions are parametric sinusoids scrolling in t:
 *     x1(row) = cx + A·cos(2π·row/P + t)
 *     x2(row) = cx − A·cos(2π·row/P + t)
 *   Depth  z  = sin(phase) determines front/back order.
 *   Front strand is drawn bold, back strand dim — gives 3-D illusion.
 *
 * CELL_AR = 0.5 aspect correction:
 *   Terminal cells are ~2× taller than wide.  Horizontal helix arm
 *   uses pitch_h = pitch_v / CELL_AR so both arms look the same scale.
 *   Plasmid uses ry = rx × CELL_AR so the ellipse appears circular.
 *
 * Theme system:
 *   Each of 6 themes supplies 5 xterm-256 indices (s1, s2, s3, bond, label).
 *   color_apply_theme() registers 5 ncurses pairs once at theme change.
 *   Falls back to 8 basic colors on terminals with COLORS < 256.
 *
 * References:
 *   Structural biology (the CONCEPTS — what each of the 10 forms is) —
 *   • Watson, J. D. & Crick, F. H. C. — "Molecular Structure of Nucleic
 *     Acids", *Nature* 171 (1953).  The B-DNA double helix, base pairing
 *     and right-handedness that scenes 0/6/9 visualise.
 *   • Wang, A. H. J. et al. — "Molecular structure of a left-handed double
 *     helical DNA fragment at atomic resolution", *Nature* 282 (1979).  The
 *     Z-DNA zigzag (scene 2); contrast with A-DNA geometry.
 *   • Sen, D. & Gilbert, W. — "Formation of parallel four-stranded
 *     complexes by guanine-rich motifs", *Nature* 334 (1988).  The
 *     G-quadruplex / G-tetrad with central K+ (scene 4).
 *   • Alberts et al. — *Molecular Biology of the Cell* (6th ed., 2014),
 *     ch. 4-5.  Textbook reference for the replication fork, Okazaki
 *     fragments, cruciforms, plasmids, triple/Hoogsteen pairing and RNA
 *     stem-loops (scenes 3,5,6,7,8).
 *   • Saenger, W. — *Principles of Nucleic Acid Structure* (Springer,
 *     1984).  Quantitative A/B/Z helix parameters: pitch, rise, groove
 *     width — the numbers behind each scene's `pitch` and `amp`.
 *
 *   Rendering (the parametric projection + terminal output) —
 *   • Foley, van Dam, Feiner & Hughes — *Computer Graphics: Principles and
 *     Practice* (2nd ed., 1990), ch. 5-6.  Parametric curves and the
 *     parallel projection of a 3-D helix to 2-D screen columns; the depth
 *     cue z = sin(phase) → front/back ordering is the painter's algorithm.
 *   • Bourke, P. — "Parametric equations of a helix / circle"
 *     (paulbourke.net).  The x = cx + A·cos(θ), z = sin(θ) backbone and the
 *     CELL_AR aspect correction (ry = rx·CELL_AR) used by every scene.
 *   • Strang, G. — *Introduction to Linear Algebra* (5th ed.), §8 on
 *     rotations.  The phase term `row·k·hand + t` is a rotation about the
 *     helix axis; `hand = ±1` flips chirality.
 *   • Raymond, E. S. & Ben-Halim, Z. — *Writing Programs with NCURSES*.
 *     attrset / COLOR_PAIR / A_BOLD / A_DIM (front=bold, back=dim) and the
 *     init_pair theme model in §3.
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── ARCHITECTURE (separated layers) ──────────────────────────────────── *
 *
 * This is a RENDER-dominated program: ten draw routines turn ONE scalar of
 * state (a phase clock) into pictures.  Almost everything is RENDER; the only
 * per-tick state advance is incrementing that clock.
 *
 *   LAYER        SECTION              MUTATES
 *   ─────        ───────              ───────
 *   PERFORMANCE  §2 clock, §8 app     loop timing / fps counter / target fps —
 *                                     never scene state
 *   RENDER       §3 color (setup),    the ncurses screen + colour pairs ONLY;
 *                §4 draw, §7 render   reads Scene (dna_type, theme, t, rows, cols),
 *                                     writes no state
 *   SIMULATION   §6 simulation        Scene.t — the phase clock; the ONLY
 *                                     per-tick state advance (scene_tick)
 *
 *   LOGIC   — no standalone layer.  The helix math (phase, x1/x2 backbone,
 *             depth z = sin(phase), the triple-strand depth sort) is pure, but
 *             it is computed INLINE inside the RENDER draws and never stored —
 *             nothing to isolate or corrupt.
 *   EFFECTS — none stored.  Pulses, front/back bold-vs-dim, Okazaki-fragment
 *             gaps and the ladder scroll are all DERIVED at draw time from
 *             Scene.t and the row index; no cosmetic state survives a frame.
 *   DELAYS  — trivial.  The only "timer" is the `paused` flag in main(): while
 *             set, scene_tick is skipped so Scene.t freezes (draws still run).
 *
 * PER-TICK COMBINE — main() (§8) is the ONE place the layers meet, in this
 * fixed per-frame order; nothing else advances Scene.t:
 *     (resize?) → drain input → scene_tick (SIM, unless paused)
 *               → erase + scene_draw + screen_hud + present (RENDER)
 *               → sleep to frame cap + sample fps (PERFORMANCE)
 *
 * USER EVENTS (NOT part of the tick) mutate state on input/resize only — the
 * main switch (§8) and scene_init/scene_resize (§5):
 *     n / p      Scene.dna_type (+ reset Scene.t = 0)
 *     t          Scene.theme (+ re-bind colour pairs via color_apply_theme, §3)
 *     space      paused
 *     ] / [      target fps (PERFORMANCE knob)
 *     SIGWINCH   scene_resize
 *
 * (The const-pointer read/mutate signature convention is deferred to step 5;
 *  here the roles are documented only.)
 *
 * ─────────────────────────────────────────────────────────────────────── */


#define _POSIX_C_SOURCE 200809L
#ifndef M_PI
#  define M_PI 3.14159265358979323846
#endif

#include <math.h>
#include <ncurses.h>
#include <signal.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

/* ===================================================================== */
/* §1  config                                                             */
/* ===================================================================== */

#define CELL_AR     0.5f   /* col_width / row_height terminal aspect ratio */
#define TAU         6.28318530f
#define RUNGS_PER_PITCH 5.f /* base-pair rungs per helical pitch → rung every
                             * pitch/RUNGS_PER_PITCH rows (see rung_interval)  */

/*
 * Compile-time sizes and bounds.  The N_* counts size the catalog tables AND
 * bound the cycle keys — n/p wrap mod N_DNA_TYPES, t wraps mod N_THEMES.
 * THEME_COLS is the palette width: the 5 colour roles per theme, matching the
 * CP_S1..CP_LB pairs.  HUD_ROWS rows are reserved for the HUD (row 0 + the
 * last row), so each scene draws into rows 1 .. rows-HUD_ROWS-1.  SIM_FPS_*
 * bound the [/] frame-rate cap (a render throttle, not simulation accuracy —
 * the phase clock advances by real dt regardless).
 */
enum {
    N_DNA_TYPES   = 10,    /* structures in the gallery; n/p cycle mod this   */
    N_THEMES      =  6,    /* colour themes; t cycles mod this                */
    THEME_COLS    =  5,    /* palette width: strand1, strand2, strand3, bond, label */
    HUD_ROWS      =  2,    /* rows reserved for HUD (top bar + bottom bar)    */

    SIM_FPS_MIN   =  5,    /* frame-cap bounds + step for the [ / ] keys      */
    SIM_FPS_DEF   = 30,
    SIM_FPS_MAX   = 60,
    SIM_FPS_STEP  =  5,
};

#define NS_PER_SEC  1000000000LL

/*
 * Theme — a named colour scheme: the five hues a structure is painted in, as a
 * 256-colour palette with an 8-colour fallback.  Indexed by Scene.theme;
 * color_apply_theme() binds the active palette to ncurses pairs CP_S1..CP_LB
 * (init_pair model — Raymond & Ben-Halim, *Writing Programs with NCURSES*).
 * WHY two palettes: the rich xterm-256 hues are chosen bright so strands read
 * over the default dark bg (front strand bold, back dim — see helix_draw); on a
 * <256-colour terminal the 8-colour row substitutes the nearest basic colour.
 * The five slots are, in order: strand1, strand2, strand3, bond, label.
 * (dna_names[] is gone — names now live on DnaStructure beside their renderer.)
 */
typedef struct {
    const char *name;                /* HUD label                                */
    short       pal256[THEME_COLS];  /* xterm-256 hues, used when COLORS >= 256   */
    short       pal8[THEME_COLS];    /* basic-8 fallback (bond/label often white) */
} Theme;

static const Theme themes[N_THEMES] = {
    /*  name        256: s1   s2   s3  bond  lbl     8-colour fallback (s1,s2,s3,bond,label)             */
    { "BioLab", {  46, 118,  51, 250, 243 }, { COLOR_GREEN,   COLOR_CYAN,   COLOR_GREEN,   COLOR_WHITE, COLOR_WHITE } },
    { "Neon",   { 201, 213, 226, 255, 245 }, { COLOR_MAGENTA, COLOR_YELLOW, COLOR_RED,     COLOR_WHITE, COLOR_WHITE } },
    { "Ocean",  {  27,  51, 159, 195, 240 }, { COLOR_BLUE,    COLOR_CYAN,   COLOR_WHITE,   COLOR_WHITE, COLOR_WHITE } },
    { "Fire",   { 196, 208, 226, 255, 243 }, { COLOR_RED,     COLOR_YELLOW, COLOR_MAGENTA, COLOR_WHITE, COLOR_WHITE } },
    { "Cosmic", {  93, 129, 171, 219, 240 }, { COLOR_MAGENTA, COLOR_BLUE,   COLOR_CYAN,    COLOR_WHITE, COLOR_WHITE } },
    { "Mono",   { 255, 231, 195, 244, 236 }, { COLOR_WHITE,   COLOR_WHITE,  COLOR_WHITE,   COLOR_WHITE, COLOR_WHITE } },
};

/* ===================================================================== */
/* §2  clock                       PERFORMANCE — monotonic time + sleep   */
/* ===================================================================== */
/* Timing primitives only.  Mutate nothing; read by the §8 main loop.     */

static int64_t clock_ns(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (int64_t)ts.tv_sec * NS_PER_SEC + ts.tv_nsec;
}

static void clock_sleep_ns(int64_t ns)
{
    if (ns <= 0) return;
    struct timespec req = {
        .tv_sec  = (time_t)(ns / NS_PER_SEC),
        .tv_nsec = (long)(ns % NS_PER_SEC),
    };
    nanosleep(&req, NULL);
}

/* ===================================================================== */
/* §3  color                       RENDER setup — theme → colour pairs    */
/* ===================================================================== */
/* Mutates ncurses colour pairs only (boot / 't' key).  No scene state.    */

/*
 * ColorPair IDs — the ncurses colour-pair slots this program registers.  They
 * start at 1 because pair 0 is the terminal default and cannot be redefined
 * (Raymond & Ben-Halim, init_pair).  CP_S1..CP_LB are the 5 themed roles, in
 * the same order as a Theme's palette (THEME_COLS); color_apply_theme rebinds
 * them on every theme change.  CP_HUD is fixed (never themed) so the HUD stays
 * legible under all themes.
 *   S1/S2/S3 = the three strands   BD = base-pair bond rung   LB = label/marker
 */
enum { CP_S1 = 1, CP_S2, CP_S3, CP_BD, CP_LB, CP_HUD };

static void color_apply_theme(const Theme *th)
{
    start_color();
    use_default_colors();
    bool hi = (COLORS >= 256);
    for (int i = 0; i < THEME_COLS; i++)
        init_pair(CP_S1 + i, hi ? th->pal256[i] : th->pal8[i], -1);
    init_pair(CP_HUD, hi ? 252 : COLOR_WHITE, -1);
}

/* ===================================================================== */
/* §4  draw                        RENDER — scene → screen (the 10 draws)  */
/* ===================================================================== */
/*
 * Reads scene parameters (cx, t, rows, cols) and writes the ncurses screen via
 * sput / mvaddch / mvprintw ONLY; never mutates scene state.  The helix math
 * (phase, x1/x2 backbone, depth ordering) is pure but computed inline here —
 * there is no separate LOGIC layer to store it in.
 */

/* safe cell write — silently clamps to drawable interior */
static inline void sput(int r, int c, chtype ch, int rows, int cols)
{
    if (r >= 1 && r < rows - 1 && c >= 0 && c < cols)
        mvaddch(r, c, ch);
}

/* backbone_pos — screen coordinate of a helix backbone at a given phase:
 * centre + amp·cos(phase), rounded to a cell.  Pass -amp for the mirror strand.
 * The parametric helix projection (Bourke; Foley/van Dam). */
static inline int backbone_pos(float center, float amp, float phase)
{
    return (int)roundf(center + amp * cosf(phase));
}

/* rung_interval — rows between base-pair rungs at a given pitch (≈
 * RUNGS_PER_PITCH rungs per pitch); helix scenes draw a rung every Nth row. */
static inline int rung_interval(float pitch)
{
    return (int)(pitch / RUNGS_PER_PITCH + 0.5f);
}

/* slope_glyph — the leaning glyph for a Z-DNA strand: which way it stepped
 * since the previous row ('/' rightward, '\' leftward, '|' at a turn). */
static inline char slope_glyph(int x, int px)
{
    return (x > px) ? '/' : (x < px ? '\\' : '|');
}

/* draw_rung — a base-pair bond: a CP_BD '-' run bridging the gap between two
 * backbone columns xa..xb on this row (interior only, not over the strands). */
static void draw_rung(int row, int xa, int xb, int rows, int cols)
{
    int lo = (xa < xb ? xa : xb) + 1;
    int hi = (xa > xb ? xa : xb) - 1;
    attrset(COLOR_PAIR(CP_BD));
    for (int c = lo; c <= hi; c++)
        sput(row, c, '-', rows, cols);
}

/*
 * helix_draw — shared backbone for B-DNA, A-DNA, Z-DNA.  Params:
 *   cx     centre column           amp    backbone amplitude (cells)
 *   pitch  rows per full turn       hand   +1 right-handed / -1 left-handed
 *   t      phase clock              rows, cols   viewport
 *   zigzag true → Z-DNA lean glyphs ('/','\'), false → round beads
 * Rung spacing is derived from pitch (rung_interval); there is no separate knob.
 */
static void helix_draw(int cx, float amp, float pitch, float hand,
                       float t, int rows, int cols, bool zigzag)
{
    float k = TAU / pitch * hand;            /* signed per-row phase step (handedness) */
    int   bstep = rung_interval(pitch);
    if (bstep < 2) bstep = 2;
    int   draw_rows = rows - HUD_ROWS - 1;

    for (int row = 1; row < draw_rows; row++) {
        float phase = k * (float)row + t;
        float prev  = k * (float)(row - 1) + t;
        int   x1 = backbone_pos((float)cx,  amp, phase);
        int   x2 = backbone_pos((float)cx, -amp, phase);
        bool  front_left = (sinf(phase) >= 0.f);   /* strand 1 nearer the viewer? */
        bool  is_rung    = ((row % bstep) == 0);

        if (is_rung)
            draw_rung(row, x1, x2, rows, cols);

        /* strand glyphs: Z-DNA leans ('/','\'), others are round beads */
        chtype ch1, ch2;
        if (zigzag) {
            int px1 = backbone_pos((float)cx, amp, prev);
            ch1 = slope_glyph(x1, px1);
            ch2 = (ch1 == '/') ? '\\' : (ch1 == '\\' ? '/' : '|');  /* opposite lean */
        } else {
            ch1 = ch2 = is_rung ? 'O' : 'o';
        }

        /* paint back strand dim, then front strand bright (bold on a rung) */
        attrset(COLOR_PAIR(front_left ? CP_S2 : CP_S1) | A_DIM);
        sput(row, front_left ? x2 : x1, front_left ? ch2 : ch1, rows, cols);

        attrset(COLOR_PAIR(front_left ? CP_S1 : CP_S2) | (is_rung ? A_BOLD : A_NORMAL));
        sput(row, front_left ? x1 : x2, front_left ? ch1 : ch2, rows, cols);
    }
    attrset(A_NORMAL);
}

/* 0: B-DNA — classic right-handed, standard pitch */
static void draw_bdna(int cx, float t, int rows, int cols)
{
    helix_draw(cx, 9.f, 20.f, 1.f, t, rows, cols, false);
}

/* 1: A-DNA — right-handed, wide amplitude, compressed pitch */
static void draw_adna(int cx, float t, int rows, int cols)
{
    helix_draw(cx, 12.f, 13.f, 1.f, t, rows, cols, false);
}

/* 2: Z-DNA — left-handed zigzag */
static void draw_zdna(int cx, float t, int rows, int cols)
{
    helix_draw(cx, 6.f, 24.f, -1.f, t, rows, cols, true);
}

/* 3: Triple Helix — three strands at 0, 2π/3, 4π/3 */
static void draw_triple(int cx, float t, int rows, int cols)
{
    float pitch = 21.f;
    float amp   = 9.f;
    float k     = TAU / pitch;
    int   bstep = rung_interval(pitch);
    int   draw_rows = rows - HUD_ROWS - 1;

    static const float ph3[3] = { 0.f, 2.094395f, 4.188790f };
    static const int   cp3[3] = { CP_S1, CP_S2, CP_S3 };

    for (int row = 1; row < draw_rows; row++) {
        float base_phase = k * (float)row + t;
        bool  bp = ((row % bstep) == 0);

        /* bond between strand 0 and strand 1 */
        if (bp) {
            int xa = backbone_pos((float)cx, amp, base_phase + ph3[0]);
            int xb = backbone_pos((float)cx, amp, base_phase + ph3[1]);
            draw_rung(row, xa, xb, rows, cols);
        }

        /*
         * Sv — one strand's vertex on this row: its screen column x, its depth
         * z = sin(phase) ∈ [-1,1] (>0 = nearer the viewer), and its colour pair
         * cp.  The three strands are insertion-sorted by z and painted back-to-
         * front so the nearest over-draws the others — the painter's algorithm
         * (Foley/van Dam), the same depth cue helix_draw uses for two strands.
         */
        typedef struct { int x; float z; int cp; } Sv;
        Sv sv[3];
        for (int i = 0; i < 3; i++) {
            float ph  = base_phase + ph3[i];
            sv[i].x   = backbone_pos((float)cx, amp, ph);
            sv[i].z   = sinf(ph);
            sv[i].cp  = cp3[i];
        }
        /* simple insertion sort by z ascending */
        for (int a = 1; a < 3; a++) {
            Sv key = sv[a];
            int b = a - 1;
            while (b >= 0 && sv[b].z > key.z) { sv[b+1] = sv[b]; b--; }
            sv[b+1] = key;
        }

        chtype bc = bp ? 'O' : 'o';
        for (int i = 0; i < 3; i++) {
            attr_t at = (i == 2) ? (bp ? A_BOLD : A_NORMAL) : A_DIM;
            attrset(COLOR_PAIR(sv[i].cp) | at);
            sput(row, sv[i].x, bc, rows, cols);
        }
    }
    attrset(A_NORMAL);
}

/* 4: G-Quadruplex — four parallel strands + periodic G-tetrad squares */
static void draw_gquad(int cx, float t, int rows, int cols)
{
    int draw_rows = rows - HUD_ROWS - 1;
    int sp = (cols > 60) ? 9 : 5;
    int xs[4] = { cx - sp - sp/2, cx - sp/2, cx + sp/2, cx + sp + sp/2 };
    static const int cp4[4] = { CP_S1, CP_S2, CP_S3, CP_S2 };
    int tetrad_step = 7;

    for (int row = 1; row < draw_rows; row++) {
        bool tetrad = ((row % tetrad_step) == 0);
        float pulse_ph = TAU * (float)row / 30.f + t;
        attr_t pulse   = (sinf(pulse_ph) > 0.f) ? A_BOLD : A_NORMAL;

        for (int i = 0; i < 4; i++) {
            attrset(COLOR_PAIR(cp4[i]) | (tetrad ? A_BOLD : pulse));
            sput(row, xs[i], tetrad ? 'G' : '|', rows, cols);
        }

        if (tetrad) {
            /* outer rectangle connecting all 4 strands */
            attrset(COLOR_PAIR(CP_BD) | A_BOLD);
            for (int c = xs[0] + 1; c < xs[3]; c++)
                sput(row, c, '-', rows, cols);
            /* central potassium ion marker */
            int mid = (xs[1] + xs[2]) / 2;
            attrset(COLOR_PAIR(CP_LB) | A_BOLD);
            sput(row, mid, 'K', rows, cols);
        }
    }
    attrset(A_NORMAL);
    (void)t;
}

/* 5: RNA Hairpin — stem-loop structure */
static void draw_hairpin(int cx, float t, int rows, int cols)
{
    int draw_rows = rows - HUD_ROWS - 1;
    int gap       = 6;
    int lx        = cx - gap;
    int rx        = cx + gap;
    int stem_rows = (draw_rows - 6) * 3 / 4;

    static const char nt_l[] = "AUGCAUGCAUGC";
    static const char nt_r[] = "UACGUACGUACG";

    /* 5' / 3' end labels */
    attrset(COLOR_PAIR(CP_LB) | A_BOLD);
    if (lx - 3 >= 0) mvprintw(1, lx - 3, "5'");
    if (rx + 2 < cols) mvprintw(1, rx + 2, "3'");

    /* stem */
    for (int i = 0; i < stem_rows; i++) {
        int row = 2 + i;
        if (row >= draw_rows) break;

        float ph = TAU * (float)i / 10.f + t * 0.4f;
        attr_t pulse = (sinf(ph) > 0.f) ? A_BOLD : A_NORMAL;

        attrset(COLOR_PAIR(CP_S1) | A_DIM);
        sput(row, lx, '|', rows, cols);
        attrset(COLOR_PAIR(CP_S2) | A_DIM);
        sput(row, rx, '|', rows, cols);

        /* base pair every 2 rows */
        if ((i % 2) == 0) {
            int ni = (i / 2) % (int)(sizeof(nt_l) - 1);
            attrset(COLOR_PAIR(CP_S1) | A_BOLD);
            sput(row, lx + 1, (chtype)nt_l[ni], rows, cols);
            attrset(COLOR_PAIR(CP_S2) | A_BOLD);
            sput(row, rx - 1, (chtype)nt_r[ni], rows, cols);
            char bch = (nt_l[ni] == 'G' || nt_l[ni] == 'C') ? '=' : '-';
            attrset(COLOR_PAIR(CP_BD) | pulse);
            for (int c = lx + 2; c < rx - 1; c++)
                sput(row, c, (chtype)bch, rows, cols);
        }
    }

    /* loop: semicircle arc from lx to rx below stem */
    int loop_top = 2 + stem_rows;
    int n_arc    = gap * 8;
    for (int i = 0; i <= n_arc; i++) {
        float a  = (float)M_PI * (float)i / (float)n_arc;
        int   lc = cx - (int)roundf((float)gap * cosf(a));
        int   lr = loop_top + (int)roundf((float)gap * 0.5f * sinf(a));
        float ph = TAU * (float)i / (float)n_arc + t;
        int   cp = (sinf(ph) > 0.f) ? CP_S1 : CP_S2;
        attrset(COLOR_PAIR(cp) | A_BOLD);
        sput(lr, lc, '*', rows, cols);
    }

    /* loop nucleotide labels at arc midpoint */
    {
        static const char loop_seq[] = "ACGU";
        int mid_row = loop_top + gap / 2;
        for (int i = 0; i < 4; i++) {
            int mc = cx - 1 + i;
            attrset(COLOR_PAIR(CP_LB) | A_BOLD);
            sput(mid_row, mc, (chtype)loop_seq[i], rows, cols);
        }
    }
    attrset(A_NORMAL);
}

/* 6: Replication Fork — Y-shape with animated fork point */
static void draw_rfork(int cx, float t, int rows, int cols)
{
    int   draw_rows = rows - HUD_ROWS - 1;
    float fork_frac = 0.35f + 0.15f * sinf(t * 0.25f);
    int   fork_row  = 1 + (int)(fork_frac * (float)(draw_rows - 2));
    float pitch = 18.f;
    float amp   = 8.f;
    float k     = TAU / pitch;

    /* parent double helix above fork */
    for (int row = 1; row < fork_row && row < draw_rows; row++) {
        float phase = k * (float)row + t;
        int   x1 = backbone_pos((float)cx,  amp, phase);
        int   x2 = backbone_pos((float)cx, -amp, phase);
        float z1 = sinf(phase);
        bool  s1f = (z1 >= 0.f);
        bool  bp  = ((row % 4) == 0);

        if (bp) draw_rung(row, x1, x2, rows, cols);
        attrset(COLOR_PAIR(s1f?CP_S2:CP_S1)|A_DIM);
        sput(row, s1f?x2:x1, bp?'O':'o', rows, cols);
        attrset(COLOR_PAIR(s1f?CP_S1:CP_S2)|(bp?A_BOLD:A_NORMAL));
        sput(row, s1f?x1:x2, bp?'O':'o', rows, cols);
    }

    /* fork point indicator */
    if (fork_row >= 1 && fork_row < draw_rows) {
        attrset(COLOR_PAIR(CP_LB) | A_BOLD);
        sput(fork_row, cx, 'Y', rows, cols);
        sput(fork_row, cx - 2, '<', rows, cols);
        sput(fork_row, cx - 1, '<', rows, cols);
        sput(fork_row, cx + 1, '>', rows, cols);
        sput(fork_row, cx + 2, '>', rows, cols);
    }

    /* two daughter strands diverging below fork */
    for (int row = fork_row + 1; row < draw_rows; row++) {
        float depth = (float)(row - fork_row);
        float div   = 0.55f * depth;

        /* leading strand — left */
        float ph_l = k * (float)row + t;
        int   xl   = backbone_pos((float)cx - div, amp * 0.5f, ph_l);
        attrset(COLOR_PAIR(CP_S1) | A_NORMAL);
        sput(row, xl, 'o', rows, cols);

        /* lagging strand — right, with Okazaki fragment gaps */
        float ph_r  = k * (float)row + t + (float)M_PI;
        int   xr    = backbone_pos((float)cx + div, amp * 0.5f, ph_r);
        int   frag  = (int)(depth) % 10;
        attr_t fattr = (frag < 7) ? A_NORMAL : A_DIM;
        chtype fch   = (frag < 7) ? 'o' : '.';
        attrset(COLOR_PAIR(CP_S2) | fattr);
        sput(row, xr, fch, rows, cols);
    }

    /* labels */
    if (fork_row + 3 < draw_rows) {
        attrset(COLOR_PAIR(CP_LB));
        if (cx - 16 >= 0) mvprintw(fork_row + 3, cx - 16, "Leading");
        if (cx + 6 < cols) mvprintw(fork_row + 3, cx + 6,  "Lagging");
    }
    attrset(A_NORMAL);
}

/* 7: Cruciform — vertical + horizontal double helices crossing */
static void draw_cruciform(int cx, float t, int rows, int cols)
{
    int   draw_rows = rows - HUD_ROWS - 1;
    int   cy        = draw_rows / 2;
    float pitch_v   = 18.f;
    float kv        = TAU / pitch_v;
    float amp_v     = 7.f;
    /* horizontal arm: compensate aspect ratio */
    float pitch_h   = pitch_v / CELL_AR;
    float kh        = TAU / pitch_h;
    float amp_h     = amp_v * CELL_AR;

    /* vertical arms — skip 3 rows near the junction */
    for (int row = 1; row < draw_rows; row++) {
        if (abs(row - cy) < 3) continue;
        float phase = kv * (float)row + t;
        int   x1 = backbone_pos((float)cx,  amp_v, phase);
        int   x2 = backbone_pos((float)cx, -amp_v, phase);
        float z1 = sinf(phase);
        bool  s1f = (z1 >= 0.f);
        bool  bp  = ((row % 4) == 0);

        if (bp) draw_rung(row, x1, x2, rows, cols);
        attrset(COLOR_PAIR(s1f?CP_S2:CP_S1)|A_DIM);
        sput(row, s1f?x2:x1, 'o', rows, cols);
        attrset(COLOR_PAIR(s1f?CP_S1:CP_S2)|(bp?A_BOLD:A_NORMAL));
        sput(row, s1f?x1:x2, 'o', rows, cols);
    }

    /* horizontal arms — skip 6 cols near junction */
    int arm_len = cols / 2 - 2;
    for (int dc = -arm_len; dc <= arm_len; dc++) {
        int col = cx + dc;
        if (col < 1 || col >= cols - 1) continue;
        if (abs(dc) < 6) continue;

        float phase = kh * (float)dc + t + (float)M_PI * 0.5f;
        int   r1 = cy + (int)roundf(amp_h * cosf(phase));
        int   r2 = cy - (int)roundf(amp_h * cosf(phase));
        float z1 = sinf(phase);
        bool  s1f = (z1 >= 0.f);
        bool  bp  = ((abs(dc) % 5) == 0);

        attrset(COLOR_PAIR(s1f?CP_S3:CP_S2)|A_DIM);
        sput(s1f?r2:r1, col, 'o', rows, cols);
        attrset(COLOR_PAIR(s1f?CP_S2:CP_S3)|(bp?A_BOLD:A_NORMAL));
        sput(s1f?r1:r2, col, 'o', rows, cols);
    }

    /* junction cross */
    attrset(COLOR_PAIR(CP_LB) | A_BOLD);
    sput(cy,     cx,     '+', rows, cols);
    sput(cy - 1, cx,     '|', rows, cols);
    sput(cy + 1, cx,     '|', rows, cols);
    sput(cy,     cx - 1, '-', rows, cols);
    sput(cy,     cx + 1, '-', rows, cols);
    attrset(A_NORMAL);
}

/* 8: Plasmid — closed circular DNA with gene regions */
static void draw_plasmid(int cx, float t, int rows, int cols)
{
    int   draw_rows = rows - HUD_ROWS - 1;
    int   cy        = draw_rows / 2;
    float max_r     = (float)((cols < draw_rows * 2 ? cols : draw_rows * 2)) * 0.27f;
    float rx        = max_r;
    float ry        = rx * CELL_AR;

    static const char *gene_names[4] = { "GeneA", "GeneB", "Ori", "GeneC" };
    static const float gene_angles[4] = { 0.78f, 2.36f, 3.93f, 5.50f };
    static const int   gene_cp[4]     = { CP_S1, CP_S2, CP_S3, CP_BD };

    /* outer strand (bold) + inner strand (dim) */
    int n_pts = 360;
    for (int i = 0; i < n_pts; i++) {
        float a = TAU * (float)i / (float)n_pts + t * 0.08f;

        /* which gene region? */
        float norm_a = fmodf(a - t * 0.08f, TAU);
        if (norm_a < 0.f) norm_a += TAU;
        int sector = (int)(norm_a / TAU * 4.f) % 4;

        int r_outer = cy + (int)roundf(ry * sinf(a));
        int c_outer = cx + (int)roundf(rx * cosf(a));
        int r_inner = cy + (int)roundf(ry * 0.82f * sinf(a));
        int c_inner = cx + (int)roundf(rx * 0.82f * cosf(a));

        attrset(COLOR_PAIR(gene_cp[sector]) | A_BOLD);
        sput(r_outer, c_outer, 'o', rows, cols);
        attrset(COLOR_PAIR(gene_cp[sector]) | A_DIM);
        sput(r_inner, c_inner, '.', rows, cols);
    }

    /* gene region labels inside ring */
    for (int g = 0; g < 4; g++) {
        float a  = gene_angles[g];
        int   lr = cy + (int)roundf(ry * 0.5f * sinf(a));
        int   lc = cx + (int)roundf(rx * 0.5f * cosf(a));
        int   len = (int)strlen(gene_names[g]);
        lc -= len / 2;
        if (lr >= 1 && lr < draw_rows && lc >= 1 && lc + len < cols - 1) {
            attrset(COLOR_PAIR(gene_cp[g]) | A_BOLD);
            mvprintw(lr, lc, "%s", gene_names[g]);
        }
    }

    /* centre label */
    attrset(COLOR_PAIR(CP_LB));
    if (cy >= 1 && cy < draw_rows) {
        int lc = cx - 3;
        if (lc >= 1 && lc + 7 < cols - 1)
            mvprintw(cy, lc, "PLASMID");
    }
    attrset(A_NORMAL);
}

/* 9: DNA Ladder — educational unrolled flat view, scrolling */
static void draw_ladder(int cx, float t, int rows, int cols)
{
    int draw_rows = rows - HUD_ROWS - 1;
    int gap       = 8;
    int lx        = cx - gap;
    int rx        = cx + gap;
    int rung_step = 3;   /* rows between base pairs */

    static const char nt_l[] = "ATGCTAGCATGCATGC";
    static const char nt_r[] = "TACGATCGTACGTACG";
    int n_seq = (int)(sizeof(nt_l) - 1);

    /* scrolling offset */
    int scroll = (int)(t * 1.8f) % (rung_step * n_seq);

    /* backbone labels */
    attrset(COLOR_PAIR(CP_S1) | A_BOLD);
    sput(1, lx - 3, '5', rows, cols);
    sput(1, lx - 2, '\'', rows, cols);
    attrset(COLOR_PAIR(CP_S2) | A_BOLD);
    sput(1, rx + 2, '3', rows, cols);
    sput(1, rx + 3, '\'', rows, cols);

    for (int row = 2; row < draw_rows; row++) {
        /* backbone lines */
        attrset(COLOR_PAIR(CP_S1) | A_NORMAL);
        sput(row, lx, '|', rows, cols);
        attrset(COLOR_PAIR(CP_S2) | A_NORMAL);
        sput(row, rx, '|', rows, cols);

        /* base pair rungs */
        int adj = (row + scroll) % (rung_step * n_seq);
        if ((adj % rung_step) == 0) {
            int ni = (adj / rung_step) % n_seq;
            float ph = TAU * (float)row / (float)(n_seq * rung_step) + t * 0.3f;
            attr_t pulse = (sinf(ph) > 0.f) ? A_BOLD : A_NORMAL;

            /* nucleotide chars at strand edges */
            attrset(COLOR_PAIR(CP_S1) | A_BOLD);
            sput(row, lx + 1, (chtype)nt_l[ni], rows, cols);
            attrset(COLOR_PAIR(CP_S2) | A_BOLD);
            sput(row, rx - 1, (chtype)nt_r[ni], rows, cols);

            /* bond type: G-C triple bond, A-T double bond */
            bool strong = (nt_l[ni] == 'G' || nt_l[ni] == 'C');
            char bch    = strong ? '=' : '-';
            attrset(COLOR_PAIR(CP_BD) | pulse);
            for (int c = lx + 2; c < rx - 1; c++)
                sput(row, c, (chtype)bch, rows, cols);

            /* bond label at centre */
            if (rx - lx > 6) {
                attrset(COLOR_PAIR(CP_LB) | pulse);
                sput(row, cx, strong ? '#' : ':', rows, cols);
            }
        }
    }

    /* 3' / 5' bottom labels */
    attrset(COLOR_PAIR(CP_S1) | A_BOLD);
    sput(draw_rows - 1, lx - 3, '3', rows, cols);
    sput(draw_rows - 1, lx - 2, '\'', rows, cols);
    attrset(COLOR_PAIR(CP_S2) | A_BOLD);
    sput(draw_rows - 1, rx + 2, '5', rows, cols);
    sput(draw_rows - 1, rx + 3, '\'', rows, cols);
    attrset(A_NORMAL);
}

/*
 * DnaStructure — one entry in the gallery: a display name and the routine that
 * renders it.  Indexed by Scene.dna_type; scene_draw calls .draw, screen_hud
 * shows .name.  Replaces the old parallel dna_names[] + draw_fns[], so the
 * label and its renderer can never drift out of order.  The set of forms (A/B/Z
 * helices, triplex, G4, cruciform, plasmid, fork, hairpin) is the structural
 * taxonomy of Saenger, *Principles of Nucleic Acid Structure*, and Alberts et
 * al., *Molecular Biology of the Cell* (see References).
 *
 * DrawFn — a scene renderer.  cx = centre column, t = the phase clock (animates
 * the helix), rows/cols = the terminal viewport.  Pure RENDER: reads these,
 * writes the screen, mutates no state.
 */
typedef void (*DrawFn)(int cx, float t, int rows, int cols);
typedef struct {
    const char *name;   /* HUD label for this form                       */
    DrawFn      draw;   /* the §4 routine that paints it                 */
} DnaStructure;

static const DnaStructure dna_structures[N_DNA_TYPES] = {
    { "B-DNA Double Helix",     draw_bdna      },
    { "A-DNA Double Helix",     draw_adna      },
    { "Z-DNA (Left-handed)",    draw_zdna      },
    { "Triple Helix",           draw_triple    },
    { "G-Quadruplex",           draw_gquad     },
    { "RNA Hairpin",            draw_hairpin   },
    { "Replication Fork",       draw_rfork     },
    { "Cruciform",              draw_cruciform },
    { "Plasmid (Circular DNA)", draw_plasmid   },
    { "DNA Ladder",             draw_ladder    },
};

/* ===================================================================== */
/* §5  scene                       STATE + USER EVENTS                    */
/* ===================================================================== */
/*
 * Scene holds all per-instance state.  scene_init / scene_resize mutate it on
 * boot and on SIGWINCH — USER EVENTS, NOT part of the per-frame tick.
 * scene_init also binds the initial theme (color_apply_theme, §3).
 */

/*
 * Scene — the whole visualisation state, read as a table of contents:
 *   WHAT is shown — dna_type, an index into dna_structures[] (n/p cycles it).
 *   HOW it looks  — theme, an index into themes[] (t cycles it).
 *   WHEN          — t, the phase clock every scene scrolls by (the sim state).
 *   WHERE         — rows, cols: the terminal viewport.
 * scene_tick advances t; scene_init / scene_resize set the rest on boot / resize.
 * `t` is the phase term added to every backbone angle (phase = row·k·hand + t,
 * cf. KEY FORMULAS): advancing it rotates the helices — a parametric sweep in
 * the sense of Foley/van Dam and Bourke (see References).
 */
typedef struct {
    int   dna_type;   /* WHAT:  index into dna_structures[]            */
    int   theme;      /* HOW:   index into themes[]                    */
    float t;          /* WHEN:  phase clock, seconds; the helix angle  */
    int   rows, cols; /* WHERE: viewport in cells                      */
} Scene;

static void scene_init(Scene *s, int rows, int cols)
{
    s->dna_type = 0;
    s->theme    = 0;
    s->t        = 0.f;
    s->rows     = rows;
    s->cols     = cols;
    color_apply_theme(&themes[s->theme]);
}

static void scene_resize(Scene *s, int rows, int cols)
{
    s->rows = rows;
    s->cols = cols;
}

/* ===================================================================== */
/* §6  simulation                  SIMULATION — the only state advance    */
/* ===================================================================== */
/* Advances Scene.t (the phase clock that scrolls every helix) and nothing  */
/* else.  Called once per frame from main (§8), and skipped while paused —  */
/* this is the entire per-tick state advance.                               */
static void scene_tick(Scene *s, float dt)
{
    s->t += dt;
}

/* ===================================================================== */
/* §7  render                      RENDER — scene → screen + terminal I/O */
/* ===================================================================== */
/*
 * scene_draw dispatches to the active §4 draw; screen_init brings up the
 * terminal (once); screen_hud overlays the two HUD bars; screen_present flushes.
 * All read-only on scene state — they write the ncurses screen only.
 */

static void scene_draw(const Scene *s)
{
    int cx = s->cols / 2;
    dna_structures[s->dna_type].draw(cx, s->t, s->rows, s->cols);
}

static void screen_init(void)
{
    initscr();
    cbreak();
    noecho();
    keypad(stdscr, TRUE);
    nodelay(stdscr, TRUE);
    curs_set(0);
}

/*
 * screen_hud — two fixed bars over the scene:
 *   top    row 0      DATA    — DNA index + name (left) and theme / fps / state
 *                              (right-aligned).
 *   bottom row rows-1 ACTIONS — the key legend.
 * Both bars are filled first, then text is clipped with "%.*s" — the left data
 * is kept clear of the right block, and nothing overruns the terminal width.
 */
static void screen_hud(const Scene *s, int target_fps, float actual_fps,
                       bool paused)
{
    int cols = s->cols;
    if (cols < 1) return;

    /* ── Top row: DATA — name (left) + theme / fps / state (right). ── */
    attrset(COLOR_PAIR(CP_HUD) | A_BOLD);
    mvhline(0, 0, ' ', cols);

    char left[64], right[48];
    snprintf(left,  sizeof left,  " DNA %d/%d: %s ",
             s->dna_type + 1, N_DNA_TYPES, dna_structures[s->dna_type].name);
    snprintf(right, sizeof right, " %s  %.0f/%d fps  %s ",
             themes[s->theme].name, actual_fps, target_fps,
             paused ? "PAUSED" : "running");
    int rx = cols - (int)strlen(right);          /* right-aligned column */
    if (rx >= 0) {
        mvprintw(0, 0,  "%.*s", rx, left);       /* left clipped clear of right */
        mvprintw(0, rx, "%s", right);
    } else {
        mvprintw(0, 0,  "%.*s", cols, left);     /* too narrow: data only */
    }

    /* ── Bottom row: ACTIONS — every interactive key. ── */
    int brow = s->rows - 1;
    if (brow > 0) {
        mvhline(brow, 0, ' ', cols);
        mvprintw(brow, 0, "%.*s", cols,
                 " n/p:type  t:theme  spc:pause  ]/[:fps  q:quit ");
    }
    attrset(A_NORMAL);
}

static void screen_present(void)
{
    refresh();
}

/* ===================================================================== */
/* §8  app                         USER EVENTS + PERFORMANCE combine      */
/* ===================================================================== */
/*
 * Signal handlers + main().  main() is the ONE per-tick combine point (see
 * ARCHITECTURE): resize → input → scene_tick (SIM) → RENDER → sleep/fps (PERF).
 * Input and resize mutate state but are USER EVENTS, NOT part of the tick.
 */

static volatile sig_atomic_t g_resize = 0;
static volatile sig_atomic_t g_quit   = 0;

static void sig_winch(int s) { (void)s; g_resize = 1; }
static void sig_int(int s)   { (void)s; g_quit   = 1; }

int main(void)
{
    signal(SIGWINCH, sig_winch);
    signal(SIGINT,   sig_int);

    screen_init();

    int rows, cols;
    getmaxyx(stdscr, rows, cols);

    Scene s;
    scene_init(&s, rows, cols);

    bool    paused     = false;
    int     fps        = SIM_FPS_DEF;
    int64_t tick_ns    = NS_PER_SEC / fps;
    int64_t last_ns    = clock_ns();
    int64_t fps_accum  = 0;
    int     fps_frames = 0;
    float   actual_fps = (float)fps;
    float   dt         = 1.f / (float)fps;

    while (!g_quit) {
        /* handle resize */
        if (g_resize) {
            g_resize = 0;
            endwin();
            refresh();
            getmaxyx(stdscr, rows, cols);
            scene_resize(&s, rows, cols);
        }

        /* input */
        int ch = getch();
        switch (ch) {
        case 'q': case 27: g_quit = 1;    break;
        case ' ':          paused = !paused; break;
        case 'n':
            s.dna_type = (s.dna_type + 1) % N_DNA_TYPES;
            s.t = 0.f;
            break;
        case 'p':
            s.dna_type = (s.dna_type + N_DNA_TYPES - 1) % N_DNA_TYPES;
            s.t = 0.f;
            break;
        case 't':
            s.theme = (s.theme + 1) % N_THEMES;
            color_apply_theme(&themes[s.theme]);
            break;
        case ']':
            fps = fps + SIM_FPS_STEP <= SIM_FPS_MAX ? fps + SIM_FPS_STEP : SIM_FPS_MAX;
            tick_ns = NS_PER_SEC / fps;
            dt      = 1.f / (float)fps;
            break;
        case '[':
            fps = fps - SIM_FPS_STEP >= SIM_FPS_MIN ? fps - SIM_FPS_STEP : SIM_FPS_MIN;
            tick_ns = NS_PER_SEC / fps;
            dt      = 1.f / (float)fps;
            break;
        default: break;
        }

        if (!paused)
            scene_tick(&s, dt);

        /* draw */
        erase();
        scene_draw(&s);
        screen_hud(&s, fps, actual_fps, paused);
        screen_present();

        /* sleep to hit target fps */
        int64_t now     = clock_ns();
        int64_t elapsed = now - last_ns;
        clock_sleep_ns(tick_ns - elapsed);
        last_ns = clock_ns();

        /* actual fps tracking */
        fps_accum += clock_ns() - now + (tick_ns - elapsed > 0 ? tick_ns - elapsed : 0);
        fps_frames++;
        if (fps_accum >= NS_PER_SEC / 2) {
            actual_fps  = (float)fps_frames * (float)NS_PER_SEC / (float)fps_accum;
            fps_accum   = 0;
            fps_frames  = 0;
        }
    }

    endwin();
    return 0;
}
