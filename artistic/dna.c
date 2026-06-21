/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * dna.c — 10 DNA/RNA structure pictures in 2D ASCII art (n/p cycles them).
 * The shapes (B/A/Z helices, triplex, G-quadruplex, cruciform, plasmid,
 * replication fork, RNA hairpin) follow Saenger, Principles of Nucleic Acid
 * Structure, and Alberts et al., Molecular Biology of the Cell.
 */

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

/* ── §1 config — sizes, bounds, and the colour-theme table ── */

#define CELL_AR     0.5f   /* terminal cells are ~twice as tall as wide; this is width/height */
#define TAU         6.28318530f
#define RUNGS_PER_PITCH 5.f /* draw about this many base-pair rungs per full helix turn (see rung_interval) */

/*
 * Counts and limits, all fixed at compile time. The N_* values both size the
 * lookup tables and bound the cycle keys (n/p wrap around N_DNA_TYPES, t wraps
 * around N_THEMES). HUD_ROWS is how many rows the top/bottom bars eat, so each
 * scene gets rows 1 .. rows-HUD_ROWS-1. The SIM_FPS_* values bound the [/] keys.
 */
enum {
    N_DNA_TYPES   = 10,    /* how many structures in the gallery              */
    N_THEMES      =  6,    /* how many colour themes                          */
    THEME_COLS    =  5,    /* colours per theme: strand1, strand2, strand3, bond, label */
    HUD_ROWS      =  2,    /* rows taken by the HUD (one bar top, one bottom) */

    SIM_FPS_MIN   =  5,    /* lowest, default, highest, and step for the fps keys */
    SIM_FPS_DEF   = 30,
    SIM_FPS_MAX   = 60,
    SIM_FPS_STEP  =  5,
};

#define NS_PER_SEC  1000000000LL

/*
 * Theme — one named colour scheme: the five colours a structure is painted in.
 * Picked by Scene.theme; color_apply_theme() loads the active one into ncurses.
 * There are two palettes because not every terminal has 256 colours: pal256 is
 * used when it does, pal8 is the nearest basic-8 stand-in otherwise. The five
 * slots, in order, are strand1, strand2, strand3, bond, label.
 */
typedef struct {
    const char *name;                /* shown in the HUD                          */
    short       pal256[THEME_COLS];  /* colour numbers used when the terminal has 256 colours */
    short       pal8[THEME_COLS];    /* fallback colours for an 8-colour terminal  */
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

/* ── §2 clock — read the clock and sleep, for frame pacing ── */

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

/* ── §3 color — load a theme into ncurses colour pairs ── */

/*
 * The colour-pair slots this program uses. They start at 1 because slot 0 is
 * the terminal's own default and can't be changed. CP_S1..CP_LB are the five
 * themed roles, in the same order as a theme's palette, and get re-loaded each
 * time the theme changes. CP_HUD never changes, so the HUD stays readable.
 *   S1/S2/S3 = the three strands   BD = base-pair rung   LB = label/marker
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

/* ── §4 draw — the ten scene renderers and their shared helpers ── */

/* Write one character, but ignore it if it would land outside the drawable area. */
static inline void sput(int r, int c, chtype ch, int rows, int cols)
{
    if (r >= 1 && r < rows - 1 && c >= 0 && c < cols)
        mvaddch(r, c, ch);
}

/* Where a strand sits on a row: the centre column nudged left/right as the
 * helix turns. Pass +amp for one strand, -amp for the one mirrored across it. */
static inline int backbone_pos(float center, float amp, float phase)
{
    return (int)roundf(center + amp * cosf(phase));
}

/* How many rows apart to draw the base-pair rungs for a given turn length. */
static inline int rung_interval(float pitch)
{
    return (int)(pitch / RUNGS_PER_PITCH + 0.5f);
}

/* Pick a leaning character for a Z-DNA strand based on which way it just moved:
 * '/' if it stepped right, '\' if left, '|' if it didn't move. */
static inline char slope_glyph(int x, int px)
{
    return (x > px) ? '/' : (x < px ? '\\' : '|');
}

/* Draw a base-pair rung: a row of '-' filling the gap between the two strand
 * columns, stopping short of the strands themselves. */
static void draw_rung(int row, int xa, int xb, int rows, int cols)
{
    int lo = (xa < xb ? xa : xb) + 1;
    int hi = (xa > xb ? xa : xb) - 1;
    attrset(COLOR_PAIR(CP_BD));
    for (int c = lo; c <= hi; c++)
        sput(row, c, '-', rows, cols);
}

/*
 * Draw a two-strand double helix; B-DNA, A-DNA and Z-DNA all use this.
 *   cx     centre column          amp    how far the strands swing sideways (cells)
 *   pitch  rows per full turn      hand   +1 right-handed, -1 left-handed
 *   t      animation clock         rows, cols   terminal size
 *   zigzag true draws Z-DNA's leaning '/','\' strands; false draws round beads.
 * The depth trick: a strand on the near side of the turn is drawn bright, the
 * far side dim, which reads as 3-D. Rung spacing comes from pitch automatically.
 */
static void helix_draw(int cx, float amp, float pitch, float hand,
                       float t, int rows, int cols, bool zigzag)
{
    float k = TAU / pitch * hand;            /* how much the angle turns per row; sign sets handedness */
    int   bstep = rung_interval(pitch);
    if (bstep < 2) bstep = 2;
    int   draw_rows = rows - HUD_ROWS - 1;

    for (int row = 1; row < draw_rows; row++) {
        float phase = k * (float)row + t;
        float prev  = k * (float)(row - 1) + t;
        int   x1 = backbone_pos((float)cx,  amp, phase);
        int   x2 = backbone_pos((float)cx, -amp, phase);
        bool  front_left = (sinf(phase) >= 0.f);   /* is strand 1 the one nearer the viewer? */
        bool  is_rung    = ((row % bstep) == 0);

        if (is_rung)
            draw_rung(row, x1, x2, rows, cols);

        chtype ch1, ch2;
        if (zigzag) {
            int px1 = backbone_pos((float)cx, amp, prev);
            ch1 = slope_glyph(x1, px1);
            ch2 = (ch1 == '/') ? '\\' : (ch1 == '\\' ? '/' : '|');  /* the other strand leans the opposite way */
        } else {
            ch1 = ch2 = is_rung ? 'O' : 'o';
        }

        /* far strand dim first, then the near strand bright (bolder on a rung) */
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

        if (bp) {
            int xa = backbone_pos((float)cx, amp, base_phase + ph3[0]);
            int xb = backbone_pos((float)cx, amp, base_phase + ph3[1]);
            draw_rung(row, xa, xb, rows, cols);
        }

        /*
         * Sv — one strand's spot on this row: its column x, how near it is to
         * the viewer (z, from -1 far to +1 near), and its colour cp. We collect
         * all three strands, sort them far-to-near, and paint in that order so
         * the nearest one lands on top — the same near-bright/far-dim depth
         * trick helix_draw uses, just with three strands instead of two.
         */
        typedef struct { int x; float z; int cp; } Sv;
        Sv sv[3];
        for (int i = 0; i < 3; i++) {
            float ph  = base_phase + ph3[i];
            sv[i].x   = backbone_pos((float)cx, amp, ph);
            sv[i].z   = sinf(ph);
            sv[i].cp  = cp3[i];
        }
        /* sort the three strands far-to-near */
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
 * DnaStructure — one entry in the gallery: a name plus the function that draws
 * it. Picked by Scene.dna_type; keeping the name and the drawer side by side in
 * one table means they can't fall out of sync.
 *
 * DrawFn — the shape of every scene renderer. cx is the centre column, t is the
 * animation clock, rows/cols are the terminal size. A renderer only paints the
 * screen; it changes no state.
 */
typedef void (*DrawFn)(int cx, float t, int rows, int cols);
typedef struct {
    const char *name;   /* name shown in the HUD          */
    DrawFn      draw;   /* the §4 function that paints it  */
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

/* ── §5 scene — all state, plus boot and resize handling ── */

/*
 * Scene — everything the program needs to know to draw a frame:
 *   which structure is on screen (dna_type, picked by n/p),
 *   which colour theme (theme, picked by t),
 *   the running clock t that makes the helices turn (this is the only thing
 *   that changes over time), and the terminal size in rows and cols.
 * scene_tick bumps t forward; scene_init and scene_resize set the rest.
 */
typedef struct {
    int   dna_type;   /* which structure: index into dna_structures[]   */
    int   theme;      /* which colours: index into themes[]             */
    float t;          /* animation clock, in seconds; turns the helix   */
    int   rows, cols; /* terminal size in cells                         */
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

/* ── §6 simulation — move the animation clock forward ── */

static void scene_tick(Scene *s, float dt)
{
    s->t += dt;
}

/* ── §7 render — terminal setup, scene dispatch, and the HUD ── */

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
 * Draw the two HUD bars: the top one shows which structure is up plus the
 * theme, fps and pause state; the bottom one lists the keys. Each bar is filled
 * with spaces first, then text is clipped with "%.*s" so it never spills past
 * the terminal edge and the left text stops short of the right block.
 */
static void screen_hud(const Scene *s, int target_fps, float actual_fps,
                       bool paused)
{
    int cols = s->cols;
    if (cols < 1) return;

    /* top bar: structure name on the left, theme/fps/state on the right */
    attrset(COLOR_PAIR(CP_HUD) | A_BOLD);
    mvhline(0, 0, ' ', cols);

    char left[64], right[48];
    snprintf(left,  sizeof left,  " DNA %d/%d: %s ",
             s->dna_type + 1, N_DNA_TYPES, dna_structures[s->dna_type].name);
    snprintf(right, sizeof right, " %s  %.0f/%d fps  %s ",
             themes[s->theme].name, actual_fps, target_fps,
             paused ? "PAUSED" : "running");
    int rx = cols - (int)strlen(right);          /* column where the right block starts */
    if (rx >= 0) {
        mvprintw(0, 0,  "%.*s", rx, left);       /* clip the left text before it reaches the right block */
        mvprintw(0, rx, "%s", right);
    } else {
        mvprintw(0, 0,  "%.*s", cols, left);     /* terminal too narrow: show the name only */
    }

    /* bottom bar: the key legend */
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

/* ── §8 app — signal handlers and the main loop ── */

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
