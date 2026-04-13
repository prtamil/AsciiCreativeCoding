/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * ca_music.c — Musical Cellular Automaton
 *
 * Rule-110 1-D cellular automaton drives a pitch row.
 * Live cells select a note from the chosen scale; dead cells rest.
 * The terminal bell (\a) is triggered on each beat for rhythmic output.
 *
 * A "cursor" column sweeps across the CA each beat period.  The cell
 * at the cursor position determines the "active" note shown in the HUD.
 * Older rows fade toward dim characters.
 *
 * Visual:
 *   Grid scrolls upward — newest row at bottom.
 *   Live cells coloured by pitch class (7-colour rainbow).
 *   Dead cells are blank.
 *
 * Keys: q quit  p pause  r reset  t/T tempo  s cycle scale
 *       1-8 rules (110 default)  SPACE randomise seed
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra misc/ca_music.c \
 *       -o ca_music -lncurses -lm
 *
 * §1 config  §2 clock  §3 color  §4 CA  §5 music  §6 draw  §7 app
 */

#define _POSIX_C_SOURCE 200809L
#include <math.h>
#include <ncurses.h>
#include <signal.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

/* ===================================================================== */
/* §1  config                                                             */
/* ===================================================================== */

#define CA_W_MAX    240    /* max CA row width                    */
#define HISTORY     40     /* rows of CA history to display       */
#define RENDER_NS   (1000000000LL / 60)
#define HUD_ROWS    3

/* Available elementary CA rules */
static const int RULE_LIST[] = {110, 30, 90, 150, 105, 60, 45, 73};
#define N_RULES 8
static int g_rule_idx = 0;   /* index into RULE_LIST */
static int g_rule     = 110;

/* Tempo: beats per second */
static float g_bps   = 4.f;   /* 4 beats/s = 240 BPM  */
#define BPS_MIN  0.5f
#define BPS_MAX 16.f

/* Scale definitions: semitone offsets from root (C) */
static const int SCALE_MAJOR[]  = {0,2,4,5,7,9,11};
static const int SCALE_MINOR[]  = {0,2,3,5,7,8,10};
static const int SCALE_PENTA[]  = {0,2,4,7,9};
static const int SCALE_BLUES[]  = {0,3,5,6,7,10};
static const int SCALE_CHROM[]  = {0,1,2,3,4,5,6,7,8,9,10,11};

static const struct { const char *name; const int *notes; int len; } SCALES[] = {
    {"Major",      SCALE_MAJOR,  7},
    {"Minor",      SCALE_MINOR,  7},
    {"Pentatonic", SCALE_PENTA,  5},
    {"Blues",      SCALE_BLUES,  6},
    {"Chromatic",  SCALE_CHROM, 12},
};
#define N_SCALES 5
static int g_scale_idx = 0;

enum { CP_C0=1, CP_C1, CP_C2, CP_C3, CP_C4, CP_C5, CP_C6,
       CP_DIM, CP_BELL, CP_HUD };

/* ===================================================================== */
/* §2  clock                                                              */
/* ===================================================================== */

static long long clock_ns(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (long long)ts.tv_sec * 1000000000LL + ts.tv_nsec;
}

static void clock_sleep_ns(long long ns)
{
    if (ns <= 0) return;
    struct timespec ts = { ns / 1000000000LL, ns % 1000000000LL };
    nanosleep(&ts, NULL);
}

/* ===================================================================== */
/* §3  color                                                              */
/* ===================================================================== */

static void color_init(void)
{
    start_color();
    use_default_colors();
    if (COLORS >= 256) {
        /* 7 rainbow pitch colors */
        init_pair(CP_C0, 196, -1);  /* red         */
        init_pair(CP_C1, 208, -1);  /* orange      */
        init_pair(CP_C2, 226, -1);  /* yellow      */
        init_pair(CP_C3,  46, -1);  /* green       */
        init_pair(CP_C4,  51, -1);  /* cyan        */
        init_pair(CP_C5,  27, -1);  /* blue        */
        init_pair(CP_C6, 129, -1);  /* purple      */
        init_pair(CP_DIM, 238, -1); /* dim dead    */
        init_pair(CP_BELL,231,-1);  /* bright bell */
        init_pair(CP_HUD, 244, -1);
    } else {
        init_pair(CP_C0, COLOR_RED,    -1);
        init_pair(CP_C1, COLOR_RED,    -1);
        init_pair(CP_C2, COLOR_YELLOW, -1);
        init_pair(CP_C3, COLOR_GREEN,  -1);
        init_pair(CP_C4, COLOR_CYAN,   -1);
        init_pair(CP_C5, COLOR_BLUE,   -1);
        init_pair(CP_C6, COLOR_MAGENTA,-1);
        init_pair(CP_DIM,  COLOR_WHITE,-1);
        init_pair(CP_BELL, COLOR_WHITE,-1);
        init_pair(CP_HUD,  COLOR_WHITE,-1);
    }
}

static int pitch_cp(int note_idx) { return CP_C0 + (note_idx % 7); }

/* ===================================================================== */
/* §4  cellular automaton                                                 */
/* ===================================================================== */

static int g_ca_w;                          /* actual CA width this session */
static unsigned char g_ca[HISTORY+1][CA_W_MAX]; /* rows; 0=dead 1=alive     */
static int g_ca_head = 0;                   /* index of newest row          */

/* Apply rule to generate next row */
static void ca_step(void)
{
    int next = (g_ca_head + 1) % (HISTORY + 1);
    const unsigned char *cur = g_ca[g_ca_head];
    unsigned char       *nxt = g_ca[next];
    for (int c = 0; c < g_ca_w; c++) {
        int l = (c > 0)        ? cur[c-1] : cur[g_ca_w-1];  /* wrap */
        int m = cur[c];
        int r = (c < g_ca_w-1) ? cur[c+1] : cur[0];
        int idx = (l << 2) | (m << 1) | r;
        nxt[c] = (g_rule >> idx) & 1;
    }
    g_ca_head = next;
}

static void ca_seed_random(void)
{
    for (int c = 0; c < g_ca_w; c++)
        g_ca[g_ca_head][c] = rand() & 1;
}

static void ca_seed_single(void)
{
    memset(g_ca[g_ca_head], 0, (size_t)g_ca_w);
    g_ca[g_ca_head][g_ca_w/2] = 1;
}

static void ca_init(bool random_seed)
{
    g_ca_head = 0;
    for (int r = 0; r <= HISTORY; r++)
        memset(g_ca[r], 0, (size_t)g_ca_w);
    if (random_seed) ca_seed_random();
    else             ca_seed_single();
}

/* ===================================================================== */
/* §5  music                                                              */
/* ===================================================================== */

static int   g_beat_col = 0;  /* which CA column the beat cursor is on */
static bool  g_beat_on  = false;
static long long g_next_beat_ns = 0;
static int   g_last_note = -1; /* last note played (-1 = rest) */

/* Map CA cell → musical note index (in current scale) */
static int cell_to_note(int alive, int col)
{
    if (!alive) return -1;  /* rest */
    int n = SCALES[g_scale_idx].len;
    return col % n;
}

/* Attempt to ring the bell and record note */
static void beat_tick(void)
{
    const unsigned char *row = g_ca[g_ca_head];
    int alive = row[g_beat_col];
    int note  = cell_to_note(alive, g_beat_col);

    if (note >= 0) {
        /* Ring terminal bell for live cells */
        putchar('\a');
        fflush(stdout);
    }

    g_last_note = note;
    g_beat_on   = true;

    /* Advance cursor */
    g_beat_col = (g_beat_col + 1) % g_ca_w;

    /* Advance CA every full sweep */
    if (g_beat_col == 0) ca_step();
}

/* ===================================================================== */
/* §6  draw                                                               */
/* ===================================================================== */

static int  g_rows, g_cols;
static bool g_paused;

static void scene_draw(void)
{
    int draw_rows = g_rows - HUD_ROWS;
    int show = draw_rows < HISTORY ? draw_rows : HISTORY;

    /* Draw CA history: newest at bottom, oldest at top */
    for (int age = 0; age < show; age++) {
        int row = g_rows - 1 - age;
        int ridx = (g_ca_head - age + HISTORY + 1) % (HISTORY + 1);
        const unsigned char *line = g_ca[ridx];

        /* Scale CA width to terminal width */
        for (int col = 0; col < g_cols; col++) {
            int ci = col * g_ca_w / g_cols;
            if (ci >= g_ca_w) ci = g_ca_w - 1;

            /* Beat cursor highlight */
            bool is_cursor = (ci == g_beat_col && age == 0);

            if (line[ci]) {
                int n = cell_to_note(1, ci);
                int cp = (is_cursor && g_beat_on) ? CP_BELL : pitch_cp(n);
                chtype ch = (age == 0) ? '#' : (age < 5) ? '+' : '.';
                attron(COLOR_PAIR(cp) | (age < 3 ? A_BOLD : 0));
                mvaddch(row, col, ch);
                attroff(COLOR_PAIR(cp) | A_BOLD);
            } else if (is_cursor) {
                attron(COLOR_PAIR(CP_DIM));
                mvaddch(row, col, '|');
                attroff(COLOR_PAIR(CP_DIM));
            }
        }
    }

    /* Note name display */
    char note_str[24] = "rest";
    if (g_last_note >= 0) {
        static const char *NOTE_NAMES[] = {"C","C#","D","Eb","E","F","F#","G","Ab","A","Bb","B"};
        int semitone = SCALES[g_scale_idx].notes[g_last_note];
        snprintf(note_str, sizeof note_str, "%s (scale°%d)", NOTE_NAMES[semitone], g_last_note+1);
    }
    g_beat_on = false;

    attron(COLOR_PAIR(CP_HUD));
    mvprintw(0, 0,
        " CA-Music  q:quit  p:pause  r:reset  spc:randomize  t/T:tempo  s:scale  1-8:rule");
    mvprintw(1, 0,
        " Rule=%d  tempo=%.1f BPM  scale=%s  note=%s",
        g_rule, (double)g_bps*60.f,
        SCALES[g_scale_idx].name, note_str);
    mvprintw(2, 0,
        " Colors=pitch  #=live beat  dim=cursor  scrolls upward");
    attroff(COLOR_PAIR(CP_HUD));
}

/* ===================================================================== */
/* §7  app                                                                */
/* ===================================================================== */

static volatile sig_atomic_t g_quit   = 0;
static volatile sig_atomic_t g_resize = 0;

static void sig_h(int s)
{
    if (s == SIGINT || s == SIGTERM) g_quit   = 1;
    if (s == SIGWINCH)               g_resize = 1;
}

static void cleanup(void) { endwin(); }

int main(void)
{
    srand((unsigned)time(NULL));
    atexit(cleanup);
    signal(SIGINT, sig_h); signal(SIGTERM, sig_h); signal(SIGWINCH, sig_h);

    initscr(); cbreak(); noecho();
    keypad(stdscr, TRUE); nodelay(stdscr, TRUE);
    curs_set(0); typeahead(-1);
    color_init();
    getmaxyx(stdscr, g_rows, g_cols);

    g_ca_w = g_cols < CA_W_MAX ? g_cols : CA_W_MAX;
    ca_init(false);

    g_next_beat_ns = clock_ns();

    while (!g_quit) {

        if (g_resize) {
            g_resize = 0;
            endwin(); refresh();
            getmaxyx(stdscr, g_rows, g_cols);
            g_ca_w = g_cols < CA_W_MAX ? g_cols : CA_W_MAX;
            ca_init(false);
        }

        int ch = getch();
        switch (ch) {
        case 'q': case 'Q': case 27: g_quit = 1; break;
        case 'p': case 'P': g_paused = !g_paused; break;
        case 'r': case 'R': ca_init(false); g_beat_col=0; break;
        case ' ': ca_seed_random(); g_beat_col=0; break;
        case 't':
            g_bps *= 0.833f; if (g_bps < BPS_MIN) g_bps = BPS_MIN; break;
        case 'T':
            g_bps *= 1.2f; if (g_bps > BPS_MAX) g_bps = BPS_MAX; break;
        case 's':
            g_scale_idx = (g_scale_idx + 1) % N_SCALES; break;
        case '1': case '2': case '3': case '4':
        case '5': case '6': case '7': case '8':
            g_rule_idx = ch - '1';
            g_rule = RULE_LIST[g_rule_idx];
            ca_init(false); g_beat_col=0;
            break;
        default: break;
        }

        long long now = clock_ns();

        if (!g_paused) {
            /* Beat trigger based on tempo */
            long long beat_ns = (long long)(1e9f / g_bps);
            if (now >= g_next_beat_ns) {
                beat_tick();
                g_next_beat_ns = now + beat_ns;
            }
        }

        erase();
        scene_draw();
        wnoutrefresh(stdscr);
        doupdate();
        clock_sleep_ns(RENDER_NS - (clock_ns() - now));
    }
    return 0;
}
