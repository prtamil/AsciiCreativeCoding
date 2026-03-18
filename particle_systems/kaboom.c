/*
 * kaboom.c  —  ncurses ASCII blast / kaboom effect
 *
 * Original algorithm by the kaboom.c author, rewritten in the
 * fireworks/matrix_rain framework:
 *   - Single stdscr, ncurses internal double buffer — no flicker
 *   - typeahead(-1) — no mid-flush input polling, no tearing
 *   - HUD written into stdscr after blast (always on top)
 *   - dt (delta-time) loop drives playback speed independently of CPU, render capped at 60 fps
 *   - SIGWINCH resize: rebuilds scene + restarts blast
 *   - Speed control:   ] = faster   [ = slower
 *   - Restart:         r = replay from frame 0
 *   - Clean signal / atexit teardown — terminal always restored
 *
 * Keys:
 *   q / ESC   quit
 *   ]  [      speed up / slow down
 *   r         replay
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra kaboom.c -o kaboom -lncurses -lm
 *
 * Sections
 * --------
 *   §1  config  — every tunable constant in one block
 *   §2  clock   — monotonic nanosecond clock, portable sleep
 *   §3  color   — color pairs; 256-color with 8-color fallback
 *   §4  blob    — one 3-D debris particle (original algorithm)
 *   §5  blast   — frame buffer + blob pool + tick + draw
 *   §6  screen  — single stdscr, ncurses internal double buffer
 *   §7  app     — dt loop, input, resize, cleanup
 */

#define _POSIX_C_SOURCE 200809L

#ifndef M_PI
#  define M_PI 3.14159265358979323846
#endif
#ifndef M_1_PI
#  define M_1_PI (1.0 / M_PI)
#endif

#include <math.h>
#include <ncurses.h>
#include <signal.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <stdio.h>

/* ===================================================================== */
/* §1  config                                                             */
/* ===================================================================== */

enum {
    SIM_FPS_MIN     =  5,
    SIM_FPS_DEFAULT = 30,
    SIM_FPS_MAX     = 60,
    SIM_FPS_STEP    =  5,

    NUM_FRAMES      = 150,
    NUM_BLOBS       = 800,

    HUD_COLS        =  28,
    FPS_UPDATE_MS   = 500,
};

#define NS_PER_SEC    1000000000LL
#define NS_PER_MS     1000000LL
#define TICK_NS(fps)  (NS_PER_SEC / (fps))

#define PERSPECTIVE   50.0

/* ===================================================================== */
/* §2  clock                                                              */
/* ===================================================================== */

static int64_t clock_ns(void)
{
    struct timespec t;
    clock_gettime(CLOCK_MONOTONIC, &t);
    return (int64_t)t.tv_sec * NS_PER_SEC + t.tv_nsec;
}

static void clock_sleep_ns(int64_t ns)
{
    if (ns <= 0) return;
    struct timespec req = {
        .tv_sec  = (time_t)(ns / NS_PER_SEC),
        .tv_nsec = (long)  (ns % NS_PER_SEC),
    };
    nanosleep(&req, NULL);
}

/* ===================================================================== */
/* §3  color                                                              */
/* ===================================================================== */

typedef enum {
    COL_FLASH  = 1,
    COL_INNER  = 2,
    COL_WAVE   = 3,
    COL_BLOB_F = 4,
    COL_BLOB_M = 5,
    COL_BLOB_N = 6,
    COL_HUD    = 7,
} ColorID;

/*
 * BlastTheme — one colour set for a full blast cycle.
 *
 * Each theme has 5 blast colours (FLASH, INNER, WAVE, BLOB_F, BLOB_M,
 * BLOB_N) in 256-color xterm indices, plus 8-color fallbacks.
 * A theme name appears in the HUD so you can see which is active.
 *
 * Themes:
 *   0 fire    — classic orange/red (original look)
 *   1 ice     — white core, cyan body, blue shockwave
 *   2 poison  — white flash, bright green inner, dark green wave
 *   3 plasma  — white flash, magenta inner, purple wave
 *   4 gold    — white flash, bright yellow, dark amber wave
 *   5 blood   — white flash, red inner, dark crimson wave
 */
typedef struct {
    const char *name;
    int flash, inner, wave, blob_f, blob_m, blob_n;    /* 256-color    */
    int f8_flash, f8_inner, f8_wave, f8_bm, f8_bn;     /* 8-color      */
} BlastTheme;

static const BlastTheme k_themes[] = {
    { "fire",   231, 214,  94, 250, 220, 196,
      COLOR_WHITE, COLOR_YELLOW, COLOR_RED,    COLOR_YELLOW, COLOR_RED    },
    { "ice",    231,  51,  21, 195, 123,  27,
      COLOR_WHITE, COLOR_CYAN,   COLOR_BLUE,   COLOR_CYAN,   COLOR_BLUE   },
    { "poison", 231,  82,  28, 193, 118,  34,
      COLOR_WHITE, COLOR_GREEN,  COLOR_GREEN,  COLOR_GREEN,  COLOR_GREEN  },
    { "plasma", 231, 201,  93, 225, 171, 129,
      COLOR_WHITE, COLOR_MAGENTA,COLOR_MAGENTA,COLOR_MAGENTA,COLOR_MAGENTA},
    { "gold",   231, 226, 136, 229, 214, 130,
      COLOR_WHITE, COLOR_YELLOW, COLOR_YELLOW, COLOR_YELLOW, COLOR_RED    },
    { "blood",  231, 196,  88, 210, 160,  52,
      COLOR_WHITE, COLOR_RED,    COLOR_RED,    COLOR_RED,    COLOR_RED    },
};

#define THEME_COUNT (int)(sizeof k_themes / sizeof k_themes[0])

/*
 * color_theme_apply() — reload blast color pairs for one theme.
 * Safe to call mid-run; takes effect on the next rendered frame.
 * COL_HUD stays yellow in every theme for consistent readability.
 */
static void color_theme_apply(int t)
{
    const BlastTheme *th = &k_themes[t];
    if (COLORS >= 256) {
        init_pair(COL_FLASH,  th->flash,  COLOR_BLACK);
        init_pair(COL_INNER,  th->inner,  COLOR_BLACK);
        init_pair(COL_WAVE,   th->wave,   COLOR_BLACK);
        init_pair(COL_BLOB_F, th->blob_f, COLOR_BLACK);
        init_pair(COL_BLOB_M, th->blob_m, COLOR_BLACK);
        init_pair(COL_BLOB_N, th->blob_n, COLOR_BLACK);
        init_pair(COL_HUD,    226,        COLOR_BLACK);
    } else {
        init_pair(COL_FLASH,  th->f8_flash, COLOR_BLACK);
        init_pair(COL_INNER,  th->f8_inner, COLOR_BLACK);
        init_pair(COL_WAVE,   th->f8_wave,  COLOR_BLACK);
        init_pair(COL_BLOB_F, th->f8_flash, COLOR_BLACK);
        init_pair(COL_BLOB_M, th->f8_bm,   COLOR_BLACK);
        init_pair(COL_BLOB_N, th->f8_bn,   COLOR_BLACK);
        init_pair(COL_HUD,    COLOR_YELLOW, COLOR_BLACK);
    }
}

static void color_init(int theme)
{
    start_color();
    color_theme_apply(theme);
}

/* ===================================================================== */
/* §4  blob                                                               */
/* ===================================================================== */

typedef struct {
    double x, y, z;
} Blob;

static double prng(void)
{
    static long long s = 1;
    s = s * 1488248101LL + 981577151LL;
    return ((s % 65536) - 32768) / 32768.0;
}

static void blob_init_pool(Blob *blobs)
{
    for (int i = 0; i < NUM_BLOBS; i++) {
        double bx = prng();
        double by = prng();
        double bz = prng();
        double br = sqrt(bx*bx + by*by + bz*bz);
        blobs[i].x = (bx / br) * (1.3 + 0.2 * prng());
        blobs[i].y = (0.5 * by / br) * (1.3 + 0.2 * prng());
        blobs[i].z = (bz / br) * (1.3 + 0.2 * prng());
    }
}

/*
 * BlastShape — per-cycle physical parameters that change the look.
 *
 * petal_n      cos(petal_n * atan2(...)) — number of symmetry lobes.
 *              4 = cross, 6 = hex star, 8 = classic, 16 = spiky ring,
 *              1 = asymmetric teardrop, 0 = smooth sphere (no petals).
 * ripple       amplitude of the angular ripple (0 = smooth, 0.5 = very jagged)
 * disc_speed   how fast the initial disc expands (1.5 = slow, 3.0 = fast)
 * y_squash     vertical squash of the blob cloud (0.3 = flat disc, 1.0 = sphere,
 *              1.8 = tall column)
 * persp        perspective depth of the blob cloud (20 = flat, 80 = deep 3D)
 * blob_speed   how fast blobs fly outward (multiplied against i0)
 * flash_chars  character sequence for the initial fireball phase
 * wave_chars   character sequence for the expanding shockwave
 * name         shown in HUD
 */
typedef struct {
    const char *name;
    double      petal_n;     /* angular frequency of lobes              */
    double      ripple;      /* lobe amplitude                          */
    double      disc_speed;  /* initial disc expansion rate             */
    double      y_squash;    /* blob cloud vertical scale               */
    double      persp;       /* blob perspective depth                  */
    double      blob_speed;  /* blob outward velocity scale             */
    const char *flash_chars; /* chars for inner fireball (frame 8-18)  */
    const char *wave_chars;  /* chars for shockwave (20 levels)         */
} BlastShape;

static const BlastShape k_shapes[] = {
    {   /* 0  classic — original algorithm */
        "classic",
        16.0, 0.3, 2.0, 0.5, 50.0, 1.0,
        "T%@W#H=+~-:.",
        " .:!HIOMW#%$&@08O=+-"
    },
    {   /* 1  star — 6-pointed, slow thick wave, tall blob column */
        "star",
        6.0,  0.45, 1.5, 1.6, 35.0, 0.8,
        "*+oO0@#%&$!^~",
        " `.-:=+*oO0#@%$"
    },
    {   /* 2  ring — smooth sphere, fast thin ring, flat disc blobs */
        "ring",
        0.0,  0.0,  3.0, 0.3, 70.0, 1.4,
        "o0OQ@#%&$()[]{}",
        " .,:;!|/\\-=+~*oO"
    },
    {   /* 3  cross — 4 lobes, medium speed, medium depth */
        "cross",
        4.0,  0.5,  2.5, 0.8, 45.0, 1.1,
        "#@WMH+|=~-:.",
        " :-=+|H#@WM0O%$"
    },
    {   /* 4  nova — 12 lobes, very fast, deep 3D blobs */
        "nova",
        12.0, 0.35, 3.5, 1.0, 80.0, 1.6,
        "%$&#@!*+~-:.",
        " .`'^-~=+*#@$%&!"
    },
    {   /* 5  pulse — asymmetric teardrop, slow, very flat blobs */
        "pulse",
        3.0,  0.6,  1.2, 0.25, 25.0, 0.7,
        "~-:.+=#@*oO0Q",
        " ..,::==++##@@%%"
    },
};

#define SHAPE_COUNT (int)(sizeof k_shapes / sizeof k_shapes[0])

/* ===================================================================== */
/* §5  blast                                                              */
/* ===================================================================== */

typedef struct {
    char    ch;
    ColorID color;
} Cell;

typedef struct {
    Blob  blobs[NUM_BLOBS];
    Cell *cells;
    int   cols;
    int   rows;
    int   frame;
    int   theme;    /* index into k_themes — changes each cycle        */
    int   shape;    /* index into k_shapes — changes each cycle        */
    bool  done;
} Blast;

static void blast_alloc_cells(Blast *b)
{
    b->cells = calloc((size_t)(b->cols * b->rows), sizeof(Cell));
}

static void blast_init(Blast *b, int cols, int rows, int theme, int shape)
{
    b->cols  = cols;
    b->rows  = rows;
    b->frame = 0;
    b->theme = theme;
    b->shape = shape;
    b->done  = false;
    blast_alloc_cells(b);
    blob_init_pool(b->blobs);
    color_theme_apply(theme);
}

static void blast_free(Blast *b)
{
    free(b->cells);
    *b = (Blast){0};
}

static void blast_render_frame(Blast *b)
{
    const int cols  = b->cols;
    const int rows  = b->rows;
    const int frame = b->frame;
    const BlastShape *sh = &k_shapes[b->shape];

    const int minx = -(cols / 2);
    const int maxx = cols + minx - 1;
    const int miny = -(rows / 2);
    const int maxy = rows + miny - 1;

    const int flash_len = (int)strlen(sh->flash_chars);
    const int wave_len  = (int)strlen(sh->wave_chars);

    Cell *p = b->cells;

    for (int y = miny; y <= maxy; y++) {
        for (int x = minx; x <= maxx; x++) {

            p->ch    = 0;
            p->color = COL_WAVE;

            if (frame == 0) {
                if (x == 0 && y == 0) {
                    p->ch    = '*';
                    p->color = COL_FLASH;
                }

            } else if (frame < 8) {
                /* Initial disc — speed controlled by disc_speed */
                double r = sqrt((double)(x*x) + 4.0*(double)(y*y));
                if (r < frame * sh->disc_speed) {
                    p->ch    = '@';
                    p->color = COL_FLASH;
                }

            } else {
                /*
                 * Blast wave — shape controlled by petal_n and ripple.
                 * petal_n == 0 means a smooth sphere (no angular modulation).
                 */
                double angle = atan2(y * 2.0 + 0.01, x + 0.01);
                double lobe  = (sh->petal_n > 0.0)
                             ? (1.0 + sh->ripple * cos(sh->petal_n * angle))
                             : 1.0;
                double r = sqrt((double)(x*x) + 4.0*(double)(y*y))
                         * (0.5 + (prng() / 3.0) * lobe * 0.3);

                int v = frame - (int)r - 7;

                if (v < 0) {
                    if (frame < 8 + flash_len) {
                        int fi = frame - 8;
                        if (fi >= 0 && fi < flash_len) {
                            p->ch    = sh->flash_chars[fi];
                            p->color = COL_INNER;
                        }
                    }
                } else if (v < wave_len) {
                    p->ch    = sh->wave_chars[v];
                    p->color = (v < wave_len / 2) ? COL_INNER : COL_WAVE;
                }
            }

            p++;
        }
    }

    /* 3-D debris blobs — y_squash and persp vary per shape */
    if (frame > 6) {
        const double persp  = sh->persp;
        const double bspeed = sh->blob_speed;
        const int    i0     = frame - 6;

        for (int j = 0; j < NUM_BLOBS; j++) {
            double bx = b->blobs[j].x * i0 * bspeed;
            double by = b->blobs[j].y * i0 * bspeed * sh->y_squash;
            double bz = b->blobs[j].z * i0 * bspeed;

            if (bz < 5.0 - persp || bz > persp) continue;

            int cx = cols / 2 + (int)(bx * persp / (bz + persp));
            int cy = rows / 2 + (int)(by * persp / (bz + persp));

            if (cx < 0 || cx >= cols || cy < 0 || cy >= rows) continue;

            Cell *c  = &b->cells[cy * cols + cx];
            c->color = (bz > persp * 0.8)  ? COL_BLOB_F
                     : (bz > -persp * 0.4)  ? COL_BLOB_M
                     :                         COL_BLOB_N;
            c->ch    = (bz > persp * 0.8) ? '.'
                     : (bz > -persp * 0.4) ? 'o'
                     :                        '@';
        }
    }
}

static bool blast_tick(Blast *b)
{
    if (b->done) return false;

    blast_render_frame(b);
    b->frame++;

    if (b->frame >= NUM_FRAMES) {
        b->done = true;
        return false;
    }

    return true;
}

/*
 * blast_draw now takes WINDOW* so it works with stdscr or any window.
 */
static void blast_draw(const Blast *b, WINDOW *w)
{
    const int cols  = b->cols;
    const int rows  = b->rows;
    const int total = cols * rows;

    for (int i = 0; i < total; i++) {
        Cell c = b->cells[i];
        if (!c.ch) continue;

        int y = i / cols;
        int x = i % cols;
        if (x >= b->cols || y >= b->rows) continue;

        attr_t attr = COLOR_PAIR(c.color);
        if (c.color == COL_FLASH) attr |= A_BOLD;

        wattron(w, attr);
        mvwaddch(w, y, x, (chtype)(unsigned char)c.ch);
        wattroff(w, attr);
    }
}

/* ===================================================================== */
/* §6  screen                                                             */
/* ===================================================================== */

/*
 * Screen — single stdscr, ncurses' internal double buffer.
 *
 * erase()             — clear newscr (back buffer), no terminal I/O
 * blast_draw(stdscr)  — write blast into newscr
 * mvprintw / attron   — write HUD into newscr after blast (on top)
 * wnoutrefresh()      — mark newscr ready, still no terminal I/O
 * doupdate()          — ONE atomic write: diff newscr vs curscr → terminal
 *
 * typeahead(-1) prevents ncurses interrupting output mid-flush to poll
 * stdin, eliminating tearing at high tick rates.
 */
typedef struct {
    int cols;
    int rows;
} Screen;

static void screen_init(Screen *s)
{
    initscr();
    noecho();
    cbreak();
    curs_set(0);
    nodelay(stdscr, TRUE);
    keypad(stdscr, TRUE);
    typeahead(-1);
    color_init(0);
    getmaxyx(stdscr, s->rows, s->cols);
}

static void screen_free(Screen *s)
{
    (void)s;
    endwin();
}

static void screen_resize(Screen *s)
{
    endwin();
    refresh();
    getmaxyx(stdscr, s->rows, s->cols);
}

static void screen_draw_blast(Screen *s, const Blast *b)
{
    erase();
    blast_draw(b, stdscr);
}

static void screen_draw_hud(Screen *s, double fps, int sim_fps,
                              int frame, int theme, int shape)
{
    char buf[HUD_COLS + 1];
    snprintf(buf, sizeof buf,
             "%4.1f fps [%s/%s] %d/%d",
             fps, k_themes[theme].name, k_shapes[shape].name,
             frame, NUM_FRAMES);

    int hx = s->cols - HUD_COLS;
    if (hx < 0) hx = 0;
    attron(COLOR_PAIR(COL_HUD) | A_BOLD);
    mvprintw(0, hx, "%s", buf);
    attroff(COLOR_PAIR(COL_HUD) | A_BOLD);
}

static void screen_present(void)
{
    wnoutrefresh(stdscr);
    doupdate();
}

/* ===================================================================== */
/* §7  app                                                                */
/* ===================================================================== */

typedef struct {
    Blast                 blast;
    Screen                screen;
    int                   sim_fps;
    int                   cycle;      /* increments each restart → next theme */
    volatile sig_atomic_t running;
    volatile sig_atomic_t need_resize;
} App;

static App g_app;

static void on_exit_signal(int sig)   { (void)sig; g_app.running = 0;     }
static void on_resize_signal(int sig) { (void)sig; g_app.need_resize = 1; }
static void cleanup(void)             { endwin(); }

static void app_do_resize(App *app)
{
    blast_free(&app->blast);
    screen_resize(&app->screen);
    blast_init(&app->blast, app->screen.cols, app->screen.rows,
               app->cycle % THEME_COUNT,
               app->cycle % SHAPE_COUNT);
    app->need_resize = 0;
}

static bool app_handle_key(App *app, int ch)
{
    switch (ch) {
    case 'q': case 'Q': case 27: return false;

    case ']':
        app->sim_fps += SIM_FPS_STEP;
        if (app->sim_fps > SIM_FPS_MAX) app->sim_fps = SIM_FPS_MAX;
        break;
    case '[':
        app->sim_fps -= SIM_FPS_STEP;
        if (app->sim_fps < SIM_FPS_MIN) app->sim_fps = SIM_FPS_MIN;
        break;

    case 'r': case 'R':
        app->cycle++;
        blast_free(&app->blast);
        blast_init(&app->blast, app->screen.cols, app->screen.rows,
                   app->cycle % THEME_COUNT,
                   app->cycle % SHAPE_COUNT);
        break;

    default: break;
    }
    return true;
}

int main(void)
{
    srand((unsigned int)clock_ns());

    atexit(cleanup);
    signal(SIGINT,   on_exit_signal);
    signal(SIGTERM,  on_exit_signal);
    signal(SIGWINCH, on_resize_signal);

    App *app     = &g_app;
    app->running = 1;
    app->sim_fps = SIM_FPS_DEFAULT;
    app->cycle   = 0;

    screen_init(&app->screen);
    blast_init(&app->blast, app->screen.cols, app->screen.rows,
               app->cycle % THEME_COUNT,
               app->cycle % SHAPE_COUNT);

    int64_t frame_time  = clock_ns();
    int64_t sim_accum   = 0;
    int64_t fps_accum   = 0;
    int     frame_count = 0;
    double  fps_display = 0.0;

    while (app->running) {

        /* ── resize ──────────────────────────────────────────────── */
        if (app->need_resize) {
            app_do_resize(app);
            frame_time = clock_ns();
            sim_accum  = 0;
        }

        /* ── dt ──────────────────────────────────────────────────── */
        int64_t now = clock_ns();
        int64_t dt  = now - frame_time;
        frame_time  = now;
        if (dt > 100 * NS_PER_MS) dt = 100 * NS_PER_MS;

        /* ── sim accumulator ─────────────────────────────────────── */
        int64_t tick_ns = TICK_NS(app->sim_fps);
        sim_accum += dt;
        while (sim_accum >= tick_ns) {
            if (!blast_tick(&app->blast)) {
                app->cycle++;
                blast_free(&app->blast);
                blast_init(&app->blast, app->screen.cols, app->screen.rows,
                           app->cycle % THEME_COUNT,
                           app->cycle % SHAPE_COUNT);
            }
            sim_accum -= tick_ns;
        }
        float alpha = (float)sim_accum / (float)tick_ns;
        (void)alpha;

        /* ── HUD counter ─────────────────────────────────────────── */
        frame_count++;
        fps_accum += dt;
        if (fps_accum >= FPS_UPDATE_MS * NS_PER_MS) {
            fps_display = (double)frame_count
                        / ((double)fps_accum / (double)NS_PER_SEC);
            frame_count = 0;
            fps_accum   = 0;
        }

        /* ── frame cap (sleep BEFORE render so I/O doesn't drift) ── */
        int64_t elapsed = clock_ns() - frame_time + dt;
        int64_t budget  = NS_PER_SEC / 60;
        clock_sleep_ns(budget - elapsed);

        /* ── render + HUD ────────────────────────────────────────── */
        screen_draw_blast(&app->screen, &app->blast);
        screen_draw_hud(&app->screen, fps_display,
                         app->sim_fps, app->blast.frame,
                         app->blast.theme, app->blast.shape);
        screen_present();

        /* ── input ───────────────────────────────────────────────── */
        int ch = getch();
        if (ch != ERR && !app_handle_key(app, ch))
            app->running = 0;
    }

    blast_free(&app->blast);
    screen_free(&app->screen);
    return 0;
}
