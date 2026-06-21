/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * phoenix.c — an owl-shaped phoenix that bursts into flame and is reborn
 * from the ash, in an endless loop. The bird is a swarm of glowing
 * particles that either trace an owl outline or fly free as fire.
 * Fire look borrows from Reeves particle systems (SIGGRAPH 1983) and the
 * temperature-to-glyph ramp from Bourke's grey-scale character mapping.
 */

#define _POSIX_C_SOURCE 200809L
#include <math.h>
#include <ncurses.h>
#include <signal.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* ── §1 config — sizes, phase timings, fire physics, palette IDs ── */

#define TARGET_FPS         60

/* Particle pools. */
#define N_BODY_MAX         600
#define N_BODY_DEFAULT     360
#define N_BODY_MIN         150
#define N_BODY_STEP         40

#define N_SPARK_MAX        400
#define N_SPARK_DEFAULT    220
#define N_SPARK_MIN         60
#define N_SPARK_STEP        30

#define N_ANCHOR_MAX       80

/* Terminal cells are taller than wide, so a circle drawn 1:1 looks
 * squashed. Multiply x by this when drawing so the owl reads round. */
#define ASPECT_X           2.0f

/* Owl shape, measured in cells from the head centre (+y points down). */
#define HEAD_CY           (-3.0f)
#define HEAD_RX            4.0f
#define HEAD_RY            3.0f
#define BODY_CY            ( 2.5f)
#define BODY_RX            3.0f
#define BODY_RY            3.5f
#define EYE_DX             2.0f
#define EYE_DY            (-3.0f)
#define BEAK_DY           (-1.4f)
#define EARTUFT_DX         3.4f
#define EARTUFT_DY        (-6.2f)

#define SEED_DX            0.0f
#define SEED_DY           (-3.0f)     /* where the reborn bird grows from */

/* How long each stage of the burn-and-rebirth cycle lasts (seconds). */
#define T_PERCH           12.0f
#define T_IGNITE           2.0f
#define T_BLAZE            3.0f
#define T_COLLAPSE         2.0f
#define T_ASH              4.0f
#define T_REBIRTH          3.0f

/* Body particle behaviour. */
#define BODY_JITTER_R      0.55f      /* random wobble around each anchor */
#define BREATH_HZ          0.25f      /* breathing speed, cycles/sec     */
#define BREATH_AMP         0.20f      /* tiny up-down bob while perched   */

/* Hot/cool temperatures the various burn stages aim particles toward. */
#define BLAZE_TEMP_MIN     0.85f
#define BLAZE_TEMP_MAX     1.00f
#define COLLAPSE_TEMP_HOT  1.00f
#define COLLAPSE_TEMP_COOL 0.30f

/* During COLLAPSE, how eagerly bound particles let go and become flying
 * embers. Higher = they scatter sooner; tuned so nearly all are loose by
 * the time the stage ends. */
#define COLLAPSE_RELEASE_RATE  6.0f

/* During REBIRTH, how eagerly flying embers get pulled back onto the owl
 * shape each second. Higher = the bird reforms faster. */
#define REBIRTH_CAPTURE_RATE   5.0f

/* How many sparks per second fly off, per burn stage. */
#define SPARK_HZ_PERCH       0.0f
#define SPARK_HZ_IGNITE     20.0f
#define SPARK_HZ_BLAZE     120.0f
#define SPARK_HZ_COLLAPSE   60.0f
#define SPARK_HZ_ASH         8.0f
#define SPARK_HZ_REBIRTH    45.0f      /* flames lick around the reforming owl */
#define SPARK_BURST_MAX     16          /* cap sparks emitted in one frame    */
#define REBIRTH_FLARE_FRAC   4         /* rebirth flare = n_spark / this      */

/* Spark physics. */
#define SPARK_LIFE_BASE     1.6f       /* how long a spark lives, seconds  */
#define SPARK_LIFE_VAR      0.5f
#define SPARK_TEMP_INIT     0.95f      /* sparks start near white-hot      */
#define SPARK_TEMP_VAR      0.10f
#define SPARK_VEL_BASE      4.0f      /* launch speed, cells/sec        */
#define SPARK_VEL_VAR       0.6f
#define SPARK_VEL_UP        2.5f      /* extra upward kick              */

/* Fire motion shared by loose body embers and sparks alike. */
#define FIRE_BUOYANCY      11.0f       /* how hard hot stuff floats up     */
#define FIRE_DRAG           1.4f       /* air resistance                   */
#define FIRE_COOL           0.85f      /* how fast things cool down        */
#define FIRE_SHEAR_AMP      6.0f       /* strength of the swirling wind    */
#define FIRE_SHEAR_HZ       1.5f       /* how fast that wind changes       */
#define FIRE_TEMP_FLOOR     0.04f      /* below this, a particle dies      */

/* Launch velocity when a body particle breaks loose during COLLAPSE. */
#define COLLAPSE_VEL_LATERAL  3.0f    /* sideways spread, ±cells/sec    */
#define COLLAPSE_VEL_DOWN     2.0f    /* baseline fall, cells/sec       */
#define COLLAPSE_VEL_UPVAR    3.5f    /* random kick (some fly upward)  */

#define DT_CAP_S           0.10f       /* cap one frame's time step, seconds */
#define N_THEMES           4

/* Colour pair IDs */
#define PAIR_HEAT_0   1
#define PAIR_HEAT_1   2
#define PAIR_HEAT_2   3
#define PAIR_HEAT_3   4
#define PAIR_HEAT_4   5
#define PAIR_HEAT_5   6
#define PAIR_PERCH    7
#define PAIR_HUD      8
#define PAIR_HINT     9

/* ── §2 performance — monotonic clock and sleep helpers ── */

static int64_t clock_ns(void)
{
    struct timespec t;
    clock_gettime(CLOCK_MONOTONIC, &t);
    return (int64_t)t.tv_sec * 1000000000LL + t.tv_nsec;
}

static void clock_sleep_ns(int64_t ns)
{
    if (ns <= 0) return;
    struct timespec t;
    t.tv_sec  = ns / 1000000000LL;
    t.tv_nsec = ns % 1000000000LL;
    nanosleep(&t, NULL);
}

/* ── §3 types — anchor template, particle pools, lifecycle, scene ── */

/* Anchor — one fixed point of the owl's outline. The owl is described as a
 * list of ~38 of these points rather than a drawn picture; a swarm of
 * particles snaps to them to make the shape appear. Listing the pose this way
 * is what makes igniting, dissolving, and regrowing the bird easy — the same
 * particles just stop or resume snapping (the anchor-template idea, also used
 * in ant_colony.c).
 *
 *   dx,dy     — position relative to the owl's centre, in cells. (x is scaled
 *               by ASPECT_X at draw time so the bird looks round.)
 *   weight    — how likely a particle is to pick this point; bigger means a
 *               denser cluster. Eyes ~5.0, plain body fill ~0.7.
 *   base_temp — resting heat, roughly 0.14 (coolest body) to 0.55 (glowing
 *               eyes); drives the resting feather colour.
 *   alive_at  — distance from the seed point, scaled 0 (at the seed) to 1
 *               (farthest). During rebirth a point only reappears once the
 *               growth reaches its alive_at, so the bird grows outward from
 *               the eyes. */
typedef struct {
    float dx, dy;      /* position relative to owl centre (cells)      */
    float weight;      /* how likely particles cluster here            */
    float base_temp;   /* resting heat, 0..1                           */
    float alive_at;    /* rebirth wake order, 0 = at seed, 1 = farthest */
} Anchor;

/* BodyParticle — one of the particles that make up the owl/fire.
 *
 * Every particle is in one of two modes. BOUND (released=false) snaps to an
 * anchor each frame, so the swarm traces the owl outline. FREE (released=true)
 * flies under the fire physics — a loose ember that broke off while burning
 * and drifts back during rebirth. The same particle just switches modes; that
 * single switch is what turns the owl into fire and back.
 *
 *   x,y        — position in world cells.
 *   vx,vy      — velocity; only meaningful while FREE.
 *   temp       — heat 0..1; picks the colour and glyph in BOTH modes (a
 *                perched owl is simply low-temp feather colours).
 *   anchor_idx — the anchor it last snapped to, while BOUND.
 *   released   — false = BOUND (tracing the owl), true = FREE (flying). */
typedef struct {
    float x, y;
    float vx, vy;
    float temp;
    int   anchor_idx;
    bool  released;
} BodyParticle;

/* Spark — a short-lived flying ember, kept in its own smaller pool.
 *
 * Body particles must always be able to return to the owl, so they never truly
 * die. Sparks can: they're the throwaway wisps thrown off the burning bird
 * that rise, cool, and wink out, plus the burst the owl emerges from at
 * rebirth. They use the same fire physics as a free body particle but with a
 * countdown so the slot frees up to be reused.
 *
 *   x,y    — position in world cells.
 *   vx,vy  — velocity in cells/sec.
 *   temp   — heat 0..1; cools every frame and picks the colour.
 *   life   — seconds remaining before it dies.
 *   active — false means this slot is empty and reusable. */
typedef struct {
    float x, y;
    float vx, vy;
    float temp;
    float life;
    bool  active;
} Spark;

/* PhoenixPhase — the stages of the burn-and-rebirth cycle, run in a fixed
 * loop: PERCH → IGNITE → BLAZE → COLLAPSE → ASH → REBIRTH → back to PERCH.
 * Each stage just decides whether particles snap to the owl or fly free and
 * how hot they get, so the whole animation is one simple counter (time in the
 * current stage versus that stage's length). */
typedef enum {
    PHX_PERCH = 0,
    PHX_IGNITE,
    PHX_BLAZE,
    PHX_COLLAPSE,
    PHX_ASH,
    PHX_REBIRTH,
    N_PHX_PHASES,
} PhoenixPhase;

/* Phoenix — the whole bird: a fixed owl outline plus the moving swarm and the
 * clock that drives the cycle. The outline (anchors) is built once and never
 * changes; only which particles snap to it, and how hot they are, varies over
 * time. Pools are fixed-size and never grown, so nothing allocates while
 * running.
 *
 *   ox,oy           — owl's centre in world cells.
 *   perch_y         — row the perch branch sits on.
 *   world_t         — total seconds since reset; drives the swirling wind.
 *   phase_t         — seconds spent in the current stage.
 *   phase           — current stage of the cycle.
 *   anchors,n_anchor— the owl outline and how many points it has.
 *   body,n_body     — body particle pool; only the first n_body are live
 *                     (',' '.' keys tune this).
 *   sparks,n_spark  — spark pool; only the first n_spark are live (';' '\''
 *                     keys tune this).
 *   spark_emit_acc  — leftover fractional spark count carried between frames
 *                     so a steady rate emits smoothly. */
typedef struct {
    float ox, oy;
    float perch_y;

    float world_t;
    float phase_t;
    PhoenixPhase phase;

    Anchor anchors[N_ANCHOR_MAX];
    int    n_anchor;

    BodyParticle body[N_BODY_MAX];
    int          n_body;
    Spark        sparks[N_SPARK_MAX];
    int          n_spark;

    float spark_emit_acc;
} Phoenix;

/* Scene — everything the program tracks in one bundle.
 *   rows,cols — terminal size.
 *   phx       — the phoenix itself.
 *   theme     — which colour palette is active.
 *   paused    — true while the animation is frozen.
 *   fps       — measured frame rate, shown in the HUD. */
typedef struct {
    int     rows, cols;
    Phoenix phx;
    int     theme;
    bool    paused;
    float   fps;
} Scene;

/* ── §4 logic — pure heat-to-colour mapping and per-phase lookups ── */

/* Turn a particle's heat into a colour, glyph, and emphasis. Cooler =
 * sparser dot and dimmer; hotter = denser character and bold, so the
 * white-hot core stands out. */
static void heat_attrs(float temp, short *pair, char *glyph, attr_t *attrs)
{
    static const char glyphs[6] = { '.', '+', '*', '#', '@', '%' };
    int b = (int)floorf(temp * 6.0f);
    if (b < 0) b = 0;
    if (b > 5) b = 5;
    *glyph = glyphs[b];
    *pair  = (short)(PAIR_HEAT_0 + b);
    *attrs = (b == 0) ? A_DIM : (b >= 4 ? A_BOLD : 0);
}

static const char *phase_name(PhoenixPhase p)
{
    switch (p) {
    case PHX_PERCH:    return "PERCH   ";
    case PHX_IGNITE:   return "IGNITE  ";
    case PHX_BLAZE:    return "BLAZE   ";
    case PHX_COLLAPSE: return "COLLAPSE";
    case PHX_ASH:      return "ASH     ";
    case PHX_REBIRTH:  return "REBIRTH ";
    default:           return "?       ";
    }
}

static float phase_duration(PhoenixPhase p)
{
    switch (p) {
    case PHX_PERCH:    return T_PERCH;
    case PHX_IGNITE:   return T_IGNITE;
    case PHX_BLAZE:    return T_BLAZE;
    case PHX_COLLAPSE: return T_COLLAPSE;
    case PHX_ASH:      return T_ASH;
    case PHX_REBIRTH:  return T_REBIRTH;
    default:           return 1.f;
    }
}

static float spark_hz_for_phase(PhoenixPhase p)
{
    switch (p) {
    case PHX_PERCH:    return SPARK_HZ_PERCH;
    case PHX_IGNITE:   return SPARK_HZ_IGNITE;
    case PHX_BLAZE:    return SPARK_HZ_BLAZE;
    case PHX_COLLAPSE: return SPARK_HZ_COLLAPSE;
    case PHX_ASH:      return SPARK_HZ_ASH;
    case PHX_REBIRTH:  return SPARK_HZ_REBIRTH;
    default:           return 0.f;
    }
}

/* ── §5 simulation — build the owl, run the fire physics and lifecycle ── */

static float frand(void)         { return (float)rand() / (float)RAND_MAX; }
static float frand_signed(void)  { return frand() * 2.f - 1.f; }

/* Place n anchors evenly around an ellipse. Returns the next free index. */
static int emit_ellipse(Anchor *out, int idx, int n, float cx, float cy,
                        float rx, float ry, float weight, float base_temp)
{
    for (int i = 0; i < n; i++) {
        float a = (float)i * (float)(2.0 * M_PI / n);
        out[idx].dx        = cx + rx * cosf(a);
        out[idx].dy        = cy + ry * sinf(a);
        out[idx].weight    = weight;
        out[idx].base_temp = base_temp;
        idx++;
    }
    return idx;
}

/* Place a single feature anchor (eye, beak, tuft). Returns the next index. */
static int emit_point(Anchor *out, int idx, float dx, float dy,
                      float weight, float base_temp)
{
    out[idx].dx        = dx;
    out[idx].dy        = dy;
    out[idx].weight    = weight;
    out[idx].base_temp = base_temp;
    return idx + 1;
}

/* Set each anchor's alive_at to its distance from the seed, scaled 0..1, so
 * rebirth fills the bird outward from the head (eyes first, ear tufts last). */
static void normalize_alive_at(Anchor *out, int n)
{
    float max_d = 0.f;
    for (int i = 0; i < n; i++) {
        float ddx = out[i].dx - SEED_DX;
        float ddy = out[i].dy - SEED_DY;
        float d   = sqrtf(ddx * ddx + ddy * ddy);
        if (d > max_d) max_d = d;
    }
    if (max_d < 0.001f) max_d = 1.f;
    for (int i = 0; i < n; i++) {
        float ddx = out[i].dx - SEED_DX;
        float ddy = out[i].dy - SEED_DY;
        float d   = sqrtf(ddx * ddx + ddy * ddy);
        out[i].alive_at = d / max_d;
    }
}

/* Build the owl outline into out[] and return how many anchors it holds. Two
 * rings sketch the head and body, a small inner ring fills it in, and a few
 * heavy feature points become the eyes, beak, and ear tufts. */
static int anchor_table_build(Anchor *out)
{
    int idx = 0;

    /* Head ring, body ring (taller than wide), and an inner fill ring. */
    idx = emit_ellipse(out, idx, 12, 0.f, HEAD_CY, HEAD_RX, HEAD_RY,
                       1.4f, 0.18f);
    idx = emit_ellipse(out, idx, 14, 0.f, BODY_CY, BODY_RX, BODY_RY,
                       1.2f, 0.16f);
    idx = emit_ellipse(out, idx,  5, 0.f, BODY_CY, BODY_RX * 0.45f, BODY_RY * 0.45f,
                       0.7f, 0.14f);

    /* Glowing eyes, a beak, and two ear tufts (heavier and hotter). */
    idx = emit_point(out, idx, -EYE_DX,     EYE_DY,     5.0f, 0.55f);
    idx = emit_point(out, idx,  EYE_DX,     EYE_DY,     5.0f, 0.55f);
    idx = emit_point(out, idx,  0.f,        BEAK_DY,    2.0f, 0.40f);
    idx = emit_point(out, idx, -EARTUFT_DX, EARTUFT_DY, 1.5f, 0.22f);
    idx = emit_point(out, idx,  EARTUFT_DX, EARTUFT_DY, 1.5f, 0.22f);

    normalize_alive_at(out, idx);
    return idx;
}

/* Pick a random anchor, with denser-weighted points chosen more often. */
static int anchor_pick(const Anchor *anch, int n)
{
    float total = 0.f;
    for (int i = 0; i < n; i++) total += anch[i].weight;
    float r = frand() * total;
    float acc = 0.f;
    for (int i = 0; i < n; i++) {
        acc += anch[i].weight;
        if (r <= acc) return i;
    }
    return n - 1;
}

/* Like anchor_pick, but only among points the rebirth has reached so far
 * (alive_at <= growth_frac). Returns -1 if none have woken yet. */
static int anchor_pick_alive(const Anchor *anch, int n, float growth_frac)
{
    float total = 0.f;
    for (int i = 0; i < n; i++)
        if (anch[i].alive_at <= growth_frac) total += anch[i].weight;
    if (total <= 0.f) return -1;
    float r = frand() * total;
    float acc = 0.f;
    int last_alive = -1;
    for (int i = 0; i < n; i++) {
        if (anch[i].alive_at <= growth_frac) {
            last_alive = i;
            acc += anch[i].weight;
            if (r <= acc) return i;
        }
    }
    return last_alive;
}

/* Snap a particle onto the given anchor (plus a little wobble) and mark it
 * bound. The caller picks the anchor, so this serves both the normal bind and
 * the rebirth alive-only bind. */
static void body_rebind_to(BodyParticle *p, const Anchor *anch, int idx,
                           float ox, float oy, float bob)
{
    p->anchor_idx = idx;
    float ax = ox + anch[idx].dx * ASPECT_X;
    float ay = oy + anch[idx].dy + bob;
    p->x = ax + frand_signed() * BODY_JITTER_R * ASPECT_X;
    p->y = ay + frand_signed() * BODY_JITTER_R;
    p->released = false;
}

/* Let a bound particle break loose into a flying ember. It keeps its current
 * spot and heat (so it looks like it crumbles, not teleports) and gets a
 * mostly-downward launch as the bird collapses onto the perch. */
static void body_release(BodyParticle *p)
{
    p->released = true;
    p->vx = frand_signed() * COLLAPSE_VEL_LATERAL;
    /* Mostly falling, but the random part lets a few embers shoot upward. */
    p->vy = COLLAPSE_VEL_DOWN + frand_signed() * COLLAPSE_VEL_UPVAR;
}

/* Move one flying ember forward by dt seconds: swirling wind, upward float
 * (stronger when hotter), air drag, then it cools. Shared by loose body
 * particles and sparks, same as fire_tornado.c / volcano.c. */
static void fire_step(float *x, float *y, float *vx, float *vy, float *temp,
                      float dt, float t)
{
    /* Swirling wind. The push depends on position so nearby embers get
     * different shoves, instead of the whole column swaying as one block. */
    float shx = sinf(FIRE_SHEAR_HZ * t + 0.30f * (*x) + 0.45f * (*y));
    float shy = cosf(FIRE_SHEAR_HZ * t + 0.45f * (*x) - 0.30f * (*y));
    *vx += FIRE_SHEAR_AMP * shx * dt;
    *vy += FIRE_SHEAR_AMP * shy * dt * 0.6f;

    /* Hot air rises (negative y is up). Even cool smoke gets a little lift. */
    float lift = FIRE_BUOYANCY * (0.20f + 0.80f * (*temp));
    *vy -= lift * dt;

    /* Air drag slows it down. */
    float k = 1.f - FIRE_DRAG * dt;
    if (k < 0.f) k = 0.f;
    *vx *= k;
    *vy *= k;

    *x += *vx * dt;
    *y += *vy * dt;

    *temp *= expf(-FIRE_COOL * dt);
    if (*temp < 0.f) *temp = 0.f;
}

/* Launch a fresh spark from (sx,sy), aimed mostly upward with some spread.
 * Life and heat are randomised so the rising trail doesn't look striped. */
static void spark_emit(Spark *s, float sx, float sy)
{
    float speed   = SPARK_VEL_BASE * (1.f + SPARK_VEL_VAR * frand_signed());
    float ang     = (float)(-M_PI / 2.0)               /* straight up */
                  + frand_signed() * 0.7f;             /* wobble ~40 degrees */
    s->x          = sx + frand_signed() * 0.6f;
    s->y          = sy + frand_signed() * 0.4f;
    s->vx         = cosf(ang) * speed;
    s->vy         = sinf(ang) * speed - SPARK_VEL_UP;
    s->temp       = SPARK_TEMP_INIT + frand_signed() * SPARK_TEMP_VAR;
    if (s->temp > 1.f) s->temp = 1.f;
    if (s->temp < 0.f) s->temp = 0.f;
    s->life       = SPARK_LIFE_BASE * (1.f + SPARK_LIFE_VAR * frand_signed());
    s->active     = true;
}

static void spark_tick(Spark *s, float dt, float t)
{
    if (!s->active) return;
    fire_step(&s->x, &s->y, &s->vx, &s->vy, &s->temp, dt, t);
    s->life -= dt;
    if (s->life <= 0.f || s->temp <= FIRE_TEMP_FLOOR) s->active = false;
}

/* Decide how hot a snapped-on particle should be right now. This is where each
 * stage gets its mood: resting feathers in PERCH, heating up through IGNITE,
 * white-hot in BLAZE, cooling through COLLAPSE, and easing from flame back to
 * feather in REBIRTH. frac is 0..1 progress through the current stage. */
static float bound_temp(PhoenixPhase phase, float frac, float base_temp,
                        float alive_at)
{
    switch (phase) {
    case PHX_PERCH:
        return base_temp + frand_signed() * 0.05f;
    case PHX_IGNITE: {
        /* Ramp from the resting feather heat up toward white-hot as the
         * stage progresses. */
        float t = base_temp + (1.0f - base_temp) * frac;
        return t + frand_signed() * 0.05f;
    }
    case PHX_BLAZE:
        return BLAZE_TEMP_MIN + (BLAZE_TEMP_MAX - BLAZE_TEMP_MIN) * frand();
    case PHX_COLLAPSE: {
        float t = COLLAPSE_TEMP_HOT
                + (COLLAPSE_TEMP_COOL - COLLAPSE_TEMP_HOT) * frac;
        return t + frand_signed() * 0.05f;
    }
    case PHX_REBIRTH: {
        /* Start each particle as flame (hottest near the seed) and let it cool
         * down to its resting feather colour as the bird finishes reforming,
         * so the owl emerges out of the fire instead of staying ablaze. */
        float hot = 0.95f - 0.30f * alive_at;
        float t   = hot + (base_temp - hot) * frac;
        return t + frand_signed() * 0.05f;
    }
    default:
        return base_temp;
    }
}

/* Set up a fresh bird: build the owl outline, place it on screen, and snap
 * every body particle onto an anchor so frame 0 already looks like an owl
 * rather than a cloud. Sparks all start empty. */
static void phoenix_init(Phoenix *p, int rows, int cols, int n_body, int n_spark)
{
    memset(p, 0, sizeof *p);
    p->ox       = (float)cols * 0.5f;
    p->oy       = (float)rows * 0.45f;
    p->perch_y  = (float)rows * 0.45f + (BODY_CY + BODY_RY + 1.0f);
    p->phase    = PHX_PERCH;
    p->n_body   = n_body;
    p->n_spark  = n_spark;
    p->n_anchor = anchor_table_build(p->anchors);

    for (int i = 0; i < p->n_body; i++) {
        int idx = anchor_pick(p->anchors, p->n_anchor);
        body_rebind_to(&p->body[i], p->anchors, idx, p->ox, p->oy, 0.f);
        p->body[i].temp = p->anchors[idx].base_temp;
        p->body[i].vx = p->body[i].vy = 0.f;
    }
    for (int i = 0; i < p->n_spark; i++) p->sparks[i].active = false;
    p->spark_emit_acc = 0.f;
}

static void phoenix_reset(Phoenix *p, int rows, int cols)
{
    int nb = p->n_body, ns = p->n_spark;
    phoenix_init(p, rows, cols, nb, ns);
}

static void phoenix_resize(Phoenix *p, int rows, int cols)
{
    p->ox      = (float)cols * 0.5f;
    p->oy      = (float)rows * 0.45f;
    p->perch_y = (float)rows * 0.45f + (BODY_CY + BODY_RY + 1.0f);
}

/* When the cycle settles back to PERCH, pull any leftover flying particles
 * home so the resting owl is whole rather than half-rebuilt. */
static void rebind_strays_to_owl(Phoenix *p)
{
    for (int i = 0; i < p->n_body; i++) {
        if (p->body[i].released) {
            int idx = anchor_pick(p->anchors, p->n_anchor);
            body_rebind_to(&p->body[i], p->anchors, idx, p->ox, p->oy, 0.f);
            p->body[i].temp = p->anchors[idx].base_temp;
            p->body[i].vx = p->body[i].vy = 0.f;
        }
    }
}

/* When REBIRTH starts, set every particle flying so the rebuild begins from a
 * clean slate with no leftover "ghost owl" from the collapse. */
static void release_all_body(Phoenix *p)
{
    for (int i = 0; i < p->n_body; i++) {
        if (!p->body[i].released) {
            p->body[i].released = true;
            p->body[i].vx = frand_signed() * 1.5f;
            p->body[i].vy = -1.0f + frand_signed() * 0.8f;
        }
    }
}

/* The upward burst of fire at the seed that the reborn owl emerges from. */
static void emit_seed_flare(Phoenix *p)
{
    float sx = p->ox + SEED_DX * ASPECT_X;
    float sy = p->oy + SEED_DY;
    int flare = p->n_spark / REBIRTH_FLARE_FRAC, lit = 0;
    for (int i = 0; i < p->n_spark && lit < flare; i++) {
        if (p->sparks[i].active) continue;
        spark_emit(&p->sparks[i], sx, sy);
        lit++;
    }
}

/* Move to the next stage of the cycle and run its one-time entry action
 * (tidy the owl on PERCH; scatter everything and flare on REBIRTH). */
static void phoenix_advance_phase(Phoenix *p)
{
    p->phase_t = 0.f;
    p->phase   = (PhoenixPhase)((int)(p->phase + 1) % N_PHX_PHASES);

    if (p->phase == PHX_PERCH)   rebind_strays_to_owl(p);
    if (p->phase == PHX_REBIRTH) { release_all_body(p); emit_seed_flare(p); }
}

/* Snap a particle to a random anchor and give it the right heat for the
 * current stage. Used while the owl is intact (PERCH/IGNITE/BLAZE). */
static void body_bind_random(Phoenix *p, BodyParticle *b, float frac, float bob)
{
    int idx = anchor_pick(p->anchors, p->n_anchor);
    body_rebind_to(b, p->anchors, idx, p->ox, p->oy, bob);
    b->temp = bound_temp(p->phase, frac,
                         p->anchors[idx].base_temp, p->anchors[idx].alive_at);
}

/* During REBIRTH, try to snap a particle onto an already-awoken anchor;
 * returns false if the growth hasn't reached any anchors yet. */
static bool body_bind_alive(Phoenix *p, BodyParticle *b, float frac, float bob)
{
    int idx = anchor_pick_alive(p->anchors, p->n_anchor, frac);
    if (idx < 0) return false;
    body_rebind_to(b, p->anchors, idx, p->ox, p->oy, bob);
    b->temp = bound_temp(PHX_REBIRTH, frac,
                         p->anchors[idx].base_temp, p->anchors[idx].alive_at);
    return true;
}

/* COLLAPSE step for one particle: a bound one has a growing chance to break
 * loose; once loose it flies as fire. By the end almost all have let go. */
static void body_tick_collapse(Phoenix *p, BodyParticle *b, float frac, float dt,
                               float bob)
{
    if (!b->released) {
        if (frand() < frac * COLLAPSE_RELEASE_RATE * dt) body_release(b);
        else                                             body_bind_random(p, b, frac, bob);
    }
    if (b->released)
        fire_step(&b->x, &b->y, &b->vx, &b->vy, &b->temp, dt, p->world_t);
}

/* ASH step: make sure the particle is loose, then let it drift up as smoke. */
static void body_tick_ash(Phoenix *p, BodyParticle *b, float dt)
{
    if (!b->released) {
        b->released = true;
        b->vx = frand_signed() * 1.0f;
        b->vy = -0.5f + frand_signed() * 0.8f;
    }
    fire_step(&b->x, &b->y, &b->vx, &b->vy, &b->temp, dt, p->world_t);
}

/* REBIRTH step: a flying particle tries each frame to land on an awoken anchor,
 * otherwise keeps drifting; an already-landed one re-snaps to an awoken one. */
static void body_tick_rebirth(Phoenix *p, BodyParticle *b, float frac, float dt,
                              float bob)
{
    if (!b->released) {
        body_bind_alive(p, b, frac, bob);
        return;
    }
    bool captured = (frand() < REBIRTH_CAPTURE_RATE * dt)
                 && body_bind_alive(p, b, frac, bob);
    if (!captured)
        fire_step(&b->x, &b->y, &b->vx, &b->vy, &b->temp, dt, p->world_t);
}

/* Update every body particle using the rule for the current stage. */
static void phoenix_tick_body(Phoenix *p, float dt, float bob)
{
    float frac = p->phase_t / phase_duration(p->phase);
    if (frac > 1.f) frac = 1.f;

    for (int i = 0; i < p->n_body; i++) {
        BodyParticle *b = &p->body[i];
        switch (p->phase) {
        case PHX_PERCH:
        case PHX_IGNITE:
        case PHX_BLAZE:    body_bind_random(p, b, frac, bob);        break;
        case PHX_COLLAPSE: body_tick_collapse(p, b, frac, dt, bob);  break;
        case PHX_ASH:      body_tick_ash(p, b, dt);                  break;
        case PHX_REBIRTH:  body_tick_rebirth(p, b, frac, dt, bob);   break;
        default: break;
        }
    }
}

/* Emit new sparks at this stage's rate, then move every live spark forward. */
static void phoenix_tick_sparks(Phoenix *p, float dt)
{
    float hz = spark_hz_for_phase(p->phase);
    p->spark_emit_acc += dt * hz;
    int to_emit = (int)p->spark_emit_acc;
    p->spark_emit_acc -= (float)to_emit;
    if (to_emit > SPARK_BURST_MAX) to_emit = SPARK_BURST_MAX;

    int emitted = 0;
    for (int i = 0; i < p->n_spark && emitted < to_emit; i++) {
        if (p->sparks[i].active) continue;
        int idx = anchor_pick(p->anchors, p->n_anchor);
        float sx = p->ox + p->anchors[idx].dx * ASPECT_X;
        float sy = p->oy + p->anchors[idx].dy;
        spark_emit(&p->sparks[i], sx, sy);
        emitted++;
    }

    for (int i = 0; i < p->n_spark; i++)
        spark_tick(&p->sparks[i], dt, p->world_t);
}

/* One full simulation step: advance the clock, move to the next stage if it's
 * over, then update the body and the sparks. */
static void phoenix_tick(Phoenix *p, float dt)
{
    p->world_t += dt;
    p->phase_t += dt;

    if (p->phase_t >= phase_duration(p->phase))
        phoenix_advance_phase(p);

    /* A gentle up-down bob so the resting owl looks like it's breathing;
     * only applied during PERCH, where the small offset is visible. */
    float bob = (p->phase == PHX_PERCH)
              ? sinf(p->world_t * (float)(2.0 * M_PI) * BREATH_HZ) * BREATH_AMP
              : 0.f;

    phoenix_tick_body(p, dt, bob);
    phoenix_tick_sparks(p, dt);
}

static void scene_init(Scene *s, int rows, int cols)
{
    memset(s, 0, sizeof *s);
    s->rows = rows;
    s->cols = cols;
    phoenix_init(&s->phx, rows, cols, N_BODY_DEFAULT, N_SPARK_DEFAULT);
}

static void scene_resize(Scene *s, int rows, int cols)
{
    s->rows = rows;
    s->cols = cols;
    phoenix_resize(&s->phx, rows, cols);
}

static void scene_tick(Scene *s, float dt)
{
    if (s->paused) return;
    phoenix_tick(&s->phx, dt);
}

/* ── §6 render — colours, particle/perch drawing, HUD, ncurses setup ── */

/* A colour theme. Because a particle's colour comes straight from its heat,
 * one six-step cool-to-hot ramp paints the whole bird: the cool end is the
 * resting feathers, the hot end is the blaze. So pressing 't' swaps both the
 * owl's plumage and its fire at once. All colours sit in the bright half of
 * the 256-colour space so nothing vanishes (see CLAUDE.md).
 *
 *   name  — label shown in the HUD.
 *   heat  — six colours from coolest (smoke) to hottest (white core).
 *   perch — colour of the branch the owl sits on. */
typedef struct {
    const char *name;
    short       heat[6];
    short       perch;
} Theme;

static const Theme g_themes[N_THEMES] = {
    /* CLASSIC — brown owl, red/orange/yellow fire, white core. */
    { "CLASSIC", { 244, 130, 166, 202, 214, 231 }, 94 },
    /* IRIS    — purple/pink phoenix. */
    { "IRIS   ", { 244,  96, 134, 170, 213, 231 }, 60 },
    /* JADE    — green-jade phoenix. */
    { "JADE   ", { 244,  28,  70, 112, 154, 231 }, 64 },
    /* GOLD    — copper/gold phoenix. */
    { "GOLD   ", { 244,  94, 130, 172, 214, 231 }, 94 },
};

static void color_init(int theme_idx)
{
    if (!has_colors()) return;
    start_color();
    use_default_colors();
    const Theme *th = &g_themes[theme_idx % N_THEMES];

    if (COLORS >= 256) {
        init_pair(PAIR_HEAT_0, th->heat[0], -1);
        init_pair(PAIR_HEAT_1, th->heat[1], -1);
        init_pair(PAIR_HEAT_2, th->heat[2], -1);
        init_pair(PAIR_HEAT_3, th->heat[3], -1);
        init_pair(PAIR_HEAT_4, th->heat[4], -1);
        init_pair(PAIR_HEAT_5, th->heat[5], -1);
        init_pair(PAIR_PERCH,  th->perch,   -1);
        init_pair(PAIR_HUD,    226,         -1);
        init_pair(PAIR_HINT,    51,         -1);
    } else {
        init_pair(PAIR_HEAT_0, COLOR_WHITE,   -1);
        init_pair(PAIR_HEAT_1, COLOR_RED,     -1);
        init_pair(PAIR_HEAT_2, COLOR_RED,     -1);
        init_pair(PAIR_HEAT_3, COLOR_YELLOW,  -1);
        init_pair(PAIR_HEAT_4, COLOR_YELLOW,  -1);
        init_pair(PAIR_HEAT_5, COLOR_WHITE,   -1);
        init_pair(PAIR_PERCH,  COLOR_YELLOW,  -1);
        init_pair(PAIR_HUD,    COLOR_YELLOW,  -1);
        init_pair(PAIR_HINT,   COLOR_CYAN,    -1);
    }
}

static void body_draw(const BodyParticle *p, int rows, int cols)
{
    int cx = (int)lroundf(p->x);
    int cy = (int)lroundf(p->y);
    if (cx < 0 || cx >= cols || cy < 0 || cy >= rows) return;

    short  pair;
    char   gl;
    attr_t at;
    heat_attrs(p->temp, &pair, &gl, &at);
    attron(COLOR_PAIR(pair) | at);
    mvaddch(cy, cx, (chtype)(unsigned char)gl);
    attroff(COLOR_PAIR(pair) | at);
}

static void spark_draw(const Spark *s, int rows, int cols)
{
    if (!s->active) return;
    int cx = (int)lroundf(s->x);
    int cy = (int)lroundf(s->y);
    if (cx < 0 || cx >= cols || cy < 0 || cy >= rows) return;

    short  pair;
    char   gl;
    attr_t at;
    heat_attrs(s->temp, &pair, &gl, &at);
    attron(COLOR_PAIR(pair) | at);
    mvaddch(cy, cx, (chtype)(unsigned char)gl);
    attroff(COLOR_PAIR(pair) | at);
}

/* Draw the branch the owl perches on, a little wider than the bird, with
 * upturned ends and a drop strut so it reads as a branch rather than a beam. */
static void draw_perch(const Scene *s)
{
    int row = (int)lroundf(s->phx.perch_y);
    if (row < 0 || row >= s->rows) return;
    int half = (int)(BODY_RX * ASPECT_X) + 4;
    int cx   = (int)lroundf(s->phx.ox);
    int x0   = cx - half;
    int x1   = cx + half;
    if (x0 < 0)        x0 = 0;
    if (x1 >= s->cols) x1 = s->cols - 1;

    attron(COLOR_PAIR(PAIR_PERCH) | A_BOLD);
    for (int x = x0; x <= x1; x++)
        mvaddch(row, x, (chtype)(unsigned char)'-');
    /* Slight upturn at each end. */
    if (x0 - 1 >= 0)        mvaddch(row, x0 - 1,
                                    (chtype)(unsigned char)'/');
    if (x1 + 1 <  s->cols)  mvaddch(row, x1 + 1,
                                    (chtype)(unsigned char)'\\');
    /* A short strut below so the branch feels grounded. */
    if (row + 1 < s->rows)
        mvaddch(row + 1, cx, (chtype)(unsigned char)'|');
    attroff(COLOR_PAIR(PAIR_PERCH) | A_BOLD);
}

/* Status line top-right (stage, theme, counts, fps) and key hints bottom-left. */
static void draw_hud(const Scene *s)
{
    char buf[120];
    snprintf(buf, sizeof buf,
             " PHOENIX  %s   %s  body:%d  spark:%d  %5.1f fps ",
             g_themes[s->theme].name,
             phase_name(s->phx.phase),
             s->phx.n_body, s->phx.n_spark,
             s->fps);
    int len = (int)strlen(buf);
    if (len > s->cols) len = s->cols;
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, s->cols - len, "%s", buf);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

    attron(COLOR_PAIR(PAIR_HINT) | A_BOLD);
    mvprintw(s->rows - 1, 0,
             " q:quit  spc:pause  r:reset  s:skip  i:ignite  t/T:theme  ,/.:body  ;/':sparks ");
    attroff(COLOR_PAIR(PAIR_HINT) | A_BOLD);
}

/* Draw one frame. Order matters: the bound owl outline goes down first, then
 * the flying embers on top of it, then the sparks, then the status line. */
static void scene_draw(const Scene *s)
{
    erase();
    draw_perch(s);

    /* The owl outline (particles snapped to anchors). */
    for (int i = 0; i < s->phx.n_body; i++)
        if (!s->phx.body[i].released)
            body_draw(&s->phx.body[i], s->rows, s->cols);

    /* Flying embers, drawn over the outline they peeled off of. */
    for (int i = 0; i < s->phx.n_body; i++)
        if (s->phx.body[i].released)
            body_draw(&s->phx.body[i], s->rows, s->cols);

    /* Sparks on top. */
    for (int i = 0; i < s->phx.n_spark; i++)
        spark_draw(&s->phx.sparks[i], s->rows, s->cols);

    draw_hud(s);
}

static void screen_init(int theme_idx)
{
    initscr();
    cbreak();
    noecho();
    keypad(stdscr, TRUE);
    nodelay(stdscr, TRUE);
    typeahead(-1);
    curs_set(0);
    color_init(theme_idx);
}

static void screen_cleanup(void)
{
    if (!isendwin()) {
        curs_set(1);
        endwin();
    }
}

/* ── §7 app — signals, key handling, the main loop and frame cap ── */

static volatile sig_atomic_t g_running     = 1;
static volatile sig_atomic_t g_need_resize = 0;

static void on_signal(int sig)
{
    if (sig == SIGWINCH) g_need_resize = 1;
    else                 g_running = 0;
}

static void install_signals(void)
{
    struct sigaction sa = {0};
    sa.sa_handler = on_signal;
    sigemptyset(&sa.sa_mask);
    sigaction(SIGINT,  &sa, NULL);
    sigaction(SIGTERM, &sa, NULL);
    sigaction(SIGWINCH,&sa, NULL);
}

static void atexit_cleanup(void) { screen_cleanup(); }

/* Jump straight to the end of the current stage so the next tick advances it
 * (the 's' key). */
static void phoenix_skip_phase(Phoenix *p)
{
    p->phase_t = phase_duration(p->phase);
}

static void phoenix_jump_to_ignite(Phoenix *p)
{
    p->phase   = PHX_IGNITE;
    p->phase_t = 0.f;
}

/* Handle one keypress. Returns false only on quit. */
static bool app_handle_key(Scene *s, int ch)
{
    switch (ch) {
    case 'q': case 'Q': case 27 /* ESC */: return false;
    case ' ':
    case 'p': case 'P': s->paused = !s->paused;                       break;
    case 'r': case 'R': phoenix_reset(&s->phx, s->rows, s->cols);     break;
    case 's': case 'S': phoenix_skip_phase(&s->phx);                  break;
    case 'i': case 'I': phoenix_jump_to_ignite(&s->phx);              break;

    case 't':
        s->theme = (s->theme + 1) % N_THEMES;
        color_init(s->theme);
        break;
    case 'T':
        s->theme = (s->theme + N_THEMES - 1) % N_THEMES;
        color_init(s->theme);
        break;

    case '.': case '>':
        if (s->phx.n_body + N_BODY_STEP <= N_BODY_MAX)
            s->phx.n_body += N_BODY_STEP;
        break;
    case ',': case '<':
        if (s->phx.n_body - N_BODY_STEP >= N_BODY_MIN)
            s->phx.n_body -= N_BODY_STEP;
        break;

    case '\'': case '"':
        if (s->phx.n_spark + N_SPARK_STEP <= N_SPARK_MAX)
            s->phx.n_spark += N_SPARK_STEP;
        break;
    case ';': case ':':
        if (s->phx.n_spark - N_SPARK_STEP >= N_SPARK_MIN)
            s->phx.n_spark -= N_SPARK_STEP;
        break;

    default: break;
    }
    return true;
}

static void app_do_resize(Scene *s)
{
    endwin();
    refresh();
    int rows, cols;
    getmaxyx(stdscr, rows, cols);
    scene_resize(s, rows, cols);
    g_need_resize = 0;
}

int main(void)
{
    install_signals();
    atexit(atexit_cleanup);

    int seed = (int)(clock_ns() & 0x7FFFFFFF);
    srand((unsigned)seed);

    screen_init(0);
    int rows, cols;
    getmaxyx(stdscr, rows, cols);

    Scene scene;
    scene_init(&scene, rows, cols);

    const int64_t frame_ns = 1000000000LL / TARGET_FPS;
    int64_t prev = clock_ns();

    int frames = 0;
    int64_t fps_t0 = prev;

    while (g_running) {
        if (g_need_resize) app_do_resize(&scene);

        int ch = getch();
        if (ch != ERR && !app_handle_key(&scene, ch)) g_running = 0;

        /* Time since last frame, capped so a hiccup can't make the
         * simulation lurch forward in one giant step. */
        int64_t now = clock_ns();
        float dt = (float)(now - prev) * 1e-9f;
        if (dt > DT_CAP_S) dt = DT_CAP_S;
        prev = now;

        scene_tick(&scene, dt);

        scene_draw(&scene);
        wnoutrefresh(stdscr);
        doupdate();

        /* Tally fps about twice a second. */
        frames++;
        if (now - fps_t0 >= 500000000LL) {
            scene.fps = (float)frames * 1e9f / (float)(now - fps_t0);
            frames    = 0;
            fps_t0    = now;
        }

        /* Sleep off the rest of the frame to hold the target rate. */
        int64_t sleep_ns = frame_ns - (clock_ns() - now);
        clock_sleep_ns(sleep_ns);
    }

    return 0;
}
