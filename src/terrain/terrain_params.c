/**
 * @file terrain_params.c
 *
 * HawkeyeTerrainParams default-init + apply skeleton. The apply step
 * forwards the seven SIH_TERR_* scalars to the shared terrain library's
 * terrain_set_params(), converting frequency (1/m) to wavelength (m).
 */

#include "terrain_params.h"
#include "terrain.h"  /* vendored shared terrain library — provides terrain_set_params() */

#include <stddef.h>   /* NULL */

/*
 * Minimum frequency before inverting to wavelength. Mirrors the guard
 * documented in terrain.h: `wavelength = 1.f / max(freq, 1e-6f)`. Avoids
 * generating a non-finite wavelength when a freshly-defaulted parameter
 * group arrives before the user has populated it.
 */
#define FREQ_MIN_FOR_INVERSION  1.0e-6f

/*
 * Single registered observer of every apply call. NULL when nothing has
 * registered yet (the default for pure-headless usage like the unit
 * tests). The rendering layer wires this to terrain_renderer_apply_params
 * at startup.
 */
static hawkeye_terrain_apply_cb_t g_apply_cb = NULL;

/*
 * The most recently applied params — the unified source of truth read by the
 * draw gate (scene.c) regardless of transport. Initialised to the flat
 * baseline so a query before the first apply is well-defined. Mirrors
 * hawkeye_terrain_params_default(); kept as a literal because static
 * initialisers can't call a function.
 */
static HawkeyeTerrainParams g_current = {
    .mode      = HAWKEYE_TERRAIN_MODE_OFF,
    .amp       = 0.0f,
    .freq      = 0.005f,
    .seed      = 0,
    .home_n_m  = 0.0f,
    .home_e_m  = 0.0f,
    .plane_deg = 0.0f,
};

void hawkeye_terrain_set_apply_callback(hawkeye_terrain_apply_cb_t cb)
{
    g_apply_cb = cb;
}

const HawkeyeTerrainParams *hawkeye_terrain_current_params(void)
{
    return &g_current;
}

HawkeyeTerrainParams hawkeye_terrain_params_default(void)
{
    /*
     * Defaults sourced from the SIH parameter table (the upstream defaults
     * for SIH_TERR_*). `enabled = false` is the safe baseline: until a log
     * or live link confirms SIH_TERR_EN=1, the renderer treats the world
     * as flat.
     */
    HawkeyeTerrainParams p = {
        .mode       = HAWKEYE_TERRAIN_MODE_OFF,
        .amp        = 0.0f,    /* SIH_TERR_AMP default — flat even in TERRAIN mode */
        .freq       = 0.005f,  /* SIH_TERR_FREQ default (1/m) */
        .seed       = 0,
        .home_n_m   = 0.0f,
        .home_e_m   = 0.0f,
        .plane_deg  = 0.0f,    /* SIH_TERR_PLANE default; non-zero → tilted plane mode */
    };
    return p;
}

void hawkeye_terrain_apply_params(const HawkeyeTerrainParams *p)
{
    if (!p) {
        HawkeyeTerrainParams d = hawkeye_terrain_params_default();
        hawkeye_terrain_apply_params(&d);
        return;
    }

    /* Record as the unified current params before touching the library, so
     * any observer reads a consistent value on this same tick. */
    g_current = *p;

    /*
     * terrain_set_params is 6-arg: (amp, wavelength, seed,
     * home_n, home_e, plane_deg). Frequency is inverted to wavelength
     * with the same guard the library uses internally.
     *
     * tri-state: amp is forced to 0 unless mode == TERRAIN
     * (matches PX4 sih.cpp: `terr_amp = (terr_mode == 1) ? AMP : 0`).
     * Walls only render when mode == WALLS — scene.c gates the wall
     * renderer on that field directly.
     */
    float amp_effective = (p->mode == HAWKEYE_TERRAIN_MODE_TERRAIN) ? p->amp : 0.0f;
    float freq          = (p->freq > FREQ_MIN_FOR_INVERSION) ? p->freq : FREQ_MIN_FOR_INVERSION;
    float wavelength    = 1.0f / freq;

    terrain_set_params(amp_effective,
                       wavelength,
                       p->seed,
                       p->home_n_m,
                       p->home_e_m,
                       p->plane_deg);

    /*
     * Notify the registered observer (typically the renderer's apply)
     * AFTER the underlying library is in its new state, so observers
     * that consult the library on the same tick read consistent values.
     */
    if (g_apply_cb) {
        g_apply_cb(p);
    }
}
