/**
 * @file terrain_params.h
 *
 * HawkeyeTerrainParams — the viewer-side mirror of the SIH_TERR_* parameter
 * group. Populated from one of two sources (ULog parameter snapshot for
 * replay, MAVLink PARAM_VALUE stream for live SIH) and pushed into the
 * shared terrain library via hawkeye_terrain_apply_params().
 *
 * ## Frame / units
 *
 * - `amp` in metres (peak amplitude of terrain variation; 0 = flat).
 * - `freq` in 1/m. The shared terrain library takes wavelength, not
 *   frequency, so apply_params() inverts via `wavelength = 1 / max(freq, eps)`.
 * - `home_n_m` / `home_e_m` are the NED offset of the SIH spawn point
 *   (SIH_LOC_LAT0/LON0) from whatever world origin the viewer uses. The
 *   shared library pins `terrain(home_n, home_e) == 0` from these.
 *
 * ## Schema freeze
 *
 * This struct is FROZEN at commit. Renderer code consumes it
 * directly; later additions require an API Change Proposal on the shared
 * upstream.
 */

#ifndef HAWKEYE_TERRAIN_PARAMS_H_
#define HAWKEYE_TERRAIN_PARAMS_H_

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* SIH_TERR_EN tri-state hotfix — walls and terrain are MUTUALLY
 * EXCLUSIVE ground modes (not coexisting siblings, as originally
 * shipped). SIH_TERR_PLANE remains independent of the mode selector. */
typedef enum {
    HAWKEYE_TERRAIN_MODE_OFF     = 0,  /* flat ground, no walls (baseline) */
    HAWKEYE_TERRAIN_MODE_TERRAIN = 1,  /* fBm hills via AMP/FREQ; no walls */
    HAWKEYE_TERRAIN_MODE_WALLS   = 2,  /* flat + wall lattice; fBm off */
} HawkeyeTerrainMode;

/* SIH_TERR_* shape, hotfixed:
 *   - mode:      tri-state selector (off / terrain / walls). Mutually exclusive.
 *   - Elevation: terrain(N,E) returns Quilez-fBm only when mode == TERRAIN.
 *     AMP=0 produces a flat surface even in TERRAIN mode (used for
 *     differential regression tests). The library applies its own
 *     home-offset so terrain(home_n, home_e) == 0 to float precision.
 *   - Walls:     scene_wall_at_cell() materialises a lattice of vertical
 *     boxes only when mode == WALLS.
 *   - PLANE:     optional planar slope. Honoured in any mode (independent
 *     of the selector); non-zero replaces the heightfield with a uniform
 *     tilt h(N,E) = tan(plane_deg) * N. */
typedef struct {
    int   mode;        /* SIH_TERR_EN      [enum] HawkeyeTerrainMode (0/1/2) */
    float amp;         /* SIH_TERR_AMP     [m]   peak amplitude of terrain variation; 0 = flat */
    float freq;        /* SIH_TERR_FREQ    [1/m] base spatial frequency; library wants wavelength = 1/freq */
    int   seed;        /* SIH_TERR_SEED    [-]   integer hash seed (drives both walls and elevation) */
    float home_n_m;    /* NED north of SIH spawn (SIH_LOC_LAT0) from viewer origin [m] */
    float home_e_m;    /* NED east  of SIH spawn (SIH_LOC_LON0) from viewer origin [m] */
    float plane_deg;   /* SIH_TERR_PLANE   [deg] tilt — non-zero replaces fBm with planar slope
                          h(N,E) = tan(plane_deg) * N; honoured in any mode */
} HawkeyeTerrainParams;

/**
 * Return a default-initialised struct with terrain disabled.
 *
 * Defaults match the SIH_TERR_* defaults from the upstream parameter table:
 * enabled=false, amp=0, freq=0.005, oct=6, hurst=0.7, erosion=1.0, seed=0,
 * home=(0,0). When applied, this leaves the terrain library returning 0
 * everywhere (per the upstream `amp == 0 -> flat` contract) and signals to
 * the renderer that no terrain mesh should be drawn.
 */
HawkeyeTerrainParams hawkeye_terrain_params_default(void);

/**
 * Push parameters into the shared terrain library.
 *
 * Calls terrain_set_params() with the inverted wavelength (1/freq) and the
 * home reference. Safe for `p == NULL` (treats it as defaults / mode OFF).
 * Safe for `freq <= 0` (clamped before inversion).
 *
 * tri-state: when `p->mode != TERRAIN` this forces `amp = 0`
 * so the library returns a flat surface even if a non-zero amplitude is
 * configured. Walls are independent — scene.c gates the wall renderer
 * on `mode == WALLS` directly, not via the apply chain.
 *
 * If a callback has been registered via hawkeye_terrain_set_apply_callback(),
 * it is invoked at the end of this call with the same pointer, so the
 * renderer (or any other observer) can mirror the apply without each
 * ingestion site having to know about downstream consumers.
 */
void hawkeye_terrain_apply_params(const HawkeyeTerrainParams *p);

/**
 * The most recently applied terrain params — the unified, transport-agnostic
 * source of truth. Updated on every hawkeye_terrain_apply_params() call, so it
 * reflects whichever ingestion path last ran: ULog replay extraction, live
 * MAVLink, or the --terrain CLI override. Callers that need to decide whether
 * to draw terrain MUST consult this rather than a transport-specific store
 * (e.g. the live-MAVLink listener), which is only populated on a live link and
 * is therefore always OFF in replay. Never NULL; defaults to a flat baseline
 * until the first apply.
 */
const HawkeyeTerrainParams *hawkeye_terrain_current_params(void);

/**
 * Callback type for observers that need to mirror every apply.
 *
 * The pointer is the SAME one passed to hawkeye_terrain_apply_params(),
 * so a callback can read every field exactly as the underlying library
 * was told.
 */
typedef void (*hawkeye_terrain_apply_cb_t)(const HawkeyeTerrainParams *p);

/**
 * Register a callback to be invoked at the end of every successful
 * hawkeye_terrain_apply_params() call.
 *
 * The intended consumer is the renderer's apply function — the rendering
 * layer registers `terrain_renderer_apply_params` here once at startup,
 * after raylib init and before any data source is created, so both the
 * ULog and live-MAVLink ingestion paths automatically push state into
 * the renderer's local cache.
 *
 * Passing NULL clears the registration. Only one callback is held at a
 * time; later registrations replace earlier ones. Not thread-safe — the
 * receive dispatch and the registration are both expected to run on the
 * main thread.
 */
void hawkeye_terrain_set_apply_callback(hawkeye_terrain_apply_cb_t cb);

#ifdef __cplusplus
}
#endif

#endif /* HAWKEYE_TERRAIN_PARAMS_H_ */
