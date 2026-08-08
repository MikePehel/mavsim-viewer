/**
 * @file terrain_renderer.h
 *
 * Clipmap terrain renderer. Three concentric LOD rings of a shared 255x255
 * vertex mesh are drawn at different scales around the camera. Each ring
 * samples a CPU-evaluated R32F heightmap of `terrain(N, E)` so the shared
 * procedural function (vendored from PX4 at src/terrain/terrain.h) drives
 * the visible surface.
 *
 * The renderer reads a cached copy of HawkeyeTerrainParams that the
 * ingestion layer (ULog parser or MAVLink listener) pushes via
 * terrain_renderer_apply_params(). Drawing is a no-op unless
 * params->mode == TERRAIN or params->plane_deg != 0 — i.e. unless the
 * heightfield actually has a non-flat surface to draw. In OFF and
 * WALLS modes (with plane = 0) the existing flat grid plane remains
 * the only ground surface.
 *
 * Frame convention: the renderer evaluates terrain() in NED and emits
 * raylib-native Y-up positions. The transform is applied per vertex in
 * the shader; see src/terrain/shaders/terrain.vs.
 */

#ifndef HAWKEYE_TERRAIN_RENDERER_H_
#define HAWKEYE_TERRAIN_RENDERER_H_

#include "raylib.h"
#include "terrain_params.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Allocate GPU mesh + per-ring heightmap textures and load shaders.
 *
 * Must be called once after the raylib window is initialised and before
 * the first draw. Safe to call without parameters applied yet; the first
 * draw with params->mode == OFF (the default) is a no-op.
 */
void terrain_renderer_init(void);

/**
 * Cache the current parameter set for use by the next draw call.
 *
 * Intended to be invoked alongside hawkeye_terrain_apply_params() so the
 * renderer's local copy and the shared library's static state stay in sync.
 * Safe for p == NULL (treats as defaults / disabled).
 */
void terrain_renderer_apply_params(const HawkeyeTerrainParams *p);

/**
 * Push the active theme's wireframe colour pair, fill, and fog.
 *
 * The fragment shader's wireframe edge colour is a mix between `wireA`
 * (low elevation) and `wireB` (high elevation) blended by
 * `smoothstep(-amp, +amp, fragWorldPos.y)` so themes that want a
 * gradient (the 1988 theme's pink→cyan look ) get one, and
 * themes that want a single colour just pass `wireA == wireB`. The fill
 * and fog uniforms cover the dark interior and distance-fog colours.
 *
 * Suggested mapping from src/theme.h:
 *   - wireA = theme.grid_minor          (low-elevation line colour)
 *   - wireB = theme.hud_highlight       (high-elevation line colour, 1988 only;
 *                                        other themes use grid_minor here too)
 *   - fill  = theme.ground
 *   - fog   = theme.fog
 *
 * Theme switches are rare so the renderer's setter can be called once
 * per frame without measurable cost.
 *
 * Defaults (when never called): built-in teal palette --
 *   wireA = wireB = #019CE3 teal, fill = near-black, fog = dark blue --
 * so the renderer keeps rendering reasonably until the scene-side
 * integration hooks in the actual theme colours.
 */
void terrain_renderer_set_theme_colors(Color wireA, Color wireB,
                                       Color fill, Color fog);

/**
 * Toggle solid Lambertian terrain mode.
 *
 * When `enable` is true the fragment shader skips the wireframe path
 * entirely and outputs `fillColor * lambert + ambient` using the same
 * theme `fillColor` (theme.ground) the wireframe path uses for its
 * dark fill. The grid pattern is provided by the grid_plane below the
 * terrain mesh, not by the terrain mesh itself. When `enable` is false
 * (default) wireframe path runs.
 *
 * supersedes 's textured solid mode: the terrain mesh
 * is intentionally NOT textured even in solid mode. The Texture2D
 * parameter from signature is dropped.
 *
 * Distance fog, keep-band discard, and geomorph all continue to apply
 * in solid mode so the LOD transitions stay continuous.
 *
 * Safe to call every frame; the renderer caches the flag cheaply.
 */
void terrain_renderer_set_solid_mode(bool enable);

/**
 * Solid-mode shading variant. Selects which BRDF / occlusion path the
 * fragment shader takes inside the solid-mode branch; ignored when
 * wireframe is active. Both modes share a per-fragment normal
 * computation from the heightmap and only differ in the lighting model
 * on top.
 *
 *   0 — Curvature AO   (4-tap concavity AO + Lambert; ground texture ok)
 *   1 — Toon (3-band)  (stylized cel with hard band edges; scene-side
 *                       suppresses the ground texture under this mode)
 *
 * NAMES / COST / LOOK arrays expose HUD-ready labels in the same order
 * for the debug panel to consume.
 */
#define TERRAIN_SHADING_MODE_COUNT 2
extern const char *const TERRAIN_SHADING_MODE_NAMES[TERRAIN_SHADING_MODE_COUNT];
extern const char *const TERRAIN_SHADING_MODE_COST [TERRAIN_SHADING_MODE_COUNT];
extern const char *const TERRAIN_SHADING_MODE_LOOK [TERRAIN_SHADING_MODE_COUNT];
void terrain_renderer_set_shading_mode(int mode);
int  terrain_renderer_get_shading_mode(void);

/**
 * Push the current drone velocity (NED, m/s) for clipmap prefetch.
 *
 * When the magnitude of (vN, vE) exceeds 1 m/s the renderer shifts the
 * mid + far ring centres forward by `velocity * lookahead` (0.5 s) on
 * the next draw, so the per-shift heightmap re-evaluation lands one
 * frame before the camera arrives at the new vertex grid cell. With the
 * drone stationary or unset (default zero), only the camera-look forward
 * bias applies. Safe to call every frame.
 */
void terrain_renderer_set_drone_velocity_ned(float vN_mps, float vE_mps, float vD_mps);

/**
 * Draw the three clipmap rings centred on the camera.
 *
 * Call between BeginMode3D / EndMode3D, after the flat grid plane and
 * before any 2D HUD overlay. The renderer re-evaluates per-ring heightmaps
 * lazily: when the camera position has translated by less than one vertex
 * step since the previous draw, the heightmap is reused unchanged.
 *
 * No-op unless mode == TERRAIN or plane_deg != 0; otherwise the
 * heightfield is flat and the grid plane below already represents it.
 */
void terrain_renderer_draw(Camera3D camera);

/**
 * Free GPU resources. Safe to call multiple times.
 */
void terrain_renderer_shutdown(void);

/* per-frame instrumentation. */

/* Public upper bound on the per-frame `tris_per_ring` array. The actual
 * number of active rings is reported via `num_rings`. Sized larger than
 * the current 2-ring layout so future ring changes don't break callers. */
#define TERRAIN_RENDERER_MAX_RINGS  4

typedef struct {
    int  num_rings;                              /* actual active ring count */
    int  tris_drawn_total;                       /* sum across rings + billboard */
    int  tris_per_ring[TERRAIN_RENDERER_MAX_RINGS];
    int  tris_billboard;                         /* horizon cylinder triangle count */
    int  patches_drawn;                          /* per-frame after frustum cull */
    int  patches_culled;
    int  heightmap_evals;                        /* ring_evaluate_heightmap calls this frame */
} terrain_render_stats_t;

/**
 * Read the most recent frame's render stats. Writes into *out; safe for
 * `out == NULL` (no-op). Values reflect the last completed
 * `terrain_renderer_draw()` call; calling between frames returns the
 * counters as the renderer left them.
 */
void terrain_renderer_get_stats(terrain_render_stats_t *out);

#ifdef __cplusplus
}
#endif

#endif /* HAWKEYE_TERRAIN_RENDERER_H_ */
