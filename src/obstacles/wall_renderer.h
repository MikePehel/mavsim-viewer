/**
 * @file wall_renderer.h
 *
 * Render the procedural SDF wall scene from the vendored
 * `src/terrain_sdf/scene.{c,h}`. The four walls come out of
 * `scene_wall(i)` as `sdf_prim_t` records in NED + altitude-up
 * coordinates; this module transforms them into raylib world space
 * (+X = East, +Y = Up, +Z = -North) and draws each as a textured
 * cube.
 *
 * The renderer is gated on the same "terrain enabled" flag the
 * listener tracks for the procedural heightfield — `SIH_TERR_EN`.
 * Walls only materialise on the PX4 side when `SIH_TERR_EN = 1`, so
 * Hawkeye mirrors that: no terrain enabled, no walls shown.
 *
 * The scene depends on `SIH_TERR_SEED` (consumed by `scene_wall()` via
 * `terrain_get_seed()` → `hash3d`). The seed flows in through the
 * existing terrain-params listener; the renderer doesn't read it
 * directly. As long as `hawkeye_terrain_apply_params()` has been
 * called once with the live seed, `scene_wall(i)` returns the same
 * wall the PX4 simulator placed.
 */
#ifndef HAWKEYE_OBSTACLES_WALL_RENDERER_H_
#define HAWKEYE_OBSTACLES_WALL_RENDERER_H_

#include <stdbool.h>

#include "raylib.h"
#include "theme.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Draw the SCENE_N_WALLS walls into the current 3D pass. No-op when
 * `enabled` is false (matches PX4: walls only exist when
 * SIH_TERR_EN = 1).
 *
 * `theme` provides the fill colour; the renderer derives a darkened
 * variant for the cube body and a slightly brighter edge tint so the
 * walls read as obstacles against the terrain rather than fading into
 * the ground colour at any theme palette.
 */
void wall_renderer_draw(bool enabled, const theme_t *theme);

#ifdef __cplusplus
}
#endif

#endif /* HAWKEYE_OBSTACLES_WALL_RENDERER_H_ */
