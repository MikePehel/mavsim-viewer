/**
 * @file wall_renderer.c
 *
 * lattice walls. See header for the contract.
 *
 * Iterate the cell grid `(i, j) ∈ [-WALL_CELL_RADIUS_CELLS, +K]²`,
 * ask `scene_wall_at_cell` whether each cell hosts a wall, and render
 * the returned primitive as a yawed cube. The primitive carries
 * precomputed `cos_yaw` / `sin_yaw` so we don't evaluate trig per
 * frame — we just feed them to a Y-axis rotation matrix and let raylib
 * draw the cube in the rotated local frame.
 *
 * NED→raylib coordinate convention (matches vehicle.c:547-555):
 *     raylib_x =  east
 *     raylib_y =  altitude (PX4 alt is already up-positive)
 *     raylib_z = -north
 *
 * The wall's local-frame N axis maps to raylib's -Z direction, and its
 * E axis to raylib's +X. The yaw angle in the SDF lives in the N-E
 * plane: a positive yaw rotates the N axis toward the E axis. After
 * the NED→raylib transform, that's a rotation in the X-Z plane about
 * the Y axis (raylib's vertical). Standard raylib idiom: translate to
 * centre, rotate about Y by yaw_rad, draw axis-aligned cube in local
 * extents, pop matrix.
 */
#include "wall_renderer.h"

#include <math.h>

#include "raymath.h"
#include "rlgl.h"

#include "scene.h"          /* vendored — scene_wall_at_cell, WALL_CELL_RADIUS_CELLS */
#include "terrain_sdf.h"    /* vendored — sdf_prim_t, SDF_PRIM_BOX */

static Color darken(Color c, float factor) {
    return (Color){
        (unsigned char)((float)c.r * factor),
        (unsigned char)((float)c.g * factor),
        (unsigned char)((float)c.b * factor),
        c.a
    };
}

void wall_renderer_draw(bool enabled, const theme_t *theme) {
    if (!enabled || !theme) return;

    const Color body = darken(theme->ground, 0.55f);
    const Color edge = theme->hud_accent;

    /* Iterate the lattice grid the way SIH does. WALL_CELL_RADIUS_CELLS
     * = 5 ⇒ 11×11 = 121 candidate cells; only ~50% (WALL_PRESENCE_P)
     * survive the presence hash, and a handful get culled by the
     * home-clearance check inside scene_wall_at_cell. */
    for (int32_t i = -WALL_CELL_RADIUS_CELLS; i <= WALL_CELL_RADIUS_CELLS; i++) {
        for (int32_t j = -WALL_CELL_RADIUS_CELLS; j <= WALL_CELL_RADIUS_CELLS; j++) {
            sdf_prim_t w;
            if (!scene_wall_at_cell(i, j, &w)) continue;
            if (w.type != SDF_PRIM_BOX) continue;

            /* NED centre → raylib centre. */
            const Vector3 pos = (Vector3){ w.center.e, w.center.alt, -w.center.n };

            /* Local-frame full extents along the wall's own N / E / alt
             * axes. The N axis ends up as raylib -Z, E as raylib +X,
             * alt as raylib +Y after the yaw rotation. */
            const Vector3 size = (Vector3){
                2.0f * w.extent.e,    /* local E → raylib X */
                2.0f * w.extent.alt,  /* local alt → raylib Y */
                2.0f * w.extent.n,    /* local N → raylib Z (sign cancels with rotation) */
            };

            /* Convert the (cos, sin) yaw pair into degrees for
             * raylib's rlRotatef. atan2 here is a one-shot per visible
             * wall (≤ 64 per frame); cheap. The negation matches the
             * NED→raylib Z-flip: a positive yaw in NED N-E plane
             * (N rotates toward E) is a negative rotation about
             * raylib's +Y axis (which points up; right-hand rule). */
            const float yaw_deg = -atan2f(w.sin_yaw, w.cos_yaw) * (180.0f / 3.14159265358979323846f);

            rlPushMatrix();
            rlTranslatef(pos.x, pos.y, pos.z);
            rlRotatef(yaw_deg, 0.0f, 1.0f, 0.0f);
            /* Cube draws centred on the current matrix origin. */
            DrawCubeV((Vector3){0, 0, 0}, size, body);
            DrawCubeWiresV((Vector3){0, 0, 0}, size, edge);
            rlPopMatrix();
        }
    }
}
