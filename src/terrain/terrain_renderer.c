/**
 * @file terrain_renderer.c
 *
 * Clipmap terrain renderer — three concentric LOD rings of a shared
 * 255 x 255 vertex mesh, each ring backed by a CPU-evaluated R32F
 * heightmap. See terrain_renderer.h for the public API contract and
 * src/terrain/shaders/terrain.{vs,fs} for the GPU side.
 *
 * The procedural terrain function is the vendored upstream
 * `terrain(north_m, east_m)`. See `src/terrain/terrain.h` for the API
 * contract and `src/terrain/terrain.c` for the implementation; both are
 * physically shared with the PX4 firmware and must not be modified
 * locally (changes go through an upstream API Change Proposal).
 *
 * Frame conversion: raylib uses Y-up with +X = East, +Y = Up,
 * +Z = -North (matches Hawkeye-wide convention; see vehicle.c:546-555).
 * The terrain() function consumes NED. Conversion is one negation at
 * the heightmap evaluation call site.
 *
 * Patch model: each ring renders as one or more patches that
 * tile its [-1, +1]^2 local space. The inner ring is one full patch (no
 * culling). Mid + far rings are split into four quadrants for CPU-side
 * frustum culling; in non-orthographic camera modes their ring centre is
 * also biased forward of the camera so the same vertex budget covers
 * more terrain ahead.
 */

#include "terrain_renderer.h"
#include "terrain.h"
#include "asset_path.h"
#include "raymath.h"
#include "rlgl.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

/* Shared mesh resolution. 255 x 255 keeps the vertex buffer ~0.5 MB and
 * each per-ring heightmap ~260 KB (R32F). */
#define RING_VERTS  255

/*
 * Three LOD rings.
 *
 * History:
 * - dropped the original 3-ring inner/mid/far layout to a 2-ring
 *    inner+outer to fix the LOD-disagreement seam in the overlap zone.
 * - added keep-band discard + vertex-Y geomorph so the rings
 *    cover disjoint annuli (no overlap) and share Y at the boundary
 *    (no cliff at the transition).
 * - reintroduces a middle ring -- machinery makes
 *    each ring's transition to the next coarser one seamless, so adding
 *    a level just needs another keep-band split and another geomorph
 *    target. With three levels the inner->middle->outer detail falloff
 *    is gradual enough that the foreground does not abruptly drop into
 *    the coarse outer-ring surface.
 *
 * Radii / Nyquist octave caps for the ring layout:
 *   inner   256 m,  9 octaves, single patch, camera-locked
 *   middle 1024 m,  6 octaves, 4 quadrants
 *   outer  3000 m,  4 octaves, 4 quadrants
 *
 * Keep-band boundaries (in world metres from the camera):
 *   inner:  [0,         256]
 *   middle: [256,      1024]
 *   outer:  [1024,    +inf]
 *
 * Geomorph targets (the "next coarser ring" each ring blends Y toward):
 *   inner  -> middle  (in the inner ring's fragRingDist [0.85, 1.0])
 *   middle -> outer   (in the middle ring's fragRingDist [0.85, 1.0])
 *   outer  -> (none; no coarser ring to converge to)
 */
#define NUM_RINGS   3

/* Maximum patches per ring. Inner uses 1; outer uses 4 quadrants. */
#define MAX_PATCHES_PER_RING  4

/*
 * Forward bias factor (legacy 's 3-ring layout) for rings whose
 * `forward_biased` flag is true. The 2-ring layout sets the outer ring's
 * flag to false because the ring is large enough (3 km radius) to cover
 * camera-relative directions in any orientation without a forward shift;
 * any positive bias would expose ground behind the camera that the ring
 * no longer reaches. The constant is retained so a future re-introduction
 * of a forward-biased ring need only flip a flag.
 */
#define FORWARD_BIAS_MID_FAR  0.30f

/* Frustum culling slack: each patch's bounding sphere is expanded by this
 * extra radius (metres) to keep silhouettes that just leave the frustum
 * from popping out. */
#define FRUSTUM_SLACK_M       16.0f

/* LOD-boundary crossfade. Each ring's patches fade alpha 1 -> 0
 * starting at fragRingDist == RING_FADE_START and reaching 0 at
 * RING_FADE_END. The inner ring's outer fade overlaps with the mid ring's
 * fully-opaque centre so the visible transition is shader-blended rather
 * than a hard pop. The constants are in fragRingDist units (the local
 * vertex distance from the ring centre); for an unsubdivided ring the
 * max fragRingDist is sqrt(2), for a quadrant patch it is sqrt(2)/2. */
#define RING_FADE_START       0.85f
#define RING_FADE_END         1.00f

/* velocity-aware prefetch lookahead. The mid + far ring centres
 * are pushed `drone_velocity * VELOCITY_LOOKAHEAD_S` ahead of the camera
 * so the heightmap snap-and-eval lands one frame before the camera
 * arrives at the new vertex grid cell. */
#define VELOCITY_LOOKAHEAD_S  0.5f
#define VELOCITY_MIN_MPS      1.0f

/*
 * Horizon billboard: a cylindrical panoramic strip that sits just outside
 * the outer ring's far edge and provides the silhouette for terrain beyond
 * the meshed coverage. Sampling `terrain()` along angular bearings + radii
 * produces a silhouette per column; the silhouette is drawn into a
 * 1024 x 256 RGBA texture that the cylinder mesh samples in horizon.fs.
 *
 * sizing: the cylinder wall moved in from 9 km to 5 km so it
 * butts up against the outer ring (3 km half-extent) with a small overlap
 * margin. The previous 9 km wall left a 6 km uncovered gap in front of
 * the camera which exposed the flat grid_plane. Sampling starts at 3.5 km
 * (just past the outer ring's silhouette-relevant edge) and runs to
 * 14 km, which lets the silhouette pick up far-away peaks that an
 * observer at the camera height could plausibly see over the outer ring's
 * own terrain.
 */
#define BILLBOARD_TEX_W       1024     /* columns -- one per ~0.35 deg */
#define BILLBOARD_TEX_H       256
#define BILLBOARD_RADIUS_M    5000.0f  /* cylinder radius (just outside the 3 km outer ring) */
#define BILLBOARD_HALF_M      400.0f   /* cylinder half-height */
#define BILLBOARD_QUADS       12       /* 30 deg per quad */
#define BILLBOARD_SAMPLE_MIN_M 3500.0f
#define BILLBOARD_SAMPLE_MAX_M 14000.0f
#define BILLBOARD_RADIAL_SAMPLES 18
#define BILLBOARD_REGEN_YAW_RAD  (10.0f * (float)M_PI / 180.0f)  /* spec: 10 deg */
#define BILLBOARD_REGEN_POS_M    100.0f                            /* spec: 100 m */

typedef struct {
    /* Patch sub-region in ring-local [-1, +1] coords. */
    float       offset_x;       /* in [-1, +1] */
    float       offset_z;       /* in [-1, +1] */
    float       half_extent;    /* in (0, 1]; 1.0 = full ring */
    /* Heightmap sub-region (texCoord space). */
    float       tex_offset_u;
    float       tex_offset_v;
    float       tex_half_extent;
} patch_t;

typedef struct {
    float       radius_m;       /* half-extent in metres */
    int         max_octaves;    /* per-ring Nyquist cap on terrain() octaves */
    float       center_x;       /* raylib world centre, x (= East) */
    float       center_z;       /* raylib world centre, z (= -North) */
    float       vertex_step_m;  /* world distance between adjacent vertices */
    Texture2D   height_tex;     /* R32F GPU heightmap */
    float      *height_cpu;     /* CPU-side mirror, row-major (RING_VERTS * RING_VERTS) */
    bool        dirty;          /* needs re-evaluation before the next draw */
    /* patch subdivision for frustum culling. */
    int         num_patches;
    patch_t     patches[MAX_PATCHES_PER_RING];
    bool        forward_biased; /* true for mid + far in perspective mode */
} ring_t;

typedef struct {
    Mesh        mesh;            /* cylinder of BILLBOARD_QUADS quads */
    Model       model;
    Shader      shader;
    int         loc_panoramaTex;
    int         loc_baseColor;
    int         loc_cylCenterXYZ;
    Texture2D   panorama_tex;
    unsigned char *panorama_cpu; /* BILLBOARD_TEX_W * BILLBOARD_TEX_H * 4 bytes (RGBA) */
    /* Last-regen camera state for trigger comparison. */
    bool        regenerated_once;
    float       last_regen_yaw_rad;
    Vector3     last_regen_pos;
} billboard_t;

typedef struct {
    bool        initialised;
    Mesh        mesh;           /* GenMeshPlane(2, 2, RING_VERTS-1, RING_VERTS-1) */
    Model       model;          /* mesh + material/shader binding */
    Shader      shader;
    /* Uniform locations */
    int         loc_ringScale;
    int         loc_ringCenterXZ;
    int         loc_patchOffset;
    int         loc_patchScale;
    int         loc_texCoordOffset;
    int         loc_texCoordScale;
    int         loc_heightTex;
    int         loc_camPos;
    int         loc_baseColor;
    int         loc_lightDir;
    int         loc_ambient;
    int         loc_ringFadeStart;
    int         loc_ringFadeEnd;
    /* continuous-LOD uniforms. */
    int         loc_geomorphActive;
    int         loc_outerHeightTex;
    int         loc_outerRingCenterXZ;
    int         loc_outerRingScale;
    int         loc_keepBandMin;
    int         loc_keepBandMax;
    int         loc_meshResolution;
    int         loc_wireColorA;
    int         loc_wireColorB;
    int         loc_wireYMin;
    int         loc_wireYMax;
    int         loc_fillColor;
    int         loc_fogColor;
    /* theme-driven wireframe colours. wireA = low-Y line,
     * wireB = high-Y line; equal values disable the gradient. Defaults
     * use the built-in default palette so the renderer keeps rendering
     * reasonably until the scene-side integration calls the setter. */
    float       theme_wire_a[3];
    float       theme_wire_b[3];
    float       theme_fill[3];
    float       theme_fog[3];
    /* solid terrain toggle (F-key in scene.c).
     * removed the texture sampling -- solid mode is now a flat
     * Lambertian shade in `fillColor`, so no texture id is cached. */
    bool        solid_mode_enabled;
    int         loc_uSolidMode;
    int         shading_mode;       /* 0..TERRAIN_SHADING_MODE_COUNT-1 */
    int         loc_uShadingMode;
    /* Per-ring state */
    ring_t      rings[NUM_RINGS];
    /* Far-horizon billboard -- replaces the far ring's geometry. */
    billboard_t billboard;
    /* Cached params (set by terrain_renderer_apply_params). */
    HawkeyeTerrainParams params;
    /* drone velocity in raylib XZ (East, -North), m/s. */
    float       drone_vel_x;
    float       drone_vel_z;
    /* per-frame render stats. Reset at the top of
     * terrain_renderer_draw, incremented in the eval / cull / draw paths,
     * exposed to callers via terrain_renderer_get_stats. */
    terrain_render_stats_t stats;
} terrain_renderer_t;

static terrain_renderer_t g_tr;

/* Billboard helpers defined further down; forward-declared so the
 * top-level init/draw/shutdown can reference them without reorder. */
static void billboard_init(billboard_t *bb);
static void billboard_destroy(billboard_t *bb);
static void billboard_maybe_regenerate(billboard_t *bb, Camera3D camera, Vector2 fwd_xz);
static void billboard_draw(billboard_t *bb, Camera3D camera);

/* ------------------------------------------------------------------ */
/* Initialisation helpers                                              */
/* ------------------------------------------------------------------ */

static void ring_set_single_patch(ring_t *ring) {
    ring->num_patches = 1;
    ring->patches[0] = (patch_t){
        .offset_x        = 0.0f,
        .offset_z        = 0.0f,
        .half_extent     = 1.0f,
        .tex_offset_u    = 0.0f,
        .tex_offset_v    = 0.0f,
        .tex_half_extent = 1.0f,
    };
}

static void ring_set_four_quadrants(ring_t *ring) {
    /*
     * Four quadrants of the [-1, +1]^2 ring local space. Each quadrant
     * has half_extent = 0.5 and centre at +/- 0.5 in each axis. The
     * heightmap layout pairs u with raylib X (East) and v with raylib Z
     * (-North = South), so the texcoord offsets mirror the world offsets
     * rescaled into [0, 1].
     *
     *   ring-local quadrants (looking down +Y onto the XZ plane):
     *     NW (X<0, Z<0)        NE (X>0, Z<0)
     *     SW (X<0, Z>0)        SE (X>0, Z>0)
     */
    ring->num_patches = 4;
    const float h = 0.5f;
    /* NW */
    ring->patches[0] = (patch_t){-h, -h, h, 0.0f, 0.0f, h};
    /* NE */
    ring->patches[1] = (patch_t){+h, -h, h, h,    0.0f, h};
    /* SW */
    ring->patches[2] = (patch_t){-h, +h, h, 0.0f, h,    h};
    /* SE */
    ring->patches[3] = (patch_t){+h, +h, h, h,    h,    h};
}

static void ring_init(ring_t *ring, float radius_m, int max_octaves,
                      bool subdivide) {
    ring->radius_m       = radius_m;
    ring->max_octaves    = max_octaves;
    ring->vertex_step_m  = (2.0f * radius_m) / (float)(RING_VERTS - 1);
    ring->center_x       = 0.0f;
    ring->center_z       = 0.0f;
    ring->dirty          = true;
    ring->forward_biased = subdivide;  /* same rings that get culled also get bias */

    if (subdivide) {
        ring_set_four_quadrants(ring);
    } else {
        ring_set_single_patch(ring);
    }

    /* Allocate CPU heightmap. zero-init so the first draw before a
     * re-evaluation shows a flat ring rather than garbage. */
    size_t n = (size_t)RING_VERTS * (size_t)RING_VERTS;
    ring->height_cpu = (float *)calloc(n, sizeof(float));

    /* Allocate the GPU texture from a zero image. We use a synthesised
     * Image struct rather than GenImageColor() so the format is R32F. */
    Image img = {
        .data    = ring->height_cpu,
        .width   = RING_VERTS,
        .height  = RING_VERTS,
        .mipmaps = 1,
        .format  = PIXELFORMAT_UNCOMPRESSED_R32,
    };
    ring->height_tex = LoadTextureFromImage(img);
    /* Bilinear filtering for sub-vertex smoothness. WebGL2 needs the
     * OES_texture_float_linear extension; if absent the driver falls
     * back to nearest, which still renders correctly. */
    SetTextureFilter(ring->height_tex, TEXTURE_FILTER_BILINEAR);
}

static void ring_destroy(ring_t *ring) {
    if (ring->height_tex.id != 0) {
        UnloadTexture(ring->height_tex);
        ring->height_tex.id = 0;
    }
    free(ring->height_cpu);
    ring->height_cpu = NULL;
}

void terrain_renderer_init(void) {
    if (g_tr.initialised) return;

    /* Build the shared mesh: a flat plane in [-1, 1] x [-1, 1]. */
    g_tr.mesh  = GenMeshPlane(2.0f, 2.0f, RING_VERTS - 1, RING_VERTS - 1);
    g_tr.model = LoadModelFromMesh(g_tr.mesh);

    /* Load the shader pair. */
    char vs_path[512], fs_path[512];
    asset_path("shaders/terrain.vs", vs_path, sizeof(vs_path));
    asset_path("shaders/terrain.fs", fs_path, sizeof(fs_path));
    g_tr.shader = LoadShader(vs_path, fs_path);

    /* Cache uniform locations. */
    g_tr.loc_ringScale      = GetShaderLocation(g_tr.shader, "ringScale");
    g_tr.loc_ringCenterXZ   = GetShaderLocation(g_tr.shader, "ringCenterXZ");
    g_tr.loc_patchOffset    = GetShaderLocation(g_tr.shader, "patchOffset");
    g_tr.loc_patchScale     = GetShaderLocation(g_tr.shader, "patchScale");
    g_tr.loc_texCoordOffset = GetShaderLocation(g_tr.shader, "texCoordOffset");
    g_tr.loc_texCoordScale  = GetShaderLocation(g_tr.shader, "texCoordScale");
    g_tr.loc_heightTex      = GetShaderLocation(g_tr.shader, "heightTex");
    g_tr.loc_camPos         = GetShaderLocation(g_tr.shader, "camPos");
    g_tr.loc_baseColor      = GetShaderLocation(g_tr.shader, "baseColor");
    g_tr.loc_lightDir       = GetShaderLocation(g_tr.shader, "lightDir");
    g_tr.loc_ambient        = GetShaderLocation(g_tr.shader, "ambient");
    g_tr.loc_ringFadeStart  = GetShaderLocation(g_tr.shader, "ringFadeStart");
    g_tr.loc_ringFadeEnd    = GetShaderLocation(g_tr.shader, "ringFadeEnd");
    g_tr.loc_geomorphActive    = GetShaderLocation(g_tr.shader, "geomorphActive");
    g_tr.loc_outerHeightTex    = GetShaderLocation(g_tr.shader, "outerHeightTex");
    g_tr.loc_outerRingCenterXZ = GetShaderLocation(g_tr.shader, "outerRingCenterXZ");
    g_tr.loc_outerRingScale    = GetShaderLocation(g_tr.shader, "outerRingScale");
    g_tr.loc_keepBandMin       = GetShaderLocation(g_tr.shader, "keepBandMin");
    g_tr.loc_keepBandMax       = GetShaderLocation(g_tr.shader, "keepBandMax");
    g_tr.loc_meshResolution    = GetShaderLocation(g_tr.shader, "meshResolution");
    g_tr.loc_wireColorA        = GetShaderLocation(g_tr.shader, "wireColorA");
    g_tr.loc_wireColorB        = GetShaderLocation(g_tr.shader, "wireColorB");
    g_tr.loc_wireYMin          = GetShaderLocation(g_tr.shader, "wireYMin");
    g_tr.loc_wireYMax          = GetShaderLocation(g_tr.shader, "wireYMax");
    g_tr.loc_fillColor         = GetShaderLocation(g_tr.shader, "fillColor");
    g_tr.loc_fogColor          = GetShaderLocation(g_tr.shader, "fogColor");
    g_tr.loc_uSolidMode        = GetShaderLocation(g_tr.shader, "uSolidMode");
    g_tr.loc_uShadingMode      = GetShaderLocation(g_tr.shader, "uShadingMode");

    /* Default theme colours = built-in teal palette so the renderer
     * keeps rendering reasonably until scene-side integration calls
     * terrain_renderer_set_theme_colors() with the active theme. wireA
     * and wireB both default to the same teal -- equal endpoints
     * disable gradient and give a single uniform wire colour. */
    g_tr.theme_wire_a[0] = 0.004f; g_tr.theme_wire_a[1] = 0.612f; g_tr.theme_wire_a[2] = 0.890f;
    g_tr.theme_wire_b[0] = 0.004f; g_tr.theme_wire_b[1] = 0.612f; g_tr.theme_wire_b[2] = 0.890f;
    g_tr.theme_fill[0]   = 0.015f; g_tr.theme_fill[1]   = 0.015f; g_tr.theme_fill[2]   = 0.040f;
    g_tr.theme_fog[0]    = 0.030f; g_tr.theme_fog[1]    = 0.030f; g_tr.theme_fog[2]    = 0.080f;

    g_tr.model.materials[0].shader = g_tr.shader;

    /* Three concentric rings. Radii + octave caps:
     *   inner    256 m,  9 octaves, single patch (camera-locked)
     *   middle  1024 m,  6 octaves, 4 quadrants, NO forward bias
     *   outer   3000 m,  4 octaves, 4 quadrants, NO forward bias
     * Each ring's octave cap is enforced CPU-side by clamping
     * params->octaves against the ring's max when calling terrain(),
     * so the heightmap stays Nyquist-safe at the local vertex step. */
    ring_init(&g_tr.rings[0],  256.0f,  9, /* subdivide */ false);
    ring_init(&g_tr.rings[1], 1024.0f,  6, /* subdivide */ true);
    ring_init(&g_tr.rings[2], 3000.0f,  4, /* subdivide */ true);
    /* `forward_biased` defaults to true inside ring_init when subdivide is
     * true; layout wants both subdivided rings centred on the
     * camera (full disc, no asymmetric gap behind), so we clear those flags. */
    g_tr.rings[1].forward_biased = false;
    g_tr.rings[2].forward_biased = false;

    /* Far-horizon billboard replaces the far ring's geometry rendering
     *. The far ring's data structures stay allocated as a
     * fall-back / debug toggle hook; only the per-frame eval + draw are
     * gated below. */
    billboard_init(&g_tr.billboard);

    /*
     * Default params only when the renderer has not already been seeded
     * by a pre-init `terrain_renderer_apply_params(...)` call. The
     * apply-callback registered by the ingestion layer fires for any
     * ULog `data_source_ulog_create` that runs before `scene_init`; the
     * renderer caches those params during that pre-init call so we must
     * not clobber them here. Check `g_tr.params.amp` as the "was it
     * touched" probe -- both BSS zero-init AND `hawkeye_terrain_params_default()`
     * leave amp at exactly 0.0f, so the only way amp is non-zero is a
     * deliberate apply with `enabled=true` set. For the disabled case
     * the defaults are functionally identical regardless of which path
     * populated them.
     */
    if (g_tr.params.mode == HAWKEYE_TERRAIN_MODE_OFF &&
        g_tr.params.seed == 0 && g_tr.params.plane_deg == 0.0f) {
        g_tr.params = hawkeye_terrain_params_default();
    }
    g_tr.initialised  = true;
}

void terrain_renderer_apply_params(const HawkeyeTerrainParams *p) {
    /*
     * Cache the params unconditionally — including before the renderer is
     * initialised. When the apply-callback registered by the ingestion
     * layer fires from `hawkeye_terrain_apply_params()` during a ULog
     * load, the call sequence is:
     *
     *   1. data_source_ulog_create() -> hawkeye_terrain_apply_params(...)
     *      -> registered callback (i.e. this function)
     *   2. scene_init() -> terrain_renderer_init()
     *
     * If we early-returned on `!initialised` the ULog params would be
     * silently dropped at step 1 and step 2 would leave the renderer
     * cache at defaults. Storing into the static struct now and letting
     * `terrain_renderer_init` preserve it instead is bit-correct for
     * BSS-zero-initialised state (rings[i].dirty is just a bool flag).
     */
    if (!p) {
        g_tr.params = hawkeye_terrain_params_default();
    } else {
        g_tr.params = *p;
    }
    /* Force a re-eval of every ring on the next draw. (Pre-init this is
     * a harmless write to zero-initialised struct fields; ring_init
     * sets dirty=true again so there's no double-effect.) */
    for (int i = 0; i < NUM_RINGS; i++) {
        g_tr.rings[i].dirty = true;
    }
}

void terrain_renderer_set_solid_mode(bool enable) {
    g_tr.solid_mode_enabled = enable;
}

const char *const TERRAIN_SHADING_MODE_NAMES[TERRAIN_SHADING_MODE_COUNT] = {
    "Curvature AO",
    "Toon 3-band",
};
const char *const TERRAIN_SHADING_MODE_COST[TERRAIN_SHADING_MODE_COUNT] = {
    "0 tex / ~12 alu",
    "0 tex / ~5 alu",
};
const char *const TERRAIN_SHADING_MODE_LOOK[TERRAIN_SHADING_MODE_COUNT] = {
    "real-ish depth AO",
    "stylized hard cel",
};

void terrain_renderer_set_shading_mode(int mode) {
    if (mode < 0) mode = 0;
    if (mode >= TERRAIN_SHADING_MODE_COUNT) mode = TERRAIN_SHADING_MODE_COUNT - 1;
    g_tr.shading_mode = mode;
}

int terrain_renderer_get_shading_mode(void) {
    return g_tr.shading_mode;
}

void terrain_renderer_set_theme_colors(Color wireA, Color wireB,
                                       Color fill, Color fog) {
    g_tr.theme_wire_a[0] = (float)wireA.r / 255.0f;
    g_tr.theme_wire_a[1] = (float)wireA.g / 255.0f;
    g_tr.theme_wire_a[2] = (float)wireA.b / 255.0f;
    g_tr.theme_wire_b[0] = (float)wireB.r / 255.0f;
    g_tr.theme_wire_b[1] = (float)wireB.g / 255.0f;
    g_tr.theme_wire_b[2] = (float)wireB.b / 255.0f;
    g_tr.theme_fill[0]   = (float)fill.r  / 255.0f;
    g_tr.theme_fill[1]   = (float)fill.g  / 255.0f;
    g_tr.theme_fill[2]   = (float)fill.b  / 255.0f;
    g_tr.theme_fog[0]    = (float)fog.r   / 255.0f;
    g_tr.theme_fog[1]    = (float)fog.g   / 255.0f;
    g_tr.theme_fog[2]    = (float)fog.b   / 255.0f;
}

void terrain_renderer_set_drone_velocity_ned(float vN_mps, float vE_mps, float vD_mps) {
    (void)vD_mps;  /* vertical velocity is irrelevant for the XZ-plane prefetch */
    /* NED -> raylib XZ: x = East = vE, z = -North = -vN. */
    g_tr.drone_vel_x = vE_mps;
    g_tr.drone_vel_z = -vN_mps;
}

/* ------------------------------------------------------------------ */
/* Per-frame heightmap evaluation                                      */
/* ------------------------------------------------------------------ */

static void ring_evaluate_heightmap(ring_t *ring) {
    g_tr.stats.heightmap_evals++;
    float *dst = ring->height_cpu;
    const float r = ring->radius_m;
    const float cx = ring->center_x;
    const float cz = ring->center_z;
    const float inv_max_idx = 1.0f / (float)(RING_VERTS - 1);

    for (int j = 0; j < RING_VERTS; j++) {
        const float v = (float)j * inv_max_idx;
        /* Shader: vertexPosition.z = 2v - 1; world raylib z = cz + vertex.z * r */
        const float world_z = cz + (2.0f * v - 1.0f) * r;
        const float worldN  = -world_z;

        for (int i = 0; i < RING_VERTS; i++) {
            const float u       = (float)i * inv_max_idx;
            const float world_x = cx + (2.0f * u - 1.0f) * r;
            const float worldE  = world_x;
            dst[j * RING_VERTS + i] = terrain(worldN, worldE);
        }
    }

    UpdateTexture(ring->height_tex, ring->height_cpu);
    ring->dirty = false;
}

/* Snap a world coordinate to the ring's vertex grid. The clipmap centre
 * lives on an integer multiple of `step` so that incremental scrolling
 * is alias-free; without snapping, sub-step camera motion would shimmer
 * the heightmap each frame. */
static float snap_to_step(float v, float step) {
    return roundf(v / step) * step;
}

/* ------------------------------------------------------------------ */
/* Frustum extraction + per-patch culling                              */
/* ------------------------------------------------------------------ */

/*
 * A frustum plane in the form ax + by + cz + d >= 0 (inside). Six planes
 * derived from the view-projection matrix via the Gribb-Hartmann method
 * — standard textbook math, works for both perspective and orthographic
 * projections.
 */
typedef struct {
    float a, b, c, d;
} plane_t;

typedef struct {
    plane_t planes[6];
} frustum_t;

/* raylib's Matrix is row-major in source but column-major in memory
 * (OpenGL convention). MatrixToFloat exposes it as a 16-float
 * column-major array; the Gribb-Hartmann formulas index it that way. */
static plane_t plane_normalize(plane_t p) {
    float mag = sqrtf(p.a * p.a + p.b * p.b + p.c * p.c);
    if (mag < 1.0e-12f) return p;
    float inv = 1.0f / mag;
    return (plane_t){p.a * inv, p.b * inv, p.c * inv, p.d * inv};
}

static frustum_t frustum_from_vp(Matrix vp) {
    /* Row vectors of the row-major matrix (raylib stores m0..m15 as
     * column-major: m[col*4 + row], but the Matrix struct fields
     * vp.m0..m15 are the row-major projection in code semantics). The
     * documented field layout in raymath.h is:
     *   m0  m4  m8  m12      row 0
     *   m1  m5  m9  m13      row 1
     *   m2  m6  m10 m14      row 2
     *   m3  m7  m11 m15      row 3
     * Gribb-Hartmann uses rows of the combined matrix in the OpenGL
     * convention. */
    const float m00 = vp.m0,  m01 = vp.m4,  m02 = vp.m8,  m03 = vp.m12;
    const float m10 = vp.m1,  m11 = vp.m5,  m12 = vp.m9,  m13 = vp.m13;
    const float m20 = vp.m2,  m21 = vp.m6,  m22 = vp.m10, m23 = vp.m14;
    const float m30 = vp.m3,  m31 = vp.m7,  m32 = vp.m11, m33 = vp.m15;

    frustum_t f;
    f.planes[0] = plane_normalize((plane_t){m30 + m00, m31 + m01, m32 + m02, m33 + m03}); /* left */
    f.planes[1] = plane_normalize((plane_t){m30 - m00, m31 - m01, m32 - m02, m33 - m03}); /* right */
    f.planes[2] = plane_normalize((plane_t){m30 + m10, m31 + m11, m32 + m12, m33 + m13}); /* bottom */
    f.planes[3] = plane_normalize((plane_t){m30 - m10, m31 - m11, m32 - m12, m33 - m13}); /* top */
    f.planes[4] = plane_normalize((plane_t){m30 + m20, m31 + m21, m32 + m22, m33 + m23}); /* near */
    f.planes[5] = plane_normalize((plane_t){m30 - m20, m31 - m21, m32 - m22, m33 - m23}); /* far */
    return f;
}

/* Sphere-vs-frustum cull. Returns true if the sphere is at least
 * partially inside (or might be — false negatives are harmless, false
 * positives would drop geometry). */
static bool sphere_in_frustum(const frustum_t *f, Vector3 c, float radius) {
    for (int i = 0; i < 6; i++) {
        const plane_t *p = &f->planes[i];
        float dist = p->a * c.x + p->b * c.y + p->c * c.z + p->d;
        if (dist < -radius) return false;
    }
    return true;
}

/* Build the combined view * projection matrix for a raylib Camera3D. */
static Matrix camera_view_projection(Camera3D camera) {
    Matrix view = GetCameraMatrix(camera);
    /*
     * raylib's BeginMode3D internally calls rlSetMatrixProjection with
     * a perspective or orthographic matrix based on camera.projection +
     * camera.fovy + screen aspect. We replicate that here so the
     * frustum we extract matches what's actually rendered.
     */
    float aspect = (float)GetScreenWidth() / (float)GetScreenHeight();
    if (aspect <= 0.0f) aspect = 1.0f;
    const float near_plane = 0.05f;   /* raylib's RL_CULL_DISTANCE_NEAR */
    const float far_plane  = 16384.0f; /* extend past far ring radius (10 km) */
    Matrix proj;
    if (camera.projection == CAMERA_ORTHOGRAPHIC) {
        const double top    = camera.fovy / 2.0;
        const double right  = top * aspect;
        proj = MatrixOrtho(-right, right, -top, top, near_plane, far_plane);
    } else {
        proj = MatrixPerspective(camera.fovy * DEG2RAD, aspect, near_plane, far_plane);
    }
    return MatrixMultiply(view, proj);
}

/* Ground-projected forward vector of the camera (raylib XZ, length 1).
 * Returns (0, 1) — arbitrary unit vector — if the camera is looking
 * straight up or straight down. */
static Vector2 camera_forward_xz(Camera3D camera) {
    Vector2 fwd = {
        camera.target.x - camera.position.x,
        camera.target.z - camera.position.z,
    };
    float mag = sqrtf(fwd.x * fwd.x + fwd.y * fwd.y);
    if (mag < 1.0e-4f) return (Vector2){0.0f, 1.0f};
    return (Vector2){fwd.x / mag, fwd.y / mag};
}

/* ------------------------------------------------------------------ */
/* Far-horizon billboard */
/* ------------------------------------------------------------------ */

/*
 * Build a 12-quad cylindrical strip centred on the world origin. The
 * cylinder is translated each frame in the vertex shader so it follows
 * the camera. Vertex layout: 4 verts per quad (no sharing), 2 triangles
 * per quad with CCW winding viewed from OUTSIDE the cylinder so back-face
 * culling drops the inside surface.
 */
static Mesh billboard_build_cylinder(void) {
    Mesh mesh = {0};
    mesh.vertexCount   = BILLBOARD_QUADS * 4;
    mesh.triangleCount = BILLBOARD_QUADS * 2;

    mesh.vertices  = (float *)RL_MALLOC((size_t)mesh.vertexCount * 3 * sizeof(float));
    mesh.texcoords = (float *)RL_MALLOC((size_t)mesh.vertexCount * 2 * sizeof(float));
    mesh.indices   = (unsigned short *)RL_MALLOC((size_t)mesh.triangleCount * 3 * sizeof(unsigned short));

    const float R = BILLBOARD_RADIUS_M;
    const float H = BILLBOARD_HALF_M;
    const float two_pi = 6.28318530718f;

    for (int q = 0; q < BILLBOARD_QUADS; q++) {
        const float t0 = (float)q       / (float)BILLBOARD_QUADS;
        const float t1 = (float)(q + 1) / (float)BILLBOARD_QUADS;
        const float th0 = t0 * two_pi;
        const float th1 = t1 * two_pi;
        const float x0 = sinf(th0) * R, z0 = -cosf(th0) * R;
        const float x1 = sinf(th1) * R, z1 = -cosf(th1) * R;

        const int base = q * 4;
        /* v00 bottom-left  (u, v) = (t0, 1) */
        mesh.vertices[(base + 0) * 3 + 0] = x0;
        mesh.vertices[(base + 0) * 3 + 1] = -H;
        mesh.vertices[(base + 0) * 3 + 2] = z0;
        mesh.texcoords[(base + 0) * 2 + 0] = t0;
        mesh.texcoords[(base + 0) * 2 + 1] = 1.0f;
        /* v10 bottom-right (u, v) = (t1, 1) */
        mesh.vertices[(base + 1) * 3 + 0] = x1;
        mesh.vertices[(base + 1) * 3 + 1] = -H;
        mesh.vertices[(base + 1) * 3 + 2] = z1;
        mesh.texcoords[(base + 1) * 2 + 0] = t1;
        mesh.texcoords[(base + 1) * 2 + 1] = 1.0f;
        /* v11 top-right    (u, v) = (t1, 0) */
        mesh.vertices[(base + 2) * 3 + 0] = x1;
        mesh.vertices[(base + 2) * 3 + 1] = +H;
        mesh.vertices[(base + 2) * 3 + 2] = z1;
        mesh.texcoords[(base + 2) * 2 + 0] = t1;
        mesh.texcoords[(base + 2) * 2 + 1] = 0.0f;
        /* v01 top-left     (u, v) = (t0, 0) */
        mesh.vertices[(base + 3) * 3 + 0] = x0;
        mesh.vertices[(base + 3) * 3 + 1] = +H;
        mesh.vertices[(base + 3) * 3 + 2] = z0;
        mesh.texcoords[(base + 3) * 2 + 0] = t0;
        mesh.texcoords[(base + 3) * 2 + 1] = 0.0f;

        /* CCW from outside: bottom-left -> bottom-right -> top-right -> top-left. */
        const int ib = q * 6;
        mesh.indices[ib + 0] = (unsigned short)(base + 0);
        mesh.indices[ib + 1] = (unsigned short)(base + 1);
        mesh.indices[ib + 2] = (unsigned short)(base + 2);
        mesh.indices[ib + 3] = (unsigned short)(base + 0);
        mesh.indices[ib + 4] = (unsigned short)(base + 2);
        mesh.indices[ib + 5] = (unsigned short)(base + 3);
    }

    UploadMesh(&mesh, false);
    return mesh;
}

static void billboard_init(billboard_t *bb) {
    bb->mesh  = billboard_build_cylinder();
    bb->model = LoadModelFromMesh(bb->mesh);

    char vs_path[512], fs_path[512];
    asset_path("shaders/horizon.vs", vs_path, sizeof(vs_path));
    asset_path("shaders/horizon.fs", fs_path, sizeof(fs_path));
    bb->shader = LoadShader(vs_path, fs_path);
    bb->model.materials[0].shader = bb->shader;

    bb->loc_panoramaTex  = GetShaderLocation(bb->shader, "panoramaTex");
    bb->loc_baseColor    = GetShaderLocation(bb->shader, "baseColor");
    bb->loc_cylCenterXYZ = GetShaderLocation(bb->shader, "cylCenterXYZ");

    /* Allocate the panoramic RGBA strip. Memory ~ 1 MB (1024 * 256 * 4). */
    const size_t bytes = (size_t)BILLBOARD_TEX_W * BILLBOARD_TEX_H * 4;
    bb->panorama_cpu = (unsigned char *)calloc(bytes, 1);

    Image img = {
        .data    = bb->panorama_cpu,
        .width   = BILLBOARD_TEX_W,
        .height  = BILLBOARD_TEX_H,
        .mipmaps = 1,
        .format  = PIXELFORMAT_UNCOMPRESSED_R8G8B8A8,
    };
    bb->panorama_tex = LoadTextureFromImage(img);
    SetTextureFilter(bb->panorama_tex, TEXTURE_FILTER_BILINEAR);
    SetTextureWrap(bb->panorama_tex, TEXTURE_WRAP_REPEAT);

    bb->regenerated_once   = false;
    bb->last_regen_yaw_rad = 0.0f;
    bb->last_regen_pos     = (Vector3){0};
}

static void billboard_destroy(billboard_t *bb) {
    if (bb->panorama_tex.id != 0) {
        UnloadTexture(bb->panorama_tex);
        bb->panorama_tex.id = 0;
    }
    if (bb->shader.id != 0) {
        UnloadShader(bb->shader);
        bb->shader.id = 0;
    }
    UnloadModel(bb->model);
    free(bb->panorama_cpu);
    bb->panorama_cpu = NULL;
}

/*
 * Re-render the panoramic strip from the camera's current position. For
 * each texture column c (one per ~0.35 deg of azimuth), we ray-cast
 * `terrain()` at BILLBOARD_RADIAL_SAMPLES radii between 1.5 km and 14 km
 * and keep the maximum apparent silhouette angle. That angle is then
 * mapped to a pixel row in the column and the column is filled below
 * with a simple brightness gradient (top of silhouette brighter,
 * lower portions darker) and alpha 1.0; everything above the silhouette
 * is alpha 0.0 so the existing sky shows through.
 */
static void billboard_regenerate_texture(billboard_t *bb, Vector3 cam_pos) {
    const float two_pi      = 6.28318530718f;
    const float radial_step = (BILLBOARD_SAMPLE_MAX_M - BILLBOARD_SAMPLE_MIN_M)
                            / (float)(BILLBOARD_RADIAL_SAMPLES - 1);

    /* Pixel-row mapping. The cylinder has half-height H at radius R, so
     * the vertical visual half-extent equals H / R radians (tan-approx is
     * exact for small angles). A silhouette angle `a` maps to a pixel row
     *
     *     row = H_tex/2 * (1 - a * R / H)
     *
     * where H_tex = BILLBOARD_TEX_H. Anything above row=0 / below row=H_tex
     * is clamped. */
    const float row_per_radian = (float)BILLBOARD_TEX_H * 0.5f
                               * (BILLBOARD_RADIUS_M / BILLBOARD_HALF_M);

    for (int c = 0; c < BILLBOARD_TEX_W; c++) {
        const float theta = ((float)c / (float)BILLBOARD_TEX_W) * two_pi;
        const float fwd_x = sinf(theta);
        const float fwd_z = -cosf(theta);

        /* Find the max apparent silhouette angle. */
        float max_tan_angle = -1.0e6f;
        for (int s = 0; s < BILLBOARD_RADIAL_SAMPLES; s++) {
            const float r = BILLBOARD_SAMPLE_MIN_M + (float)s * radial_step;
            const float wx = cam_pos.x + fwd_x * r;
            const float wz = cam_pos.z + fwd_z * r;
            const float worldE = wx;
            const float worldN = -wz;
            const float h = terrain(worldN, worldE);
            const float dy = h - cam_pos.y;
            const float tan_angle = dy / r;
            if (tan_angle > max_tan_angle) max_tan_angle = tan_angle;
        }

        int row_sil = (int)((float)BILLBOARD_TEX_H * 0.5f - max_tan_angle * row_per_radian);
        if (row_sil < 0)               row_sil = 0;
        if (row_sil > BILLBOARD_TEX_H) row_sil = BILLBOARD_TEX_H;

        /* Fill column: above silhouette = transparent sky; below = terrain. */
        const int below_pixels = BILLBOARD_TEX_H - row_sil;
        for (int r = 0; r < BILLBOARD_TEX_H; r++) {
            const int idx = (r * BILLBOARD_TEX_W + c) * 4;
            if (r < row_sil) {
                bb->panorama_cpu[idx + 0] = 0;
                bb->panorama_cpu[idx + 1] = 0;
                bb->panorama_cpu[idx + 2] = 0;
                bb->panorama_cpu[idx + 3] = 0;
            } else {
                /* Brighter near the silhouette top, darker toward the bottom
                 * (fakes atmospheric perspective + Lambert without normals). */
                const float t = (below_pixels > 1)
                              ? (float)(r - row_sil) / (float)(below_pixels - 1)
                              : 0.0f;
                const float shade01 = 0.95f - 0.35f * t;
                bb->panorama_cpu[idx + 0] = (unsigned char)(shade01 * 255.0f);
                bb->panorama_cpu[idx + 1] = 0;
                bb->panorama_cpu[idx + 2] = 0;
                bb->panorama_cpu[idx + 3] = 255;
            }
        }
    }

    UpdateTexture(bb->panorama_tex, bb->panorama_cpu);
}

static void billboard_maybe_regenerate(billboard_t *bb,
                                       Camera3D camera, Vector2 fwd_xz) {
    const float current_yaw = atan2f(fwd_xz.x, -fwd_xz.y); /* +Z is south */
    Vector3 pos = camera.position;

    if (!bb->regenerated_once) {
        billboard_regenerate_texture(bb, pos);
        bb->regenerated_once   = true;
        bb->last_regen_yaw_rad = current_yaw;
        bb->last_regen_pos     = pos;
        return;
    }

    float dyaw = current_yaw - bb->last_regen_yaw_rad;
    /* Wrap into [-pi, +pi]. */
    if (dyaw >  3.14159265f) dyaw -= 2.0f * 3.14159265f;
    if (dyaw < -3.14159265f) dyaw += 2.0f * 3.14159265f;

    const float dx = pos.x - bb->last_regen_pos.x;
    const float dz = pos.z - bb->last_regen_pos.z;
    const float dpos = sqrtf(dx * dx + dz * dz);

    if (fabsf(dyaw) > BILLBOARD_REGEN_YAW_RAD ||
        dpos       > BILLBOARD_REGEN_POS_M) {
        billboard_regenerate_texture(bb, pos);
        bb->last_regen_yaw_rad = current_yaw;
        bb->last_regen_pos     = pos;
    }
}

static void billboard_draw(billboard_t *bb, Camera3D camera) {
    /* Anchor the cylinder to the camera's ground XZ (slightly below eye
     * level along Y so the visible silhouette matches what the meshed rings
     * show). The shader's `cylCenterXYZ` adds this to every vertex. */
    const float cyl_centre[3] = {
        camera.position.x,
        camera.position.y - BILLBOARD_HALF_M * 0.25f, /* small downshift */
        camera.position.z,
    };
    const float baseColor[4] = { 0.55f, 0.46f, 0.35f, 1.0f };

    SetShaderValue(bb->shader, bb->loc_cylCenterXYZ, cyl_centre, SHADER_UNIFORM_VEC3);
    SetShaderValue(bb->shader, bb->loc_baseColor,    baseColor,  SHADER_UNIFORM_VEC4);

    rlActiveTextureSlot(1);
    rlEnableTexture(bb->panorama_tex.id);
    SetShaderValue(bb->shader, bb->loc_panoramaTex, (int[]){1}, SHADER_UNIFORM_INT);

    DrawModel(bb->model, (Vector3){0, 0, 0}, 1.0f, WHITE);

    rlActiveTextureSlot(1);
    rlDisableTexture();
    rlActiveTextureSlot(0);
}

/* ------------------------------------------------------------------ */
/* Draw                                                                 */
/* ------------------------------------------------------------------ */

void terrain_renderer_draw(Camera3D camera) {
    if (!g_tr.initialised) return;
    /* Terrain mesh renders only in TERRAIN mode or when a planar slope is
     * configured. scene.c gates the call too, but defending in depth here
     * means a direct external call can't accidentally draw a flat plane. */
    if (g_tr.params.mode != HAWKEYE_TERRAIN_MODE_TERRAIN &&
        g_tr.params.plane_deg == 0.0f) {
        return;
    }

    /* reset per-frame counters at the top of every draw. The
     * counters are incremented in the heightmap-eval / cull / draw paths
     * below and read by terrain_renderer_get_stats(). num_rings is
     * stamped from the compile-time NUM_RINGS so callers can iterate
     * tris_per_ring without re-querying the renderer layout. */
    memset(&g_tr.stats, 0, sizeof(g_tr.stats));
    g_tr.stats.num_rings = NUM_RINGS;

    const bool is_perspective = (camera.projection != CAMERA_ORTHOGRAPHIC);
    Vector2 fwd_xz = camera_forward_xz(camera);

    /*
     * Ring centre selection (two-ring layout).
     *
     * - Inner ring (index 0): locked to the camera position for FPV
     *   pivot precision. No bias of any kind.
     * - Outer ring (index 1): centred on the camera (no forward bias).
     *   With a 3 km radius the disc covers every camera-relative
     *   direction; an asymmetric shift would expose ground behind the
     *   camera that the ring no longer reaches and reintroduce the
     *   pre-redesign gap. Velocity prefetch is still applied when the
     *   drone is moving so the per-shift heightmap eval lands one frame
     *   before the grid snap.
     *
     * The forward-bias machinery is preserved (gated
     * on `ring->forward_biased`) so a future ring layout can re-enable
     * it by setting the flag.
     */
    const float vmag = sqrtf(g_tr.drone_vel_x * g_tr.drone_vel_x +
                             g_tr.drone_vel_z * g_tr.drone_vel_z);
    const bool has_velocity_prefetch = (vmag > VELOCITY_MIN_MPS);

    for (int i = 0; i < NUM_RINGS; i++) {
        ring_t *ring = &g_tr.rings[i];

        float bias_m = 0.0f;
        if (ring->forward_biased && is_perspective) {
            bias_m = ring->radius_m * FORWARD_BIAS_MID_FAR;
        }
        float prefetch_x = 0.0f;
        float prefetch_z = 0.0f;
        if (has_velocity_prefetch && i > 0) {
            /* Inner ring stays on camera for pivot precision; outer ring
             * picks up the velocity lookahead regardless of forward_bias. */
            prefetch_x = g_tr.drone_vel_x * VELOCITY_LOOKAHEAD_S;
            prefetch_z = g_tr.drone_vel_z * VELOCITY_LOOKAHEAD_S;
        }
        const float target_x_raw = camera.position.x + fwd_xz.x * bias_m + prefetch_x;
        const float target_z_raw = camera.position.z + fwd_xz.y * bias_m + prefetch_z;

        /* Snap target centre to the ring's vertex grid. */
        const float target_x = snap_to_step(target_x_raw, ring->vertex_step_m);
        const float target_z = snap_to_step(target_z_raw, ring->vertex_step_m);

        if (ring->dirty ||
            target_x != ring->center_x ||
            target_z != ring->center_z) {
            ring->center_x = target_x;
            ring->center_z = target_z;
            ring_evaluate_heightmap(ring);
        }
    }

    /* Frustum for per-patch culling. Computed once per frame. */
    Matrix vp = camera_view_projection(camera);
    frustum_t frustum = frustum_from_vp(vp);

    /* Theme tint — fixed warm-tan placeholder; wires to scene theme in
     * a follow-up. */
    const float baseColor[4] = { 0.50f, 0.42f, 0.32f, 1.0f };
    const float lightDir[3]  = { 0.30f, 0.80f, 0.40f };
    const float ambient      = 0.25f;
    /*
     * alpha-fade crossfade between rings is no longer
     * meaningful under the continuous-LOD architecture (the keep-band
     * discard + geomorph make the inner/outer transition geometry-clean,
     * so there is no overlap for a fade to hide). Set the uniforms to
     * values fragRingDist can never reach (it tops out at sqrt(2)) so
     * the smoothstep always returns 0 and alpha stays 1.0.
     */
    const float ringFadeStart = 100.0f;
    const float ringFadeEnd   = 101.0f;
    const float camPos[3]     = { camera.position.x, camera.position.y, camera.position.z };

    SetShaderValue(g_tr.shader, g_tr.loc_baseColor,     baseColor,     SHADER_UNIFORM_VEC4);
    SetShaderValue(g_tr.shader, g_tr.loc_lightDir,      lightDir,      SHADER_UNIFORM_VEC3);
    SetShaderValue(g_tr.shader, g_tr.loc_ambient,       &ambient,      SHADER_UNIFORM_FLOAT);
    SetShaderValue(g_tr.shader, g_tr.loc_ringFadeStart, &ringFadeStart, SHADER_UNIFORM_FLOAT);
    SetShaderValue(g_tr.shader, g_tr.loc_ringFadeEnd,   &ringFadeEnd,  SHADER_UNIFORM_FLOAT);
    SetShaderValue(g_tr.shader, g_tr.loc_camPos,        camPos,        SHADER_UNIFORM_VEC3);

    /* mesh resolution so the fragment shader can scale the per-quad
     * fract UV into [0, 1] within each mesh quad for edge detection. */
    const float meshResolution = (float)(RING_VERTS - 1);
    SetShaderValue(g_tr.shader, g_tr.loc_meshResolution, &meshResolution, SHADER_UNIFORM_FLOAT);

    /* theme-driven wireframe palette. wireA + wireB
     * define the low-Y / high-Y endpoints of the elevation gradient; the
     * fragment shader blends them by smoothstep over [wireYMin, wireYMax]
     * derived from the active terrain amplitude. Themes that want a
     * single uniform colour pass wireA == wireB. */
    SetShaderValue(g_tr.shader, g_tr.loc_wireColorA, g_tr.theme_wire_a, SHADER_UNIFORM_VEC3);
    SetShaderValue(g_tr.shader, g_tr.loc_wireColorB, g_tr.theme_wire_b, SHADER_UNIFORM_VEC3);
    SetShaderValue(g_tr.shader, g_tr.loc_fillColor,  g_tr.theme_fill,   SHADER_UNIFORM_VEC3);
    SetShaderValue(g_tr.shader, g_tr.loc_fogColor,   g_tr.theme_fog,    SHADER_UNIFORM_VEC3);

    /* fBm elevation restored, so the wireframe gradient again
     * tracks the active amplitude — [-amp, +amp] covers ~99 % of fBm
     * samples by construction. A small floor prevents the gradient from
     * collapsing to a degenerate band when amp = 0 (planar or flat
     * modes): every sample lands at 0 which is the midpoint of an
     * artificial 1 m band, harmless. */
    const float amp_band = (g_tr.params.amp > 1.0f) ? g_tr.params.amp : 1.0f;
    const float wireYMin = -amp_band;
    const float wireYMax = +amp_band;
    SetShaderValue(g_tr.shader, g_tr.loc_wireYMin, &wireYMin, SHADER_UNIFORM_FLOAT);
    SetShaderValue(g_tr.shader, g_tr.loc_wireYMax, &wireYMax, SHADER_UNIFORM_FLOAT);

    /* solid-mode toggle. When enabled the fragment
     * shader skips the wireframe path and outputs the theme's `fillColor`
     * with Lambertian shading from the existing `lightDir` + `ambient`
     * uniforms. removed the texture sampling, so no slot binding
     * is needed for solid mode. */
    const int solid_mode_flag = g_tr.solid_mode_enabled ? 1 : 0;
    SetShaderValue(g_tr.shader, g_tr.loc_uSolidMode, &solid_mode_flag,
                   SHADER_UNIFORM_INT);
    SetShaderValue(g_tr.shader, g_tr.loc_uShadingMode, &g_tr.shading_mode,
                   SHADER_UNIFORM_INT);

    /*
     * boundary radii shared between the rings:
     *   inner  -> middle boundary = inner.radius_m  = 256 m
     *   middle -> outer  boundary = middle.radius_m = 1024 m
     * Each ring's vertex-shader geomorph in its outer 15 % converges
     * Y to the next coarser ring's heightmap, so all transitions agree
     * at exactly the boundary radius.
     */
    const float ring_outer_boundary_m[NUM_RINGS] = {
        g_tr.rings[0].radius_m,           /* inner outer edge */
        g_tr.rings[1].radius_m,           /* middle outer edge */
        1.0e9f,                           /* outer extends to infinity */
    };

    /* Draw inner -> outer so the dense ring's depth wins at overlapping
     * pixels (the keep-band discard normally prevents overlap, but the
     * inner-first order is also the natural front-to-back order for
     * early-Z). The billboard below picks up the silhouette beyond the
     * outer ring. */
    for (int i = 0; i < NUM_RINGS; i++) {
        ring_t *ring = &g_tr.rings[i];

        const float ringCenterXZ[2] = { ring->center_x, ring->center_z };
        SetShaderValue(g_tr.shader, g_tr.loc_ringScale,    &ring->radius_m, SHADER_UNIFORM_FLOAT);
        SetShaderValue(g_tr.shader, g_tr.loc_ringCenterXZ, ringCenterXZ,    SHADER_UNIFORM_VEC2);

        /* Bind the per-ring heightmap to texture slot 1. */
        rlActiveTextureSlot(1);
        rlEnableTexture(ring->height_tex.id);
        SetShaderValue(g_tr.shader, g_tr.loc_heightTex, (int[]){1}, SHADER_UNIFORM_INT);

        /*
         * per-ring continuous-LOD uniforms (generalised
         * to support N levels). Each ring carries:
         *   - A keep-band [keepMin, keepMax] in world metres. Fragments
         *     outside discard so rings cover disjoint annuli.
         *   - A geomorph target (the next coarser ring) bound to texture
         *     slot 2. The vertex shader blends this ring's Y toward the
         *     target ring's heightmap in fragRingDist [0.85, 1.0] so the
         *     edge converges to the target's surface. The outermost ring
         *     has no target (geomorphActive = 0).
         */
        const float keepMin = (i == 0) ? 0.0f : ring_outer_boundary_m[i - 1];
        const float keepMax = ring_outer_boundary_m[i];
        SetShaderValue(g_tr.shader, g_tr.loc_keepBandMin, &keepMin, SHADER_UNIFORM_FLOAT);
        SetShaderValue(g_tr.shader, g_tr.loc_keepBandMax, &keepMax, SHADER_UNIFORM_FLOAT);

        const bool has_geomorph_target = (i + 1 < NUM_RINGS);
        if (has_geomorph_target) {
            ring_t *target = &g_tr.rings[i + 1];
            rlActiveTextureSlot(2);
            rlEnableTexture(target->height_tex.id);
            SetShaderValue(g_tr.shader, g_tr.loc_outerHeightTex, (int[]){2},
                           SHADER_UNIFORM_INT);

            const float targetCenter[2] = { target->center_x, target->center_z };
            SetShaderValue(g_tr.shader, g_tr.loc_outerRingCenterXZ, targetCenter,
                           SHADER_UNIFORM_VEC2);
            SetShaderValue(g_tr.shader, g_tr.loc_outerRingScale, &target->radius_m,
                           SHADER_UNIFORM_FLOAT);
            SetShaderValue(g_tr.shader, g_tr.loc_geomorphActive, (int[]){1},
                           SHADER_UNIFORM_INT);
        } else {
            SetShaderValue(g_tr.shader, g_tr.loc_geomorphActive, (int[]){0},
                           SHADER_UNIFORM_INT);
        }

        for (int p = 0; p < ring->num_patches; p++) {
            const patch_t *patch = &ring->patches[p];

            /*
             * Patch world-space bounding sphere. Centre is the ring centre
             * shifted by patch.offset * ringScale; radius bounds the
             * patch corners (sqrt(2) for a square) plus vertical slack
             * for terrain amplitude and a small safety margin.
             */
            const Vector3 patch_centre = {
                ring->center_x + patch->offset_x * ring->radius_m,
                0.0f,
                ring->center_z + patch->offset_z * ring->radius_m,
            };
            const float patch_world_half = patch->half_extent * ring->radius_m;
            /* terrain is flat (default) or planar (`plane_deg`),
             * so vertical-extent slack only needs to cover the planar
             * slope rise across the patch. Use the patch radius as a
             * conservative slack (tan(45°) = 1, slope rarely exceeds that)
             * with a 1 m floor for the flat case. */
            const float vertical_slack   = fmaxf(patch_world_half, 1.0f);
            const float patch_radius     = patch_world_half * 1.41421356f
                                         + vertical_slack + FRUSTUM_SLACK_M;

            /*
             * In orthographic mode we always draw every patch — the
             * top-down view typically wants the full disc and the
             * frustum-extraction math is less informative for an ortho
             * projection that does not converge to a point.
             */
            if (is_perspective &&
                !sphere_in_frustum(&frustum, patch_centre, patch_radius)) {
                g_tr.stats.patches_culled++;
                continue;
            }

            const float patchOffset[2]    = { patch->offset_x, patch->offset_z };
            const float texCoordOffset[2] = { patch->tex_offset_u, patch->tex_offset_v };
            SetShaderValue(g_tr.shader, g_tr.loc_patchOffset,    patchOffset,
                           SHADER_UNIFORM_VEC2);
            SetShaderValue(g_tr.shader, g_tr.loc_patchScale,     &patch->half_extent,
                           SHADER_UNIFORM_FLOAT);
            SetShaderValue(g_tr.shader, g_tr.loc_texCoordOffset, texCoordOffset,
                           SHADER_UNIFORM_VEC2);
            SetShaderValue(g_tr.shader, g_tr.loc_texCoordScale,  &patch->tex_half_extent,
                           SHADER_UNIFORM_FLOAT);

            DrawModel(g_tr.model, (Vector3){0, 0, 0}, 1.0f, WHITE);

            /* stats: shared mesh is a (RING_VERTS-1) x (RING_VERTS-1)
             * quad grid drawn as two triangles per quad. The same mesh is
             * reused for every patch / ring so the per-patch contribution
             * is a compile-time constant. */
            const int patch_tris = (RING_VERTS - 1) * (RING_VERTS - 1) * 2;
            g_tr.stats.patches_drawn++;
            g_tr.stats.tris_per_ring[i] += patch_tris;
            g_tr.stats.tris_drawn_total += patch_tris;
        }

        rlActiveTextureSlot(1);
        rlDisableTexture();
        /* also clear texture slot 2 if this ring's geomorph
         * bound a target ring's heightmap there (every ring except the
         * outermost). */
        if (i + 1 < NUM_RINGS) {
            rlActiveTextureSlot(2);
            rlDisableTexture();
        }
        rlActiveTextureSlot(0);
    }

    /* Far-horizon panoramic billboard sits beyond the outer ring. */
    billboard_maybe_regenerate(&g_tr.billboard, camera, fwd_xz);
    billboard_draw(&g_tr.billboard, camera);
    /* Billboard mesh is BILLBOARD_QUADS quads (two tris each), always
     * drawn in one DrawModel call -- not subject to per-patch culling. */
    g_tr.stats.tris_billboard = BILLBOARD_QUADS * 2;
    g_tr.stats.tris_drawn_total += g_tr.stats.tris_billboard;
}

void terrain_renderer_get_stats(terrain_render_stats_t *out) {
    if (!out) return;
    *out = g_tr.stats;
}

void terrain_renderer_shutdown(void) {
    if (!g_tr.initialised) return;

    billboard_destroy(&g_tr.billboard);
    for (int i = 0; i < NUM_RINGS; i++) {
        ring_destroy(&g_tr.rings[i]);
    }
    UnloadModel(g_tr.model);  /* this also unloads the mesh */
    UnloadShader(g_tr.shader);

    g_tr.initialised = false;
}
