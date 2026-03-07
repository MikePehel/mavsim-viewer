#include "scene.h"
#include "raymath.h"
#include "rlgl.h"
#include <stdlib.h>
#include <stdio.h>
#include <math.h>

#define GROUND_SIZE 5000.0f   // half-size in meters (10km total, matching jMAVSim)
#define GROUND_TILES 100.0f   // texture repeat count
#define SKY_RADIUS 6000.0f    // sky sphere radius (larger than ground diagonal)

// Grid shared settings
#define GRID_EXTENT      500.0f
#define GRID_SPACING     10.0f
#define GRID_MAJOR_EVERY 5

// Grid mode colors
#define GRID_SKY       (Color){ 56,  56,  60, 255 }
#define GRID_GROUND    (Color){ 32,  32,  34, 255 }
#define GRID_MINOR     (Color){ 97,  97,  97, 128 }
#define GRID_MAJOR     (Color){ 143, 143, 143, 128 }
#define GRID_AXIS_X    (Color){ 200,  60,  60, 180 }
#define GRID_AXIS_Z    (Color){ 60,   60, 200, 180 }

// Rez mode colors
#define REZ_SKY        (Color){ 12,  12,  18, 255 }  // near-black
#define REZ_GROUND     (Color){ 2,    2,   4, 255 }  // black
#define REZ_MINOR      (Color){ 0,  204, 218, 50 }   // teal, subtle
#define REZ_MAJOR      (Color){ 0,  204, 218, 140 }  // teal, bright
#define REZ_AXIS_X     (Color){ 0,  204, 218, 220 }  // teal, full
#define REZ_AXIS_Z     (Color){ 0,  204, 218, 220 }  // teal, full

// 1988 mode colors (synthwave)
#define SYNTH_SKY      (Color){ 8,   8,  20, 255 }
#define SYNTH_GROUND   (Color){ 5,   5,  16, 255 }
#define SYNTH_MINOR    (Color){ 255, 20, 100, 50 }   // hot pink, subtle
#define SYNTH_MAJOR    (Color){ 255, 20, 100, 160 }   // hot pink, bright
#define SYNTH_AXIS_X   (Color){ 255, 20, 100, 220 }   // hot pink, full
#define SYNTH_AXIS_Z   (Color){ 255, 20, 100, 220 }   // hot pink, full

// 1988 mountain settings
#define MTN_COLOR      (Color){ 1, 156, 227, 255 }    // #019CE3 teal
#define MTN_COLS       40       // subdivisions along edge
#define MTN_ROWS       12       // subdivisions deep
#define MTN_DEPTH      400.0f   // how far mountains extend from grid edge
#define MTN_PEAK       350.0f   // max mountain height

static float mtn_hash(int x, int y) {
    unsigned int h = (unsigned int)(x * 7919 + y * 104729 + 31337);
    h ^= h >> 13;
    h *= 0x5bd1e995;
    h ^= h >> 15;
    return (float)(h & 0xFFFF) / 65535.0f;
}

// Smooth value noise
static float mtn_vnoise(float x, float y) {
    int ix = (int)floorf(x), iy = (int)floorf(y);
    float fx = x - ix, fy = y - iy;
    fx = fx * fx * (3.0f - 2.0f * fx);
    fy = fy * fy * (3.0f - 2.0f * fy);
    float a = mtn_hash(ix, iy), b = mtn_hash(ix+1, iy);
    float c = mtn_hash(ix, iy+1), d = mtn_hash(ix+1, iy+1);
    return a + (b-a)*fx + (c-a)*fy + (a-b-c+d)*fx*fy;
}

// FBM noise with 3 octaves
static float mtn_fbm(float x, float y) {
    float v = 0.0f;
    v += mtn_vnoise(x, y) * 1.0f;
    v += mtn_vnoise(x*2.0f + 5.3f, y*2.0f + 1.7f) * 0.5f;
    v += mtn_vnoise(x*4.0f + 9.1f, y*4.0f + 3.2f) * 0.25f;
    return v / 1.75f;
}

// Get height at a point on an edge strip
static float mtn_get_height(float along, float deep, int edge) {
    // along: 0-1 along edge, deep: 0-1 from grid edge outward
    // Depth envelope: 0 at grid edge, rises, stays, tapers at back
    float env = sinf(deep * 3.14159f);
    env = powf(env, 0.6f); // broader peak

    // Along-edge taper: fade to ground at both ends
    float edge_taper = 1.0f;
    if (along < 0.15f) edge_taper = along / 0.15f;
    else if (along > 0.85f) edge_taper = (1.0f - along) / 0.15f;
    edge_taper = edge_taper * edge_taper; // smooth quadratic
    env *= edge_taper;

    float ox = along * 5.0f + edge * 13.7f;
    float oy = deep * 3.0f + edge * 7.3f;
    float n = mtn_fbm(ox, oy);
    // Ridged noise for sharp jagged peaks
    float ridge = 1.0f - fabsf(n * 2.0f - 1.0f);
    ridge = powf(ridge, 2.0f);
    // Extra altitude variation along the ridgeline
    float variation = mtn_vnoise(along * 3.0f + edge * 5.1f, deep * 1.5f + edge * 2.3f);
    variation = 0.3f + 0.7f * variation; // range 0.3 to 1.0
    float h = env * ridge * variation * MTN_PEAK;
    // Gently push low areas down
    if (h < MTN_PEAK * 0.15f) h *= 0.7f;
    return h;
}

static void mtn_vert(Mesh *m, int vi, float x, float y, float z,
                     float bx, float by, unsigned char r, unsigned char g, unsigned char b) {
    m->vertices[vi*3+0] = x;  m->vertices[vi*3+1] = y;  m->vertices[vi*3+2] = z;
    m->texcoords[vi*2+0] = bx; m->texcoords[vi*2+1] = by;
    m->colors[vi*4+0] = r; m->colors[vi*4+1] = g; m->colors[vi*4+2] = b; m->colors[vi*4+3] = 255;
}

static Mesh gen_mountains(void) {
    // 4 edges, each MTN_COLS x MTN_ROWS quads, 2 tris per quad, 3 verts per tri
    int total_tris = 4 * MTN_COLS * MTN_ROWS * 2;
    int vert_count = total_tris * 3;

    Mesh mesh = { 0 };
    mesh.triangleCount = total_tris;
    mesh.vertexCount = vert_count;
    mesh.vertices = RL_CALLOC(vert_count * 3, sizeof(float));
    mesh.texcoords = RL_CALLOC(vert_count * 2, sizeof(float));
    mesh.colors = RL_CALLOC(vert_count * 4, sizeof(unsigned char));

    float ext = GRID_EXTENT;
    unsigned char cr = MTN_COLOR.r, cg = MTN_COLOR.g, cb = MTN_COLOR.b;

    // Edge: start_x, start_z, along_dx, along_dz, out_nx, out_nz
    float edges[4][6] = {
        { -ext, -ext,  1,  0,  0, -1 },  // south
        {  ext, -ext,  0,  1,  1,  0 },  // east
        {  ext,  ext, -1,  0,  0,  1 },  // north
        { -ext,  ext,  0, -1, -1,  0 },  // west
    };

    int vi = 0;
    for (int e = 0; e < 4; e++) {
        float sx = edges[e][0], sz = edges[e][1];
        float adx = edges[e][2], adz = edges[e][3];
        float onx = edges[e][4], onz = edges[e][5];
        float elen = 2.0f * ext;

        for (int c = 0; c < MTN_COLS; c++) {
            for (int r = 0; r < MTN_ROWS; r++) {
                float a0 = (float)c / MTN_COLS, a1 = (float)(c+1) / MTN_COLS;
                float d0 = (float)r / MTN_ROWS, d1 = (float)(r+1) / MTN_ROWS;

                // World positions of 4 quad corners
                float x00 = sx + adx*a0*elen + onx*d0*MTN_DEPTH;
                float z00 = sz + adz*a0*elen + onz*d0*MTN_DEPTH;
                float y00 = mtn_get_height(a0, d0, e);

                float x10 = sx + adx*a1*elen + onx*d0*MTN_DEPTH;
                float z10 = sz + adz*a1*elen + onz*d0*MTN_DEPTH;
                float y10 = mtn_get_height(a1, d0, e);

                float x01 = sx + adx*a0*elen + onx*d1*MTN_DEPTH;
                float z01 = sz + adz*a0*elen + onz*d1*MTN_DEPTH;
                float y01 = mtn_get_height(a0, d1, e);

                float x11 = sx + adx*a1*elen + onx*d1*MTN_DEPTH;
                float z11 = sz + adz*a1*elen + onz*d1*MTN_DEPTH;
                float y11 = mtn_get_height(a1, d1, e);

                // Tri 1: 00, 10, 01 — barycentric (1,0), (0,1), (0,0)
                mtn_vert(&mesh, vi++, x00,y00,z00, 1,0, cr,cg,cb);
                mtn_vert(&mesh, vi++, x10,y10,z10, 0,1, cr,cg,cb);
                mtn_vert(&mesh, vi++, x01,y01,z01, 0,0, cr,cg,cb);
                // Tri 2: 10, 11, 01
                mtn_vert(&mesh, vi++, x10,y10,z10, 1,0, cr,cg,cb);
                mtn_vert(&mesh, vi++, x11,y11,z11, 0,1, cr,cg,cb);
                mtn_vert(&mesh, vi++, x01,y01,z01, 0,0, cr,cg,cb);
            }
        }
    }

    UploadMesh(&mesh, false);
    return mesh;
}

void scene_init(scene_t *s) {
    s->cam_mode = CAM_MODE_CHASE;
    s->view_mode = VIEW_GRID;
    s->chase_distance = 3.0f;
    s->chase_yaw = 0.0f;
    s->chase_pitch = 0.4f;  // ~23° above horizontal

    // Camera
    s->camera = (Camera3D){
        .position = (Vector3){0.0f, 5.0f, 10.0f},
        .target   = (Vector3){0.0f, 0.0f, 0.0f},
        .up       = (Vector3){0.0f, 1.0f, 0.0f},
        .fovy     = 60.0f,
        .projection = CAMERA_PERSPECTIVE,
    };

    // Ground plane mesh (10x10 subdivisions to avoid diagonal tearing)
    Mesh ground_mesh = GenMeshPlane(GROUND_SIZE * 2, GROUND_SIZE * 2, 10, 10);
    s->ground = LoadModelFromMesh(ground_mesh);

    // Ground texture
    Image grass_img = LoadImage("textures/grass3.jpg");
    if (grass_img.data != NULL) {
        s->ground_tex = LoadTextureFromImage(grass_img);
        SetTextureFilter(s->ground_tex, TEXTURE_FILTER_TRILINEAR);
        SetTextureWrap(s->ground_tex, TEXTURE_WRAP_REPEAT);
        UnloadImage(grass_img);

        s->ground.materials[0].maps[MATERIAL_MAP_DIFFUSE].texture = s->ground_tex;

        // Scale UVs for tiling
        Mesh *m = &s->ground.meshes[0];
        for (int i = 0; i < m->vertexCount; i++) {
            m->texcoords[i * 2 + 0] *= GROUND_TILES;
            m->texcoords[i * 2 + 1] *= GROUND_TILES;
        }
        UpdateMeshBuffer(*m, 1, m->texcoords, m->vertexCount * 2 * sizeof(float), 0);
    }

    // Sky sphere — we draw from inside with backface culling disabled
    Mesh sky_mesh = GenMeshSphere(SKY_RADIUS, 36, 36);
    s->sky_sphere = LoadModelFromMesh(sky_mesh);

    // Sky texture
    Image sky_img = LoadImage("textures/HDR_040_Field_Bg.jpg");
    if (sky_img.data != NULL) {
        s->sky_tex = LoadTextureFromImage(sky_img);
        SetTextureFilter(s->sky_tex, TEXTURE_FILTER_BILINEAR);
        UnloadImage(sky_img);

        s->sky_sphere.materials[0].maps[MATERIAL_MAP_DIFFUSE].texture = s->sky_tex;
    }

    // Grid shader and plane (100x100 subdivisions for fwidth() precision)
    s->grid_shader = LoadShader("shaders/grid.vs", "shaders/grid.fs");
    s->loc_colGround  = GetShaderLocation(s->grid_shader, "colGround");
    s->loc_colMinor   = GetShaderLocation(s->grid_shader, "colMinor");
    s->loc_colMajor   = GetShaderLocation(s->grid_shader, "colMajor");
    s->loc_colAxisX   = GetShaderLocation(s->grid_shader, "colAxisX");
    s->loc_colAxisZ   = GetShaderLocation(s->grid_shader, "colAxisZ");
    s->loc_spacing    = GetShaderLocation(s->grid_shader, "spacing");
    s->loc_majorEvery = GetShaderLocation(s->grid_shader, "majorEvery");
    s->loc_axisWidth  = GetShaderLocation(s->grid_shader, "axisWidth");
    s->loc_matModel   = GetShaderLocation(s->grid_shader, "matModel");

    Mesh grid_mesh = GenMeshPlane(GROUND_SIZE * 2, GROUND_SIZE * 2, 100, 100);
    s->grid_plane = LoadModelFromMesh(grid_mesh);
    s->grid_plane.materials[0].shader = s->grid_shader;

    // 1988 mountains
    s->mtn_shader = LoadShader("shaders/mountain.vs", "shaders/mountain.fs");
    Mesh mtn_mesh = gen_mountains();
    s->mountains = LoadModelFromMesh(mtn_mesh);
    s->mountains.materials[0].shader = s->mtn_shader;
}

static void update_chase_camera(scene_t *s, Vector3 pos) {
    float dist = s->chase_distance;

    // Spherical coordinates around the target
    float cam_x = pos.x + dist * cosf(s->chase_pitch) * sinf(s->chase_yaw);
    float cam_y = pos.y + dist * sinf(s->chase_pitch);
    float cam_z = pos.z + dist * cosf(s->chase_pitch) * cosf(s->chase_yaw);

    s->camera.target = pos;
    s->camera.position = (Vector3){cam_x, cam_y, cam_z};
    s->camera.up = (Vector3){0, 1, 0};
}

static void update_fpv_camera(scene_t *s, Vector3 pos, Quaternion rot) {
    s->camera.position = pos;
    Vector3 forward = Vector3RotateByQuaternion((Vector3){0, 0, -1}, rot);
    s->camera.target = Vector3Add(pos, forward);
    Vector3 up = Vector3RotateByQuaternion((Vector3){0, 1, 0}, rot);
    s->camera.up = up;
}

void scene_update_camera(scene_t *s, Vector3 vehicle_pos, Quaternion vehicle_rot) {
    switch (s->cam_mode) {
        case CAM_MODE_CHASE:
            update_chase_camera(s, vehicle_pos);
            break;
        case CAM_MODE_FPV:
            update_fpv_camera(s, vehicle_pos, vehicle_rot);
            break;
        default:
            break;
    }
}

void scene_handle_input(scene_t *s) {
    if (IsKeyPressed(KEY_C)) {
        s->cam_mode = (s->cam_mode + 1) % CAM_MODE_COUNT;
        const char *names[] = {"Chase", "FPV"};
        printf("Camera: %s\n", names[s->cam_mode]);

        s->camera.up = (Vector3){0, 1, 0};
    }

    if (IsKeyPressed(KEY_V)) {
        // If in hidden mode, return to Grid; otherwise cycle public modes
        if (s->view_mode >= VIEW_COUNT)
            s->view_mode = VIEW_GRID;
        else
            s->view_mode = (s->view_mode + 1) % VIEW_COUNT;
        const char *names[] = {"Grid", "jMAVSim", "Rez"};
        printf("View: %s\n", names[s->view_mode]);
    }

    // Ctrl+1988 sequence detection for hidden mode
    if (IsKeyDown(KEY_LEFT_CONTROL)) {
        int expected[] = { KEY_ONE, KEY_NINE, KEY_EIGHT, KEY_EIGHT };
        if (s->seq_1988 < 4 && IsKeyPressed(expected[s->seq_1988])) {
            s->seq_1988++;
            if (s->seq_1988 == 4) {
                s->view_mode = (s->view_mode == VIEW_1988) ? VIEW_GRID : VIEW_1988;
                s->seq_1988 = 0;
            }
        } else if (IsKeyPressed(KEY_ONE) || IsKeyPressed(KEY_TWO) || IsKeyPressed(KEY_THREE) ||
                   IsKeyPressed(KEY_FOUR) || IsKeyPressed(KEY_FIVE) || IsKeyPressed(KEY_SIX) ||
                   IsKeyPressed(KEY_SEVEN) || IsKeyPressed(KEY_EIGHT) || IsKeyPressed(KEY_NINE) ||
                   IsKeyPressed(KEY_ZERO)) {
            s->seq_1988 = IsKeyPressed(KEY_ONE) ? 1 : 0;
        }
    } else {
        s->seq_1988 = 0;
    }

    // Mouse drag to orbit (left button)
    if (s->cam_mode == CAM_MODE_CHASE && IsMouseButtonDown(MOUSE_BUTTON_LEFT)) {
        Vector2 delta = GetMouseDelta();
        s->chase_yaw   -= delta.x * 0.005f;
        s->chase_pitch += delta.y * 0.005f;

        // Clamp pitch to avoid flipping
        if (s->chase_pitch < -1.2f) s->chase_pitch = -1.2f;
        if (s->chase_pitch > 1.4f) s->chase_pitch = 1.4f;
    }

    // Scroll wheel FOV zoom
    float wheel = GetMouseWheelMove();
    if (wheel != 0.0f) {
        s->camera.fovy -= wheel * 5.0f;
        if (s->camera.fovy < 10.0f) s->camera.fovy = 10.0f;
        if (s->camera.fovy > 120.0f) s->camera.fovy = 120.0f;
    }
}

static void color_to_vec4(Color c, float out[4]) {
    out[0] = c.r / 255.0f;
    out[1] = c.g / 255.0f;
    out[2] = c.b / 255.0f;
    out[3] = c.a / 255.0f;
}

static void draw_shader_grid(const scene_t *s,
    Color ground, Color minor, Color major, Color axis_x, Color axis_z)
{
    float v[4];
    color_to_vec4(ground, v); SetShaderValue(s->grid_shader, s->loc_colGround, v, SHADER_UNIFORM_VEC4);
    color_to_vec4(minor, v);  SetShaderValue(s->grid_shader, s->loc_colMinor, v, SHADER_UNIFORM_VEC4);
    color_to_vec4(major, v);  SetShaderValue(s->grid_shader, s->loc_colMajor, v, SHADER_UNIFORM_VEC4);
    color_to_vec4(axis_x, v); SetShaderValue(s->grid_shader, s->loc_colAxisX, v, SHADER_UNIFORM_VEC4);
    color_to_vec4(axis_z, v); SetShaderValue(s->grid_shader, s->loc_colAxisZ, v, SHADER_UNIFORM_VEC4);

    float spacing = GRID_SPACING;
    float majorEvery = (float)GRID_MAJOR_EVERY;
    float axisWidth = 1.5f;
    SetShaderValue(s->grid_shader, s->loc_spacing, &spacing, SHADER_UNIFORM_FLOAT);
    SetShaderValue(s->grid_shader, s->loc_majorEvery, &majorEvery, SHADER_UNIFORM_FLOAT);
    SetShaderValue(s->grid_shader, s->loc_axisWidth, &axisWidth, SHADER_UNIFORM_FLOAT);

    // Pass identity model matrix (plane is at origin)
    Matrix model = MatrixIdentity();
    SetShaderValueMatrix(s->grid_shader, s->loc_matModel, model);

    DrawModel(s->grid_plane, (Vector3){0, 0, 0}, 1.0f, WHITE);
}

void scene_draw(const scene_t *s) {
    if (s->view_mode == VIEW_GRID) {
        draw_shader_grid(s, GRID_GROUND, GRID_MINOR, GRID_MAJOR, GRID_AXIS_X, GRID_AXIS_Z);
    } else if (s->view_mode == VIEW_REZ) {
        draw_shader_grid(s, REZ_GROUND, REZ_MINOR, REZ_MAJOR, REZ_AXIS_X, REZ_AXIS_Z);
    } else if (s->view_mode == VIEW_1988) {
        draw_shader_grid(s, SYNTH_GROUND, SYNTH_MINOR, SYNTH_MAJOR, SYNTH_AXIS_X, SYNTH_AXIS_Z);
        // Draw mountains with wireframe shader
        rlDisableBackfaceCulling();
        DrawModel(s->mountains, (Vector3){0, 0, 0}, 1.0f, WHITE);
        rlEnableBackfaceCulling();
    } else {
        // Sky sphere centered on camera — disable culling so we see it from inside
        rlDisableBackfaceCulling();
        DrawModel(s->sky_sphere, s->camera.position, 1.0f, WHITE);
        rlEnableBackfaceCulling();

        // Ground
        DrawModel(s->ground, (Vector3){0, 0, 0}, 1.0f, WHITE);


    }
}

void scene_draw_sky(const scene_t *s) {
    switch (s->view_mode) {
        case VIEW_GRID: ClearBackground(GRID_SKY); break;
        case VIEW_REZ:  ClearBackground(REZ_SKY);   break;
        case VIEW_1988: ClearBackground(SYNTH_SKY); break;
        default:        ClearBackground((Color){135, 206, 235, 255}); break;
    }
}

void scene_cleanup(scene_t *s) {
    UnloadModel(s->ground);
    UnloadModel(s->sky_sphere);
    UnloadModel(s->grid_plane);
    UnloadShader(s->grid_shader);
    if (s->ground_tex.id > 0) UnloadTexture(s->ground_tex);
    if (s->sky_tex.id > 0) UnloadTexture(s->sky_tex);
    UnloadModel(s->mountains);
    UnloadShader(s->mtn_shader);
}
