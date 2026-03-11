#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#ifdef _MSC_VER
#define _USE_MATH_DEFINES
#endif
#include <math.h>

#include "raylib.h"
#include "raymath.h"
#include "data_source.h"
#include "ulog_replay.h"
#include "vehicle.h"
#include "scene.h"
#include "hud.h"
#include "debug_panel.h"
#include "ortho_panel.h"
#include "asset_path.h"

#define MAX_VEHICLES 17
#define MISSILE_COUNT 16

static const Color vehicle_colors[MAX_VEHICLES] = {
    {230, 230, 230, 255}, // 0: white (default single)
    {230,  41,  55, 255}, // 1: red
    {  0, 228,  48, 255}, // 2: green
    {  0, 121, 241, 255}, // 3: blue
    {253, 249,   0, 255}, // 4: yellow
    {255,   0, 255, 255}, // 5: magenta
    {  0, 255, 255, 255}, // 6: cyan
    {255, 161,   0, 255}, // 7: orange
    {200, 122, 255, 255}, // 8: purple
    {127, 106,  79, 255}, // 9: brown
    {255, 109, 194, 255}, // 10: pink
    {  0, 182, 172, 255}, // 11: teal
    {135, 206, 235, 255}, // 12: sky blue
    {255, 203, 164, 255}, // 13: peach
    {170, 255, 128, 255}, // 14: lime
    {200, 200, 200, 255}, // 15: silver
    {255,  50,  50, 255}, // 16: target (red)
};

// ── Anime missile barrage simulation ────────────────────────────────────────
// 16 drones launch from a near-single origin, arc outward in a spherical fan,
// then heat-seek a VTOL target with oscillating sidewind for visual drama.
typedef struct {
    float theta;        // spherical azimuth of fan-out direction
    float phi;          // spherical elevation of fan-out direction
    float phase;        // oscillation phase offset (unique per missile)
    float osc_freq;     // sidewind oscillation frequency
    float osc_amp;      // sidewind amplitude
    float arc_radius;   // how far it arcs out before seeking
    float speed;        // base speed multiplier
} missile_params_t;

static void missile_barrage_init(missile_params_t params[MISSILE_COUNT]) {
    for (int i = 0; i < MISSILE_COUNT; i++) {
        // Distribute missiles in a spherical fan (golden angle spiral)
        float golden = (float)M_PI * (3.0f - sqrtf(5.0f));
        float y = 1.0f - ((float)i / (MISSILE_COUNT - 1)) * 1.4f; // -0.4 to 1.0
        float radius_at_y = sqrtf(1.0f - y * y);
        float angle = (float)i * golden;

        params[i].theta = angle;
        params[i].phi = asinf(y * 0.6f + 0.2f); // bias upward slightly
        params[i].phase = (float)i * 0.7f + (float)i * (float)i * 0.03f;
        params[i].osc_freq = 3.0f + (float)(i % 5) * 1.2f;
        params[i].osc_amp = 1.5f + (float)(i % 3) * 0.8f;
        params[i].arc_radius = 15.0f + (float)(i % 4) * 5.0f;
        params[i].speed = 0.9f + (float)(i % 7) * 0.05f;
    }
}

// Generate hil_state for a missile at time t (seconds)
static void missile_barrage_state(const missile_params_t *p, float t,
                                  Vector3 origin, Vector3 target,
                                  hil_state_t *out_state)
{
    // Phase 1: fan out in spherical arc (0 to ~3s)
    // Phase 2: seek target with oscillation (~3s onward)
    float fan_duration = 2.5f * (1.0f / p->speed);
    float seek_start = fan_duration;
    float total_flight = seek_start + 6.0f; // converge over ~6s

    if (t > total_flight) t = total_flight; // clamp

    float progress = t / total_flight; // 0 to 1 overall

    // Fan-out direction in world space
    float ct = cosf(p->theta), st = sinf(p->theta);
    float cp = cosf(p->phi), sp = sinf(p->phi);
    Vector3 fan_dir = { ct * cp, sp, st * cp };

    // Arc apex point
    Vector3 apex = {
        origin.x + fan_dir.x * p->arc_radius,
        origin.y + fan_dir.y * p->arc_radius + 5.0f,
        origin.z + fan_dir.z * p->arc_radius
    };

    Vector3 pos;
    if (t < seek_start) {
        // Phase 1: accelerating arc from origin toward apex
        float u = t / seek_start;
        float ease = u * u * (3.0f - 2.0f * u); // smoothstep
        pos.x = origin.x + (apex.x - origin.x) * ease;
        pos.y = origin.y + (apex.y - origin.y) * ease;
        pos.z = origin.z + (apex.z - origin.z) * ease;
    } else {
        // Phase 2: curved seek from apex toward target
        float u = (t - seek_start) / (total_flight - seek_start);
        float ease = u * u * (3.0f - 2.0f * u); // smoothstep into target

        pos.x = apex.x + (target.x - apex.x) * ease;
        pos.y = apex.y + (target.y - apex.y) * ease;
        pos.z = apex.z + (target.z - apex.z) * ease;
    }

    // Sidewind oscillation (perpendicular to travel direction)
    Vector3 to_target = { target.x - pos.x, target.y - pos.y, target.z - pos.z };
    float dist = sqrtf(to_target.x*to_target.x + to_target.y*to_target.y + to_target.z*to_target.z);
    if (dist > 0.1f) {
        to_target.x /= dist; to_target.y /= dist; to_target.z /= dist;
    }
    // Perpendicular vectors for oscillation
    Vector3 up = {0, 1, 0};
    Vector3 side = {
        to_target.y * up.z - to_target.z * up.y,
        to_target.z * up.x - to_target.x * up.z,
        to_target.x * up.y - to_target.y * up.x
    };
    float slen = sqrtf(side.x*side.x + side.y*side.y + side.z*side.z);
    if (slen > 0.001f) { side.x /= slen; side.y /= slen; side.z /= slen; }

    Vector3 up2 = {
        to_target.y * side.z - to_target.z * side.y,
        to_target.z * side.x - to_target.x * side.z,
        to_target.x * side.y - to_target.y * side.x
    };

    // Oscillation decays as missile approaches target
    float decay = 1.0f - progress * progress;
    float osc_h = sinf(t * p->osc_freq + p->phase) * p->osc_amp * decay;
    float osc_v = cosf(t * p->osc_freq * 0.7f + p->phase * 1.3f) * p->osc_amp * 0.6f * decay;

    pos.x += side.x * osc_h + up2.x * osc_v;
    pos.y += side.y * osc_h + up2.y * osc_v;
    pos.z += side.z * osc_h + up2.z * osc_v;

    // Convert to lat/lon/alt for hil_state
    // Use a simple NED offset: origin at (0,0,alt), position is meters offset
    // We'll set lat/lon relative to a base and encode as degE7
    double base_lat = 47.397742;
    double base_lon = 8.545594;
    double base_alt = 489.4;

    double meters_per_deg_lat = 111132.0;
    double meters_per_deg_lon = 111132.0 * cos(base_lat * M_PI / 180.0);

    out_state->lat = (int32_t)((base_lat + (double)pos.z / meters_per_deg_lat) * 1e7);
    out_state->lon = (int32_t)((base_lon + (double)pos.x / meters_per_deg_lon) * 1e7);
    out_state->alt = (int32_t)((base_alt + (double)pos.y) * 1000.0);

    // Velocity (finite difference approximation via direction)
    float spd = 8.0f + progress * 15.0f; // accelerating
    out_state->vx = (int16_t)(to_target.z * spd * 100.0f); // NED north = +Z in our coords
    out_state->vy = (int16_t)(to_target.x * spd * 100.0f); // NED east = +X
    out_state->vz = (int16_t)(-to_target.y * spd * 100.0f); // NED down = -Y
    out_state->ind_airspeed = (uint16_t)(spd * 100.0f);
    out_state->true_airspeed = out_state->ind_airspeed;

    // Simple orientation: nose toward target
    out_state->quaternion[0] = 1.0f;
    out_state->quaternion[1] = 0.0f;
    out_state->quaternion[2] = 0.0f;
    out_state->quaternion[3] = 0.0f;
    // Compute yaw quaternion from to_target
    float yaw = atan2f(to_target.x, to_target.z);
    float pitch = -asinf(to_target.y);
    float cy = cosf(yaw * 0.5f), sy = sinf(yaw * 0.5f);
    float cp2 = cosf(pitch * 0.5f), sp2 = sinf(pitch * 0.5f);
    out_state->quaternion[0] = cy * cp2;
    out_state->quaternion[1] = 0;
    out_state->quaternion[2] = sy * cp2;
    out_state->quaternion[3] = cy * sp2;

    out_state->valid = true;
    out_state->time_usec = (uint64_t)(t * 1e6);
}

static void print_usage(const char *prog) {
    printf("Usage: %s [options]\n", prog);
    printf("  -udp <port>    UDP base port (default: 19410)\n");
    printf("  -n <count>     Number of vehicles (default: 1, max: %d)\n", MAX_VEHICLES);
    printf("  -mc            Multicopter model (default)\n");
    printf("  -fw            Fixed-wing model\n");
    printf("  -ts            Tailsitter model\n");
    printf("  -origin <lat> <lon> <alt>  NED origin in degrees/meters (default: PX4 SIH)\n");
    printf("  --replay <file.ulg>  Replay ULog file\n");
    printf("  -w <width>     Window width (default: 1280)\n");
    printf("  -h <height>    Window height (default: 720)\n");
}

int main(int argc, char *argv[]) {
    uint16_t base_port = 19410;
    int vehicle_count = 1;
    int model_idx = MODEL_QUADROTOR;
    int win_w = 1280;
    int win_h = 720;
    bool debug = false;
    // PX4 SIH default spawn position
    double origin_lat = 47.397742;
    double origin_lon = 8.545594;
    double origin_alt = 489.4;
    bool origin_specified = false;
    const char *replay_file = NULL;
    bool missile_mode = false;
    bool fake_mode = false;

    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "-udp") == 0 && i + 1 < argc) {
            base_port = (uint16_t)atoi(argv[++i]);
        } else if (strcmp(argv[i], "-n") == 0 && i + 1 < argc) {
            vehicle_count = atoi(argv[++i]);
            if (vehicle_count < 1) vehicle_count = 1;
            if (vehicle_count > MAX_VEHICLES) vehicle_count = MAX_VEHICLES;
        } else if (strcmp(argv[i], "-origin") == 0 && i + 3 < argc) {
            origin_lat = atof(argv[++i]);
            origin_lon = atof(argv[++i]);
            origin_alt = atof(argv[++i]);
            origin_specified = true;
        } else if (strcmp(argv[i], "-mc") == 0) {
            model_idx = MODEL_QUADROTOR;
        } else if (strcmp(argv[i], "-fw") == 0) {
            model_idx = MODEL_FIXEDWING;
        } else if (strcmp(argv[i], "-ts") == 0) {
            model_idx = MODEL_TAILSITTER;
        } else if (strcmp(argv[i], "-d") == 0) {
            debug = true;
        } else if (strcmp(argv[i], "-w") == 0 && i + 1 < argc) {
            win_w = atoi(argv[++i]);
        } else if (strcmp(argv[i], "-h") == 0 && i + 1 < argc) {
            win_h = atoi(argv[++i]);
        } else if (strcmp(argv[i], "--replay") == 0 && i + 1 < argc) {
            replay_file = argv[++i];
        } else if (strcmp(argv[i], "-missile") == 0) {
            missile_mode = true;
        } else if (strcmp(argv[i], "-fake") == 0) {
            fake_mode = true;
        } else if (strcmp(argv[i], "--help") == 0) {
            print_usage(argv[0]);
            return 0;
        }
    }

    asset_path_init();

    // Init Raylib
    SetConfigFlags(FLAG_MSAA_4X_HINT | FLAG_WINDOW_RESIZABLE);
    InitWindow(win_w, win_h, "MAVSim Viewer");
    SetTargetFPS(60);

    // Init data sources
    data_source_t sources[MAX_VEHICLES];
    memset(sources, 0, sizeof(sources));
    bool is_replay = (replay_file != NULL);

    // Missile mode overrides vehicle count
    missile_params_t missile_params[MISSILE_COUNT];
    float missile_time = 0.0f;
    if (missile_mode) {
        vehicle_count = MISSILE_COUNT + 1; // 16 missiles + 1 target
        missile_barrage_init(missile_params);
        origin_specified = true; // force shared origin
    }

    if (fake_mode) {
        vehicle_count = 1;
        origin_specified = true;
    }

    if (is_replay) {
        vehicle_count = 1;  // single vehicle replay (multi-file is future scope)
        if (data_source_ulog_create(&sources[0], replay_file) != 0) {
            fprintf(stderr, "Failed to open ULog: %s\n", replay_file);
            CloseWindow();
            return 1;
        }
    } else if (!missile_mode && !fake_mode) {
        for (int i = 0; i < vehicle_count; i++) {
            if (data_source_mavlink_create(&sources[i], base_port + i, (uint8_t)i, debug) != 0) {
                fprintf(stderr, "Failed to init MAVLink receiver on port %u\n", base_port + i);
                CloseWindow();
                return 1;
            }
        }
    }

    // Init vehicles
    // Init scene first (provides lighting shader for vehicles)
    scene_t scene;
    scene_init(&scene);

    vehicle_t vehicles[MAX_VEHICLES];
    if (missile_mode) {
        // Missiles: FPV quads, target: VTOL
        for (int i = 0; i < MISSILE_COUNT; i++) {
            vehicle_init(&vehicles[i], MODEL_FPV_QUAD, scene.lighting_shader);
            vehicles[i].color = vehicle_colors[i];
        }
        vehicle_init(&vehicles[MISSILE_COUNT], MODEL_VTOL, scene.lighting_shader);
        vehicles[MISSILE_COUNT].color = vehicle_colors[MISSILE_COUNT];
    } else {
        for (int i = 0; i < vehicle_count; i++) {
            vehicle_init(&vehicles[i], model_idx, scene.lighting_shader);
            vehicles[i].color = vehicle_colors[i];
        }
    }

    // For multi-vehicle or explicit origin: pre-set the NED origin on all vehicles
    if (vehicle_count > 1 || origin_specified) {
        double lat0_rad = origin_lat * (M_PI / 180.0);
        double lon0_rad = origin_lon * (M_PI / 180.0);
        for (int i = 0; i < vehicle_count; i++) {
            vehicles[i].lat0 = lat0_rad;
            vehicles[i].lon0 = lon0_rad;
            vehicles[i].alt0 = origin_alt;
            vehicles[i].origin_set = true;
        }
        printf("NED origin: lat=%.6f lon=%.6f alt=%.1f\n", origin_lat, origin_lon, origin_alt);
    }

    hud_t hud;
    hud_init(&hud);
    hud.is_replay = is_replay;

    debug_panel_t dbg_panel;
    debug_panel_init(&dbg_panel);

    ortho_panel_t ortho;
    ortho_panel_init(&ortho);

    int selected = 0;
    bool was_connected[MAX_VEHICLES];
    memset(was_connected, 0, sizeof(was_connected));
    Vector3 last_pos[MAX_VEHICLES];
    memset(last_pos, 0, sizeof(last_pos));
    bool show_hud = true;
    int trail_mode = 1;              // 0=off, 1=directional trail, 2=speed ribbon
    bool show_ground_track = false;  // ground projection off by default
    bool classic_colors = false;     // L key: toggle classic (red/blue) vs modern (yellow/purple)
    bool missile_paused = false;

    // Main loop
    while (!WindowShouldClose()) {
        // Missile mode controls
        if (missile_mode) {
            if (IsKeyPressed(KEY_SPACE)) missile_paused = !missile_paused;
        }

        // Missile barrage: generate synthetic flight data
        if (missile_mode) {
            float dt = missile_paused ? 0.0f : GetFrameTime();
            missile_time += dt;

            // Loop the animation
            float total_anim = 10.0f;
            if (missile_time > total_anim) {
                missile_time = 0.0f;
                for (int i = 0; i < vehicle_count; i++)
                    vehicle_reset_trail(&vehicles[i]);
            }

            Vector3 launch = { 0.0f, 15.0f, 0.0f };
            Vector3 target_center = { 40.0f, 20.0f, 30.0f };

            // Compute target VTOL's actual position (slow orbit)
            float tgt_a = missile_time * 0.3f;
            Vector3 target = {
                target_center.x + cosf(tgt_a) * 3.0f,
                target_center.y + sinf(missile_time * 0.5f) * 1.0f,
                target_center.z + sinf(tgt_a) * 3.0f
            };

            // Update missiles — they track the VTOL's live position
            for (int i = 0; i < MISSILE_COUNT; i++) {
                float t = missile_time - (float)i * 0.05f;
                if (t < 0.0f) t = 0.0f;

                hil_state_t state = {0};
                missile_barrage_state(&missile_params[i], t, launch, target, &state);
                vehicle_update(&vehicles[i], &state, NULL);
                vehicles[i].active = true;
            }

            // Target VTOL state
            {
                hil_state_t tgt_state = {0};
                float tx = target.x;
                float tz = target.z;
                float ty = target.y;

                double base_lat = 47.397742;
                double base_lon = 8.545594;
                double base_alt = 489.4;
                double meters_per_deg_lat = 111132.0;
                double meters_per_deg_lon = 111132.0 * cos(base_lat * M_PI / 180.0);

                tgt_state.lat = (int32_t)((base_lat + (double)tz / meters_per_deg_lat) * 1e7);
                tgt_state.lon = (int32_t)((base_lon + (double)tx / meters_per_deg_lon) * 1e7);
                tgt_state.alt = (int32_t)((base_alt + (double)ty) * 1000.0);
                tgt_state.vx = 0; tgt_state.vy = 0; tgt_state.vz = 0;
                tgt_state.quaternion[0] = cosf(tgt_a * 0.5f);
                tgt_state.quaternion[1] = 0;
                tgt_state.quaternion[2] = sinf(tgt_a * 0.5f);
                tgt_state.quaternion[3] = 0;
                tgt_state.valid = true;
                tgt_state.time_usec = (uint64_t)(missile_time * 1e6);

                vehicle_update(&vehicles[MISSILE_COUNT], &tgt_state, NULL);
                vehicles[MISSILE_COUNT].active = true;
            }
        }

        // Fake mode: WASDQE flight control
        if (fake_mode) {
            static float fake_x = 0.0f, fake_y = 10.0f, fake_z = 0.0f;
            static float fake_yaw = 0.0f;   // radians
            static float fake_pitch = 0.0f; // radians
            static float fake_roll = 0.0f;
            float dt = GetFrameTime();
            float move_speed = 20.0f * dt;  // m/s
            float rot_speed = 2.0f * dt;    // rad/s

            // Shift = boost
            if (IsKeyDown(KEY_LEFT_SHIFT) || IsKeyDown(KEY_RIGHT_SHIFT))
                move_speed *= 3.0f;

            // Rotation: arrow keys
            if (IsKeyDown(KEY_LEFT))  fake_yaw -= rot_speed;
            if (IsKeyDown(KEY_RIGHT)) fake_yaw += rot_speed;

            // Movement relative to yaw
            float fwd_x = sinf(fake_yaw);
            float fwd_z = cosf(fake_yaw);
            float right_x = cosf(fake_yaw);
            float right_z = -sinf(fake_yaw);

            // Tilt for visual feedback
            float target_pitch = 0.0f, target_roll = 0.0f;

            if (IsKeyDown(KEY_W)) { fake_x += fwd_x * move_speed; fake_z += fwd_z * move_speed; target_pitch = -0.3f; }
            if (IsKeyDown(KEY_S)) { fake_x -= fwd_x * move_speed; fake_z -= fwd_z * move_speed; target_pitch = 0.3f; }
            if (IsKeyDown(KEY_D)) { fake_x += right_x * move_speed; fake_z += right_z * move_speed; target_roll = 0.4f; }
            if (IsKeyDown(KEY_A)) { fake_x -= right_x * move_speed; fake_z -= right_z * move_speed; target_roll = -0.4f; }
            if (IsKeyDown(KEY_E)) fake_y += move_speed;
            if (IsKeyDown(KEY_Q)) fake_y -= move_speed;

            // Smooth tilt
            fake_pitch += (target_pitch - fake_pitch) * 5.0f * dt;
            fake_roll += (target_roll - fake_roll) * 5.0f * dt;

            // Build hil_state
            double meters_per_deg_lat = 111132.0;
            double meters_per_deg_lon = 111132.0 * cos(origin_lat * M_PI / 180.0);

            hil_state_t state = {0};
            state.lat = (int32_t)((origin_lat + (double)fake_z / meters_per_deg_lat) * 1e7);
            state.lon = (int32_t)((origin_lon + (double)fake_x / meters_per_deg_lon) * 1e7);
            state.alt = (int32_t)((origin_alt + (double)fake_y) * 1000.0);

            // Velocity for HUD readout
            float vx = 0, vy = 0, vz = 0;
            if (IsKeyDown(KEY_W)) { vx += fwd_z * 20.0f; vy += fwd_x * 20.0f; }
            if (IsKeyDown(KEY_S)) { vx -= fwd_z * 20.0f; vy -= fwd_x * 20.0f; }
            if (IsKeyDown(KEY_D)) { vy += right_x * 20.0f; vx += right_z * 20.0f; }  // NED east
            if (IsKeyDown(KEY_A)) { vy -= right_x * 20.0f; vx -= right_z * 20.0f; }
            float vz_ned = 0;
            if (IsKeyDown(KEY_E)) vz_ned = -20.0f; // NED down = negative for climb
            if (IsKeyDown(KEY_Q)) vz_ned = 20.0f;
            state.vx = (int16_t)(vx * 100.0f);
            state.vy = (int16_t)(vy * 100.0f);
            state.vz = (int16_t)(vz_ned * 100.0f);
            float spd = sqrtf(vx*vx + vy*vy + vz_ned*vz_ned);
            state.ind_airspeed = (uint16_t)(spd * 100.0f);
            state.true_airspeed = state.ind_airspeed;

            // Quaternion from yaw/pitch/roll (ZYX convention)
            float cy = cosf(fake_yaw * 0.5f), sy = sinf(fake_yaw * 0.5f);
            float cp = cosf(fake_pitch * 0.5f), sp = sinf(fake_pitch * 0.5f);
            float cr = cosf(fake_roll * 0.5f), sr = sinf(fake_roll * 0.5f);
            state.quaternion[0] = cr*cp*cy + sr*sp*sy;
            state.quaternion[1] = sr*cp*cy - cr*sp*sy;
            state.quaternion[2] = cr*sp*cy + sr*cp*sy;
            state.quaternion[3] = cr*cp*sy - sr*sp*cy;

            state.valid = true;
            state.time_usec = (uint64_t)(GetTime() * 1e6);

            vehicle_update(&vehicles[0], &state, NULL);
            vehicles[0].active = true;
        }

        // Poll all data sources and update vehicles
        for (int i = 0; i < vehicle_count && !missile_mode && !fake_mode; i++) {
            data_source_poll(&sources[i], GetFrameTime());

            // Reset trail and origin on reconnect
            if (sources[i].connected && !was_connected[i]) {
                vehicle_reset_trail(&vehicles[i]);
                if (sources[i].mav_type != 0)
                    vehicle_set_type(&vehicles[i], sources[i].mav_type);
                if (!origin_specified && vehicle_count == 1) {
                    vehicles[i].origin_set = false;
                    vehicles[i].origin_wait_count = 0;
                }
            }
            was_connected[i] = sources[i].connected;

            vehicle_update(&vehicles[i], &sources[i].state, &sources[i].home);
            vehicles[i].sysid = sources[i].sysid;

            // Detect position jump (new SITL connecting before disconnect timeout)
            if (vehicles[i].active && vehicles[i].trail_count > 0) {
                Vector3 delta = Vector3Subtract(vehicles[i].position, last_pos[i]);
                if (Vector3Length(delta) > 50.0f) {
                    vehicle_reset_trail(&vehicles[i]);
                }
            }
            last_pos[i] = vehicles[i].position;
        }

        // Check if any source is connected (for HUD)
        bool any_connected = fake_mode || missile_mode;
        for (int i = 0; i < vehicle_count && !any_connected; i++) {
            if (sources[i].connected) { any_connected = true; break; }
        }

        // Update HUD sim time from selected vehicle
        hud_update(&hud, sources[selected].state.time_usec,
                   sources[selected].connected, GetFrameTime());

        // Handle input
        scene_handle_input(&scene);

        // Help overlay toggle (? key = Shift+/)
        if (IsKeyPressed(KEY_SLASH) && (IsKeyDown(KEY_LEFT_SHIFT) || IsKeyDown(KEY_RIGHT_SHIFT))) {
            hud.show_help = !hud.show_help;
        }

        // Toggle HUD visibility
        if (IsKeyPressed(KEY_H)) {
            show_hud = !show_hud;
        }

        // Cycle trail mode: off → trail → speed ribbon
        if (IsKeyPressed(KEY_T)) {
            trail_mode = (trail_mode + 1) % 3;
        }

        // Toggle classic/modern arm colors
        if (IsKeyPressed(KEY_K)) {
            classic_colors = !classic_colors;
        }

        // Toggle ground track projection
        if (IsKeyPressed(KEY_G)) {
            show_ground_track = !show_ground_track;
        }

        // Toggle debug panel (Ctrl+D)
        if (IsKeyDown(KEY_LEFT_CONTROL) && IsKeyPressed(KEY_D)) {
            dbg_panel.visible = !dbg_panel.visible;
        }

        // Toggle ortho panel
        if (IsKeyPressed(KEY_O)) {
            ortho.visible = !ortho.visible;
        }

        // Cycle model for selected vehicle
        // Cycle model: M = within group, Shift+M = all models
        if (IsKeyPressed(KEY_M)) {
            if (IsKeyDown(KEY_LEFT_SHIFT) || IsKeyDown(KEY_RIGHT_SHIFT)) {
                int next = (vehicles[selected].model_idx + 1) % vehicle_model_count;
                vehicle_load_model(&vehicles[selected], next);
            } else {
                vehicle_cycle_model(&vehicles[selected]);
            }
        }

        // Vehicle selection input
        if (vehicle_count > 1) {
            if (IsKeyPressed(KEY_TAB)) {
                // Cycle to next connected vehicle, clear pins
                for (int j = 1; j <= vehicle_count; j++) {
                    int next = (selected + j) % vehicle_count;
                    if (sources[next].connected) { selected = next; break; }
                }
                hud.pinned_count = 0;
                memset(hud.pinned, -1, sizeof(hud.pinned));
            }
            if (IsKeyPressed(KEY_LEFT_BRACKET)) {
                for (int j = 1; j <= vehicle_count; j++) {
                    int prev = (selected - j + vehicle_count) % vehicle_count;
                    if (sources[prev].connected) { selected = prev; break; }
                }
                hud.pinned_count = 0;
                memset(hud.pinned, -1, sizeof(hud.pinned));
            }
            if (IsKeyPressed(KEY_RIGHT_BRACKET)) {
                for (int j = 1; j <= vehicle_count; j++) {
                    int next = (selected + j) % vehicle_count;
                    if (sources[next].connected) { selected = next; break; }
                }
                hud.pinned_count = 0;
                memset(hud.pinned, -1, sizeof(hud.pinned));
            }
            // Number keys 1-9: plain = select + clear pins, SHIFT = toggle pin
            for (int k = KEY_ONE; k <= KEY_NINE; k++) {
                if (IsKeyPressed(k)) {
                    int idx = k - KEY_ONE;
                    if (idx >= vehicle_count) continue;

                    if (IsKeyDown(KEY_LEFT_SHIFT) || IsKeyDown(KEY_RIGHT_SHIFT)) {
                        // SHIFT+number: toggle pin
                        if (idx == selected) continue;  // can't pin the primary

                        // Check if already pinned
                        int found = -1;
                        for (int p = 0; p < hud.pinned_count; p++) {
                            if (hud.pinned[p] == idx) { found = p; break; }
                        }

                        if (found >= 0) {
                            // Unpin: shift remaining
                            for (int p = found; p < hud.pinned_count - 1; p++)
                                hud.pinned[p] = hud.pinned[p + 1];
                            hud.pinned_count--;
                            hud.pinned[hud.pinned_count] = -1;
                        } else if (hud.pinned_count < HUD_MAX_PINNED && hud.pinned_count < vehicle_count - 1) {
                            hud.pinned[hud.pinned_count++] = idx;
                        }
                    } else if (!IsKeyDown(KEY_LEFT_CONTROL) && !IsKeyDown(KEY_RIGHT_CONTROL)) {
                        // Plain number: switch primary, clear pins
                        selected = idx;
                        hud.pinned_count = 0;
                        memset(hud.pinned, -1, sizeof(hud.pinned));
                    }
                    // Note: Ctrl+number is reserved for 1988 easter egg
                }
            }
        }

        // Replay playback controls
        if (is_replay) {
            if (IsKeyPressed(KEY_SPACE)) {
                if (!sources[0].connected) {
                    // Replay ended — restart from beginning
                    ulog_replay_ctx_t *rctx = (ulog_replay_ctx_t *)sources[0].impl;
                    ulog_replay_seek(rctx, 0.0f);
                    sources[0].connected = true;
                    sources[0].playback.paused = false;
                    vehicle_reset_trail(&vehicles[0]);
                    vehicles[0].origin_set = false;
                    vehicles[0].origin_wait_count = 0;
                } else {
                    sources[0].playback.paused = !sources[0].playback.paused;
                }
            }
            if (IsKeyPressed(KEY_L)) {
                sources[0].playback.looping = !sources[0].playback.looping;
            }
            if (IsKeyPressed(KEY_I)) {
                sources[0].playback.interpolation = !sources[0].playback.interpolation;
                printf("Interpolation: %s\n", sources[0].playback.interpolation ? "ON" : "OFF");
            }
            if (IsKeyPressed(KEY_EQUAL)) {
                float *spd = &sources[0].playback.speed;
                if (*spd < 0.5f) *spd = 0.5f;
                else if (*spd < 1.0f) *spd = 1.0f;
                else if (*spd < 2.0f) *spd = 2.0f;
                else if (*spd < 4.0f) *spd = 4.0f;
                else if (*spd < 8.0f) *spd = 8.0f;
                else *spd = 16.0f;
            }
            if (IsKeyPressed(KEY_MINUS)) {
                float *spd = &sources[0].playback.speed;
                if (*spd > 8.0f) *spd = 8.0f;
                else if (*spd > 4.0f) *spd = 4.0f;
                else if (*spd > 2.0f) *spd = 2.0f;
                else if (*spd > 1.0f) *spd = 1.0f;
                else if (*spd > 0.5f) *spd = 0.5f;
                else *spd = 0.25f;
            }
            if (IsKeyPressed(KEY_R)) {
                ulog_replay_ctx_t *ctx = (ulog_replay_ctx_t *)sources[0].impl;
                ulog_replay_seek(ctx, 0.0f);
                sources[0].connected = true;
                sources[0].playback.paused = false;
                vehicle_reset_trail(&vehicles[0]);
                vehicles[0].origin_set = false;
                vehicles[0].origin_wait_count = 0;
            }
            if (IsKeyPressed(KEY_RIGHT)) {
                ulog_replay_ctx_t *ctx = (ulog_replay_ctx_t *)sources[0].impl;
                float step = (IsKeyDown(KEY_LEFT_SHIFT) || IsKeyDown(KEY_RIGHT_SHIFT)) ? 30.0f : 5.0f;
                ulog_replay_seek(ctx, sources[0].playback.position_s + step);
                sources[0].playback.position_s = (float)ctx->wall_accum;
                vehicle_reset_trail(&vehicles[0]);
            }
            if (IsKeyPressed(KEY_LEFT)) {
                ulog_replay_ctx_t *ctx = (ulog_replay_ctx_t *)sources[0].impl;
                float step = (IsKeyDown(KEY_LEFT_SHIFT) || IsKeyDown(KEY_RIGHT_SHIFT)) ? 30.0f : 5.0f;
                float target = sources[0].playback.position_s - step;
                if (target < 0.0f) target = 0.0f;
                ulog_replay_seek(ctx, target);
                sources[0].playback.position_s = (float)ctx->wall_accum;
                vehicle_reset_trail(&vehicles[0]);
                vehicles[0].origin_set = false;
                vehicles[0].origin_wait_count = 0;
            }
        }

        // Update debug panel
        debug_panel_update(&dbg_panel, GetFrameTime());

        // Update camera to follow selected vehicle
        scene_update_camera(&scene, vehicles[selected].position, vehicles[selected].rotation);

        // Render ortho views to textures (before main BeginDrawing)
        if (ortho.visible) {
            ortho_panel_update(&ortho, vehicles[selected].position);
            ortho_panel_render(&ortho, &scene, vehicles, vehicle_count,
                               selected, scene.view_mode, trail_mode);
        }

        // Render
        BeginDrawing();

            // Sky background
            scene_draw_sky(&scene);

            BeginMode3D(scene.camera);
                scene_draw(&scene);
                for (int i = 0; i < vehicle_count; i++) {
                    if (vehicles[i].active || vehicle_count == 1) {
                        vehicle_draw(&vehicles[i], scene.view_mode, i == selected,
                                     trail_mode, show_ground_track, scene.camera.position,
                                     classic_colors, scene.is_underwater);
                    }
                }
            EndMode3D();

            // Ortho ground fill (2D overlay)
            scene_draw_ortho_ground(&scene, GetScreenWidth(), GetScreenHeight());

            // Underwater edge rings overlay
            scene_draw_underwater_overlay(&scene, GetScreenWidth(), GetScreenHeight());

            // HUD
            if (show_hud) {
                hud_draw(&hud, vehicles, sources, vehicle_count,
                         selected, GetScreenWidth(), GetScreenHeight(),
                         scene.view_mode, scene.is_underwater);
            }

            // Debug panel
            {
                int active_count = 0;
                int total_trail = 0;
                for (int i = 0; i < vehicle_count; i++) {
                    if (vehicles[i].active) active_count++;
                    total_trail += vehicles[i].trail_count;
                }
                debug_panel_draw(&dbg_panel, GetScreenWidth(), GetScreenHeight(),
                                 scene.view_mode, hud.font_label,
                                 vehicle_count, active_count, total_trail);
            }

            // Ortho panel overlay
            int bar_h = show_hud ? hud_bar_height(&hud, GetScreenHeight()) : 0;
            ortho_panel_draw(&ortho, GetScreenHeight(), bar_h, scene.view_mode, hud.font_label);

            // Fullscreen ortho view label
            ortho_panel_draw_fullscreen_label(GetScreenWidth(), GetScreenHeight(),
                scene.ortho_mode, scene.ortho_span, scene.view_mode, hud.font_label);

        EndDrawing();
    }

    // Cleanup
    ortho_panel_cleanup(&ortho);
    hud_cleanup(&hud);
    for (int i = 0; i < vehicle_count; i++) {
        vehicle_cleanup(&vehicles[i]);
        data_source_close(&sources[i]);
    }
    scene_cleanup(&scene);
    CloseWindow();

    return 0;
}
