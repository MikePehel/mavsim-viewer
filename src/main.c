#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#ifdef _MSC_VER
#define _USE_MATH_DEFINES
#endif
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

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

#define MAX_VEHICLES 16

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
};

// Position resolution tier for spatial conflict detection
typedef struct {
    int   tier;        // 1=home_position topic, 2=GPOS derived, 3=grid
    float confidence;  // 1.0, 0.85, or 0.5
    bool  estimated;   // true if tier 3
} position_resolution_t;

static void print_usage(const char *prog) {
    printf("Usage: %s [options]\n", prog);
    printf("  -udp <port>    UDP base port (default: 19410)\n");
    printf("  -n <count>     Number of vehicles (default: 1, max: %d)\n", MAX_VEHICLES);
    printf("  -mc            Multicopter model (default)\n");
    printf("  -fw            Fixed-wing model\n");
    printf("  -ts            Tailsitter model\n");
    printf("  -origin <lat> <lon> <alt>  NED origin in degrees/meters (default: PX4 SIH)\n");
    printf("  --replay <file1.ulg> [file2.ulg ...]  Replay ULog file(s)\n");
    printf("  --ghost        Accept position overlaps without prompting\n");
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

    // TASK 1: Multi-file replay + ghost flag
    char *replay_paths[MAX_VEHICLES] = {0};
    int num_replay_files = 0;
    bool ghost_mode = false;

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
        } else if (strcmp(argv[i], "--replay") == 0) {
            // Collect all subsequent args that look like file paths (not flags)
            while (i + 1 < argc && argv[i + 1][0] != '-') {
                if (num_replay_files >= MAX_VEHICLES) {
                    fprintf(stderr, "Too many replay files (max %d)\n", MAX_VEHICLES);
                    return 1;
                }
                replay_paths[num_replay_files++] = argv[++i];
            }
        } else if (strcmp(argv[i], "--ghost") == 0) {
            ghost_mode = true;
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
    bool is_replay = (num_replay_files > 0);

    // TASK 2: Initialize multiple replay sources
    if (is_replay) {
        for (int i = 0; i < num_replay_files; i++) {
            if (data_source_ulog_create(&sources[i], replay_paths[i]) != 0) {
                fprintf(stderr, "Failed to open ULog: %s\n", replay_paths[i]);
                CloseWindow();
                return 1;
            }
        }
        vehicle_count = num_replay_files;
    } else {
        for (int i = 0; i < vehicle_count; i++) {
            if (data_source_mavlink_create(&sources[i], base_port + i, (uint8_t)i, debug) != 0) {
                fprintf(stderr, "Failed to init MAVLink receiver on port %u\n", base_port + i);
                CloseWindow();
                return 1;
            }
        }
    }

    // Init scene first (provides lighting shader for vehicles)
    scene_t scene;
    scene_init(&scene);

    vehicle_t vehicles[MAX_VEHICLES];
    for (int i = 0; i < vehicle_count; i++) {
        vehicle_init(&vehicles[i], model_idx, scene.lighting_shader);
        vehicles[i].color = vehicle_colors[i];
    }

    // Position resolution for all vehicles (hoisted for use by HUD state population)
    position_resolution_t resolve[MAX_VEHICLES];
    for (int i = 0; i < MAX_VEHICLES; i++) {
        resolve[i] = (position_resolution_t){ .tier = 1, .confidence = 1.0f, .estimated = false };
    }

    // Spatial conflict detection + resolution (multi-file replay only)
    if (is_replay && num_replay_files > 1) {
        // Determine position resolution tier for each drone
        for (int i = 0; i < num_replay_files; i++) {
            ulog_replay_ctx_t *r = (ulog_replay_ctx_t *)sources[i].impl;
            if (r->home_from_topic) {
                resolve[i].tier = 1;
                resolve[i].confidence = 1.0f;
                resolve[i].estimated = false;
            } else if (r->home.valid) {
                resolve[i].tier = 2;
                resolve[i].confidence = 0.85f;
                resolve[i].estimated = false;
            } else {
                resolve[i].tier = 3;
                resolve[i].confidence = 0.5f;
                resolve[i].estimated = true;
            }
        }

        // Detect conflicts
        bool conflict = false;

        // Count drones with tier 3 (no position)
        int tier3_count = 0;
        for (int i = 0; i < num_replay_files; i++) {
            if (resolve[i].tier == 3) tier3_count++;
        }
        if (tier3_count > 1) conflict = true;

        // Pairwise proximity check (within 3.0m)
        if (!conflict) {
            for (int i = 0; i < num_replay_files && !conflict; i++) {
                if (!sources[i].home.valid) continue;
                for (int j = i + 1; j < num_replay_files && !conflict; j++) {
                    if (!sources[j].home.valid) continue;
                    double dlat_degE7 = (double)(sources[i].home.lat - sources[j].home.lat);
                    double dlon_degE7 = (double)(sources[i].home.lon - sources[j].home.lon);
                    double dalt_mm = (double)(sources[i].home.alt - sources[j].home.alt);
                    double dlat_m = dlat_degE7 / 1e7 * 111319.5;
                    double lat_rad = (sources[i].home.lat / 1e7) * (M_PI / 180.0);
                    double dlon_m = dlon_degE7 / 1e7 * 111319.5 * cos(lat_rad);
                    double dalt_m = dalt_mm / 1000.0;
                    double dist = sqrt(dlat_m * dlat_m + dlon_m * dlon_m + dalt_m * dalt_m);
                    if (dist < 3.0) conflict = true;
                }
            }
        }

        // Show blocking modal if conflict detected and --ghost not set
        if (conflict && !ghost_mode) {
            int choice = 0; // 0 = waiting, 1 = cancel, 2 = ghost, 3 = grid
            while (choice == 0 && !WindowShouldClose()) {
                int sw = GetScreenWidth();
                int sh = GetScreenHeight();

                int key = GetKeyPressed();
                if (key == KEY_ONE) choice = 1;
                else if (key == KEY_TWO) choice = 2;
                else if (key == KEY_THREE) choice = 3;

                BeginDrawing();
                    ClearBackground(DARKGRAY);

                    // Semi-transparent overlay
                    DrawRectangle(0, 0, sw, sh, (Color){0, 0, 0, 160});

                    // Centered dialog box
                    int box_w = 520;
                    int box_h = 260;
                    int bx = (sw - box_w) / 2;
                    int by = (sh - box_h) / 2;
                    DrawRectangle(bx, by, box_w, box_h, (Color){40, 40, 40, 240});
                    DrawRectangleLines(bx, by, box_w, box_h, RAYWHITE);

                    // Title
                    const char *title = "Position Conflict Detected";
                    int tw = MeasureText(title, 24);
                    DrawText(title, bx + (box_w - tw) / 2, by + 20, 24, YELLOW);

                    // Subtitle
                    char subtitle[128];
                    snprintf(subtitle, sizeof(subtitle),
                             "%d drones resolved to the same position.", num_replay_files);
                    int stw = MeasureText(subtitle, 18);
                    DrawText(subtitle, bx + (box_w - stw) / 2, by + 55, 18, LIGHTGRAY);

                    // Options
                    int opt_x = bx + 40;
                    int opt_y = by + 100;
                    DrawText("[1] Cancel & Reupload", opt_x, opt_y, 20, WHITE);
                    DrawText("[2] Ghost Mode - overlap accepted", opt_x, opt_y + 35, 20, WHITE);
                    DrawText("[3] Grid Offset - auto-space drones", opt_x, opt_y + 70, 20, WHITE);

                    // Footer
                    const char *footer = "Press 1, 2, or 3";
                    int fw_text = MeasureText(footer, 16);
                    DrawText(footer, bx + (box_w - fw_text) / 2, by + box_h - 30, 16, GRAY);

                EndDrawing();
            }

            if (choice == 1 || WindowShouldClose()) {
                // Cancel: close all sources, fall back to zero vehicles
                for (int i = 0; i < num_replay_files; i++) {
                    data_source_close(&sources[i]);
                }
                memset(sources, 0, sizeof(sources));
                num_replay_files = 0;
                vehicle_count = 0;
                is_replay = false;
                if (WindowShouldClose()) {
                    // Clean up and exit
                    for (int i = 0; i < vehicle_count; i++)
                        vehicle_cleanup(&vehicles[i]);
                    scene_cleanup(&scene);
                    CloseWindow();
                    return 0;
                }
            } else if (choice == 2) {
                ghost_mode = true;
            } else if (choice == 3) {
                // Grid offset: space estimated drones on a grid
                float drone_w = 3.0f;
                float gap = drone_w * 1.5f;
                int est_count = 0;
                for (int i = 0; i < num_replay_files; i++) {
                    if (resolve[i].estimated) est_count++;
                }
                int cols = (int)ceil(sqrt((double)est_count));
                int est_idx = 0;
                for (int i = 0; i < num_replay_files; i++) {
                    if (!resolve[i].estimated) continue;
                    int row = est_idx / cols;
                    int col = est_idx % cols;
                    // Offset home position in meters, converted to degE7
                    // 1 meter ≈ 1/111319.5 degrees latitude
                    double offset_n = row * gap;
                    double offset_e = col * gap;
                    double lat_rad = 0.0;
                    if (sources[0].home.valid)
                        lat_rad = (sources[0].home.lat / 1e7) * (M_PI / 180.0);
                    int32_t dlat = (int32_t)(offset_n / 111319.5 * 1e7);
                    int32_t dlon = (int32_t)(offset_e / (111319.5 * cos(lat_rad)) * 1e7);
                    sources[i].home.lat += dlat;
                    sources[i].home.lon += dlon;
                    if (!sources[i].home.valid) {
                        // Give it a position based on the first valid source
                        if (sources[0].home.valid) {
                            sources[i].home.lat = sources[0].home.lat + dlat;
                            sources[i].home.lon = sources[0].home.lon + dlon;
                            sources[i].home.alt = sources[0].home.alt;
                            sources[i].home.valid = true;
                        }
                    }
                    est_idx++;
                }
            }
        } else if (conflict && ghost_mode) {
            // --ghost was set via CLI: auto-accept overlap
            printf("Ghost mode: position overlaps accepted\n");
        }
    }

    // TASK 4: Shared origin + ground datum (multi-file replay)
    if (is_replay && num_replay_files > 1) {
        // Shared lat/lon origin = first log's home
        ulog_replay_ctx_t *ref = (ulog_replay_ctx_t *)sources[0].impl;
        if (ref->home.valid) {
            origin_lat = ref->home.lat / 1e7;
            origin_lon = ref->home.lon / 1e7;
        }

        // Ground datum = lowest altitude across all logs
        int32_t min_alt = ref->home.alt;
        bool found_valid = ref->home.valid;
        for (int i = 1; i < num_replay_files; i++) {
            ulog_replay_ctx_t *r = (ulog_replay_ctx_t *)sources[i].impl;
            if (r->home.valid) {
                if (!found_valid || r->home.alt < min_alt) {
                    min_alt = r->home.alt;
                    found_valid = true;
                }
            }
        }

        if (found_valid) {
            double lat0_rad = origin_lat * (M_PI / 180.0);
            double lon0_rad = origin_lon * (M_PI / 180.0);
            for (int i = 0; i < num_replay_files; i++) {
                vehicles[i].lat0 = lat0_rad;
                vehicles[i].lon0 = lon0_rad;
                vehicles[i].alt0 = min_alt / 1000.0;
                vehicles[i].origin_set = true;
            }
            printf("Multi-replay NED origin: lat=%.6f lon=%.6f alt=%.1f\n",
                   origin_lat, origin_lon, min_alt / 1000.0);
        }
    } else if (vehicle_count > 1 || origin_specified) {
        // Original behavior for MAVLink multi-vehicle or explicit origin
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

    // ── Temporal alignment (Task 2) ──
    if (num_replay_files > 1) {
        ulog_replay_ctx_t *ref = (ulog_replay_ctx_t *)sources[0].impl;
        uint64_t ref_anchor = ref->takeoff_anchor.anchor_usec;

        for (int i = 0; i < num_replay_files; i++) {
            ulog_replay_ctx_t *r = (ulog_replay_ctx_t *)sources[i].impl;
            if (r->takeoff_anchor.method > 0 && ref_anchor > 0) {
                r->time_offset = (int64_t)ref_anchor
                               - (int64_t)r->takeoff_anchor.anchor_usec;
            } else {
                r->time_offset = 0;
            }
        }

        printf("Temporal alignment:\n");
        for (int i = 0; i < num_replay_files; i++) {
            ulog_replay_ctx_t *r = (ulog_replay_ctx_t *)sources[i].impl;
            printf("  Replay %d: takeoff=%.3fs offset=%+.3fs conf=%.2f\n",
                   i, r->takeoff_anchor.anchor_usec / 1e6,
                   r->time_offset / 1e6, r->takeoff_anchor.confidence);
        }
    }

    // ── Populate per-vehicle HUD state (Task 3) ──
    if (is_replay) {
        for (int i = 0; i < num_replay_files; i++) {
            ulog_replay_ctx_t *r = (ulog_replay_ctx_t *)sources[i].impl;
            vehicles[i].pos_tier = resolve[i].tier;
            vehicles[i].pos_confidence = resolve[i].confidence;
            vehicles[i].pos_estimated = resolve[i].estimated;
            vehicles[i].temporal_confidence = r->takeoff_anchor.confidence;
        }
    }

    // ── Ghost alpha (Task 4) ──
    if (ghost_mode && num_replay_files > 1) {
        vehicle_set_ghost_alpha(&vehicles[0], 1.0f);
        for (int i = 1; i < num_replay_files; i++)
            vehicle_set_ghost_alpha(&vehicles[i], 0.35f);
    }

    // ── Unified timeline duration (Task 5) ──
    float unified_duration = 0;
    if (is_replay) {
        for (int i = 0; i < num_replay_files; i++) {
            ulog_replay_ctx_t *r = (ulog_replay_ctx_t *)sources[i].impl;
            float dur = ulog_replay_aligned_duration(r);
            if (dur > unified_duration) unified_duration = dur;
        }
        if (num_replay_files > 1 && unified_duration > 0) {
            for (int i = 0; i < num_replay_files; i++) {
                sources[i].playback.duration_s = unified_duration;
            }
        }
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

    // Main loop
    while (!WindowShouldClose()) {
        // Poll all data sources and update vehicles
        for (int i = 0; i < vehicle_count; i++) {
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
        bool any_connected = false;
        for (int i = 0; i < vehicle_count; i++) {
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

        // TASK 5: Replay playback controls — apply to all sources
        if (is_replay) {
            if (IsKeyPressed(KEY_SPACE)) {
                // Check if any source has ended
                bool any_ended = false;
                for (int i = 0; i < num_replay_files; i++) {
                    if (!sources[i].connected) { any_ended = true; break; }
                }
                if (any_ended) {
                    // Replay ended — restart all from beginning
                    for (int i = 0; i < num_replay_files; i++) {
                        ulog_replay_ctx_t *rctx = (ulog_replay_ctx_t *)sources[i].impl;
                        ulog_replay_seek(rctx, 0.0f);
                        sources[i].connected = true;
                        sources[i].playback.paused = false;
                        vehicle_reset_trail(&vehicles[i]);
                        vehicles[i].origin_set = false;
                        vehicles[i].origin_wait_count = 0;
                    }
                } else {
                    for (int i = 0; i < num_replay_files; i++) {
                        sources[i].playback.paused = !sources[i].playback.paused;
                    }
                }
            }
            if (IsKeyPressed(KEY_L)) {
                for (int i = 0; i < num_replay_files; i++) {
                    sources[i].playback.looping = !sources[i].playback.looping;
                }
            }
            if (IsKeyPressed(KEY_I)) {
                bool new_val = !sources[0].playback.interpolation;
                for (int i = 0; i < num_replay_files; i++) {
                    sources[i].playback.interpolation = new_val;
                }
                printf("Interpolation: %s\n", new_val ? "ON" : "OFF");
            }
            if (IsKeyPressed(KEY_EQUAL)) {
                // Speed up — use sources[0] as reference, apply to all
                float spd = sources[0].playback.speed;
                if (spd < 0.5f) spd = 0.5f;
                else if (spd < 1.0f) spd = 1.0f;
                else if (spd < 2.0f) spd = 2.0f;
                else if (spd < 4.0f) spd = 4.0f;
                else if (spd < 8.0f) spd = 8.0f;
                else spd = 16.0f;
                for (int i = 0; i < num_replay_files; i++) {
                    sources[i].playback.speed = spd;
                }
            }
            if (IsKeyPressed(KEY_MINUS)) {
                float spd = sources[0].playback.speed;
                if (spd > 8.0f) spd = 8.0f;
                else if (spd > 4.0f) spd = 4.0f;
                else if (spd > 2.0f) spd = 2.0f;
                else if (spd > 1.0f) spd = 1.0f;
                else if (spd > 0.5f) spd = 0.5f;
                else spd = 0.25f;
                for (int i = 0; i < num_replay_files; i++) {
                    sources[i].playback.speed = spd;
                }
            }
            if (IsKeyPressed(KEY_R)) {
                for (int i = 0; i < num_replay_files; i++) {
                    ulog_replay_ctx_t *ctx = (ulog_replay_ctx_t *)sources[i].impl;
                    float offset_s = (float)(ctx->time_offset / 1e6);
                    float adjusted = offset_s;  // unified time 0 + per-source offset
                    if (adjusted < 0.0f) adjusted = 0.0f;
                    ulog_replay_seek(ctx, adjusted);
                    sources[i].connected = true;
                    sources[i].playback.paused = false;
                    vehicle_reset_trail(&vehicles[i]);
                    vehicles[i].origin_set = false;
                    vehicles[i].origin_wait_count = 0;
                }
            }
            if (IsKeyPressed(KEY_RIGHT)) {
                float step = (IsKeyDown(KEY_LEFT_SHIFT) || IsKeyDown(KEY_RIGHT_SHIFT)) ? 30.0f : 5.0f;
                float seek_target = sources[selected].playback.position_s + step;
                for (int i = 0; i < num_replay_files; i++) {
                    ulog_replay_ctx_t *ctx = (ulog_replay_ctx_t *)sources[i].impl;
                    float offset_s = (float)(ctx->time_offset / 1e6);
                    float adjusted = seek_target + offset_s;
                    if (adjusted < 0.0f) adjusted = 0.0f;
                    ulog_replay_seek(ctx, adjusted);
                    sources[i].playback.position_s = (float)ctx->wall_accum;
                    vehicle_reset_trail(&vehicles[i]);
                }
            }
            if (IsKeyPressed(KEY_LEFT)) {
                float step = (IsKeyDown(KEY_LEFT_SHIFT) || IsKeyDown(KEY_RIGHT_SHIFT)) ? 30.0f : 5.0f;
                float seek_target = sources[selected].playback.position_s - step;
                if (seek_target < 0.0f) seek_target = 0.0f;
                for (int i = 0; i < num_replay_files; i++) {
                    ulog_replay_ctx_t *ctx = (ulog_replay_ctx_t *)sources[i].impl;
                    float offset_s = (float)(ctx->time_offset / 1e6);
                    float adjusted = seek_target + offset_s;
                    if (adjusted < 0.0f) adjusted = 0.0f;
                    ulog_replay_seek(ctx, adjusted);
                    sources[i].playback.position_s = (float)ctx->wall_accum;
                    vehicle_reset_trail(&vehicles[i]);
                    vehicles[i].origin_set = false;
                    vehicles[i].origin_wait_count = 0;
                }
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
                                     classic_colors);
                    }
                }
            EndMode3D();

            // Ortho ground fill (2D overlay)
            scene_draw_ortho_ground(&scene, GetScreenWidth(), GetScreenHeight());

            // HUD
            if (show_hud) {
                hud_draw(&hud, vehicles, sources, vehicle_count,
                         selected, GetScreenWidth(), GetScreenHeight(),
                         scene.view_mode);
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
