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

// Per-view-mode vehicle palettes. Slots 1-6 match trail directional colors.
// Alternating warm/cool so adjacent drones are visually distinct.
static const Color vehicle_palette_grid[MAX_VEHICLES] = {
    {230, 230, 230, 255}, // 0: white (primary)
    { 40, 120, 255, 255}, // 1: blue (cool)
    {255,  40,  80, 255}, // 2: red (warm)
    {255, 200,  50, 255}, // 3: yellow (warm)
    { 40, 255,  80, 255}, // 4: green (cool)
    {255, 140,   0, 255}, // 5: orange (warm)
    {160,  60, 255, 255}, // 6: purple (cool)
    {255, 100, 180, 255}, // 7: pink (warm)
    {  0, 180, 140, 255}, // 8: teal (cool)
    {200, 180,  80, 255}, // 9: gold (warm)
    {100, 100, 255, 255}, // 10: indigo (cool)
    {255, 180, 100, 255}, // 11: peach (warm)
    {100, 220, 200, 255}, // 12: mint (cool)
    {220,  80, 180, 255}, // 13: magenta (warm)
    {120, 200, 255, 255}, // 14: sky blue (cool)
    {180, 255,  60, 255}, // 15: lime (cool)
};
static const Color vehicle_palette_rez[MAX_VEHICLES] = {
    {200, 208, 218, 255}, { 30, 100, 240, 255}, {255,  40,  80, 255},
    {220, 180,  30, 255}, { 40, 255, 100, 255}, {255, 160,   0, 255},
    {160,  40, 240, 255}, {255,  80, 160, 255}, {  0, 160, 130, 255},
    {180, 160,  60, 255}, { 80,  90, 240, 255}, {240, 170,  90, 255},
    { 80, 200, 180, 255}, {200,  60, 160, 255}, {100, 180, 240, 255},
    {160, 240,  50, 255},
};
static const Color vehicle_palette_snow[MAX_VEHICLES] = {
    { 40,  40,  50, 255}, { 20,  80, 160, 255}, {200,  20,  60, 255},
    {200, 140,  20, 255}, { 20, 160,  40, 255}, {160,  20,  80, 255},
    {140,  20, 200, 255}, {180,  40, 100, 255}, {  0, 120, 100, 255},
    {180, 120,  40, 255}, { 60,  60, 180, 255}, {140, 120,  40, 255},
    { 40, 140, 120, 255}, {160,  40, 120, 255}, { 60, 130, 180, 255},
    {120, 160,  20, 255},
};
static const Color vehicle_palette_1988[MAX_VEHICLES] = {
    {255, 255, 255, 255}, { 60, 100, 255, 255}, {255,  40,  80, 255},
    {255, 220,  60, 255}, { 40, 255,  80, 255}, {255, 140,   0, 255},
    {180,  40, 255, 255}, {255,  20, 100, 255}, {  0, 200, 160, 255},
    {255, 180,  60, 255}, {120,  60, 255, 255}, {255, 255, 100, 255},
    { 60, 255, 200, 255}, {255,  60, 200, 255}, { 60, 200, 255, 255},
    {200, 255,  40, 255},
};

static const Color *get_vehicle_palette(view_mode_t vm) {
    switch (vm) {
        case VIEW_SNOW: return vehicle_palette_snow;
        case VIEW_REZ:  return vehicle_palette_rez;
        case VIEW_1988: return vehicle_palette_1988;
        default:        return vehicle_palette_grid;
    }
}

static void print_usage(const char *prog) {
    printf("Usage: %s [options]\n", prog);
    printf("  -udp <port>    UDP base port (default: 19410)\n");
    printf("  -n <count>     Number of vehicles (default: 1, max: %d)\n", MAX_VEHICLES);
    printf("  -mc            Multicopter model (default)\n");
    printf("  -fw            Fixed-wing model\n");
    printf("  -ts            Tailsitter model\n");
    printf("  -origin <lat> <lon> <alt>  NED origin in degrees/meters (default: PX4 SIH)\n");
    printf("  --replay <file1.ulg> [file2.ulg ...]  Replay ULog file(s)\n");
    printf("  --ghost <file1.ulg> [file2.ulg ...]   Ghost mode replay\n");
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
            while (i + 1 < argc && argv[i + 1][0] != '-') {
                if (num_replay_files >= MAX_VEHICLES) {
                    fprintf(stderr, "Too many replay files (max %d)\n", MAX_VEHICLES);
                    return 1;
                }
                replay_paths[num_replay_files++] = argv[++i];
            }
        } else if (strcmp(argv[i], "--ghost") == 0) {
            ghost_mode = true;
            while (i + 1 < argc && argv[i + 1][0] != '-') {
                if (num_replay_files >= MAX_VEHICLES) {
                    fprintf(stderr, "Too many replay files (max %d)\n", MAX_VEHICLES);
                    return 1;
                }
                replay_paths[num_replay_files++] = argv[++i];
            }
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

    // Init vehicles
    // Init scene first (provides lighting shader for vehicles)
    scene_t scene;
    scene_init(&scene);

    vehicle_t vehicles[MAX_VEHICLES];
    for (int i = 0; i < vehicle_count; i++) {
        vehicle_init(&vehicles[i], model_idx, scene.lighting_shader);
        vehicles[i].color = get_vehicle_palette(scene.view_mode)[i];
    }

    // For multi-vehicle MAVLink or explicit origin: pre-set the NED origin
    if (!is_replay && (vehicle_count > 1 || origin_specified)) {
        double lat0_rad = origin_lat * (M_PI / 180.0);
        double lon0_rad = origin_lon * (M_PI / 180.0);
        for (int i = 0; i < vehicle_count; i++) {
            vehicles[i].lat0 = lat0_rad;
            vehicles[i].lon0 = lon0_rad;
            vehicles[i].alt0 = origin_alt;
            vehicles[i].origin_set = true;
        }
        printf("NED origin: lat=%.6f lon=%.6f alt=%.1f\n", origin_lat, origin_lon, origin_alt);
    } else if (origin_specified) {
        double lat0_rad = origin_lat * (M_PI / 180.0);
        double lon0_rad = origin_lon * (M_PI / 180.0);
        for (int i = 0; i < vehicle_count; i++) {
            vehicles[i].lat0 = lat0_rad;
            vehicles[i].lon0 = lon0_rad;
            vehicles[i].alt0 = origin_alt;
            vehicles[i].origin_set = true;
        }
    }

    // Apply ghost mode: translucent non-primary drones
    // Origin sharing is handled at runtime — when vehicle[0]'s origin is set
    // during playback, we copy it to all other vehicles (see main loop below)
    if (ghost_mode && num_replay_files > 1) {
        vehicle_set_ghost_alpha(&vehicles[0], 1.0f);
        for (int i = 1; i < num_replay_files; i++)
            vehicle_set_ghost_alpha(&vehicles[i], 0.35f);
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

        // Ghost mode: once both primary and secondary drones have positions,
        // compute offset to collapse secondary onto primary's location
        if (ghost_mode && num_replay_files > 1 && vehicles[0].active) {
            for (int i = 1; i < num_replay_files; i++) {
                if (vehicles[i].active && vehicles[i].grid_offset.x == 0.0f
                    && vehicles[i].grid_offset.z == 0.0f) {
                    // First frame both are active — snap secondary to primary
                    vehicles[i].grid_offset.x = vehicles[0].position.x - vehicles[i].position.x;
                    vehicles[i].grid_offset.y = vehicles[0].position.y - vehicles[i].position.y;
                    vehicles[i].grid_offset.z = vehicles[0].position.z - vehicles[i].position.z;
                }
            }
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

        // Update vehicle colors when view mode changes
        {
            const Color *pal = get_vehicle_palette(scene.view_mode);
            for (int i = 0; i < vehicle_count; i++)
                vehicles[i].color = pal[i];
        }

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
            int max_modes = (num_replay_files > 1) ? 4 : 3;
            trail_mode = (trail_mode + 1) % max_modes;
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

        // Replay playback controls (apply to all sources)
        if (is_replay) {
            int nrf = num_replay_files > 0 ? num_replay_files : 1;
            if (IsKeyPressed(KEY_SPACE)) {
                if (!sources[selected].connected) {
                    for (int i = 0; i < nrf; i++) {
                        ulog_replay_ctx_t *rctx = (ulog_replay_ctx_t *)sources[i].impl;
                        ulog_replay_seek(rctx, 0.0f);
                        sources[i].connected = true;
                        sources[i].playback.paused = false;
                        vehicle_reset_trail(&vehicles[i]);
                    }
                } else {
                    bool p = !sources[selected].playback.paused;
                    for (int i = 0; i < nrf; i++)
                        sources[i].playback.paused = p;
                }
            }
            if (IsKeyPressed(KEY_L)) {
                bool l = !sources[selected].playback.looping;
                for (int i = 0; i < nrf; i++)
                    sources[i].playback.looping = l;
            }
            if (IsKeyPressed(KEY_I)) {
                bool interp = !sources[selected].playback.interpolation;
                for (int i = 0; i < nrf; i++)
                    sources[i].playback.interpolation = interp;
                printf("Interpolation: %s\n", interp ? "ON" : "OFF");
            }
            if (IsKeyPressed(KEY_EQUAL)) {
                float spd = sources[selected].playback.speed;
                if (spd < 0.5f) spd = 0.5f;
                else if (spd < 1.0f) spd = 1.0f;
                else if (spd < 2.0f) spd = 2.0f;
                else if (spd < 4.0f) spd = 4.0f;
                else if (spd < 8.0f) spd = 8.0f;
                else spd = 16.0f;
                for (int i = 0; i < nrf; i++)
                    sources[i].playback.speed = spd;
            }
            if (IsKeyPressed(KEY_MINUS)) {
                float spd = sources[selected].playback.speed;
                if (spd > 8.0f) spd = 8.0f;
                else if (spd > 4.0f) spd = 4.0f;
                else if (spd > 2.0f) spd = 2.0f;
                else if (spd > 1.0f) spd = 1.0f;
                else if (spd > 0.5f) spd = 0.5f;
                else spd = 0.25f;
                for (int i = 0; i < nrf; i++)
                    sources[i].playback.speed = spd;
            }
            if (IsKeyPressed(KEY_R)) {
                for (int i = 0; i < nrf; i++) {
                    ulog_replay_ctx_t *ctx = (ulog_replay_ctx_t *)sources[i].impl;
                    ulog_replay_seek(ctx, 0.0f);
                    sources[i].connected = true;
                    sources[i].playback.paused = false;
                    vehicle_reset_trail(&vehicles[i]);
                }
            }
            if (IsKeyPressed(KEY_RIGHT)) {
                float step = (IsKeyDown(KEY_LEFT_SHIFT) || IsKeyDown(KEY_RIGHT_SHIFT)) ? 30.0f : 5.0f;
                float seek_target = sources[selected].playback.position_s + step;
                for (int i = 0; i < nrf; i++) {
                    ulog_replay_ctx_t *ctx = (ulog_replay_ctx_t *)sources[i].impl;
                    ulog_replay_seek(ctx, seek_target);
                    sources[i].playback.position_s = (float)ctx->wall_accum;
                    vehicle_reset_trail(&vehicles[i]);
                }
            }
            if (IsKeyPressed(KEY_LEFT)) {
                float step = (IsKeyDown(KEY_LEFT_SHIFT) || IsKeyDown(KEY_RIGHT_SHIFT)) ? 30.0f : 5.0f;
                float target = sources[selected].playback.position_s - step;
                if (target < 0.0f) target = 0.0f;
                for (int i = 0; i < nrf; i++) {
                    ulog_replay_ctx_t *ctx = (ulog_replay_ctx_t *)sources[i].impl;
                    ulog_replay_seek(ctx, target);
                    sources[i].playback.position_s = (float)ctx->wall_accum;
                    vehicle_reset_trail(&vehicles[i]);
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
