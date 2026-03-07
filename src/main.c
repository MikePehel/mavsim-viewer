#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#ifdef _MSC_VER
#define _USE_MATH_DEFINES
#endif
#include <math.h>

#include "raylib.h"
#include "raymath.h"
#include "mavlink_receiver.h"
#include "vehicle.h"
#include "scene.h"
#include "hud.h"

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

static void print_usage(const char *prog) {
    printf("Usage: %s [options]\n", prog);
    printf("  -udp <port>    UDP base port (default: 19410)\n");
    printf("  -n <count>     Number of vehicles (default: 1, max: %d)\n", MAX_VEHICLES);
    printf("  -mc            Multicopter model (default)\n");
    printf("  -fw            Fixed-wing model\n");
    printf("  -ts            Tailsitter model\n");
    printf("  -origin <lat> <lon> <alt>  NED origin in degrees/meters (default: PX4 SIH)\n");
    printf("  -w <width>     Window width (default: 1280)\n");
    printf("  -h <height>    Window height (default: 720)\n");
}

int main(int argc, char *argv[]) {
    uint16_t base_port = 19410;
    int vehicle_count = 1;
    vehicle_type_t vtype = VEHICLE_MULTICOPTER;
    int win_w = 1280;
    int win_h = 720;
    bool debug = false;
    bool fake = false;
    // PX4 SIH default spawn position
    double origin_lat = 47.397742;
    double origin_lon = 8.545594;
    double origin_alt = 489.4;
    bool origin_specified = false;

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
            vtype = VEHICLE_MULTICOPTER;
        } else if (strcmp(argv[i], "-fw") == 0) {
            vtype = VEHICLE_FIXEDWING;
        } else if (strcmp(argv[i], "-ts") == 0) {
            vtype = VEHICLE_TAILSITTER;
        } else if (strcmp(argv[i], "-d") == 0) {
            debug = true;
        } else if (strcmp(argv[i], "-fake") == 0) {
            fake = true;
        } else if (strcmp(argv[i], "-w") == 0 && i + 1 < argc) {
            win_w = atoi(argv[++i]);
        } else if (strcmp(argv[i], "-h") == 0 && i + 1 < argc) {
            win_h = atoi(argv[++i]);
        } else if (strcmp(argv[i], "--help") == 0) {
            print_usage(argv[0]);
            return 0;
        }
    }

    // Init Raylib
    SetConfigFlags(FLAG_MSAA_4X_HINT | FLAG_WINDOW_RESIZABLE);
    InitWindow(win_w, win_h, "MAVSim Viewer");
    SetTargetFPS(60);

    // Init MAVLink receivers
    mavlink_receiver_t receivers[MAX_VEHICLES];
    memset(receivers, 0, sizeof(receivers));
    if (!fake) {
        for (int i = 0; i < vehicle_count; i++) {
            receivers[i].debug = debug;
            if (mavlink_receiver_init(&receivers[i], base_port + i, (uint8_t)i) != 0) {
                fprintf(stderr, "Failed to init MAVLink receiver on port %u\n", base_port + i);
                CloseWindow();
                return 1;
            }
        }
    }

    // Init scene first (provides lighting shader for vehicles)
    scene_t scene;
    scene_init(&scene);

    // Init vehicles
    vehicle_t vehicles[MAX_VEHICLES];
    for (int i = 0; i < vehicle_count; i++) {
        vehicle_init(&vehicles[i], vtype, scene.lighting_shader);
        vehicles[i].color = vehicle_colors[i];
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

    // Fake mode: place vehicles in a grid with dummy telemetry
    if (fake) {
        for (int i = 0; i < vehicle_count; i++) {
            float row = (float)(i / 3);
            float col = (float)(i % 3);
            vehicles[i].position = (Vector3){ (col - 1.0f) * 5.0f, 2.0f + i * 0.5f, (row - 1.0f) * 5.0f };
            vehicles[i].rotation = QuaternionIdentity();
            vehicles[i].active = true;
            vehicles[i].sysid = (uint8_t)(i + 1);
            vehicles[i].heading_deg = (float)(i * 40);
            vehicles[i].roll_deg = (float)(i * 3 - 12);
            vehicles[i].pitch_deg = (float)(i * 2 - 8);
            vehicles[i].ground_speed = 5.0f + i * 1.5f;
            vehicles[i].vertical_speed = (i % 2 == 0) ? 1.5f : -0.8f;
            vehicles[i].airspeed = 8.0f + i * 2.0f;
            vehicles[i].altitude_rel = 10.0f + i * 5.0f;
            receivers[i].connected = true;
            receivers[i].sysid = (uint8_t)(i + 1);
        }
    }

    hud_t hud;
    hud_init(&hud);

    int selected = 0;
    bool show_hud = true;
    bool show_trails = true;

    // Main loop
    while (!WindowShouldClose()) {
        // Poll all MAVLink receivers and update vehicles
        if (!fake) {
            for (int i = 0; i < vehicle_count; i++) {
                mavlink_receiver_poll(&receivers[i]);
                vehicle_update(&vehicles[i], &receivers[i].state);
                vehicles[i].sysid = receivers[i].sysid;
            }
        }

        // Check if any receiver is connected (for HUD)
        bool any_connected = false;
        for (int i = 0; i < vehicle_count; i++) {
            if (receivers[i].connected) { any_connected = true; break; }
        }

        // Update HUD sim time from selected vehicle
        hud_update(&hud, receivers[selected].state.time_usec,
                   receivers[selected].connected, GetFrameTime());

        // Handle input
        if (IsKeyPressed(KEY_T)) show_trails = !show_trails;
        scene_handle_input(&scene);

        // Help overlay toggle (? key = Shift+/)
        if (IsKeyPressed(KEY_SLASH) && (IsKeyDown(KEY_LEFT_SHIFT) || IsKeyDown(KEY_RIGHT_SHIFT))) {
            hud.show_help = !hud.show_help;
        }

        // Toggle HUD visibility
        if (IsKeyPressed(KEY_H)) {
            show_hud = !show_hud;
        }

        // Vehicle selection input
        if (vehicle_count > 1) {
            if (IsKeyPressed(KEY_TAB)) {
                // Cycle to next connected vehicle, clear pins
                for (int j = 1; j <= vehicle_count; j++) {
                    int next = (selected + j) % vehicle_count;
                    if (receivers[next].connected) { selected = next; break; }
                }
                hud.pinned_count = 0;
                memset(hud.pinned, -1, sizeof(hud.pinned));
            }
            if (IsKeyPressed(KEY_LEFT_BRACKET)) {
                for (int j = 1; j <= vehicle_count; j++) {
                    int prev = (selected - j + vehicle_count) % vehicle_count;
                    if (receivers[prev].connected) { selected = prev; break; }
                }
                hud.pinned_count = 0;
                memset(hud.pinned, -1, sizeof(hud.pinned));
            }
            if (IsKeyPressed(KEY_RIGHT_BRACKET)) {
                for (int j = 1; j <= vehicle_count; j++) {
                    int next = (selected + j) % vehicle_count;
                    if (receivers[next].connected) { selected = next; break; }
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

        // WASDQE movement for fake mode
        if (fake) {
            float spd = 5.0f * GetFrameTime();
            vehicle_t *sv = &vehicles[selected];
            float dx = 0, dy = 0, dz = 0;
            if (IsKeyDown(KEY_W)) dz -= spd;
            if (IsKeyDown(KEY_S)) dz += spd;
            if (IsKeyDown(KEY_A)) dx -= spd;
            if (IsKeyDown(KEY_D)) dx += spd;
            if (IsKeyDown(KEY_E)) dy += spd;
            if (IsKeyDown(KEY_Q)) dy -= spd;
            sv->position.x += dx;
            sv->position.y += dy;
            sv->position.z += dz;

            // Derive fake telemetry from movement
            if (dx != 0 || dz != 0) {
                sv->heading_deg = atan2f(dx, -dz) * RAD2DEG;
                if (sv->heading_deg < 0) sv->heading_deg += 360.0f;
                sv->roll_deg = dx / spd * 15.0f;
                sv->pitch_deg = dz / spd * -10.0f;
                sv->ground_speed = sqrtf(dx*dx + dz*dz) / GetFrameTime();
            } else {
                sv->roll_deg *= 0.9f;
                sv->pitch_deg *= 0.9f;
                sv->ground_speed *= 0.9f;
            }
            sv->vertical_speed = dy / GetFrameTime();
            sv->altitude_rel = sv->position.y;

            // Rebuild rotation quaternion from euler angles
            Quaternion qYaw   = QuaternionFromAxisAngle((Vector3){0,1,0}, sv->heading_deg * DEG2RAD);
            Quaternion qPitch = QuaternionFromAxisAngle((Vector3){1,0,0}, sv->pitch_deg * DEG2RAD);
            Quaternion qRoll  = QuaternionFromAxisAngle((Vector3){0,0,1}, sv->roll_deg * DEG2RAD);
            sv->rotation = QuaternionMultiply(qYaw, QuaternionMultiply(qPitch, qRoll));

            // Sample trails for all vehicles
            float dt = GetFrameTime();
            for (int i = 0; i < vehicle_count; i++) {
                vehicle_t *v = &vehicles[i];
                v->trail_timer += dt;
                if (v->trail_timer >= 0.05f) {
                    v->trail_timer = 0.0f;
                    v->trail[v->trail_head] = v->position;
                    v->trail_roll[v->trail_head] = v->roll_deg;
                    v->trail_pitch[v->trail_head] = v->pitch_deg;
                    v->trail_vert[v->trail_head] = v->vertical_speed;
                    v->trail_head = (v->trail_head + 1) % v->trail_capacity;
                    if (v->trail_count < v->trail_capacity) v->trail_count++;
                }
            }
        }

        // Update camera to follow selected vehicle
        scene_update_camera(&scene, vehicles[selected].position, vehicles[selected].rotation);

        // Render
        BeginDrawing();

            // Sky background
            scene_draw_sky(&scene);

            BeginMode3D(scene.camera);
                scene_draw(&scene);
                for (int i = 0; i < vehicle_count; i++) {
                    if (vehicles[i].active || vehicle_count == 1) {
                        vehicle_draw(&vehicles[i], scene.view_mode, i == selected, show_trails);
                    }
                }
            EndMode3D();

            // HUD
            if (show_hud) {
                hud_draw(&hud, vehicles, receivers, vehicle_count,
                         selected, GetScreenWidth(), GetScreenHeight(),
                         scene.view_mode);
            }

        EndDrawing();
    }

    // Cleanup
    hud_cleanup(&hud);
    for (int i = 0; i < vehicle_count; i++) {
        vehicle_cleanup(&vehicles[i]);
        if (!fake) mavlink_receiver_close(&receivers[i]);
    }
    scene_cleanup(&scene);
    CloseWindow();

    return 0;
}
