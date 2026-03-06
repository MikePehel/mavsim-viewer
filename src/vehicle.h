#ifndef VEHICLE_H
#define VEHICLE_H

#include "raylib.h"
#include "mavlink_receiver.h"

typedef enum {
    VEHICLE_MULTICOPTER = 0,
    VEHICLE_FIXEDWING,
    VEHICLE_TAILSITTER,
    VEHICLE_COUNT
} vehicle_type_t;

typedef struct {
    Model model;
    Vector3 position;        // Raylib coords (Y-up, right-handed)
    Quaternion rotation;     // Raylib quaternion
    vehicle_type_t type;
    bool origin_set;
    bool active;             // has received data
    double lat0;             // radians
    double lon0;             // radians
    double alt0;             // meters
    float model_scale;
    float heading_deg;       // yaw 0-360
    float roll_deg;          // roll in degrees
    float pitch_deg;         // pitch in degrees
    float ground_speed;      // m/s
    float vertical_speed;    // m/s (positive = climbing)
    float airspeed;          // m/s
    float altitude_rel;      // meters above origin
    uint8_t sysid;
    Color color;
} vehicle_t;

// Load vehicle model. type selects which OBJ to load.
void vehicle_init(vehicle_t *v, vehicle_type_t type);

// Update position/rotation from HIL_STATE_QUATERNION data.
void vehicle_update(vehicle_t *v, const hil_state_t *state);

// Draw the vehicle model. If selected is false, apply tint and draw marker sphere.
void vehicle_draw(vehicle_t *v, bool selected);

// Unload model resources.
void vehicle_cleanup(vehicle_t *v);

#endif
