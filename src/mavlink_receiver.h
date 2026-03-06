#ifndef MAVLINK_RECEIVER_H
#define MAVLINK_RECEIVER_H

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    float quaternion[4]; // w, x, y, z
    int32_t lat;         // degE7
    int32_t lon;         // degE7
    int32_t alt;         // mm
    int16_t vx, vy, vz;          // cm/s, NED
    uint16_t ind_airspeed;       // cm/s
    uint16_t true_airspeed;      // cm/s
    bool valid;
} hil_state_t;

typedef struct {
    int sockfd;
    uint16_t port;
    bool connected;
    bool debug;
    uint8_t sysid;
    hil_state_t state;
} mavlink_receiver_t;

// Initialize UDP socket on given port. Returns 0 on success.
int mavlink_receiver_init(mavlink_receiver_t *recv, uint16_t port);

// Poll for new messages (non-blocking). Call once per frame.
void mavlink_receiver_poll(mavlink_receiver_t *recv);

// Cleanup socket.
void mavlink_receiver_close(mavlink_receiver_t *recv);

#endif
