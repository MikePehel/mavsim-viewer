#ifndef MAVLINK_RECEIVER_H
#define MAVLINK_RECEIVER_H

#include <stdbool.h>
#include <stdint.h>

#ifdef _WIN32
typedef uintptr_t sock_t;
#define SOCK_INVALID (~(sock_t)0)
#else
typedef int sock_t;
#define SOCK_INVALID (-1)
#endif

typedef struct {
    float quaternion[4]; // w, x, y, z
    int32_t lat;         // degE7
    int32_t lon;         // degE7
    int32_t alt;         // mm
    int16_t vx, vy, vz;          // cm/s, NED
    uint16_t ind_airspeed;       // cm/s
    uint16_t true_airspeed;      // cm/s
    uint64_t time_usec;          // timestamp (time since boot), microseconds
    bool valid;
} hil_state_t;

typedef struct {
    int32_t lat;         // degE7
    int32_t lon;         // degE7
    int32_t alt;         // mm (AMSL)
    bool valid;
} home_position_t;

typedef struct {
    sock_t sockfd;
    uint16_t port;
    uint8_t channel;
    bool connected;
    bool debug;
    uint8_t sysid;
    uint8_t mav_type;            // MAV_TYPE from heartbeat
    double last_msg_time;        // wall-clock time of last received message
    hil_state_t state;
    home_position_t home;
    bool sender_known;           // true once we've seen a packet
    uint8_t sender_addr[16];     // sockaddr_in stored as opaque bytes

    // SIH_TERR_* request-retry state. After the initial PARAM_REQUEST_READ
    // batch on connect, the poll loop watches the listener's missing-mask
    // and re-issues PARAM_REQUEST_READ for the specific names that haven't
    // come back yet, with exponential backoff (0.5s, 1s, 2s, 4s, 8s).
    // UDP can drop one of the 5 PARAM_VALUE replies and PX4 doesn't
    // notice; this layer makes Hawkeye notice instead.
    int    terr_retry_level;     // 0 = just issued initial; 1..5 = retry N pending
    double terr_next_check_time; // wall-clock; check + maybe retry at/after this
    bool   terr_retry_done;      // set true once initial sync settled OR backoff exhausted

    // Periodic re-sync of the SIH_TERR_* set, layered on top of the
    // initial retry. After initial sync, the poll loop fires a fresh
    // PARAM_REQUEST_READ for all 5 names every TERR_REFRESH_INTERVAL_S
    // seconds, so runtime PARAM_SETs (e.g. from MAVSDK scripts) reflect
    // in the viewer within one interval even though PX4 doesn't
    // broadcast PARAM_VALUE on set. The freshness layer (per-name
    // received-at timestamps in the listener) logs a STALE warning if
    // any name's last-seen exceeds STALE_THRESHOLD — that catches the
    // case where the periodic loop fires but the replies are vanishing.
    double terr_next_refresh_time;        // wall-clock; next periodic refresh
    bool   terr_stale_logged[8];          // per-name "currently logged stale" gate
                                          // (size 8 to avoid #include of the listener
                                          // count; actual count is 5)
} mavlink_receiver_t;

// Initialize UDP socket on given port with MAVLink parse channel. Returns 0 on success.
int mavlink_receiver_init(mavlink_receiver_t *recv, uint16_t port, uint8_t channel);

// Poll for new messages (non-blocking). Call once per frame.
void mavlink_receiver_poll(mavlink_receiver_t *recv);

// Cleanup socket.
void mavlink_receiver_close(mavlink_receiver_t *recv);

#endif
