#include "mavlink_receiver.h"

#include <stdio.h>
#include <string.h>
#include <errno.h>
#ifndef _WIN32
#include <time.h>
#endif

#ifdef _WIN32
#include <ws2tcpip.h>
#define SOCK_CLOSE(s) closesocket(s)
#else
#include <unistd.h>
#include <fcntl.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#define SOCK_CLOSE(s) close(s)
#endif

// MAVLink config: use common message set
// MAVLINK_COMM_NUM_BUFFERS=16 set via CMake for multi-vehicle support
#include <mavlink.h>

#include "mavlink/terrain_params_listener.h"

#define DISCONNECT_TIMEOUT_S 2.0

static void request_home_position(mavlink_receiver_t *recv) {
    if (!recv->sender_known) return;

    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];

    mavlink_msg_command_long_pack(255, 0, &msg,
        recv->sysid, 1,               // target system, component
        MAV_CMD_REQUEST_MESSAGE,       // command 512
        0,                             // confirmation
        MAVLINK_MSG_ID_HOME_POSITION,  // param1: message id to request
        0, 0, 0, 0, 0, 0);            // params 2-7 unused

    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    sendto(recv->sockfd, (char *)buf, len, 0,
           (struct sockaddr *)recv->sender_addr, sizeof(struct sockaddr_in));
    printf("Requested HOME_POSITION from system %u\n", recv->sysid);
}

/*
 * PX4 never spontaneously broadcasts PARAM_VALUE — they only fly in
 * response to PARAM_SET (echo to sender) or PARAM_REQUEST_LIST/READ.
 * Hawkeye's listener is purely passive, so without an explicit read
 * here the terrain mesh never paints in live SITL even though the
 * firmware is consuming the params (drone visibly hits invisible geo).
 * Target the 5 SIH_TERR_* names the listener handles by name lookup
 * (param_index = -1) rather than dumping the full ~900-param table.
 */
static void request_terrain_params(mavlink_receiver_t *recv) {
    if (!recv->sender_known) return;

    /* SIH_TERR_* — fBm elevation restored on top of
     * wall lattice; both consume the same SEED. Five params on the wire:
     *   EN     — master enable (gates walls + elevation)
     *   AMP    — fBm peak amplitude [m] (0 = flat even when EN=1)
     *   FREQ   — fBm base spatial frequency [1/m]
     *   SEED   — integer hash seed (drives wall layout AND fBm pattern)
     *   PLANE  — optional planar slope [deg]; non-zero replaces fBm */
    static const char *const k_terrain_param_ids[] = {
        "SIH_TERR_EN",
        "SIH_TERR_AMP",
        "SIH_TERR_FREQ",
        "SIH_TERR_SEED",
        "SIH_TERR_PLANE",
    };
    const size_t n = sizeof(k_terrain_param_ids) / sizeof(k_terrain_param_ids[0]);

    for (size_t i = 0; i < n; i++) {
        mavlink_message_t msg;
        uint8_t buf[MAVLINK_MAX_PACKET_LEN];

        mavlink_msg_param_request_read_pack(255, 0, &msg,
            recv->sysid, 1,             // target system, component
            k_terrain_param_ids[i],
            -1);                         // lookup by name

        uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
        sendto(recv->sockfd, (char *)buf, len, 0,
               (struct sockaddr *)recv->sender_addr, sizeof(struct sockaddr_in));
    }
    printf("Requested %zu SIH_TERR_* params from system %u\n", n, recv->sysid);
}

/*
 * Send a PARAM_REQUEST_READ for one named SIH_TERR_* param. The retry
 * layer above this function calls it for the specific names the
 * listener hasn't seen yet, so we don't re-ask for the ones that
 * already came through on the initial batch.
 */
static void request_one_terrain_param(mavlink_receiver_t *recv, const char *name) {
    if (!recv->sender_known || !name) return;

    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];

    mavlink_msg_param_request_read_pack(255, 0, &msg,
        recv->sysid, 1,
        name,
        -1);

    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    sendto(recv->sockfd, (char *)buf, len, 0,
           (struct sockaddr *)recv->sender_addr, sizeof(struct sockaddr_in));
}

static double get_wall_time(void) {
#ifdef _WIN32
    LARGE_INTEGER freq, count;
    QueryPerformanceFrequency(&freq);
    QueryPerformanceCounter(&count);
    return (double)count.QuadPart / (double)freq.QuadPart;
#else
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (double)ts.tv_sec + (double)ts.tv_nsec * 1e-9;
#endif
}

static int set_nonblocking(sock_t s) {
#ifdef _WIN32
    u_long mode = 1;
    return ioctlsocket(s, FIONBIO, &mode);
#else
    int flags = fcntl(s, F_GETFL, 0);
    return fcntl(s, F_SETFL, flags | O_NONBLOCK);
#endif
}

int mavlink_receiver_init(mavlink_receiver_t *recv, uint16_t port, uint8_t channel) {
    bool debug = recv->debug;
    memset(recv, 0, sizeof(*recv));
    recv->debug = debug;
    recv->port = port;
    recv->channel = channel;
    recv->sockfd = SOCK_INVALID;

#ifdef _WIN32
    WSADATA wsa;
    if (WSAStartup(MAKEWORD(2, 2), &wsa) != 0) {
        fprintf(stderr, "WSAStartup failed: %d\n", WSAGetLastError());
        return -1;
    }
#endif

    recv->sockfd = socket(AF_INET, SOCK_DGRAM, 0);
    if (recv->sockfd == SOCK_INVALID) {
        perror("socket");
        return -1;
    }

    set_nonblocking(recv->sockfd);

    struct sockaddr_in addr;
    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = INADDR_ANY;
    addr.sin_port = htons(port);

    if (bind(recv->sockfd, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        perror("bind");
        SOCK_CLOSE(recv->sockfd);
        recv->sockfd = SOCK_INVALID;
        return -1;
    }

    printf("Listening for MAVLink on UDP port %u\n", port);
    return 0;
}

void mavlink_receiver_poll(mavlink_receiver_t *recv) {
    uint8_t buf[2048];
    struct sockaddr_in sender;
    socklen_t sender_len = sizeof(sender);
    bool got_data = false;

    for (;;) {
        int n = recvfrom(recv->sockfd, (char *)buf, sizeof(buf), 0,
                         (struct sockaddr *)&sender, &sender_len);
        if (n <= 0) break;

        got_data = true;

        if (!recv->sender_known) {
            memcpy(recv->sender_addr, &sender, sizeof(struct sockaddr_in));
            recv->sender_known = true;
        }

        mavlink_message_t msg;
        mavlink_status_t status;

        for (int i = 0; i < n; i++) {
            if (mavlink_parse_char(recv->channel, buf[i], &msg, &status)) {
                if (recv->debug) {
                    printf("[MAVLink] msgid=%u sysid=%u compid=%u seq=%u len=%u\n",
                           msg.msgid, msg.sysid, msg.compid, msg.seq, msg.len);
                }

                /*
                 * Fire the initial requests on the FIRST message of any
                 * type, not just on HEARTBEAT. PX4's SIH custom mavlink
                 * stream (see ROMFS/.../px4-rc.mavlink, the sihsim
                 * `-m custom` block) only streams HIL_ACTUATOR_CONTROLS
                 * and HIL_STATE_QUATERNION — no HEARTBEAT — so a
                 * heartbeat-gated trigger would never fire on the link
                 * that actually carries the SIH_TERR_* state we need.
                 */
                if (!recv->connected) {
                    recv->connected = true;
                    recv->sysid = msg.sysid;
                    printf("Connected to system %u (first msgid %u)\n",
                           msg.sysid, msg.msgid);
                    request_home_position(recv);
                    request_terrain_params(recv);
                    /* Arm the retry timer for any PARAM_VALUE that
                     * doesn't make the round trip within 0.5 s. */
                    recv->terr_retry_level = 0;
                    recv->terr_next_check_time = get_wall_time() + 0.5;
                    recv->terr_retry_done = false;
                }

                switch (msg.msgid) {
                    case MAVLINK_MSG_ID_HEARTBEAT: {
                        mavlink_heartbeat_t hb;
                        mavlink_msg_heartbeat_decode(&msg, &hb);
                        recv->mav_type = hb.type;
                        break;
                    }

                    case MAVLINK_MSG_ID_HOME_POSITION: {
                        mavlink_home_position_t hp;
                        mavlink_msg_home_position_decode(&msg, &hp);
                        recv->home.lat = hp.latitude;
                        recv->home.lon = hp.longitude;
                        recv->home.alt = hp.altitude;
                        recv->home.valid = true;
                        printf("Home position: lat=%.7f lon=%.7f alt=%.1fm\n",
                               hp.latitude * 1e-7, hp.longitude * 1e-7, hp.altitude * 1e-3);
                        break;
                    }

                    case MAVLINK_MSG_ID_PARAM_VALUE: {
                        mavlink_param_value_t pv;
                        mavlink_msg_param_value_decode(&msg, &pv);
                        terrain_params_on_param_value(&pv);
                        break;
                    }

                    case MAVLINK_MSG_ID_HIL_STATE_QUATERNION: {
                        mavlink_hil_state_quaternion_t hil;
                        mavlink_msg_hil_state_quaternion_decode(&msg, &hil);

                        recv->state.quaternion[0] = hil.attitude_quaternion[0];
                        recv->state.quaternion[1] = hil.attitude_quaternion[1];
                        recv->state.quaternion[2] = hil.attitude_quaternion[2];
                        recv->state.quaternion[3] = hil.attitude_quaternion[3];
                        recv->state.lat = hil.lat;
                        recv->state.lon = hil.lon;
                        recv->state.alt = hil.alt;
                        recv->state.vx = hil.vx;
                        recv->state.vy = hil.vy;
                        recv->state.vz = hil.vz;
                        recv->state.ind_airspeed = hil.ind_airspeed;
                        recv->state.true_airspeed = hil.true_airspeed;
                        recv->state.time_usec = hil.time_usec;
                        recv->state.valid = true;

                        if (recv->debug) {
                            printf("  HIL_STATE_Q: lat=%d lon=%d alt=%d q=[%.3f,%.3f,%.3f,%.3f]\n",
                                   hil.lat, hil.lon, hil.alt,
                                   hil.attitude_quaternion[0], hil.attitude_quaternion[1],
                                   hil.attitude_quaternion[2], hil.attitude_quaternion[3]);
                        }
                        break;
                    }
                }
            }
        }
    }

    if (got_data) {
        recv->last_msg_time = get_wall_time();
    } else if (recv->connected && recv->last_msg_time > 0) {
        double now = get_wall_time();
        if (now - recv->last_msg_time > DISCONNECT_TIMEOUT_S) {
            recv->connected = false;
            recv->state.valid = false;
            recv->home.valid = false;
            recv->sender_known = false;
            printf("Disconnected from system %u\n", recv->sysid);
        }
    }

    /*
     * SIH_TERR_* sync layers.
     *
     * initial-sync retry with exponential backoff. PX4 unicasts
     * PARAM_VALUE only to the setter (or, for our PARAM_REQUEST_READ,
     * back to us once each), so a single UDP drop strands one name. We
     * re-request the missing ones at 0.5/1/2/4/8 s and give up at 5
     * retries.
     *
     * periodic refresh + stale detection. After initial sync
     * completes, fire a fresh PARAM_REQUEST_READ for all 5 names every
     * TERR_REFRESH_INTERVAL_S so runtime PARAM_SETs (test scripts
     * mutating SIH_TERR_*) reflect in the viewer within one interval.
     * Per-name freshness is tracked by the listener; if any name's
     * last-seen is older than TERR_STALE_THRESHOLD_S we log a STALE
     * warning so a silently broken refresh loop becomes visible. The
     * stale-state is edge-triggered per name so the log doesn't spam
     * once stale → either fresh again or still stale.
     */
    const double TERR_REFRESH_INTERVAL_S = 5.0;
    const double TERR_STALE_THRESHOLD_S  = 12.0; /* > 2x interval */

    if (recv->connected && !recv->terr_retry_done) {
        /* initial-sync retry. */
        double now = get_wall_time();
        unsigned int missing = terrain_params_listener_missing_mask();

        if (missing == 0) {
            printf("[terrain] all SIH_TERR_* in sync from system %u\n",
                   recv->sysid);
            recv->terr_retry_done = true;
            recv->terr_next_refresh_time = now + TERR_REFRESH_INTERVAL_S;
        } else if (now >= recv->terr_next_check_time) {
            const int MAX_RETRIES = 5;
            if (recv->terr_retry_level >= MAX_RETRIES) {
                printf("[terrain] giving up after %d retries; "
                       "missing mask=0x%02x (some SIH_TERR_* will be stale)\n",
                       MAX_RETRIES, missing);
                recv->terr_retry_done = true;
                recv->terr_next_refresh_time = now + TERR_REFRESH_INTERVAL_S;
            } else {
                int n_count = terrain_params_listener_param_count();
                int n_resent = 0;
                for (int i = 0; i < n_count; i++) {
                    if (missing & (1u << i)) {
                        const char *name = terrain_params_listener_name_for_index(i);
                        if (name) {
                            request_one_terrain_param(recv, name);
                            n_resent++;
                        }
                    }
                }
                recv->terr_retry_level++;
                double delay = 0.5 * (double)(1u << recv->terr_retry_level);
                recv->terr_next_check_time = now + delay;
                printf("[terrain] retry %d/%d: re-requested %d missing "
                       "(mask=0x%02x), next check in %.1fs\n",
                       recv->terr_retry_level, MAX_RETRIES, n_resent,
                       missing, delay);
            }
        }
    } else if (recv->connected && recv->terr_retry_done) {
        /* periodic refresh + stale detection. */
        double now = get_wall_time();
        int n_count = terrain_params_listener_param_count();

        if (now >= recv->terr_next_refresh_time) {
            /* Silently re-request all 5 names. */
            for (int i = 0; i < n_count; i++) {
                const char *name = terrain_params_listener_name_for_index(i);
                if (name) request_one_terrain_param(recv, name);
            }
            recv->terr_next_refresh_time = now + TERR_REFRESH_INTERVAL_S;
        }

        /* Per-name stale check. Edge-triggered logging: log on the
         * transition fresh→stale and on stale→fresh, but not every poll
         * while stale stays stale. */
        for (int i = 0; i < n_count; i++) {
            double age = terrain_params_listener_seconds_since_received(i);
            if (age < 0.0) continue;  /* never received — reported */
            const char *name = terrain_params_listener_name_for_index(i);
            bool is_stale = (age > TERR_STALE_THRESHOLD_S);
            if (is_stale && !recv->terr_stale_logged[i]) {
                printf("[terrain] STALE: %s last seen %.1fs ago (>%.0fs threshold) — "
                       "refresh round is failing for this name\n",
                       name ? name : "(?)", age, TERR_STALE_THRESHOLD_S);
                recv->terr_stale_logged[i] = true;
            } else if (!is_stale && recv->terr_stale_logged[i]) {
                printf("[terrain] RECOVERED: %s fresh again (last seen %.1fs ago)\n",
                       name ? name : "(?)", age);
                recv->terr_stale_logged[i] = false;
            }
        }
    }
}

void mavlink_receiver_close(mavlink_receiver_t *recv) {
    if (recv->sockfd != SOCK_INVALID) {
        SOCK_CLOSE(recv->sockfd);
        recv->sockfd = SOCK_INVALID;
    }
#ifdef _WIN32
    WSACleanup();
#endif
}
