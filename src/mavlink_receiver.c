#include "mavlink_receiver.h"

#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

// MAVLink config: use common message set
#include <mavlink.h>

int mavlink_receiver_init(mavlink_receiver_t *recv, uint16_t port) {
    bool debug = recv->debug;
    memset(recv, 0, sizeof(*recv));
    recv->debug = debug;
    recv->port = port;

    recv->sockfd = socket(AF_INET, SOCK_DGRAM, 0);
    if (recv->sockfd < 0) {
        perror("socket");
        return -1;
    }

    // Set non-blocking
    int flags = fcntl(recv->sockfd, F_GETFL, 0);
    fcntl(recv->sockfd, F_SETFL, flags | O_NONBLOCK);

    struct sockaddr_in addr;
    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = INADDR_ANY;
    addr.sin_port = htons(port);

    if (bind(recv->sockfd, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        perror("bind");
        close(recv->sockfd);
        return -1;
    }

    printf("Listening for MAVLink on UDP port %u\n", port);
    return 0;
}

void mavlink_receiver_poll(mavlink_receiver_t *recv) {
    uint8_t buf[2048];
    struct sockaddr_in sender;
    socklen_t sender_len = sizeof(sender);

    for (;;) {
        ssize_t n = recvfrom(recv->sockfd, buf, sizeof(buf), 0,
                             (struct sockaddr *)&sender, &sender_len);
        if (n <= 0) break;

        mavlink_message_t msg;
        mavlink_status_t status;

        for (ssize_t i = 0; i < n; i++) {
            if (mavlink_parse_char(MAVLINK_COMM_0, buf[i], &msg, &status)) {
                if (recv->debug) {
                    printf("[MAVLink] msgid=%u sysid=%u compid=%u seq=%u len=%u\n",
                           msg.msgid, msg.sysid, msg.compid, msg.seq, msg.len);
                }

                switch (msg.msgid) {
                    case MAVLINK_MSG_ID_HEARTBEAT:
                        if (!recv->connected) {
                            recv->connected = true;
                            recv->sysid = msg.sysid;
                            printf("Connected to system %u\n", msg.sysid);
                        }
                        break;

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
}

void mavlink_receiver_close(mavlink_receiver_t *recv) {
    if (recv->sockfd >= 0) {
        close(recv->sockfd);
        recv->sockfd = -1;
    }
}
