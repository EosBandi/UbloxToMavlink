/*
 * serial.c - POSIX serial port and UDP link helpers
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
#define _GNU_SOURCE
#include "serial.h"
#include "log.h"

#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <termios.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <netdb.h>
#include <arpa/inet.h>

/* implemented in serial_baud.c (needs asm/termbits.h which clashes with termios.h) */
int serial_set_custom_baud(int fd, uint32_t baud);

static speed_t baud_to_speed(uint32_t baud)
{
    switch (baud) {
    case 4800:    return B4800;
    case 9600:    return B9600;
    case 19200:   return B19200;
    case 38400:   return B38400;
    case 57600:   return B57600;
    case 115200:  return B115200;
    case 230400:  return B230400;
#ifdef B460800
    case 460800:  return B460800;
#endif
#ifdef B500000
    case 500000:  return B500000;
#endif
#ifdef B921600
    case 921600:  return B921600;
#endif
#ifdef B1000000
    case 1000000: return B1000000;
#endif
#ifdef B1500000
    case 1500000: return B1500000;
#endif
#ifdef B2000000
    case 2000000: return B2000000;
#endif
    default:      return 0;
    }
}

int serial_set_baud(int fd, uint32_t baud)
{
    struct termios tio;
    if (tcgetattr(fd, &tio) != 0) return -1;
    speed_t sp = baud_to_speed(baud);
    if (sp == 0) {
        return serial_set_custom_baud(fd, baud);
    }
    cfsetispeed(&tio, sp);
    cfsetospeed(&tio, sp);
    if (tcsetattr(fd, TCSANOW, &tio) != 0) return -1;
    return 0;
}

int serial_open(const char *dev, uint32_t baud)
{
    int fd = open(dev, O_RDWR | O_NOCTTY | O_NONBLOCK | O_CLOEXEC);
    if (fd < 0) return -1;

    struct termios tio;
    if (tcgetattr(fd, &tio) != 0) {
        /* not a tty (e.g. a FIFO used in tests): still usable */
        return fd;
    }
    cfmakeraw(&tio);
    tio.c_cflag |= CLOCAL | CREAD;
    tio.c_cflag &= ~(CSTOPB | PARENB | CRTSCTS);
    tio.c_cflag |= CS8;
    tio.c_cc[VMIN] = 0;
    tio.c_cc[VTIME] = 0;
    if (tcsetattr(fd, TCSANOW, &tio) != 0) {
        close(fd);
        return -1;
    }
    if (serial_set_baud(fd, baud) != 0) {
        close(fd);
        return -1;
    }
    tcflush(fd, TCIOFLUSH);
    return fd;
}

void serial_flush(int fd) { tcflush(fd, TCIOFLUSH); }
void serial_drain(int fd) { tcdrain(fd); }

/* ------------------------------------------------------------------ */
/* generic link                                                       */
/* ------------------------------------------------------------------ */
static int parse_hostport(const char *s, char *host, size_t hostlen, char *port, size_t portlen)
{
    const char *colon = strrchr(s, ':');
    if (!colon || colon == s) return -1;
    size_t hl = (size_t)(colon - s);
    if (hl >= hostlen) return -1;
    memcpy(host, s, hl);
    host[hl] = 0;
    snprintf(port, portlen, "%s", colon + 1);
    return 0;
}

static int udp_open(link_t *l, const char *hostport, bool server)
{
    char host[128], port[16];
    if (parse_hostport(hostport, host, sizeof(host), port, sizeof(port)) != 0) {
        LOGE("bad UDP spec '%s' (expected host:port)", hostport);
        return -1;
    }
    struct addrinfo hints, *res = NULL;
    memset(&hints, 0, sizeof(hints));
    hints.ai_family = AF_UNSPEC;
    hints.ai_socktype = SOCK_DGRAM;
    if (server) hints.ai_flags = AI_PASSIVE;
    int rc = getaddrinfo(host, port, &hints, &res);
    if (rc != 0 || !res) {
        LOGE("getaddrinfo(%s:%s): %s", host, port, gai_strerror(rc));
        return -1;
    }
    int fd = socket(res->ai_family, SOCK_DGRAM | SOCK_NONBLOCK | SOCK_CLOEXEC, 0);
    if (fd < 0) {
        LOGE("socket: %s", strerror(errno));
        freeaddrinfo(res);
        return -1;
    }
    if (server) {
        int one = 1;
        setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));
        if (bind(fd, res->ai_addr, res->ai_addrlen) != 0) {
            LOGE("bind(%s:%s): %s", host, port, strerror(errno));
            close(fd);
            freeaddrinfo(res);
            return -1;
        }
        l->have_peer = false;
    } else {
        memcpy(l->peer, res->ai_addr, res->ai_addrlen);
        l->peer_len = res->ai_addrlen;
        l->have_peer = true;
    }
    freeaddrinfo(res);
    l->fd = fd;
    l->is_udp = true;
    l->udp_server = server;
    return 0;
}

int link_open(link_t *l, const char *spec, uint32_t baud)
{
    memset(l, 0, sizeof(*l));
    l->fd = -1;
    snprintf(l->spec, sizeof(l->spec), "%s", spec);
    l->baud = baud;
    if (strncmp(spec, "udp://", 6) == 0) return udp_open(l, spec + 6, false);
    if (strncmp(spec, "udpin://", 8) == 0) return udp_open(l, spec + 8, true);
    if (strncmp(spec, "serial://", 9) == 0) spec += 9;
    l->fd = serial_open(spec, baud);
    if (l->fd < 0) return -1;
    return 0;
}

void link_close(link_t *l)
{
    if (l->fd >= 0) close(l->fd);
    l->fd = -1;
}

ssize_t link_read(link_t *l, uint8_t *buf, size_t len)
{
    if (l->fd < 0) return -1;
    if (l->is_udp) {
        struct sockaddr_storage from;
        socklen_t fl = sizeof(from);
        ssize_t n = recvfrom(l->fd, buf, len, 0, (struct sockaddr *)&from, &fl);
        if (n < 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR || errno == ECONNREFUSED) return 0;
            return -1;
        }
        if (l->udp_server && fl <= sizeof(l->peer)) {
            memcpy(l->peer, &from, fl);
            l->peer_len = fl;
            l->have_peer = true;
        }
        return n;
    }
    ssize_t n = read(l->fd, buf, len);
    if (n < 0) {
        if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR) return 0;
        return -1;
    }
    if (n == 0) {
        /* EOF on a tty means the device went away (USB unplugged) */
        return -1;
    }
    return n;
}

ssize_t link_write(link_t *l, const uint8_t *buf, size_t len)
{
    if (l->fd < 0) return -1;
    if (l->is_udp) {
        if (!l->have_peer) return (ssize_t)len;   /* nobody to talk to yet */
        ssize_t n = sendto(l->fd, buf, len, 0, (const struct sockaddr *)l->peer, l->peer_len);
        if (n < 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR || errno == ECONNREFUSED
                || errno == ENETUNREACH || errno == EHOSTUNREACH) return 0;
            return -1;
        }
        return n;
    }
    size_t off = 0;
    int spins = 0;
    while (off < len) {
        ssize_t n = write(l->fd, buf + off, len - off);
        if (n < 0) {
            if (errno == EINTR) continue;
            if (errno == EAGAIN || errno == EWOULDBLOCK) {
                /* tx buffer full: wait briefly a few times, then drop the rest */
                if (++spins > 20) break;
                usleep(1000);
                continue;
            }
            return -1;
        }
        off += (size_t)n;
    }
    return (ssize_t)off;
}
