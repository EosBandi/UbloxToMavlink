/*
 * serial.h - POSIX serial port and UDP link helpers
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
#ifndef SERIAL_H
#define SERIAL_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <sys/types.h>

/* Open a serial device raw/non-blocking at the given baud. Returns fd or -1. */
int  serial_open(const char *dev, uint32_t baud);
/* Change the baud rate of an open port. Returns 0 on success. */
int  serial_set_baud(int fd, uint32_t baud);
/* Discard pending input (and output). */
void serial_flush(int fd);
/* Block until all queued output has been transmitted. */
void serial_drain(int fd);

/*
 * A generic byte link: serial device or UDP socket.
 *   "/dev/ttyAMA0"            serial, baud from the separate argument
 *   "udp://host:port"         UDP client: send to host:port, receive from anyone
 *   "udpin://0.0.0.0:port"    UDP server: bind to port, reply to the last sender
 */
typedef struct {
    int  fd;
    bool is_udp;
    bool udp_server;
    bool have_peer;
    uint8_t peer[128];      /* struct sockaddr_storage, opaque here */
    unsigned peer_len;
    char spec[256];
    uint32_t baud;
} link_t;

int     link_open(link_t *l, const char *spec, uint32_t baud);
void    link_close(link_t *l);
/* returns bytes read, 0 if none, -1 on fatal error (link should be reopened) */
ssize_t link_read(link_t *l, uint8_t *buf, size_t len);
/* returns bytes written (may be short for serial), -1 on fatal error */
ssize_t link_write(link_t *l, const uint8_t *buf, size_t len);

#endif
