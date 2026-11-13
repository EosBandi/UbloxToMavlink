/*
 * serial_baud.c - non-standard baud rates via termios2 (Linux only).
 * Kept in its own translation unit because <asm/termbits.h> conflicts
 * with <termios.h>.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
#include <stdint.h>
#ifdef __linux__
#include <sys/ioctl.h>
#include <asm/termbits.h>
#include <asm/ioctls.h>

int serial_set_custom_baud(int fd, uint32_t baud)
{
    struct termios2 tio;
    if (ioctl(fd, TCGETS2, &tio) != 0) return -1;
    tio.c_cflag &= ~CBAUD;
    tio.c_cflag |= BOTHER;
    tio.c_ispeed = baud;
    tio.c_ospeed = baud;
    if (ioctl(fd, TCSETS2, &tio) != 0) return -1;
    return 0;
}
#else
int serial_set_custom_baud(int fd, uint32_t baud)
{
    (void)fd; (void)baud;
    return -1;
}
#endif
