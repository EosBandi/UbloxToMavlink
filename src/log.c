/* log.c - tiny leveled logger
 * SPDX-License-Identifier: GPL-3.0-or-later */
#define _GNU_SOURCE
#include "log.h"
#include <stdio.h>
#include <time.h>
#include <string.h>

static int g_level = LOG_INFO;
static const char *names[] = { "ERR", "WRN", "INF", "DBG", "TRC" };

void log_set_level(int level)
{
    if (level < LOG_ERROR) level = LOG_ERROR;
    if (level > LOG_TRACE) level = LOG_TRACE;
    g_level = level;
}
int log_get_level(void) { return g_level; }

void log_msg(int level, const char *fmt, ...)
{
    if (level > g_level) return;
    struct timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);
    struct tm tm;
    localtime_r(&ts.tv_sec, &tm);
    char tbuf[32];
    strftime(tbuf, sizeof(tbuf), "%H:%M:%S", &tm);
    char line[1024];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(line, sizeof(line), fmt, ap);
    va_end(ap);
    fprintf(stderr, "%s.%03ld [%s] %s\n", tbuf, ts.tv_nsec / 1000000L, names[level], line);
    fflush(stderr);
}

void log_hexdump(int level, const char *prefix, const void *data, unsigned len)
{
    if (level > g_level) return;
    const unsigned char *d = data;
    char line[3 * 64 + 1];
    unsigned n = len > 64 ? 64 : len;
    for (unsigned i = 0; i < n; i++) snprintf(&line[i * 3], 4, "%02X ", d[i]);
    log_msg(level, "%s (%u bytes): %s%s", prefix, len, line, len > 64 ? "..." : "");
}
