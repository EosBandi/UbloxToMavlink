/* log.h - tiny leveled logger to stderr (journald-friendly)
 * SPDX-License-Identifier: GPL-3.0-or-later */
#ifndef LOG_H
#define LOG_H
#include <stdarg.h>

enum log_level { LOG_ERROR = 0, LOG_WARN, LOG_INFO, LOG_DEBUG, LOG_TRACE };

void log_set_level(int level);
int  log_get_level(void);
void log_msg(int level, const char *fmt, ...) __attribute__((format(printf, 2, 3)));
void log_hexdump(int level, const char *prefix, const void *data, unsigned len);

#define LOGE(...) log_msg(LOG_ERROR, __VA_ARGS__)
#define LOGW(...) log_msg(LOG_WARN,  __VA_ARGS__)
#define LOGI(...) log_msg(LOG_INFO,  __VA_ARGS__)
#define LOGD(...) log_msg(LOG_DEBUG, __VA_ARGS__)
#define LOGT(...) log_msg(LOG_TRACE, __VA_ARGS__)

#endif
