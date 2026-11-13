/*
 * ubx2mav - u-blox GNSS receiver to ArduPilot MAVLink GPS_INPUT bridge
 *
 * Reads UBX from a u-blox module on a serial port, configures it the way
 * ArduPilot's own driver would, and feeds the flight controller with
 * MAVLink GPS_INPUT messages (ArduPilot GPS_TYPE = 14, "MAV").
 * RTCM corrections arriving from the GCS as GPS_RTCM_DATA / GPS_INJECT_DATA
 * are re-assembled and injected into the receiver.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
#define _GNU_SOURCE
#include "bridge.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>
#include <errno.h>
#include <getopt.h>
#include <unistd.h>
#include <poll.h>
#include <time.h>
#include <math.h>

volatile sig_atomic_t g_stop = 0;

static void on_signal(int sig)
{
    (void)sig;
    g_stop = 1;
}

/* ------------------------------------------------------------------ */
/* time                                                               */
/* ------------------------------------------------------------------ */
uint64_t now_ms(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000ULL + (uint64_t)ts.tv_nsec / 1000000ULL;
}

uint64_t now_us_realtime(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);
    return (uint64_t)ts.tv_sec * 1000000ULL + (uint64_t)ts.tv_nsec / 1000ULL;
}

/* ------------------------------------------------------------------ */
/* link management                                                    */
/* ------------------------------------------------------------------ */
static void open_links(bridge_t *b)
{
    uint64_t t = now_ms();
    if (b->gps.fd < 0 && t >= b->gps_retry_ms) {
        if (link_open(&b->gps, b->opt.gps_dev, b->opt.gps_baud) == 0) {
            LOGI("GPS: opened %s", b->opt.gps_dev);
            b->gps_state = GPS_DOWN;
            ubx_parser_init(&b->ubxp);
        } else {
            if (b->gps_retry_ms == 0) LOGE("GPS: cannot open %s: %s (retrying)", b->opt.gps_dev, strerror(errno));
            b->gps_retry_ms = t + 1000;
        }
    }
    if (b->fc.fd < 0 && t >= b->fc_retry_ms) {
        if (link_open(&b->fc, b->opt.fc_spec, b->opt.fc_baud) == 0) {
            LOGI("FC: opened %s", b->opt.fc_spec);
            mav_parser_init(&b->mavp);
            b->st.fc_reconnects++;
        } else {
            if (b->fc_retry_ms == 0) LOGE("FC: cannot open %s: %s (retrying)", b->opt.fc_spec, strerror(errno));
            b->fc_retry_ms = t + 1000;
        }
    }
}

static void gps_link_lost(bridge_t *b, const char *why)
{
    LOGW("GPS: link lost (%s), reopening %s", why, b->opt.gps_dev);
    link_close(&b->gps);
    b->gps_state = GPS_DOWN;
    b->ep.pending = false;
    b->gps_retry_ms = now_ms() + 500;
    fc_statustext(b, MAV_SEVERITY_WARNING, "ubx2mav: GPS link lost");
}

static void fc_link_lost(bridge_t *b, const char *why)
{
    LOGW("FC: link lost (%s), reopening %s", why, b->opt.fc_spec);
    link_close(&b->fc);
    b->fc_retry_ms = now_ms() + 500;
}

bool gps_write(bridge_t *b, const uint8_t *data, size_t len)
{
    if (b->gps.fd < 0) return false;
    ssize_t n = link_write(&b->gps, data, len);
    if (n < 0) {
        gps_link_lost(b, strerror(errno));
        return false;
    }
    return (size_t)n == len;
}

static bool fc_write(bridge_t *b, const uint8_t *data, size_t len)
{
    if (b->fc.fd < 0) return false;
    ssize_t n = link_write(&b->fc, data, len);
    if (n < 0) {
        fc_link_lost(b, strerror(errno));
        return false;
    }
    return (size_t)n == len;
}

/* ------------------------------------------------------------------ */
/* MAVLink output                                                     */
/* ------------------------------------------------------------------ */
static void send_heartbeat(bridge_t *b)
{
    mav_heartbeat_t hb = {
        .custom_mode = 0,
        .type = MAV_TYPE_GPS,
        .autopilot = MAV_AUTOPILOT_INVALID,
        .base_mode = 0,
        .system_status = (b->gps_state == GPS_RUNNING) ? MAV_STATE_ACTIVE : MAV_STATE_STANDBY,
        .mavlink_version = 3,
    };
    uint8_t f[MAV_MAX_FRAME];
    size_t n = mav_pack_heartbeat(&b->chan, &hb, f, sizeof(f));
    if (fc_write(b, f, n)) b->st.hb_tx++;
}

void fc_statustext(bridge_t *b, uint8_t severity, const char *fmt, ...)
{
    char text[128];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(text, sizeof(text), fmt, ap);
    va_end(ap);
    LOGI("STATUSTEXT(%u): %s", severity, text);
    if (!b->opt.statustext) return;
    uint8_t f[MAV_MAX_FRAME];
    size_t n = mav_pack_statustext(&b->chan, severity, text, f, sizeof(f));
    if (fc_write(b, f, n)) b->st.statustext_tx++;
}

/*
 * Convert the collected UBX epoch into GPS_INPUT, mirroring the field
 * handling of ArduPilot's AP_GPS_UBLOX (producer) and AP_GPS_MAV (consumer).
 */
void gps_send_epoch(bridge_t *b)
{
    epoch_t *ep = &b->ep;
    const ubx_nav_pvt_t *p = &ep->pvt;
    ep->pending = false;

    mav_gps_input_t gi;
    memset(&gi, 0, sizeof(gi));
    gi.gps_id = (uint8_t)b->opt.gps_id;
    gi.ignore_flags = 0;

    /* fix type: identical mapping to AP_GPS_UBLOX::_parse_gps MSG_PVT */
    uint8_t fix;
    switch (p->fix_type) {
    case 2:  fix = GPS_FIX_TYPE_2D_FIX; break;
    case 3:
        fix = GPS_FIX_TYPE_3D_FIX;
        if (p->flags & UBX_PVT_FLAGS_DIFFSOLN) fix = GPS_FIX_TYPE_DGPS;
        if ((p->flags & UBX_PVT_FLAGS_CARRSOLN_MASK) == UBX_PVT_FLAGS_CARRSOLN_FLOAT) fix = GPS_FIX_TYPE_RTK_FLOAT;
        if ((p->flags & UBX_PVT_FLAGS_CARRSOLN_MASK) == UBX_PVT_FLAGS_CARRSOLN_FIXED) fix = GPS_FIX_TYPE_RTK_FIXED;
        break;
    case 4:  fix = GPS_FIX_TYPE_3D_FIX; break;   /* GNSS + dead reckoning */
    default: fix = GPS_FIX_TYPE_NO_FIX; break;    /* 0, 1, 5 (time only) */
    }
    if (b->opt.require_fix_ok && fix >= GPS_FIX_TYPE_2D_FIX && !(p->flags & UBX_PVT_FLAGS_GNSSFIXOK))
        fix = GPS_FIX_TYPE_NO_FIX;
    /* never send 0: AP_GPS_MAV casts fix_type straight into AP_GPS_FixType where 0 = NO_GPS */
    gi.fix_type = fix;

    gi.time_week_ms = p->itow;
    gi.time_week = ep->week_valid ? ep->week : 0;

    gi.lat = p->lat;
    gi.lon = p->lon;
    gi.alt = (float)p->h_msl * 0.001f;                 /* AMSL, metres */

    if (ep->dop_valid) {
        gi.hdop = (float)ep->dop.h_dop * 0.01f;
        gi.vdop = (float)ep->dop.v_dop * 0.01f;
    } else {
        /* AP_GPS_UBLOX uses pDOP for both until a NAV-DOP arrives */
        gi.hdop = (float)p->p_dop * 0.01f;
        gi.vdop = (float)p->p_dop * 0.01f;
    }

    gi.vn = (float)p->vel_n * 0.001f;
    gi.ve = (float)p->vel_e * 0.001f;
    gi.vd = (float)p->vel_d * 0.001f;
    gi.speed_accuracy = (float)p->s_acc * 0.001f;
    gi.horiz_accuracy = (float)p->h_acc * 0.001f;
    gi.vert_accuracy  = (float)p->v_acc * 0.001f;
    gi.satellites_visible = p->num_sv;

    /* yaw from moving-baseline RELPOSNED, with ArduPilot's validity mask */
    gi.yaw = 0;
    if (b->opt.yaw_relposned && ep->relposned_valid && ep->relposned.version >= 1 &&
        ep->relposned.itow == p->itow) {
        const uint32_t valid_mask = UBX_RELPOSNED_RELPOSHEADINGVALID | UBX_RELPOSNED_RELPOSVALID |
                                    UBX_RELPOSNED_GNSSFIXOK | UBX_RELPOSNED_ISMOVING | UBX_RELPOSNED_CARRSOLN_FIXED;
        const uint32_t invalid_mask = UBX_RELPOSNED_REFPOSMISS | UBX_RELPOSNED_REFOBSMISS | UBX_RELPOSNED_CARRSOLN_FLOAT;
        uint32_t fl = ep->relposned.flags;
        if ((fl & valid_mask) == valid_mask && (fl & invalid_mask) == 0) {
            double hdg = ep->relposned.rel_pos_heading * 1e-5 + b->opt.yaw_offset_deg;
            hdg = fmod(hdg, 360.0);
            if (hdg < 0) hdg += 360.0;
            long cdeg = lround(hdg * 100.0);
            if (cdeg <= 0 || cdeg >= 36000) cdeg = 36000;   /* 0 means "no yaw" on the wire */
            gi.yaw = (uint16_t)cdeg;
        }
    }

    /* time_usec: UTC derived from GPS time when known, else the Pi's clock.
     * ArduPilot ignores this field; it is informational. */
    if (ep->week_valid && fix >= GPS_FIX_TYPE_2D_FIX) {
        uint64_t gps_s = (uint64_t)ep->week * 604800ULL + p->itow / 1000ULL;
        uint64_t utc_s = gps_s + 315964800ULL - (uint64_t)(int64_t)ep->leap_s;
        gi.time_usec = utc_s * 1000000ULL + (uint64_t)(p->itow % 1000) * 1000ULL;
    } else {
        gi.time_usec = now_us_realtime();
    }

    uint8_t f[MAV_MAX_FRAME];
    size_t n = mav_pack_gps_input(&b->chan, &gi, f, sizeof(f));
    if (fc_write(b, f, n)) b->st.gps_input_tx++;

    LOGD("GPS_INPUT fix=%u sats=%u lat=%d lon=%d alt=%.1f hdop=%.2f vel=(%.2f,%.2f,%.2f) hacc=%.2f week=%u tow=%u yaw=%u",
         gi.fix_type, gi.satellites_visible, gi.lat, gi.lon, (double)gi.alt, (double)gi.hdop,
         (double)gi.vn, (double)gi.ve, (double)gi.vd, (double)gi.horiz_accuracy, gi.time_week, gi.time_week_ms, gi.yaw);
}

/* ------------------------------------------------------------------ */
/* RTCM injection (re-assembly identical to AP_GPS::handle_gps_rtcm_fragment) */
/* ------------------------------------------------------------------ */
static void inject_rtcm(bridge_t *b, const uint8_t *data, uint16_t len)
{
    if (len == 0) return;
    if (b->gps_state != GPS_RUNNING) {
        LOGD("RTCM: %u bytes dropped, GPS not ready", len);
        return;
    }
    if (gps_write(b, data, len)) {
        b->st.rtcm_msgs++;
        b->st.rtcm_bytes += len;
        LOGT("RTCM: injected %u bytes", len);
    }
}

static void handle_rtcm_fragment(bridge_t *b, uint8_t flags, const uint8_t *data, uint8_t len)
{
    rtcm_buf_t *r = &b->rtcm;
    if ((flags & 1) == 0) {
        inject_rtcm(b, data, len);
        return;
    }
    const uint8_t fragment = (flags >> 1) & 0x03;
    const uint8_t sequence = (flags >> 3) & 0x1F;
    uint8_t *start = &r->buffer[MAVLINK_MSG_GPS_RTCM_DATA_FIELD_DATA_LEN * (uint16_t)fragment];
    bool clear = false;

    if (r->fragments_received) {
        bool seq_changed = r->sequence != sequence;
        bool seen = r->fragments_received & (1u << fragment);
        if (!seq_changed && seen && memcmp(start, data, len) == 0) return;   /* duplicate */
        clear = seq_changed || seen;
    }
    if (clear) {
        r->fragment_count = 0;
        b->st.rtcm_frag_discarded += (uint32_t)__builtin_popcount(r->fragments_received);
        r->fragments_received = 0;
    }
    r->sequence = sequence;
    r->fragments_received |= (uint8_t)(1u << fragment);
    memcpy(start, data, len);

    if (len < MAVLINK_MSG_GPS_RTCM_DATA_FIELD_DATA_LEN) {
        r->fragment_count = (uint8_t)(fragment + 1);
        r->total_length = (uint16_t)(MAVLINK_MSG_GPS_RTCM_DATA_FIELD_DATA_LEN * fragment + len);
    } else if (r->fragments_received == 0x0F) {
        r->fragment_count = 4;
        r->total_length = MAVLINK_MSG_GPS_RTCM_DATA_FIELD_DATA_LEN * 4;
    }
    if (r->fragment_count != 0 && r->fragments_received == (1u << r->fragment_count) - 1) {
        inject_rtcm(b, r->buffer, r->total_length);
        r->fragment_count = 0;
        r->fragments_received = 0;
    }
}

/* ------------------------------------------------------------------ */
/* MAVLink input                                                      */
/* ------------------------------------------------------------------ */
static void fc_handle_msg(bridge_t *b)
{
    const mav_message_t *m = &b->mavp.msg;
    b->st.fc_msgs_rx++;

    switch (m->msgid) {
    case MAVLINK_MSG_ID_HEARTBEAT: {
        mav_heartbeat_t hb;
        mav_decode_heartbeat(m, &hb);
        /* only autopilots count as "the flight controller" */
        if (hb.type == MAV_TYPE_GCS || hb.autopilot == MAV_AUTOPILOT_INVALID) break;
        if (m->compid != 1) break;
        b->st.fc_hb_rx++;
        b->last_fc_hb_ms = now_ms();
        if (!b->fc_seen) {
            b->fc_seen = true;
            b->fc_sysid = m->sysid;
            b->fc_compid = m->compid;
            LOGI("FC: heartbeat from sysid %u compid %u (autopilot %u, type %u)%s",
                 m->sysid, m->compid, hb.autopilot, hb.type,
                 m->magic == MAV_STX_V2 ? "" : " [MAVLink 1 - set SERIALn_PROTOCOL=2]");
            if (b->opt.sysid < 0 && b->chan.sysid != m->sysid) {
                LOGI("FC: adopting system id %u so we appear as a component of the vehicle", m->sysid);
                b->chan.sysid = m->sysid;
            }
        } else if (m->sysid != b->fc_sysid) {
            LOGD("FC: heartbeat from another autopilot sysid %u ignored", m->sysid);
        }
        break;
    }
    case MAVLINK_MSG_ID_GPS_RTCM_DATA: {
        if (!b->opt.rtcm_enable) break;
        mav_gps_rtcm_data_t d;
        mav_decode_gps_rtcm_data(m, &d);
        LOGT("GPS_RTCM_DATA flags=0x%02X len=%u from %u/%u", d.flags, d.len, m->sysid, m->compid);
        handle_rtcm_fragment(b, d.flags, d.data, d.len);
        break;
    }
    case MAVLINK_MSG_ID_GPS_INJECT_DATA: {
        if (!b->opt.rtcm_enable) break;
        mav_gps_inject_data_t d;
        mav_decode_gps_inject_data(m, &d);
        handle_rtcm_fragment(b, 0, d.data, d.len);
        break;
    }
    default:
        break;
    }
}

/* ------------------------------------------------------------------ */
/* I/O pump and periodic work                                         */
/* ------------------------------------------------------------------ */
void pump(bridge_t *b, int timeout_ms)
{
    struct pollfd pfd[2];
    int n = 0;
    int gps_idx = -1, fc_idx = -1;
    if (b->gps.fd >= 0) { pfd[n].fd = b->gps.fd; pfd[n].events = POLLIN; gps_idx = n++; }
    if (b->fc.fd >= 0)  { pfd[n].fd = b->fc.fd;  pfd[n].events = POLLIN; fc_idx = n++; }

    if (n == 0) {
        usleep((useconds_t)timeout_ms * 1000);
        return;
    }
    int r = poll(pfd, (nfds_t)n, timeout_ms);
    if (r < 0) {
        if (errno != EINTR) LOGE("poll: %s", strerror(errno));
        return;
    }
    uint8_t buf[2048];
    if (gps_idx >= 0 && (pfd[gps_idx].revents & (POLLIN | POLLERR | POLLHUP))) {
        ssize_t got = link_read(&b->gps, buf, sizeof(buf));
        if (got < 0) {
            gps_link_lost(b, "read error");
        } else {
            for (ssize_t i = 0; i < got; i++) {
                ubx_parse_result_t pr = ubx_parse_byte(&b->ubxp, buf[i]);
                if (pr == UBX_PARSE_UBX) gps_handle_frame(b);
                else if (pr == UBX_PARSE_NMEA) b->det_nmea_seen = true;
                if (b->gps.fd < 0) break;
            }
        }
    }
    if (fc_idx >= 0 && b->fc.fd >= 0 && (pfd[fc_idx].revents & (POLLIN | POLLERR | POLLHUP))) {
        ssize_t got = link_read(&b->fc, buf, sizeof(buf));
        if (got < 0) {
            fc_link_lost(b, "read error");
        } else {
            for (ssize_t i = 0; i < got; i++) {
                if (mav_parse_byte(&b->mavp, buf[i])) fc_handle_msg(b);
            }
        }
    }
}

static void log_stats(bridge_t *b)
{
    uint64_t t = now_ms();
    double dt = (double)(t - b->last_stats_ms) / 1000.0;
    if (dt <= 0) dt = 1;
    double epoch_rate = (b->st.epochs - b->last_stats_epochs) / dt;
    double tx_rate = (b->st.gps_input_tx - b->last_stats_gps_input) / dt;
    b->last_stats_ms = t;
    b->last_stats_epochs = b->st.epochs;
    b->last_stats_gps_input = b->st.gps_input_tx;

    const ubx_nav_pvt_t *p = &b->ep.pvt;
    LOGI("stats: GPS %s fix=%u sats=%u hdop=%.2f hacc=%.2fm %.1fHz | GPS_INPUT %.1fHz total=%u | "
         "FC hb=%u%s | RTCM in=%u msgs/%u B used=%u crcfail=%u | ubx crc=%u mav crc=%u eoe_to=%u nak=%u",
         b->gps_state == GPS_RUNNING ? "ok" : "DOWN", p->fix_type, p->num_sv,
         b->ep.dop_valid ? b->ep.dop.h_dop * 0.01 : p->p_dop * 0.01, p->h_acc * 0.001, epoch_rate,
         tx_rate, b->st.gps_input_tx,
         b->st.fc_hb_rx, (b->fc_seen && t - b->last_fc_hb_ms > 5000) ? " (STALE)" : "",
         b->st.rtcm_msgs, b->st.rtcm_bytes, b->st.rtcm_gps_used, b->st.rtcm_gps_crc_fail,
         b->ubxp.rx_crc_err, b->mavp.rx_crc_err, b->st.eoe_timeouts, b->st.cfg_nak);
}

static void periodic(bridge_t *b)
{
    uint64_t t = now_ms();

    if (t - b->last_hb_tx_ms >= 1000) {
        b->last_hb_tx_ms = t;
        send_heartbeat(b);
    }

    gps_epoch_tick(b);

    if (b->gps_state == GPS_RUNNING && (int)(t - b->last_pvt_ms) > b->opt.gps_timeout_ms) {
        LOGW("GPS: no NAV-PVT for %d ms, re-detecting receiver", b->opt.gps_timeout_ms);
        fc_statustext(b, MAV_SEVERITY_WARNING, "ubx2mav: GPS data timeout");
        b->gps_state = GPS_DOWN;
        b->ep.pending = false;
    }

    if (b->opt.stats_interval_s > 0 && t - b->last_stats_ms >= (uint64_t)b->opt.stats_interval_s * 1000ULL) {
        log_stats(b);
    }
}

/* ------------------------------------------------------------------ */
/* options                                                            */
/* ------------------------------------------------------------------ */
static void usage(const char *prog)
{
    printf(
"ubx2mav %s - u-blox GPS to ArduPilot MAVLink GPS_INPUT bridge\n"
"\n"
"Usage: %s [options]\n"
"\n"
"GPS side:\n"
"  -g, --gps-dev DEV        serial device of the u-blox receiver (default /dev/ttyAMA0)\n"
"  -b, --gps-baud N         target baud rate (default 230400, like ArduPilot)\n"
"      --no-autobaud        only try --gps-baud, do not scan other rates\n"
"      --keep-baud          do not change the baud rate of the module\n"
"      --gps-port P         module port we are wired to: uart1|uart2|usb|i2c|spi (default auto)\n"
"      --rate-ms N          measurement period in ms (ArduPilot GPS_RATE_MS, default 200)\n"
"      --dynmodel N         dynamic model (GPS_NAVFILTER, default 8 = airborne <4g; -1 = leave)\n"
"      --min-elev N         minimum satellite elevation in deg (GPS_MIN_ELEV, default -100 = leave)\n"
"      --gnss-mode MASK     constellation bitmask (GPS_GNSS_MODE: 1 GPS 2 SBAS 4 Galileo 8 BeiDou\n"
"                           32 QZSS 64 GLONASS 128 NavIC; default 0 = leave as configured)\n"
"      --sbas-mode N        0 disable, 1 enable, 2 leave as is (GPS_SBAS_MODE, default 2)\n"
"      --save-config        store the configuration in the module (GPS_SAVE_CFG)\n"
"      --no-config          do not configure the module at all, only parse what it sends\n"
"      --yaw-relposned      forward moving-baseline heading from NAV-RELPOSNED as GPS yaw\n"
"      --yaw-offset DEG     add this offset to the RELPOSNED heading (antenna orientation)\n"
"      --require-fix-ok     treat NAV-PVT without gnssFixOK as no fix (ArduPilot does not)\n"
"      --send-on pvt|eoe    when to emit GPS_INPUT (default eoe: after NAV-EOE, falls back to pvt)\n"
"      --eoe-timeout MS     send anyway this long after NAV-PVT (default 40)\n"
"      --gps-timeout MS     re-detect the receiver after this long without NAV-PVT (default 3000)\n"
"\n"
"Flight controller side:\n"
"  -f, --fc DEV|udp://H:P   MAVLink link: serial device, udp://host:port or udpin://0.0.0.0:port\n"
"                           (default /dev/ttyACM0)\n"
"  -B, --fc-baud N          serial baud rate towards the flight controller (default 115200)\n"
"      --sysid N            MAVLink system id (default: adopt the autopilot's, 1 until seen)\n"
"      --compid N           MAVLink component id (default 220 = MAV_COMP_ID_GPS)\n"
"      --gps-id N           GPS_INPUT.gps_id = ArduPilot GPS instance (0 for GPS1, 1 for GPS2)\n"
"      --no-rtcm            do not inject GPS_RTCM_DATA / GPS_INJECT_DATA into the receiver\n"
"      --no-statustext      do not send STATUSTEXT messages to the GCS\n"
"\n"
"General:\n"
"      --stats N            log a statistics line every N seconds (default 30, 0 = off)\n"
"  -v, --verbose            more logging (repeat for trace)\n"
"  -q, --quiet              warnings and errors only\n"
"  -V, --version            print version and exit\n"
"  -h, --help               this text\n"
"\n"
"Flight controller parameters: GPS_TYPE=14 (GPS1_TYPE on 4.6+), SERIALn_PROTOCOL=2 on the\n"
"port connected to the Pi, optionally GPS_DELAY_MS=120 (u-blox M8/F9 lag used by ArduPilot).\n",
    UBX2MAV_VERSION, prog);
}

static int parse_port(const char *s)
{
    if (!strcasecmp(s, "uart1")) return UBX_PORT_UART1;
    if (!strcasecmp(s, "uart2")) return UBX_PORT_UART2;
    if (!strcasecmp(s, "usb"))   return UBX_PORT_USB;
    if (!strcasecmp(s, "i2c") || !strcasecmp(s, "ddc")) return UBX_PORT_I2C;
    if (!strcasecmp(s, "spi"))   return UBX_PORT_SPI;
    if (!strcasecmp(s, "auto"))  return -1;
    return -2;
}

static void defaults(options_t *o)
{
    memset(o, 0, sizeof(*o));
    o->gps_dev = "/dev/ttyAMA0";
    o->gps_baud = 230400;
    o->gps_autobaud = true;
    o->gps_set_baud = true;
    o->gps_port = -1;
    o->fc_spec = "/dev/ttyACM0";
    o->fc_baud = 115200;
    o->sysid = -1;
    o->compid = MAV_COMP_ID_GPS;
    o->gps_id = 0;
    o->rate_ms = 200;
    o->dynmodel = 8;
    o->min_elev = -100;
    o->gnss_mode = 0;
    o->sbas_mode = 2;
    o->save_config = false;
    o->no_config = false;
    o->yaw_relposned = false;
    o->yaw_offset_deg = 0;
    o->require_fix_ok = false;
    o->send_mode = SEND_EOE;
    o->eoe_timeout_ms = 40;
    o->rtcm_enable = true;
    o->statustext = true;
    o->stats_interval_s = 30;
    o->gps_timeout_ms = 3000;
}

enum {
    OPT_NO_AUTOBAUD = 1000, OPT_KEEP_BAUD, OPT_GPS_PORT, OPT_RATE_MS, OPT_DYNMODEL, OPT_MIN_ELEV,
    OPT_GNSS_MODE, OPT_SBAS_MODE, OPT_SAVE_CONFIG, OPT_NO_CONFIG, OPT_YAW_RELPOSNED, OPT_YAW_OFFSET,
    OPT_REQUIRE_FIX_OK, OPT_SEND_ON, OPT_EOE_TIMEOUT, OPT_GPS_TIMEOUT, OPT_SYSID, OPT_COMPID,
    OPT_GPS_ID, OPT_NO_RTCM, OPT_NO_STATUSTEXT, OPT_STATS
};

static const struct option long_opts[] = {
    { "gps-dev",        required_argument, 0, 'g' },
    { "gps-baud",       required_argument, 0, 'b' },
    { "no-autobaud",    no_argument,       0, OPT_NO_AUTOBAUD },
    { "keep-baud",      no_argument,       0, OPT_KEEP_BAUD },
    { "gps-port",       required_argument, 0, OPT_GPS_PORT },
    { "rate-ms",        required_argument, 0, OPT_RATE_MS },
    { "dynmodel",       required_argument, 0, OPT_DYNMODEL },
    { "min-elev",       required_argument, 0, OPT_MIN_ELEV },
    { "gnss-mode",      required_argument, 0, OPT_GNSS_MODE },
    { "sbas-mode",      required_argument, 0, OPT_SBAS_MODE },
    { "save-config",    no_argument,       0, OPT_SAVE_CONFIG },
    { "no-config",      no_argument,       0, OPT_NO_CONFIG },
    { "yaw-relposned",  no_argument,       0, OPT_YAW_RELPOSNED },
    { "yaw-offset",     required_argument, 0, OPT_YAW_OFFSET },
    { "require-fix-ok", no_argument,       0, OPT_REQUIRE_FIX_OK },
    { "send-on",        required_argument, 0, OPT_SEND_ON },
    { "eoe-timeout",    required_argument, 0, OPT_EOE_TIMEOUT },
    { "gps-timeout",    required_argument, 0, OPT_GPS_TIMEOUT },
    { "fc",             required_argument, 0, 'f' },
    { "fc-baud",        required_argument, 0, 'B' },
    { "sysid",          required_argument, 0, OPT_SYSID },
    { "compid",         required_argument, 0, OPT_COMPID },
    { "gps-id",         required_argument, 0, OPT_GPS_ID },
    { "no-rtcm",        no_argument,       0, OPT_NO_RTCM },
    { "no-statustext",  no_argument,       0, OPT_NO_STATUSTEXT },
    { "stats",          required_argument, 0, OPT_STATS },
    { "verbose",        no_argument,       0, 'v' },
    { "quiet",          no_argument,       0, 'q' },
    { "version",        no_argument,       0, 'V' },
    { "help",           no_argument,       0, 'h' },
    { 0, 0, 0, 0 }
};

static int parse_args(int argc, char **argv, options_t *o)
{
    int c, level = LOG_INFO;
    while ((c = getopt_long(argc, argv, "g:b:f:B:vqVh", long_opts, NULL)) != -1) {
        switch (c) {
        case 'g': o->gps_dev = optarg; break;
        case 'b': o->gps_baud = (uint32_t)strtoul(optarg, NULL, 10); break;
        case OPT_NO_AUTOBAUD: o->gps_autobaud = false; break;
        case OPT_KEEP_BAUD: o->gps_set_baud = false; break;
        case OPT_GPS_PORT:
            o->gps_port = parse_port(optarg);
            if (o->gps_port == -2) { fprintf(stderr, "bad --gps-port %s\n", optarg); return -1; }
            break;
        case OPT_RATE_MS: o->rate_ms = atoi(optarg); break;
        case OPT_DYNMODEL: o->dynmodel = atoi(optarg); break;
        case OPT_MIN_ELEV: o->min_elev = atoi(optarg); break;
        case OPT_GNSS_MODE: o->gnss_mode = atoi(optarg); break;
        case OPT_SBAS_MODE: o->sbas_mode = atoi(optarg); break;
        case OPT_SAVE_CONFIG: o->save_config = true; break;
        case OPT_NO_CONFIG: o->no_config = true; break;
        case OPT_YAW_RELPOSNED: o->yaw_relposned = true; break;
        case OPT_YAW_OFFSET: o->yaw_offset_deg = atof(optarg); break;
        case OPT_REQUIRE_FIX_OK: o->require_fix_ok = true; break;
        case OPT_SEND_ON:
            if (!strcasecmp(optarg, "pvt")) o->send_mode = SEND_PVT;
            else if (!strcasecmp(optarg, "eoe")) o->send_mode = SEND_EOE;
            else { fprintf(stderr, "bad --send-on %s\n", optarg); return -1; }
            break;
        case OPT_EOE_TIMEOUT: o->eoe_timeout_ms = atoi(optarg); break;
        case OPT_GPS_TIMEOUT: o->gps_timeout_ms = atoi(optarg); break;
        case 'f': o->fc_spec = optarg; break;
        case 'B': o->fc_baud = (uint32_t)strtoul(optarg, NULL, 10); break;
        case OPT_SYSID: o->sysid = atoi(optarg); break;
        case OPT_COMPID: o->compid = atoi(optarg); break;
        case OPT_GPS_ID: o->gps_id = atoi(optarg); break;
        case OPT_NO_RTCM: o->rtcm_enable = false; break;
        case OPT_NO_STATUSTEXT: o->statustext = false; break;
        case OPT_STATS: o->stats_interval_s = atoi(optarg); break;
        case 'v': level++; break;
        case 'q': level = LOG_WARN; break;
        case 'V': printf("ubx2mav %s\n", UBX2MAV_VERSION); exit(0);
        case 'h': usage(argv[0]); exit(0);
        default: usage(argv[0]); return -1;
        }
    }
    log_set_level(level);

    if (o->rate_ms < 50 || o->rate_ms > 1000) { fprintf(stderr, "--rate-ms must be 50..1000\n"); return -1; }
    if (o->sbas_mode < 0 || o->sbas_mode > 2) { fprintf(stderr, "--sbas-mode must be 0, 1 or 2\n"); return -1; }
    if (o->gps_id < 0 || o->gps_id > 1) { fprintf(stderr, "--gps-id must be 0 or 1\n"); return -1; }
    if (o->eoe_timeout_ms < 5) o->eoe_timeout_ms = 5;
    if (o->eoe_timeout_ms > o->rate_ms / 2) o->eoe_timeout_ms = o->rate_ms / 2;
    if (o->compid < 1 || o->compid > 255) { fprintf(stderr, "--compid must be 1..255\n"); return -1; }
    if (o->sysid > 255) { fprintf(stderr, "--sysid must be 1..255\n"); return -1; }
    return 0;
}

/* ------------------------------------------------------------------ */
/* main                                                               */
/* ------------------------------------------------------------------ */
int main(int argc, char **argv)
{
    static bridge_t b;
    memset(&b, 0, sizeof(b));
    defaults(&b.opt);
    if (parse_args(argc, argv, &b.opt) != 0) return 2;

    b.gps.fd = -1;
    b.fc.fd = -1;
    b.chan.sysid = (uint8_t)(b.opt.sysid > 0 ? b.opt.sysid : 1);
    b.chan.compid = (uint8_t)b.opt.compid;
    b.gps_state = GPS_DOWN;
    b.ubx_port = UBX_PORT_UNKNOWN;
    ubx_parser_init(&b.ubxp);
    mav_parser_init(&b.mavp);
    b.last_stats_ms = now_ms();

    signal(SIGINT, on_signal);
    signal(SIGTERM, on_signal);
    signal(SIGPIPE, SIG_IGN);

    LOGI("ubx2mav %s starting: GPS %s (target %lu baud) -> FC %s, sysid %s compid %d gps_id %d",
         UBX2MAV_VERSION, b.opt.gps_dev, (unsigned long)b.opt.gps_baud, b.opt.fc_spec,
         b.opt.sysid > 0 ? "fixed" : "auto", b.opt.compid, b.opt.gps_id);

    while (!g_stop) {
        open_links(&b);

        if (b.gps.fd >= 0 && b.gps_state != GPS_RUNNING) {
            gps_setup(&b);
            continue;
        }
        pump(&b, 20);
        periodic(&b);
    }

    LOGI("ubx2mav stopping: %u GPS_INPUT sent, %u RTCM messages injected", b.st.gps_input_tx, b.st.rtcm_msgs);
    link_close(&b.gps);
    link_close(&b.fc);
    return 0;
}
