/*
 * gps.c - u-blox detection, configuration and NAV message handling.
 *
 * The configuration performed here mirrors what ArduPilot's AP_GPS_UBLOX
 * driver does with GPS_AUTO_CONFIG=1:
 *   - find the module at any common baud rate and move it to 230400
 *   - UBX + RTCM3 input, UBX-only output on our port (NMEA off)
 *   - CFG-RATE 200 ms / nav 1 / UTC time reference    (GPS_RATE_MS)
 *   - CFG-NAV5 dynamic model 8 (airborne <4g)            (GPS_NAVFILTER)
 *   - optional minimum elevation, GNSS constellation mask and SBAS mode
 *   - NAV-PVT, NAV-DOP, NAV-TIMEGPS enabled; POSLLH/STATUS/SOL/VELNED off
 *   - NAV-EOE enabled so a complete epoch can be forwarded with minimum delay
 *   - VALSET keys on F9/M9/M10, legacy CFG-* messages on M8 and older
 *   - optional save to non-volatile memory                (GPS_SAVE_CFG)
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
#define _GNU_SOURCE
#include "bridge.h"

#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <math.h>

/* ------------------------------------------------------------------ */
/* helpers                                                            */
/* ------------------------------------------------------------------ */
const char *gps_generation_name(int gen)
{
    switch (gen) {
    case GEN_UBLOX6: return "u-blox 6";
    case GEN_UBLOX7: return "u-blox 7";
    case GEN_M8:     return "u-blox M8";
    case GEN_F9:     return "u-blox F9";
    case GEN_M9:     return "u-blox M9";
    case GEN_M10:    return "u-blox M10";
    default:         return "u-blox (unknown generation)";
    }
}

static const char *port_name(uint8_t port)
{
    switch (port) {
    case UBX_PORT_I2C:   return "I2C";
    case UBX_PORT_UART1: return "UART1";
    case UBX_PORT_UART2: return "UART2";
    case UBX_PORT_USB:   return "USB";
    case UBX_PORT_SPI:   return "SPI";
    default:             return "unknown";
    }
}

static bool send_frame(bridge_t *b, const uint8_t *frame, size_t len, const char *what)
{
    if (len == 0) {
        LOGE("failed to build %s", what);
        return false;
    }
    log_hexdump(LOG_TRACE, what, frame, (unsigned)len);
    return gps_write(b, frame, len);
}

/*
 * Pump I/O until a UBX frame (cls,id) arrives or timeout expires.
 * The matched payload is copied into b->wait_payload/b->wait_len.
 */
static bool wait_frame(bridge_t *b, uint8_t cls, uint8_t id, int timeout_ms)
{
    b->wait_active = true;
    b->wait_cls = cls;
    b->wait_id = id;
    b->wait_matched = false;
    uint64_t deadline = now_ms() + (uint64_t)timeout_ms;
    while (!g_stop && !b->wait_matched && now_ms() < deadline) {
        uint64_t left = deadline - now_ms();
        pump(b, left > 20 ? 20 : (int)left);
        if (b->gps.fd < 0) break;
    }
    b->wait_active = false;
    return b->wait_matched;
}

/* Wait for ACK/NAK for a CFG message. Returns 1 ACK, -1 NAK, 0 timeout. */
static int wait_ack(bridge_t *b, uint8_t cls, uint8_t id, int timeout_ms)
{
    b->ack_active = true;
    b->ack_cls = cls;
    b->ack_id = id;
    b->ack_result = 0;
    uint64_t deadline = now_ms() + (uint64_t)timeout_ms;
    while (!g_stop && b->ack_result == 0 && now_ms() < deadline) {
        uint64_t left = deadline - now_ms();
        pump(b, left > 20 ? 20 : (int)left);
        if (b->gps.fd < 0) break;
    }
    b->ack_active = false;
    return b->ack_result;
}

/* Send a CFG message and wait for its ACK, retrying on timeout. */
static int send_cfg(bridge_t *b, const uint8_t *frame, size_t len, const char *what)
{
    for (int attempt = 0; attempt < 3 && !g_stop; attempt++) {
        if (!send_frame(b, frame, len, what)) return 0;
        int r = wait_ack(b, frame[2], frame[3], 600);
        if (r == 1) {
            LOGD("%s: ACK", what);
            return 1;
        }
        if (r == -1) {
            LOGD("%s: NAK", what);
            b->st.cfg_nak++;
            return -1;
        }
        LOGD("%s: no ACK (attempt %d)", what, attempt + 1);
    }
    return 0;
}

/* Poll a CFG message and wait for the reply. */
static bool poll_cfg(bridge_t *b, uint8_t cls, uint8_t id, const char *what)
{
    uint8_t f[16];
    size_t n = ubx_build_poll(f, sizeof(f), cls, id);
    for (int attempt = 0; attempt < 3 && !g_stop; attempt++) {
        b->ack_result = 0;
        if (!send_frame(b, f, n, what)) return false;
        if (wait_frame(b, cls, id, 600)) return true;
        /* a NAK means the module does not support this message (e.g. M10 + CFG-PRT) */
        if (!b->ack_active && b->ack_result == -1 && b->ack_cls == cls && b->ack_id == id) {
            LOGD("%s: NAK (unsupported)", what);
            return false;
        }
    }
    return false;
}

/* ------------------------------------------------------------------ */
/* MON-VER parsing                                                    */
/* ------------------------------------------------------------------ */
static void parse_version(bridge_t *b)
{
    const ubx_mon_ver_t *v = &b->ver;
    b->module[0] = 0;
    b->protver[0] = 0;
    b->protver_num = 0;
    for (int i = 0; i < v->num_ext; i++) {
        const char *e = v->extension[i];
        if (strncmp(e, "PROTVER=", 8) == 0) snprintf(b->protver, sizeof(b->protver), "%s", e + 8);
        else if (strncmp(e, "PROTVER ", 8) == 0) snprintf(b->protver, sizeof(b->protver), "%s", e + 8);
        else if (strncmp(e, "MOD=", 4) == 0) snprintf(b->module, sizeof(b->module), "%s", e + 4);
    }
    if (b->protver[0]) b->protver_num = atof(b->protver);

    b->generation = GEN_UNKNOWN;
    if (strncmp(v->hw_version, "00040007", 8) == 0) b->generation = GEN_UBLOX6;
    else if (strncmp(v->hw_version, "00070000", 8) == 0) b->generation = GEN_UBLOX7;
    else if (strncmp(v->hw_version, "00080000", 8) == 0) b->generation = GEN_M8;
    else if (strncmp(v->hw_version, "00190000", 8) == 0) {
        if (strncmp(v->sw_version, "EXT CORE 1", 10) == 0) b->generation = GEN_F9;
        else if (strncmp(v->sw_version, "EXT CORE 4", 10) == 0) b->generation = GEN_M9;
        else b->generation = GEN_F9;
    } else if (strncmp(v->hw_version, "000A0000", 8) == 0) b->generation = GEN_M10;

    b->is_zed = (strncmp(b->module, "ZED-", 4) == 0);

    /* The configuration interface (VALSET/VALGET) exists from protocol 23.01
     * (F9). M10 (protocol 34) only supports VALSET. M9 supports both, and
     * ArduPilot configures it with the legacy messages, so we do the same. */
    b->valset = false;
    if (b->generation == GEN_M10 || b->generation == GEN_F9) b->valset = true;
    else if (b->generation == GEN_UNKNOWN && b->protver_num >= 23.01) b->valset = true;

    LOGI("GPS: %s%s%s HW %s SW %s PROTVER %s -> %s configuration",
         gps_generation_name(b->generation),
         b->module[0] ? " " : "", b->module,
         v->hw_version, v->sw_version, b->protver[0] ? b->protver : "?",
         b->valset ? "VALSET" : "legacy CFG");
}

/* ------------------------------------------------------------------ */
/* baud rate detection                                                */
/* ------------------------------------------------------------------ */
static bool probe_baud(bridge_t *b, uint32_t baud, int listen_ms)
{
    if (serial_set_baud(b->gps.fd, baud) != 0) {
        LOGW("GPS: cannot set %lu baud on %s", (unsigned long)baud, b->opt.gps_dev);
        return false;
    }
    usleep(20000);
    serial_flush(b->gps.fd);
    ubx_parser_init(&b->ubxp);
    b->det_ubx_seen = false;
    b->det_nmea_seen = false;

    uint8_t f[16];
    size_t n = ubx_build_poll(f, sizeof(f), UBX_CLASS_MON, UBX_MON_VER);
    send_frame(b, f, n, "MON-VER poll");

    uint64_t deadline = now_ms() + (uint64_t)listen_ms;
    while (!g_stop && now_ms() < deadline && b->gps.fd >= 0) {
        pump(b, 20);
        if (b->det_ubx_seen) return true;
    }
    if (b->det_nmea_seen) return true;
    return false;
}

static bool is_usb_device(const bridge_t *b)
{
    return strstr(b->opt.gps_dev, "ttyACM") != NULL;
}

/* Switch the module from `from` baud to the target baud. */
static bool switch_baud(bridge_t *b, uint32_t from)
{
    uint32_t target = b->opt.gps_baud;
    uint8_t f[64];
    size_t n;

    LOGI("GPS: found at %lu baud, switching module to %lu", (unsigned long)from, (unsigned long)target);

    /* 1. Legacy CFG-PRT with the full current port configuration (works on
     *    u-blox 6/7/M8/M9/F9). Poll first so we keep the existing mode bits. */
    if (poll_cfg(b, UBX_CLASS_CFG, UBX_CFG_PRT, "CFG-PRT poll")) {
        if (ubx_decode_cfg_prt(b->wait_payload, b->wait_len, &b->prt)) {
            b->have_prt = true;
            b->ubx_port = b->prt.port_id;
            if (b->prt.port_id == UBX_PORT_UART1 || b->prt.port_id == UBX_PORT_UART2) {
                ubx_cfg_prt_t p = b->prt;
                p.baudrate = target;
                p.in_proto_mask |= UBX_PROTO_UBX | UBX_PROTO_RTCM3;
                p.out_proto_mask = UBX_PROTO_UBX;
                n = ubx_build_cfg_prt(f, sizeof(f), &p);
                send_frame(b, f, n, "CFG-PRT set baud");
            }
        }
    }

    /* 2. $PUBX,41 (what ArduPilot's init blob uses) - works whenever NMEA
     *    input is enabled, including M10. portId 1 = UART1, 2 = UART2 */
    uint8_t pubx_port = (b->have_prt && b->prt.port_id == UBX_PORT_UART2) ? 2 : 1;
    char s[64];
    n = ubx_build_pubx41(s, sizeof(s), pubx_port,
                         UBX_PROTO_UBX | UBX_PROTO_NMEA | UBX_PROTO_RTCM3, UBX_PROTO_UBX, target);
    if (n) gps_write(b, (const uint8_t *)s, n);

    /* 3. VALSET CFG-UARTx-BAUDRATE (F9/M9/M10) */
    uint32_t key = (pubx_port == 2) ? UBX_KEY_CFG_UART2_BAUDRATE : UBX_KEY_CFG_UART1_BAUDRATE;
    n = ubx_build_valset(f, sizeof(f), UBX_LAYER_RAM, key, target);
    send_frame(b, f, n, "VALSET baud");

    serial_drain(b->gps.fd);
    usleep(150000);

    if (probe_baud(b, target, 1200) && b->det_ubx_seen) {
        LOGI("GPS: module now talking UBX at %lu baud", (unsigned long)target);
        return true;
    }
    LOGW("GPS: no UBX at %lu after switching baud rate", (unsigned long)target);
    return false;
}

/* Returns true when a module answering UBX has been found at b->gps_actual_baud */
static bool detect(bridge_t *b)
{
    static const uint32_t common[] = { 38400, 9600, 115200, 57600, 460800, 19200, 921600, 4800 };
    uint32_t list[12];
    int count = 0;

    list[count++] = b->opt.gps_baud;
    if (b->opt.gps_autobaud && !is_usb_device(b)) {
        for (size_t i = 0; i < sizeof(common) / sizeof(common[0]); i++)
            if (common[i] != b->opt.gps_baud) list[count++] = common[i];
    }

    for (int i = 0; i < count && !g_stop && b->gps.fd >= 0; i++) {
        LOGD("GPS: probing %lu baud", (unsigned long)list[i]);
        if (!probe_baud(b, list[i], 900)) continue;

        b->gps_actual_baud = list[i];
        if (b->det_ubx_seen) LOGI("GPS: UBX detected at %lu baud", (unsigned long)list[i]);
        else LOGI("GPS: NMEA detected at %lu baud", (unsigned long)list[i]);

        if (is_usb_device(b)) {
            /* USB CDC ignores the baud rate; nothing to switch */
            if (!b->det_ubx_seen) {
                /* NMEA-only USB output: UBX input still works, confirm with a second poll */
                probe_baud(b, list[i], 900);
            }
            return true;
        }
        if (list[i] != b->opt.gps_baud && b->opt.gps_set_baud) {
            if (switch_baud(b, list[i])) {
                b->gps_actual_baud = b->opt.gps_baud;
                return true;
            }
            /* module may have switched anyway: restart the whole scan */
            return false;
        }
        if (!b->det_ubx_seen) {
            /* NMEA only at the right baud: make sure UBX input works */
            if (!probe_baud(b, list[i], 900) || !b->det_ubx_seen) {
                LOGW("GPS: NMEA seen but no UBX reply; is UBX input enabled on this port?");
            }
        }
        return true;
    }
    return false;
}

/* ------------------------------------------------------------------ */
/* configuration - legacy CFG-* messages (u-blox 6/7/M8/M9)           */
/* ------------------------------------------------------------------ */
static void cfg_msg_rate(bridge_t *b, uint8_t cls, uint8_t id, uint8_t rate, const char *name, bool optional)
{
    uint8_t f[16];
    char what[48];
    snprintf(what, sizeof(what), "CFG-MSG %s=%u", name, rate);
    size_t n = ubx_build_cfg_msg(f, sizeof(f), cls, id, rate);
    int r = send_cfg(b, f, n, what);
    if (r == -1 && !optional) LOGW("GPS: module rejected %s", what);
    if (r == 0) LOGW("GPS: no answer to %s", what);
}

static void configure_legacy(bridge_t *b)
{
    const options_t *o = &b->opt;
    uint8_t f[128];
    size_t n;

    /* --- port protocols --- */
    if (!b->have_prt && poll_cfg(b, UBX_CLASS_CFG, UBX_CFG_PRT, "CFG-PRT poll")) {
        if (ubx_decode_cfg_prt(b->wait_payload, b->wait_len, &b->prt)) {
            b->have_prt = true;
            b->ubx_port = b->prt.port_id;
        }
    }
    if (b->have_prt) {
        LOGI("GPS: connected on port %s, in=0x%04X out=0x%04X baud=%lu", port_name(b->prt.port_id),
             b->prt.in_proto_mask, b->prt.out_proto_mask, (unsigned long)b->prt.baudrate);
        uint16_t want_in = b->prt.in_proto_mask | UBX_PROTO_UBX | UBX_PROTO_RTCM3;
        if (want_in != b->prt.in_proto_mask || b->prt.out_proto_mask != UBX_PROTO_UBX) {
            ubx_cfg_prt_t p = b->prt;
            p.in_proto_mask = want_in;
            p.out_proto_mask = UBX_PROTO_UBX;
            n = ubx_build_cfg_prt(f, sizeof(f), &p);
            send_cfg(b, f, n, "CFG-PRT protocols");
            b->prt = p;
        }
    } else {
        LOGW("GPS: CFG-PRT poll failed; leaving port protocols unchanged");
    }

    /* --- navigation rate --- */
    bool rate_ok = false;
    if (poll_cfg(b, UBX_CLASS_CFG, UBX_CFG_RATE, "CFG-RATE poll")) {
        ubx_cfg_rate_t r;
        if (ubx_decode_cfg_rate(b->wait_payload, b->wait_len, &r)) {
            LOGD("GPS: current rate meas=%u nav=%u timeref=%u", r.meas_rate_ms, r.nav_rate, r.time_ref);
            rate_ok = (r.meas_rate_ms == o->rate_ms && r.nav_rate == 1 && r.time_ref == 0);
        }
    }
    if (!rate_ok) {
        n = ubx_build_cfg_rate(f, sizeof(f), (uint16_t)o->rate_ms, 1, 0);
        send_cfg(b, f, n, "CFG-RATE");
        /* the module ACKs invalid rates too: read back like ArduPilot does */
        if (poll_cfg(b, UBX_CLASS_CFG, UBX_CFG_RATE, "CFG-RATE verify")) {
            ubx_cfg_rate_t r;
            if (ubx_decode_cfg_rate(b->wait_payload, b->wait_len, &r) && r.meas_rate_ms != o->rate_ms)
                LOGW("GPS: module kept %u ms measurement rate (requested %d)", r.meas_rate_ms, o->rate_ms);
        }
    }

    /* --- CFG-NAV5: dynamic model / min elevation --- */
    if (poll_cfg(b, UBX_CLASS_CFG, UBX_CFG_NAV5, "CFG-NAV5 poll") && b->wait_len >= 36) {
        uint8_t p[36];
        memcpy(p, b->wait_payload, 36);
        uint16_t mask = 0;
        LOGD("GPS: current dynModel=%u minElev=%d", p[2], (int8_t)p[12]);
        if (o->dynmodel >= 0 && p[2] != (uint8_t)o->dynmodel) { p[2] = (uint8_t)o->dynmodel; mask |= 0x0001; }
        if (o->min_elev != -100 && (int8_t)p[12] != o->min_elev) { p[12] = (uint8_t)(int8_t)o->min_elev; mask |= 0x0002; }
        if (mask) {
            le_put_u16(p, mask);
            n = ubx_frame(UBX_CLASS_CFG, UBX_CFG_NAV5, p, 36, f, sizeof(f));
            send_cfg(b, f, n, "CFG-NAV5");
        }
    } else {
        LOGW("GPS: CFG-NAV5 poll failed");
    }

    /* --- SBAS --- */
    if (o->sbas_mode != 2) {
        if (poll_cfg(b, UBX_CLASS_CFG, UBX_CFG_SBAS, "CFG-SBAS poll") && b->wait_len >= 8) {
            uint8_t p[8];
            memcpy(p, b->wait_payload, 8);
            if (p[0] != (uint8_t)o->sbas_mode) {
                p[0] = (uint8_t)o->sbas_mode;
                n = ubx_frame(UBX_CLASS_CFG, UBX_CFG_SBAS, p, 8, f, sizeof(f));
                send_cfg(b, f, n, "CFG-SBAS");
            }
        }
    }

    /* --- GNSS constellations (ArduPilot GPS_GNSS_MODE algorithm) --- */
    if (o->gnss_mode != 0) {
        if (poll_cfg(b, UBX_CLASS_CFG, UBX_CFG_GNSS, "CFG-GNSS poll") && b->wait_len >= 4) {
            uint8_t p[4 + 8 * 8];
            uint16_t len = b->wait_len > sizeof(p) ? sizeof(p) : b->wait_len;
            memcpy(p, b->wait_payload, len);
            uint8_t num_trk_hw = p[1];
            uint8_t blocks = p[3];
            if (4 + 8 * blocks > len) blocks = (uint8_t)((len - 4) / 8);
            int gnss_count = 0;
            for (int i = 0; i < 8; i++)
                if ((o->gnss_mode & (1 << i)) && i != UBX_GNSS_SBAS) gnss_count++;
            if (gnss_count == 0) gnss_count = 1;
            uint8_t orig[sizeof(p)];
            memcpy(orig, p, sizeof(p));
            for (int i = 0; i < blocks; i++) {
                uint8_t *blk = &p[4 + 8 * i];
                uint8_t gnss_id = blk[0];
                uint32_t flags = le_get_u32(blk + 4);
                if (o->gnss_mode & (1 << gnss_id)) {
                    if (gnss_id != UBX_GNSS_SBAS && (b->generation > GEN_M8 || gnss_id != UBX_GNSS_GAL)) {
                        blk[1] = (uint8_t)((num_trk_hw - 3) / (gnss_count * 2));
                        blk[2] = num_trk_hw;
                    } else if (gnss_id == UBX_GNSS_SBAS) {
                        blk[1] = 1; blk[2] = 3;
                    } else { /* Galileo on M8: max 8 channels recommended */
                        blk[1] = (uint8_t)((num_trk_hw - 3) / (gnss_count * 2));
                        blk[2] = 8;
                    }
                    flags |= 1;
                } else {
                    blk[1] = 0; blk[2] = 0;
                    flags &= ~1u;
                }
                le_put_u32(blk + 4, flags);
            }
            if (memcmp(orig, p, 4 + 8 * blocks) != 0) {
                n = ubx_frame(UBX_CLASS_CFG, UBX_CFG_GNSS, p, (uint16_t)(4 + 8 * blocks), f, sizeof(f));
                send_cfg(b, f, n, "CFG-GNSS");
                usleep(500000); /* GNSS reconfiguration restarts the engine */
            }
        }
    }

    /* --- message rates (3-byte CFG-MSG applies to the current port) --- */
    cfg_msg_rate(b, UBX_CLASS_NAV, UBX_NAV_PVT,     1, "NAV-PVT", false);
    cfg_msg_rate(b, UBX_CLASS_NAV, UBX_NAV_DOP,     1, "NAV-DOP", false);
    cfg_msg_rate(b, UBX_CLASS_NAV, UBX_NAV_TIMEGPS, 1, "NAV-TIMEGPS", false);
    cfg_msg_rate(b, UBX_CLASS_NAV, UBX_NAV_EOE,     1, "NAV-EOE", true);
    cfg_msg_rate(b, UBX_CLASS_NAV, UBX_NAV_RELPOSNED, o->yaw_relposned ? 1 : 0, "NAV-RELPOSNED", true);
    cfg_msg_rate(b, UBX_CLASS_NAV, UBX_NAV_POSLLH,  0, "NAV-POSLLH", false);
    cfg_msg_rate(b, UBX_CLASS_NAV, UBX_NAV_STATUS,  0, "NAV-STATUS", false);
    cfg_msg_rate(b, UBX_CLASS_NAV, UBX_NAV_SOL,     0, "NAV-SOL", true);
    cfg_msg_rate(b, UBX_CLASS_NAV, UBX_NAV_VELNED,  0, "NAV-VELNED", false);
    cfg_msg_rate(b, UBX_CLASS_NAV, UBX_NAV_SVINFO,  0, "NAV-SVINFO", true);
    cfg_msg_rate(b, UBX_CLASS_NAV, UBX_NAV_SAT,     0, "NAV-SAT", true);
    cfg_msg_rate(b, UBX_CLASS_RXM, UBX_RXM_RTCM,    o->rtcm_enable ? 1 : 0, "RXM-RTCM", true);

    if (o->save_config) {
        n = ubx_build_cfg_cfg_save(f, sizeof(f));
        if (send_cfg(b, f, n, "CFG-CFG save") == 1) LOGI("GPS: configuration saved to non-volatile memory");
        else LOGW("GPS: saving configuration failed");
    }
}

/* ------------------------------------------------------------------ */
/* configuration - VALSET (F9/M10)                                    */
/* ------------------------------------------------------------------ */
static int valset(bridge_t *b, uint8_t layers, uint32_t key, uint64_t value, const char *name, bool optional)
{
    uint8_t f[32];
    char what[64];
    snprintf(what, sizeof(what), "VALSET %s(0x%08lX)=%llu", name, (unsigned long)key, (unsigned long long)value);
    size_t n = ubx_build_valset(f, sizeof(f), layers, key, value);
    int r = send_cfg(b, f, n, what);
    if (r == -1 && !optional) LOGW("GPS: module rejected %s", what);
    if (r == 0) LOGW("GPS: no answer to %s", what);
    return r;
}

static void configure_valset(bridge_t *b)
{
    const options_t *o = &b->opt;
    uint8_t layers = UBX_LAYER_RAM | (o->save_config ? (UBX_LAYER_BBR | UBX_LAYER_FLASH) : 0);
    uint8_t port = b->ubx_port;
    if (port > UBX_PORT_SPI) port = UBX_PORT_UART1;

    static const uint32_t inprot_base[5] = {
        UBX_KEY_CFG_I2CINPROT_BASE, UBX_KEY_CFG_UART1INPROT_BASE, UBX_KEY_CFG_UART2INPROT_BASE,
        UBX_KEY_CFG_USBINPROT_BASE, UBX_KEY_CFG_SPIINPROT_BASE };
    static const uint32_t outprot_base[5] = {
        UBX_KEY_CFG_I2COUTPROT_BASE, UBX_KEY_CFG_UART1OUTPROT_BASE, UBX_KEY_CFG_UART2OUTPROT_BASE,
        UBX_KEY_CFG_USBOUTPROT_BASE, UBX_KEY_CFG_SPIOUTPROT_BASE };

    LOGI("GPS: configuring via VALSET for port %s (layers 0x%02X)", port_name(port), layers);

    /* port protocols: UBX + RTCM3 in, UBX only out */
    valset(b, layers, inprot_base[port] + UBX_KEY_PROT_UBX, 1, "INPROT-UBX", false);
    valset(b, layers, inprot_base[port] + UBX_KEY_PROT_RTCM3X, 1, "INPROT-RTCM3X", false);
    valset(b, layers, outprot_base[port] + UBX_KEY_PROT_UBX, 1, "OUTPROT-UBX", false);
    valset(b, layers, outprot_base[port] + UBX_KEY_PROT_NMEA, 0, "OUTPROT-NMEA", false);

    /* rate and navigation engine */
    valset(b, layers, UBX_KEY_CFG_RATE_MEAS, (uint64_t)o->rate_ms, "RATE-MEAS", false);
    valset(b, layers, UBX_KEY_CFG_RATE_NAV, 1, "RATE-NAV", false);
    valset(b, layers, UBX_KEY_CFG_RATE_TIMEREF, 0, "RATE-TIMEREF", false);
    if (o->dynmodel >= 0)
        valset(b, layers, UBX_KEY_CFG_NAVSPG_DYNMODEL, (uint64_t)o->dynmodel, "NAVSPG-DYNMODEL", false);
    if (o->min_elev != -100)
        valset(b, layers, UBX_KEY_CFG_NAVSPG_INFIL_MINELEV, (uint8_t)(int8_t)o->min_elev, "NAVSPG-INFIL_MINELEV", false);

    /* message output on our port */
    valset(b, layers, UBX_KEY_MSGOUT_NAV_PVT_I2C + port,     1, "MSGOUT-NAV-PVT", false);
    valset(b, layers, UBX_KEY_MSGOUT_NAV_DOP_I2C + port,     1, "MSGOUT-NAV-DOP", false);
    valset(b, layers, UBX_KEY_MSGOUT_NAV_TIMEGPS_I2C + port, 1, "MSGOUT-NAV-TIMEGPS", false);
    valset(b, layers, UBX_KEY_MSGOUT_NAV_EOE_I2C + port,     1, "MSGOUT-NAV-EOE", true);
    valset(b, layers, UBX_KEY_MSGOUT_NAV_RELPOSNED_I2C + port, o->yaw_relposned ? 1 : 0, "MSGOUT-NAV-RELPOSNED", true);
    valset(b, layers, UBX_KEY_MSGOUT_NAV_STATUS_I2C + port,  0, "MSGOUT-NAV-STATUS", true);
    valset(b, layers, UBX_KEY_MSGOUT_NAV_POSLLH_I2C + port,  0, "MSGOUT-NAV-POSLLH", true);
    valset(b, layers, UBX_KEY_MSGOUT_NAV_VELNED_I2C + port,  0, "MSGOUT-NAV-VELNED", true);
    valset(b, layers, UBX_KEY_MSGOUT_NAV_SOL_I2C + port,     0, "MSGOUT-NAV-SOL", true);
    valset(b, layers, UBX_KEY_MSGOUT_NAV_SAT_I2C + port,     0, "MSGOUT-NAV-SAT", true);
    valset(b, layers, UBX_KEY_MSGOUT_RXM_RTCM_I2C + port,    o->rtcm_enable ? 1 : 0, "MSGOUT-RXM-RTCM", true);

    /* SBAS (ArduPilot uses the signal keys on F9/M10) */
    if (o->sbas_mode != 2) {
        uint64_t ena = (o->sbas_mode == 1) ? 1 : 0;
        valset(b, layers, UBX_KEY_SIGNAL_SBAS_ENA, ena, "SIGNAL-SBAS_ENA", true);
        valset(b, layers, UBX_KEY_SIGNAL_SBAS_L1CA_ENA, ena, "SIGNAL-SBAS_L1CA_ENA", true);
    }

    /* GNSS constellations (ArduPilot populate_F9_gnss) */
    if (o->gnss_mode != 0) {
        int mode = o->gnss_mode | (1 << UBX_GNSS_GPS) | (1 << UBX_GNSS_QZSS);
        mode &= ~(1 << UBX_GNSS_IMES);
        bool changed = false;
        for (int i = 0; i < 8; i++) {
            uint64_t ena = (mode & (1 << i)) ? 1 : 0;
            switch (i) {
            case UBX_GNSS_SBAS:
                if (o->sbas_mode == 2) {
                    valset(b, layers, UBX_KEY_SIGNAL_SBAS_ENA, ena, "SIGNAL-SBAS_ENA", true);
                    valset(b, layers, UBX_KEY_SIGNAL_SBAS_L1CA_ENA, ena, "SIGNAL-SBAS_L1CA_ENA", true);
                }
                changed = true;
                break;
            case UBX_GNSS_GAL:
                valset(b, layers, UBX_KEY_SIGNAL_GAL_ENA, ena, "SIGNAL-GAL_ENA", true);
                valset(b, layers, UBX_KEY_SIGNAL_GAL_E1_ENA, ena, "SIGNAL-GAL_E1_ENA", true);
                valset(b, layers, b->is_zed ? UBX_KEY_SIGNAL_GAL_E5B_ENA : UBX_KEY_SIGNAL_GAL_E5A_ENA, ena, "SIGNAL-GAL_E5x_ENA", true);
                changed = true;
                break;
            case UBX_GNSS_BDS:
                valset(b, layers, UBX_KEY_SIGNAL_BDS_ENA, ena, "SIGNAL-BDS_ENA", true);
                valset(b, layers, UBX_KEY_SIGNAL_BDS_B1_ENA, ena, "SIGNAL-BDS_B1_ENA", true);
                valset(b, layers, b->is_zed ? UBX_KEY_SIGNAL_BDS_B2_ENA : UBX_KEY_SIGNAL_BDS_B2A_ENA, ena, "SIGNAL-BDS_B2x_ENA", true);
                changed = true;
                break;
            case UBX_GNSS_GLO:
                valset(b, layers, UBX_KEY_SIGNAL_GLO_ENA, ena, "SIGNAL-GLO_ENA", true);
                valset(b, layers, UBX_KEY_SIGNAL_GLO_L1_ENA, ena, "SIGNAL-GLO_L1_ENA", true);
                if (b->is_zed) valset(b, layers, UBX_KEY_SIGNAL_GLO_L2_ENA, ena, "SIGNAL-GLO_L2_ENA", true);
                changed = true;
                break;
            case UBX_GNSS_NAVIC:
                valset(b, layers, UBX_KEY_SIGNAL_NAVIC_ENA, ena, "SIGNAL-NAVIC_ENA", true);
                valset(b, layers, UBX_KEY_SIGNAL_NAVIC_L5_ENA, ena, "SIGNAL-NAVIC_L5_ENA", true);
                changed = true;
                break;
            default: /* GPS and QZSS are left alone, IMES unsupported */
                break;
            }
        }
        if (changed) usleep(500000); /* GNSS takes ~0.5 s to restart */
    }

    if (o->save_config) LOGI("GPS: configuration written to RAM, BBR and flash layers");
}

/* ------------------------------------------------------------------ */
/* setup entry point                                                  */
/* ------------------------------------------------------------------ */
void gps_setup(bridge_t *b)
{
    b->gps_state = GPS_SETUP;
    b->have_ver = false;
    b->have_prt = false;
    b->generation = GEN_UNKNOWN;
    b->valset = false;
    memset(&b->ep, 0, sizeof(b->ep));
    b->ep.leap_s = 18; /* current GPS-UTC offset, refined by NAV-TIMEGPS */

    if (b->gps.fd < 0) return;

    if (!detect(b)) {
        if (!g_stop) LOGW("GPS: no u-blox found on %s, retrying", b->opt.gps_dev);
        usleep(500000);
        return;
    }

    /* identify the module */
    uint8_t f[16];
    size_t n = ubx_build_poll(f, sizeof(f), UBX_CLASS_MON, UBX_MON_VER);
    for (int i = 0; i < 3 && !b->have_ver && !g_stop; i++) {
        send_frame(b, f, n, "MON-VER poll");
        if (wait_frame(b, UBX_CLASS_MON, UBX_MON_VER, 800) &&
            ubx_decode_mon_ver(b->wait_payload, b->wait_len, &b->ver)) {
            b->have_ver = true;
            parse_version(b);
        }
    }
    if (!b->have_ver) LOGW("GPS: no MON-VER reply, assuming legacy configuration");

    /* which port are we on? (needed for VALSET message-output keys) */
    if (!b->have_prt && poll_cfg(b, UBX_CLASS_CFG, UBX_CFG_PRT, "CFG-PRT poll")) {
        if (ubx_decode_cfg_prt(b->wait_payload, b->wait_len, &b->prt)) {
            b->have_prt = true;
            b->ubx_port = b->prt.port_id;
        }
    }
    if (!b->have_prt) {
        if (b->opt.gps_port >= 0) b->ubx_port = (uint8_t)b->opt.gps_port;
        else b->ubx_port = is_usb_device(b) ? UBX_PORT_USB : UBX_PORT_UART1;
        LOGI("GPS: port not reported by module, assuming %s", port_name(b->ubx_port));
    } else if (b->opt.gps_port >= 0 && b->opt.gps_port != b->ubx_port) {
        LOGW("GPS: module reports port %s but --gps-port says %s; using the module's answer",
             port_name(b->ubx_port), port_name((uint8_t)b->opt.gps_port));
    }

    if (b->opt.no_config) {
        LOGI("GPS: --no-config given, module left as is");
    } else if (b->valset) {
        configure_valset(b);
    } else {
        configure_legacy(b);
    }

    serial_flush(b->gps.fd);
    ubx_parser_init(&b->ubxp);
    b->gps_state = GPS_RUNNING;
    b->last_pvt_ms = now_ms();   /* start the data timeout from here */
    b->st.gps_reconnects++;

    LOGI("GPS: ready - %s %s at %lu baud, %d ms rate, dynmodel %d, port %s",
         gps_generation_name(b->generation), b->module[0] ? b->module : "",
         (unsigned long)b->gps_actual_baud, b->opt.rate_ms, b->opt.dynmodel, port_name(b->ubx_port));
    if (b->have_ver)
        fc_statustext(b, MAV_SEVERITY_INFO, "u-blox %s%sHW:%.8s PROT %s",
                      b->module, b->module[0] ? " " : "", b->ver.hw_version, b->protver[0] ? b->protver : "?");
    else
        fc_statustext(b, MAV_SEVERITY_INFO, "ubx2mav: GPS online at %lu baud", (unsigned long)b->gps_actual_baud);
}

/* ------------------------------------------------------------------ */
/* time helpers                                                       */
/* ------------------------------------------------------------------ */
/* days since 1970-01-01 for a proleptic Gregorian date (H. Hinnant) */
static int64_t days_from_civil(int y, unsigned m, unsigned d)
{
    y -= m <= 2;
    const int64_t era = (y >= 0 ? y : y - 399) / 400;
    const unsigned yoe = (unsigned)(y - era * 400);
    const unsigned doy = (153 * (m + (m > 2 ? -3 : 9)) + 2) / 5 + d - 1;
    const unsigned doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;
    return era * 146097 + (int64_t)doe - 719468;
}

#define GPS_EPOCH_UNIX 315964800LL
#define SECS_PER_WEEK  604800LL

/* GPS week from the UTC date/time in NAV-PVT */
static bool week_from_pvt_utc(const ubx_nav_pvt_t *p, int leap_s, uint16_t *week)
{
    if ((p->valid & (UBX_PVT_VALID_DATE | UBX_PVT_VALID_TIME)) != (UBX_PVT_VALID_DATE | UBX_PVT_VALID_TIME)) return false;
    if (p->year < 2015 || p->month < 1 || p->month > 12 || p->day < 1 || p->day > 31) return false;
    int64_t unix_s = days_from_civil(p->year, p->month, p->day) * 86400 + p->hour * 3600 + p->min * 60 + p->sec;
    int64_t gps_s = unix_s - GPS_EPOCH_UNIX + leap_s;
    /* resolve the week using the iTOW to avoid the +/- leap second ambiguity at week edges */
    int64_t w = gps_s / SECS_PER_WEEK;
    int64_t tow_from_utc = gps_s % SECS_PER_WEEK;
    int64_t tow_from_itow = p->itow / 1000;
    if (tow_from_utc - tow_from_itow > SECS_PER_WEEK / 2) w++;
    else if (tow_from_itow - tow_from_utc > SECS_PER_WEEK / 2) w--;
    if (w < 1800 || w > 65535) return false;
    *week = (uint16_t)w;
    return true;
}

/* ------------------------------------------------------------------ */
/* NAV message handling                                               */
/* ------------------------------------------------------------------ */
static void handle_pvt(bridge_t *b)
{
    ubx_nav_pvt_t m;
    if (!ubx_decode_nav_pvt(b->ubxp.payload, b->ubxp.len, &m)) return;
    epoch_t *ep = &b->ep;

    if (ep->pending) {
        /* previous epoch never got its EOE: send it now rather than lose it */
        gps_send_epoch(b);
    }

    /* week number: NAV-TIMEGPS is authoritative, roll over on iTOW wrap like ArduPilot */
    if (ep->week_valid && ep->last_pvt_itow != 0 && m.itow < ep->last_pvt_itow &&
        ep->last_pvt_itow > 604000000u && m.itow < 1000000u) {
        ep->week++;
    }
    if (!ep->week_valid || !(ep->timegps_valid && ep->timegps.itow == m.itow)) {
        uint16_t w;
        if (week_from_pvt_utc(&m, ep->leap_s, &w)) {
            if (ep->week_valid && w != ep->week && !ep->timegps_valid) {
                LOGD("GPS: week from UTC %u differs from tracked %u", w, ep->week);
            }
            if (!ep->week_valid) {
                ep->week = w;
                ep->week_valid = true;
            }
        }
    }
    ep->last_pvt_itow = m.itow;

    ep->pvt = m;
    ep->itow = m.itow;
    ep->pvt_rx_ms = now_ms();
    ep->pending = true;
    b->last_pvt_ms = ep->pvt_rx_ms;
    b->st.epochs++;

    LOGT("PVT itow=%u fix=%u flags=0x%02X sats=%u lat=%d lon=%d hmsl=%d hacc=%u",
         m.itow, m.fix_type, m.flags, m.num_sv, m.lat, m.lon, m.h_msl, m.h_acc);

    if (b->gps_state != GPS_RUNNING) {
        ep->pending = false;   /* don't forward while configuring */
        return;
    }
    if (b->opt.send_mode == SEND_PVT || !ep->eoe_seen) {
        if (b->opt.send_mode == SEND_EOE && ep->epochs_without_eoe < 20) {
            /* give NAV-EOE a chance to show up for the first few epochs */
            ep->epochs_without_eoe++;
            return;
        }
        gps_send_epoch(b);
    }
}

static void handle_dop(bridge_t *b)
{
    ubx_nav_dop_t m;
    if (!ubx_decode_nav_dop(b->ubxp.payload, b->ubxp.len, &m)) return;
    b->ep.dop = m;
    b->ep.dop_valid = true;
    LOGT("DOP itow=%u hdop=%u vdop=%u pdop=%u", m.itow, m.h_dop, m.v_dop, m.p_dop);
}

static void handle_timegps(bridge_t *b)
{
    ubx_nav_timegps_t m;
    if (!ubx_decode_nav_timegps(b->ubxp.payload, b->ubxp.len, &m)) return;
    epoch_t *ep = &b->ep;
    if (m.valid & UBX_TIMEGPS_VALID_LEAPS) {
        ep->leap_s = m.leap_s;
        ep->leap_valid = true;
    }
    if (m.valid & UBX_TIMEGPS_VALID_WEEK) {
        ep->timegps = m;
        ep->timegps_valid = true;
        if (!ep->week_valid || ep->week != (uint16_t)m.week) {
            LOGD("GPS: week %d (leap %d s)", m.week, m.leap_s);
        }
        ep->week = (uint16_t)m.week;
        ep->week_valid = true;
    }
    LOGT("TIMEGPS itow=%u week=%d leap=%d valid=0x%02X", m.itow, m.week, m.leap_s, m.valid);
}

static void handle_relposned(bridge_t *b)
{
    ubx_nav_relposned_t m;
    if (!ubx_decode_nav_relposned(b->ubxp.payload, b->ubxp.len, &m)) return;
    b->ep.relposned = m;
    b->ep.relposned_valid = true;
    b->ep.relposned_rx_ms = now_ms();
    LOGT("RELPOSNED itow=%u heading=%d len=%d flags=0x%08X", m.itow, m.rel_pos_heading, m.rel_pos_length, m.flags);
}

static void handle_eoe(bridge_t *b)
{
    epoch_t *ep = &b->ep;
    uint32_t itow = b->ubxp.len >= 4 ? le_get_u32(b->ubxp.payload) : 0;
    if (!ep->eoe_seen) {
        LOGI("GPS: NAV-EOE available, forwarding complete epochs");
        ep->eoe_seen = true;
    }
    ep->epochs_without_eoe = 0;
    if (ep->pending && b->gps_state == GPS_RUNNING) {
        if (itow != ep->itow) LOGD("GPS: EOE itow %u != PVT itow %u", itow, ep->itow);
        gps_send_epoch(b);
    }
}

static void handle_rxm_rtcm(bridge_t *b)
{
    ubx_rxm_rtcm_t m;
    if (!ubx_decode_rxm_rtcm(b->ubxp.payload, b->ubxp.len, &m)) return;
    if (m.flags & 0x01) b->st.rtcm_gps_crc_fail++;
    else if (((m.flags >> 1) & 0x03) == 2) b->st.rtcm_gps_used++;
    LOGD("GPS: RTCM %u from station %u %s", m.msg_type, m.ref_station,
         (m.flags & 1) ? "CRC FAILED" : ((((m.flags >> 1) & 3) == 2) ? "used" : "not used"));
}

void gps_handle_frame(bridge_t *b)
{
    ubx_parser_t *p = &b->ubxp;
    b->last_ubx_ms = now_ms();
    b->det_ubx_seen = true;

    /* synchronous waits used during configuration */
    if (b->wait_active && p->cls == b->wait_cls && p->id == b->wait_id) {
        memcpy(b->wait_payload, p->payload, p->len);
        b->wait_len = p->len;
        b->wait_matched = true;
    }
    if (p->cls == UBX_CLASS_ACK) {
        ubx_ack_t a;
        if (ubx_decode_ack(p->payload, p->len, &a)) {
            if (b->ack_active && a.cls_id == b->ack_cls && a.msg_id == b->ack_id)
                b->ack_result = (p->id == UBX_ACK_ACK) ? 1 : -1;
            if (!b->ack_active || a.cls_id != b->ack_cls || a.msg_id != b->ack_id)
                LOGT("unsolicited %s for 0x%02X 0x%02X", p->id == UBX_ACK_ACK ? "ACK" : "NAK", a.cls_id, a.msg_id);
            /* remember NAKs for poll_cfg() */
            if (p->id == UBX_ACK_NAK) { b->ack_cls = a.cls_id; b->ack_id = a.msg_id; b->ack_result = -1; }
        }
        return;
    }

    if (p->cls == UBX_CLASS_NAV) {
        switch (p->id) {
        case UBX_NAV_PVT:       handle_pvt(b); break;
        case UBX_NAV_DOP:       handle_dop(b); break;
        case UBX_NAV_TIMEGPS:   handle_timegps(b); break;
        case UBX_NAV_RELPOSNED: handle_relposned(b); break;
        case UBX_NAV_EOE:       handle_eoe(b); break;
        default:
            LOGT("NAV 0x%02X len %u ignored", p->id, p->len);
            break;
        }
        return;
    }
    if (p->cls == UBX_CLASS_RXM && p->id == UBX_RXM_RTCM) {
        handle_rxm_rtcm(b);
        return;
    }
    if (p->cls == UBX_CLASS_MON && p->id == UBX_MON_VER) {
        if (!b->have_ver && ubx_decode_mon_ver(p->payload, p->len, &b->ver)) {
            b->have_ver = true;
            parse_version(b);
        }
        return;
    }
    if (p->cls == UBX_CLASS_INF) {
        char txt[128];
        unsigned n = p->len < sizeof(txt) - 1 ? p->len : sizeof(txt) - 1;
        memcpy(txt, p->payload, n);
        txt[n] = 0;
        LOGD("GPS INF-%02X: %s", p->id, txt);
        return;
    }
    LOGT("UBX 0x%02X 0x%02X len %u ignored", p->cls, p->id, p->len);
}

/* called from the main loop: end-of-epoch timeout fallback */
void gps_epoch_tick(bridge_t *b)
{
    epoch_t *ep = &b->ep;
    if (!ep->pending || b->gps_state != GPS_RUNNING) return;
    uint64_t age = now_ms() - ep->pvt_rx_ms;
    if ((int)age >= b->opt.eoe_timeout_ms) {
        if (ep->eoe_seen) {
            b->st.eoe_timeouts++;
            LOGD("GPS: EOE timeout after %llu ms, forwarding epoch %u", (unsigned long long)age, ep->itow);
        }
        gps_send_epoch(b);
    }
}
