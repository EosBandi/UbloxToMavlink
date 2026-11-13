/*
 * bridge.h - shared state for the u-blox -> MAVLink GPS bridge
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
#ifndef BRIDGE_H
#define BRIDGE_H

#include <stdint.h>
#include <stdbool.h>
#include <signal.h>

#include "mavlink_min.h"
#include "ubx.h"
#include "serial.h"
#include "log.h"

#define UBX2MAV_VERSION "1.0.0"

enum gps_state { GPS_DOWN = 0, GPS_SETUP, GPS_RUNNING };
enum send_mode { SEND_EOE = 0, SEND_PVT };
enum ubx_generation { GEN_UNKNOWN = 0, GEN_UBLOX6, GEN_UBLOX7, GEN_M8, GEN_F9, GEN_M9, GEN_M10 };

/* ArduPilot defaults: GPS_RATE_MS=200, GPS_NAVFILTER=8 (airborne 4g),
 * GPS_MIN_ELEV=-100 (leave), GPS_GNSS_MODE=0 (leave), GPS_SBAS_MODE=2 (leave) */
typedef struct {
    const char *gps_dev;
    uint32_t    gps_baud;        /* target baud rate (ArduPilot uses 230400) */
    bool        gps_autobaud;
    bool        gps_set_baud;    /* switch the module to gps_baud if found elsewhere */
    int         gps_port;        /* UBX_PORT_* or -1 for auto */
    const char *fc_spec;
    uint32_t    fc_baud;
    int         sysid;           /* -1 = adopt the flight controller's sysid */
    int         compid;
    int         gps_id;          /* GPS_INPUT.gps_id, matches ArduPilot GPS instance */
    int         rate_ms;
    int         dynmodel;
    int         min_elev;
    int         gnss_mode;
    int         sbas_mode;       /* 0 off, 1 on, 2 do not change */
    bool        save_config;
    bool        no_config;
    bool        yaw_relposned;
    double      yaw_offset_deg;
    bool        require_fix_ok;  /* honour NAV-PVT gnssFixOK (ArduPilot does not) */
    int         send_mode;
    int         eoe_timeout_ms;
    bool        rtcm_enable;
    bool        statustext;
    int         stats_interval_s;
    int         gps_timeout_ms;
} options_t;

typedef struct {
    bool     pending;
    uint32_t itow;
    uint64_t pvt_rx_ms;
    ubx_nav_pvt_t pvt;
    ubx_nav_dop_t dop;       bool dop_valid;
    ubx_nav_timegps_t timegps; bool timegps_valid;
    ubx_nav_relposned_t relposned; bool relposned_valid; uint64_t relposned_rx_ms;
    uint16_t week;  bool week_valid;
    int8_t   leap_s; bool leap_valid;
    uint32_t last_pvt_itow;
    bool     eoe_seen;
    uint32_t epochs_without_eoe;
} epoch_t;

typedef struct {
    uint8_t  buffer[MAVLINK_MSG_GPS_RTCM_DATA_FIELD_DATA_LEN * 4];
    uint8_t  fragments_received;
    uint8_t  fragment_count;
    uint8_t  sequence;
    uint16_t total_length;
} rtcm_buf_t;

typedef struct {
    uint32_t gps_input_tx;
    uint32_t hb_tx;
    uint32_t statustext_tx;
    uint32_t fc_hb_rx;
    uint32_t fc_msgs_rx;
    uint32_t rtcm_msgs;
    uint32_t rtcm_bytes;
    uint32_t rtcm_frag_discarded;
    uint32_t rtcm_gps_used;
    uint32_t rtcm_gps_crc_fail;
    uint32_t epochs;
    uint32_t eoe_timeouts;
    uint32_t cfg_nak;
    uint32_t gps_reconnects;
    uint32_t fc_reconnects;
} stats_t;

typedef struct bridge {
    options_t opt;

    link_t gps;
    link_t fc;
    uint64_t gps_retry_ms;
    uint64_t fc_retry_ms;

    ubx_parser_t ubxp;
    mav_parser_t mavp;
    mav_channel_t chan;

    int gps_state;
    uint64_t last_pvt_ms;
    uint64_t last_ubx_ms;

    /* module identification */
    bool   have_ver;
    ubx_mon_ver_t ver;
    char   module[24];
    char   protver[12];
    double protver_num;
    int    generation;
    bool   is_zed;
    bool   valset;         /* use CFG-VALSET (F9/M9/M10) instead of legacy CFG-* */
    uint8_t ubx_port;      /* UBX_PORT_* we are connected to */
    bool   have_prt;
    ubx_cfg_prt_t prt;
    uint32_t gps_actual_baud;

    /* synchronous wait support (used while configuring) */
    bool    wait_active;
    uint8_t wait_cls, wait_id;
    bool    wait_matched;
    uint8_t wait_payload[UBX_MAX_PAYLOAD];
    uint16_t wait_len;
    bool    ack_active;
    uint8_t ack_cls, ack_id;
    int     ack_result;    /* 0 pending, 1 ACK, -1 NAK */
    bool    det_ubx_seen;
    bool    det_nmea_seen;

    epoch_t ep;

    /* flight controller side */
    uint64_t last_hb_tx_ms;
    uint64_t last_fc_hb_ms;
    bool     fc_seen;
    uint8_t  fc_sysid;
    uint8_t  fc_compid;
    rtcm_buf_t rtcm;

    stats_t st;
    uint64_t last_stats_ms;
    uint32_t last_stats_epochs;
    uint32_t last_stats_gps_input;
} bridge_t;

extern volatile sig_atomic_t g_stop;

/* main.c */
uint64_t now_ms(void);
uint64_t now_us_realtime(void);
void pump(bridge_t *b, int timeout_ms);
bool gps_write(bridge_t *b, const uint8_t *data, size_t len);
void fc_statustext(bridge_t *b, uint8_t severity, const char *fmt, ...) __attribute__((format(printf, 3, 4)));
void gps_send_epoch(bridge_t *b);

/* gps.c */
void gps_setup(bridge_t *b);
void gps_handle_frame(bridge_t *b);
void gps_epoch_tick(bridge_t *b);
const char *gps_generation_name(int gen);

#endif
