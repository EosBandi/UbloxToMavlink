/*
 * mavlink_min.h - minimal, dependency-free MAVLink v2 encoder/decoder.
 *
 * Only the handful of messages this bridge needs are implemented, with
 * CRC_EXTRA values taken from the official common.xml (verified against
 * pymavlink 2.4.48). Wire layout is the standard MAVLink field ordering
 * (fields sorted by size, extensions appended).
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
#ifndef MAVLINK_MIN_H
#define MAVLINK_MIN_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#define MAV_STX_V1 0xFE
#define MAV_STX_V2 0xFD
#define MAV_MAX_PAYLOAD 255
#define MAV_V2_HDR_LEN 10
#define MAV_SIG_LEN 13
#define MAV_MAX_FRAME (MAV_V2_HDR_LEN + MAV_MAX_PAYLOAD + 2 + MAV_SIG_LEN)

/* message ids */
#define MAVLINK_MSG_ID_HEARTBEAT        0
#define MAVLINK_MSG_ID_SYSTEM_TIME      2
#define MAVLINK_MSG_ID_GPS_INJECT_DATA  123
#define MAVLINK_MSG_ID_GPS_INPUT        232
#define MAVLINK_MSG_ID_GPS_RTCM_DATA    233
#define MAVLINK_MSG_ID_STATUSTEXT       253

/* MAV_TYPE / MAV_AUTOPILOT / MAV_STATE / component ids */
#define MAV_TYPE_GCS                 6
#define MAV_TYPE_GPS                 41
#define MAV_AUTOPILOT_ARDUPILOTMEGA  3
#define MAV_AUTOPILOT_INVALID        8
#define MAV_STATE_BOOT               1
#define MAV_STATE_STANDBY            3
#define MAV_STATE_ACTIVE             4
#define MAV_STATE_CRITICAL           5
#define MAV_COMP_ID_GPS              220
#define MAV_COMP_ID_GPS2             221

/* MAV_SEVERITY */
#define MAV_SEVERITY_EMERGENCY 0
#define MAV_SEVERITY_ALERT     1
#define MAV_SEVERITY_CRITICAL  2
#define MAV_SEVERITY_ERROR     3
#define MAV_SEVERITY_WARNING   4
#define MAV_SEVERITY_NOTICE    5
#define MAV_SEVERITY_INFO      6
#define MAV_SEVERITY_DEBUG     7

/* GPS_FIX_TYPE as used by GPS_INPUT.fix_type (== AP_GPS_FixType) */
#define GPS_FIX_TYPE_NO_GPS    0
#define GPS_FIX_TYPE_NO_FIX    1
#define GPS_FIX_TYPE_2D_FIX    2
#define GPS_FIX_TYPE_3D_FIX    3
#define GPS_FIX_TYPE_DGPS      4
#define GPS_FIX_TYPE_RTK_FLOAT 5
#define GPS_FIX_TYPE_RTK_FIXED 6

/* GPS_INPUT_IGNORE_FLAGS */
#define GPS_INPUT_IGNORE_FLAG_ALT                 1
#define GPS_INPUT_IGNORE_FLAG_HDOP                2
#define GPS_INPUT_IGNORE_FLAG_VDOP                4
#define GPS_INPUT_IGNORE_FLAG_VEL_HORIZ           8
#define GPS_INPUT_IGNORE_FLAG_VEL_VERT            16
#define GPS_INPUT_IGNORE_FLAG_SPEED_ACCURACY      32
#define GPS_INPUT_IGNORE_FLAG_HORIZONTAL_ACCURACY 64
#define GPS_INPUT_IGNORE_FLAG_VERTICAL_ACCURACY   128

#define MAVLINK_MSG_GPS_RTCM_DATA_FIELD_DATA_LEN   180
#define MAVLINK_MSG_GPS_INJECT_DATA_FIELD_DATA_LEN 110

/* decoded message (payload is zero-extended to the full message length) */
typedef struct {
    uint8_t  magic;      /* 0xFE or 0xFD */
    uint8_t  len;        /* payload length on the wire */
    uint8_t  incompat_flags;
    uint8_t  compat_flags;
    uint8_t  seq;
    uint8_t  sysid;
    uint8_t  compid;
    uint32_t msgid;
    uint8_t  payload[MAV_MAX_PAYLOAD];
    bool     crc_ok;     /* false when the message id is unknown to us */
} mav_message_t;

typedef struct {
    uint8_t  state;
    uint8_t  buf[MAV_MAX_FRAME];
    uint16_t idx;
    uint16_t need;       /* total expected frame bytes once the header is known */
    mav_message_t msg;
    /* statistics */
    uint32_t rx_ok;
    uint32_t rx_crc_err;
    uint32_t rx_unknown;
    uint32_t rx_v1;
} mav_parser_t;

/* ---- channel/encoder state ---- */
typedef struct {
    uint8_t sysid;
    uint8_t compid;
    uint8_t seq;
} mav_channel_t;

/* GPS_INPUT message in host form */
typedef struct {
    uint64_t time_usec;
    uint32_t time_week_ms;
    int32_t  lat;
    int32_t  lon;
    float    alt;
    float    hdop;
    float    vdop;
    float    vn;
    float    ve;
    float    vd;
    float    speed_accuracy;
    float    horiz_accuracy;
    float    vert_accuracy;
    uint16_t ignore_flags;
    uint16_t time_week;
    uint8_t  gps_id;
    uint8_t  fix_type;
    uint8_t  satellites_visible;
    uint16_t yaw;
} mav_gps_input_t;

typedef struct {
    uint8_t flags;
    uint8_t len;
    uint8_t data[MAVLINK_MSG_GPS_RTCM_DATA_FIELD_DATA_LEN];
} mav_gps_rtcm_data_t;

typedef struct {
    uint8_t target_system;
    uint8_t target_component;
    uint8_t len;
    uint8_t data[MAVLINK_MSG_GPS_INJECT_DATA_FIELD_DATA_LEN];
} mav_gps_inject_data_t;

typedef struct {
    uint32_t custom_mode;
    uint8_t  type;
    uint8_t  autopilot;
    uint8_t  base_mode;
    uint8_t  system_status;
    uint8_t  mavlink_version;
} mav_heartbeat_t;

/* CRC helpers (X.25 / CRC-16-CCITT as used by MAVLink) */
void     mav_crc_init(uint16_t *crc);
void     mav_crc_accumulate(uint8_t data, uint16_t *crc);
uint16_t mav_crc_calculate(const uint8_t *buf, size_t len);

/* returns crc_extra for a known message id, or -1 */
int mav_crc_extra(uint32_t msgid);
/* returns the full (max) payload length for a known message id, or -1 */
int mav_msg_len(uint32_t msgid);

/* Generic v2 frame builder. payload is `len` bytes (full message length);
 * trailing zeros are trimmed as required by MAVLink 2. Returns frame length. */
size_t mav_pack_frame(mav_channel_t *ch, uint32_t msgid, const uint8_t *payload,
                      size_t len, uint8_t *out, size_t out_size);

size_t mav_pack_heartbeat(mav_channel_t *ch, const mav_heartbeat_t *hb,
                          uint8_t *out, size_t out_size);
size_t mav_pack_gps_input(mav_channel_t *ch, const mav_gps_input_t *gi,
                          uint8_t *out, size_t out_size);
size_t mav_pack_statustext(mav_channel_t *ch, uint8_t severity, const char *text,
                           uint8_t *out, size_t out_size);

/* Parser: feed one byte at a time, returns true when a complete frame
 * with a good CRC (for a message we know) has been decoded into p->msg. */
void mav_parser_init(mav_parser_t *p);
bool mav_parse_byte(mav_parser_t *p, uint8_t c);

/* decoders */
void mav_decode_heartbeat(const mav_message_t *m, mav_heartbeat_t *hb);
void mav_decode_gps_rtcm_data(const mav_message_t *m, mav_gps_rtcm_data_t *d);
void mav_decode_gps_inject_data(const mav_message_t *m, mav_gps_inject_data_t *d);

/* little-endian helpers (exported for tests) */
void     le_put_u16(uint8_t *b, uint16_t v);
void     le_put_u32(uint8_t *b, uint32_t v);
void     le_put_u64(uint8_t *b, uint64_t v);
void     le_put_i32(uint8_t *b, int32_t v);
void     le_put_f32(uint8_t *b, float v);
uint16_t le_get_u16(const uint8_t *b);
uint32_t le_get_u32(const uint8_t *b);
uint64_t le_get_u64(const uint8_t *b);
int32_t  le_get_i32(const uint8_t *b);
int16_t  le_get_i16(const uint8_t *b);
float    le_get_f32(const uint8_t *b);

#endif /* MAVLINK_MIN_H */
