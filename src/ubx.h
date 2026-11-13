/*
 * ubx.h - u-blox UBX protocol framing, parsing and message definitions.
 *
 * Message layouts follow the u-blox M8/F9/M10 interface descriptions and
 * mirror the structures ArduPilot's AP_GPS_UBLOX driver uses.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
#ifndef UBX_H
#define UBX_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#define UBX_SYNC1 0xB5
#define UBX_SYNC2 0x62
#define UBX_MAX_PAYLOAD 1024   /* plenty for everything we enable */

/* classes */
#define UBX_CLASS_NAV 0x01
#define UBX_CLASS_RXM 0x02
#define UBX_CLASS_INF 0x04
#define UBX_CLASS_ACK 0x05
#define UBX_CLASS_CFG 0x06
#define UBX_CLASS_MON 0x0A
#define UBX_CLASS_TIM 0x0D

/* NAV */
#define UBX_NAV_POSLLH    0x02
#define UBX_NAV_STATUS    0x03
#define UBX_NAV_DOP       0x04
#define UBX_NAV_SOL       0x06
#define UBX_NAV_PVT       0x07
#define UBX_NAV_VELNED    0x12
#define UBX_NAV_TIMEGPS   0x20
#define UBX_NAV_SVINFO    0x30
#define UBX_NAV_SAT       0x35
#define UBX_NAV_RELPOSNED 0x3C
#define UBX_NAV_EOE       0x61
/* ACK */
#define UBX_ACK_NAK 0x00
#define UBX_ACK_ACK 0x01
/* CFG */
#define UBX_CFG_PRT    0x00
#define UBX_CFG_MSG    0x01
#define UBX_CFG_RST    0x04
#define UBX_CFG_RATE   0x08
#define UBX_CFG_CFG    0x09
#define UBX_CFG_SBAS   0x16
#define UBX_CFG_NAV5   0x24
#define UBX_CFG_GNSS   0x3E
#define UBX_CFG_VALSET 0x8A
#define UBX_CFG_VALGET 0x8B
/* MON */
#define UBX_MON_VER 0x04
#define UBX_MON_HW  0x09
/* RXM */
#define UBX_RXM_RTCM 0x32

/* CFG-PRT port ids (also the port index for CFG-MSGOUT-* VALSET keys) */
#define UBX_PORT_I2C   0
#define UBX_PORT_UART1 1
#define UBX_PORT_UART2 2
#define UBX_PORT_USB   3
#define UBX_PORT_SPI   4
#define UBX_PORT_UNKNOWN 0xFF

/* CFG-PRT protocol masks */
#define UBX_PROTO_UBX   0x0001
#define UBX_PROTO_NMEA  0x0002
#define UBX_PROTO_RTCM2 0x0004
#define UBX_PROTO_RTCM3 0x0020

/* VALSET layers */
#define UBX_LAYER_RAM   0x01
#define UBX_LAYER_BBR   0x02
#define UBX_LAYER_FLASH 0x04

/* CFG-CFG save mask as used by ArduPilot (SAVE_CFG_ALL) */
#define UBX_SAVE_CFG_ALL 0x0000061F

/* NAV-PVT flags */
#define UBX_PVT_FLAGS_GNSSFIXOK      0x01
#define UBX_PVT_FLAGS_DIFFSOLN       0x02
#define UBX_PVT_FLAGS_HEADVEHVALID   0x20
#define UBX_PVT_FLAGS_CARRSOLN_MASK  0xC0
#define UBX_PVT_FLAGS_CARRSOLN_FLOAT 0x40
#define UBX_PVT_FLAGS_CARRSOLN_FIXED 0x80
/* NAV-PVT valid */
#define UBX_PVT_VALID_DATE 0x01
#define UBX_PVT_VALID_TIME 0x02
#define UBX_PVT_VALID_FULLYRESOLVED 0x04
/* NAV-TIMEGPS valid */
#define UBX_TIMEGPS_VALID_TOW   0x01
#define UBX_TIMEGPS_VALID_WEEK  0x02
#define UBX_TIMEGPS_VALID_LEAPS 0x04
/* NAV-RELPOSNED flags (v1) */
#define UBX_RELPOSNED_GNSSFIXOK          (1u << 0)
#define UBX_RELPOSNED_DIFFSOLN           (1u << 1)
#define UBX_RELPOSNED_RELPOSVALID        (1u << 2)
#define UBX_RELPOSNED_CARRSOLN_FLOAT     (1u << 3)
#define UBX_RELPOSNED_CARRSOLN_FIXED     (2u << 3)
#define UBX_RELPOSNED_CARRSOLN_MASK      (3u << 3)
#define UBX_RELPOSNED_ISMOVING           (1u << 5)
#define UBX_RELPOSNED_REFPOSMISS         (1u << 6)
#define UBX_RELPOSNED_REFOBSMISS         (1u << 7)
#define UBX_RELPOSNED_RELPOSHEADINGVALID (1u << 8)
#define UBX_RELPOSNED_RELPOSNORMALIZED   (1u << 9)

/* GNSS ids (CFG-GNSS gnssId / GPS_GNSS_MODE bit positions) */
#define UBX_GNSS_GPS   0
#define UBX_GNSS_SBAS  1
#define UBX_GNSS_GAL   2
#define UBX_GNSS_BDS   3
#define UBX_GNSS_IMES  4
#define UBX_GNSS_QZSS  5
#define UBX_GNSS_GLO   6
#define UBX_GNSS_NAVIC 7

/* ------------------------------------------------------------------ */
/* VALSET configuration keys (u-blox F9/M9/M10, protocol >= 23.01)    */
/* Verified against the u-blox interface descriptions / pyubx2.      */
/* ------------------------------------------------------------------ */
#define UBX_KEY_CFG_RATE_MEAS              0x30210001u /* U2 ms */
#define UBX_KEY_CFG_RATE_NAV               0x30210002u /* U2 */
#define UBX_KEY_CFG_RATE_TIMEREF           0x20210003u /* E1 0=UTC */
#define UBX_KEY_CFG_NAVSPG_DYNMODEL        0x20110021u /* E1 */
#define UBX_KEY_CFG_NAVSPG_INFIL_MINELEV   0x201100A4u /* I1 deg */
#define UBX_KEY_CFG_TMODE_MODE             0x20030001u /* E1 */
#define UBX_KEY_CFG_UART1_BAUDRATE         0x40520001u /* U4 */
#define UBX_KEY_CFG_UART2_BAUDRATE         0x40530001u /* U4 */
/* IN/OUT protocol keys: base + 1 (UBX) / +2 (NMEA) / +4 (RTCM3X) */
#define UBX_KEY_CFG_I2CINPROT_BASE    0x10710000u
#define UBX_KEY_CFG_I2COUTPROT_BASE   0x10720000u
#define UBX_KEY_CFG_UART1INPROT_BASE  0x10730000u
#define UBX_KEY_CFG_UART1OUTPROT_BASE 0x10740000u
#define UBX_KEY_CFG_UART2INPROT_BASE  0x10750000u
#define UBX_KEY_CFG_UART2OUTPROT_BASE 0x10760000u
#define UBX_KEY_CFG_USBINPROT_BASE    0x10770000u
#define UBX_KEY_CFG_USBOUTPROT_BASE   0x10780000u
#define UBX_KEY_CFG_SPIINPROT_BASE    0x10790000u
#define UBX_KEY_CFG_SPIOUTPROT_BASE   0x107A0000u
#define UBX_KEY_PROT_UBX    1u
#define UBX_KEY_PROT_NMEA   2u
#define UBX_KEY_PROT_RTCM3X 4u
/* CFG-MSGOUT-* keys for the I2C port; add the port index (0..4) for
 * I2C, UART1, UART2, USB, SPI respectively. */
#define UBX_KEY_MSGOUT_NAV_PVT_I2C        0x20910006u
#define UBX_KEY_MSGOUT_NAV_DOP_I2C        0x20910038u
#define UBX_KEY_MSGOUT_NAV_TIMEGPS_I2C    0x20910047u
#define UBX_KEY_MSGOUT_NAV_EOE_I2C        0x2091015Fu
#define UBX_KEY_MSGOUT_NAV_STATUS_I2C     0x2091001Au
#define UBX_KEY_MSGOUT_NAV_POSLLH_I2C     0x20910029u
#define UBX_KEY_MSGOUT_NAV_VELNED_I2C     0x20910042u
#define UBX_KEY_MSGOUT_NAV_SOL_I2C        0x20910001u
#define UBX_KEY_MSGOUT_NAV_SAT_I2C        0x20910015u
#define UBX_KEY_MSGOUT_NAV_RELPOSNED_I2C  0x2091008Du
#define UBX_KEY_MSGOUT_RXM_RTCM_I2C       0x20910268u
#define UBX_KEY_MSGOUT_MON_HW_I2C         0x209101B4u
/* GNSS signal enables */
#define UBX_KEY_SIGNAL_GPS_ENA       0x1031001Fu
#define UBX_KEY_SIGNAL_SBAS_ENA      0x10310020u
#define UBX_KEY_SIGNAL_SBAS_L1CA_ENA 0x10310005u
#define UBX_KEY_SIGNAL_GAL_ENA       0x10310021u
#define UBX_KEY_SIGNAL_GAL_E1_ENA    0x10310007u
#define UBX_KEY_SIGNAL_GAL_E5A_ENA   0x10310009u
#define UBX_KEY_SIGNAL_GAL_E5B_ENA   0x1031000Au
#define UBX_KEY_SIGNAL_BDS_ENA       0x10310022u
#define UBX_KEY_SIGNAL_BDS_B1_ENA    0x1031000Du
#define UBX_KEY_SIGNAL_BDS_B2_ENA    0x1031000Eu
#define UBX_KEY_SIGNAL_BDS_B2A_ENA   0x10310028u
#define UBX_KEY_SIGNAL_QZSS_ENA      0x10310024u
#define UBX_KEY_SIGNAL_GLO_ENA       0x10310025u
#define UBX_KEY_SIGNAL_GLO_L1_ENA    0x10310018u
#define UBX_KEY_SIGNAL_GLO_L2_ENA    0x1031001Au
#define UBX_KEY_SIGNAL_NAVIC_ENA     0x10310026u
#define UBX_KEY_SIGNAL_NAVIC_L5_ENA  0x1031001Du

/* ------------------------------------------------------------------ */
/* decoded messages                                                   */
/* ------------------------------------------------------------------ */
typedef struct {
    uint32_t itow;
    uint16_t year; uint8_t month, day, hour, min, sec;
    uint8_t  valid;
    uint32_t t_acc;
    int32_t  nano;
    uint8_t  fix_type;
    uint8_t  flags;
    uint8_t  flags2;
    uint8_t  num_sv;
    int32_t  lon, lat;           /* 1e-7 deg */
    int32_t  h_ellipsoid, h_msl; /* mm */
    uint32_t h_acc, v_acc;       /* mm */
    int32_t  vel_n, vel_e, vel_d, g_speed; /* mm/s */
    int32_t  head_mot;           /* 1e-5 deg */
    uint32_t s_acc;              /* mm/s */
    uint32_t head_acc;           /* 1e-5 deg */
    uint16_t p_dop;              /* 0.01 */
    uint16_t flags3;
    int32_t  head_veh;           /* 1e-5 deg */
} ubx_nav_pvt_t;

typedef struct {
    uint32_t itow;
    uint16_t g_dop, p_dop, t_dop, v_dop, h_dop, n_dop, e_dop; /* 0.01 */
} ubx_nav_dop_t;

typedef struct {
    uint32_t itow;
    int32_t  ftow;
    int16_t  week;
    int8_t   leap_s;
    uint8_t  valid;
    uint32_t t_acc;
} ubx_nav_timegps_t;

typedef struct {
    uint8_t  version;
    uint16_t ref_station_id;
    uint32_t itow;
    int32_t  rel_pos_n, rel_pos_e, rel_pos_d; /* cm */
    int32_t  rel_pos_length;                  /* cm */
    int32_t  rel_pos_heading;                 /* 1e-5 deg */
    uint32_t acc_n, acc_e, acc_d, acc_length; /* 0.1 mm */
    uint32_t acc_heading;                     /* 1e-5 deg */
    uint32_t flags;
} ubx_nav_relposned_t;

typedef struct {
    uint32_t itow;
    uint8_t  gps_fix;
    uint8_t  flags;
    uint8_t  fix_stat;
    uint8_t  flags2;
    uint32_t ttff;
    uint32_t msss;
} ubx_nav_status_t;

#define UBX_MON_VER_MAX_EXT 12
typedef struct {
    char sw_version[31];
    char hw_version[11];
    char extension[UBX_MON_VER_MAX_EXT][31];
    int  num_ext;
} ubx_mon_ver_t;

typedef struct {
    uint8_t  port_id;
    uint16_t tx_ready;
    uint32_t mode;
    uint32_t baudrate;
    uint16_t in_proto_mask;
    uint16_t out_proto_mask;
    uint16_t flags;
} ubx_cfg_prt_t;

typedef struct {
    uint16_t meas_rate_ms;
    uint16_t nav_rate;
    uint16_t time_ref;
} ubx_cfg_rate_t;

typedef struct {
    uint8_t cls_id;
    uint8_t msg_id;
} ubx_ack_t;

typedef struct {
    uint8_t  version;
    uint8_t  flags;      /* bit0 crcFailed, bits1-2 msgUsed (2=used) */
    uint16_t sub_type;
    uint16_t ref_station;
    uint16_t msg_type;
} ubx_rxm_rtcm_t;

/* ------------------------------------------------------------------ */
/* parser                                                             */
/* ------------------------------------------------------------------ */
typedef struct {
    uint8_t  state;
    uint8_t  cls;
    uint8_t  id;
    uint16_t len;
    uint16_t idx;
    uint8_t  ck_a, ck_b;
    uint8_t  payload[UBX_MAX_PAYLOAD];
    /* NMEA sentence detector (used for baud rate detection) */
    uint8_t  nmea_state;
    uint16_t nmea_idx;
    uint8_t  nmea_ck;
    char     nmea_hex[2];
    /* statistics */
    uint32_t rx_ok;
    uint32_t rx_crc_err;
    uint32_t rx_too_long;
    uint32_t rx_nmea;
    uint32_t rx_bytes;
} ubx_parser_t;

typedef enum { UBX_PARSE_NONE = 0, UBX_PARSE_UBX, UBX_PARSE_NMEA } ubx_parse_result_t;

void ubx_parser_init(ubx_parser_t *p);
/* Returns UBX_PARSE_UBX when a full valid UBX frame is in p->cls/id/len/payload,
 * UBX_PARSE_NMEA when a checksum-valid NMEA sentence has been seen. */
ubx_parse_result_t ubx_parse_byte(ubx_parser_t *p, uint8_t c);

/* frame builder: returns bytes written (8 + len) or 0 if out too small */
size_t ubx_frame(uint8_t cls, uint8_t id, const uint8_t *payload, uint16_t len,
                 uint8_t *out, size_t out_size);

/* decoders: return false when payload too short */
bool ubx_decode_nav_pvt(const uint8_t *p, uint16_t len, ubx_nav_pvt_t *m);
bool ubx_decode_nav_dop(const uint8_t *p, uint16_t len, ubx_nav_dop_t *m);
bool ubx_decode_nav_timegps(const uint8_t *p, uint16_t len, ubx_nav_timegps_t *m);
bool ubx_decode_nav_relposned(const uint8_t *p, uint16_t len, ubx_nav_relposned_t *m);
bool ubx_decode_nav_status(const uint8_t *p, uint16_t len, ubx_nav_status_t *m);
bool ubx_decode_mon_ver(const uint8_t *p, uint16_t len, ubx_mon_ver_t *m);
bool ubx_decode_cfg_prt(const uint8_t *p, uint16_t len, ubx_cfg_prt_t *m);
bool ubx_decode_cfg_rate(const uint8_t *p, uint16_t len, ubx_cfg_rate_t *m);
bool ubx_decode_ack(const uint8_t *p, uint16_t len, ubx_ack_t *m);
bool ubx_decode_rxm_rtcm(const uint8_t *p, uint16_t len, ubx_rxm_rtcm_t *m);

/* encoders for configuration messages (complete frames) */
size_t ubx_build_cfg_prt(uint8_t *out, size_t out_size, const ubx_cfg_prt_t *prt);
size_t ubx_build_cfg_rate(uint8_t *out, size_t out_size, uint16_t meas_ms, uint16_t nav_rate, uint16_t time_ref);
size_t ubx_build_cfg_msg(uint8_t *out, size_t out_size, uint8_t cls, uint8_t id, uint8_t rate);
size_t ubx_build_cfg_cfg_save(uint8_t *out, size_t out_size);
/* VALSET with a single key; value size is derived from the key */
size_t ubx_build_valset(uint8_t *out, size_t out_size, uint8_t layers, uint32_t key, uint64_t value);
size_t ubx_build_valget(uint8_t *out, size_t out_size, uint8_t layer, uint32_t key);
/* generic poll (empty payload) */
size_t ubx_build_poll(uint8_t *out, size_t out_size, uint8_t cls, uint8_t id);
/* NMEA $PUBX,41 port configuration sentence */
size_t ubx_build_pubx41(char *out, size_t out_size, uint8_t port_id, uint16_t in_proto,
                        uint16_t out_proto, uint32_t baud);

/* size in bytes of a VALSET value for a key (1,2,4,8) */
uint8_t ubx_key_size(uint32_t key);

#endif /* UBX_H */
