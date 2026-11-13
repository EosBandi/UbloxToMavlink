/*
 * ubx.c - u-blox UBX framing, parsing and message codecs
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
#include "ubx.h"
#include "mavlink_min.h"   /* little-endian helpers */
#include <string.h>
#include <stdio.h>

/* ------------------------------------------------------------------ */
/* framing                                                            */
/* ------------------------------------------------------------------ */
static void ubx_checksum(const uint8_t *data, size_t len, uint8_t *ck_a, uint8_t *ck_b)
{
    uint8_t a = 0, b = 0;
    for (size_t i = 0; i < len; i++) {
        a = (uint8_t)(a + data[i]);
        b = (uint8_t)(b + a);
    }
    *ck_a = a;
    *ck_b = b;
}

size_t ubx_frame(uint8_t cls, uint8_t id, const uint8_t *payload, uint16_t len,
                 uint8_t *out, size_t out_size)
{
    if (out_size < (size_t)len + 8) return 0;
    out[0] = UBX_SYNC1;
    out[1] = UBX_SYNC2;
    out[2] = cls;
    out[3] = id;
    out[4] = (uint8_t)(len & 0xFF);
    out[5] = (uint8_t)(len >> 8);
    if (len) memcpy(&out[6], payload, len);
    ubx_checksum(&out[2], (size_t)len + 4, &out[6 + len], &out[7 + len]);
    return (size_t)len + 8;
}

/* ------------------------------------------------------------------ */
/* parser                                                             */
/* ------------------------------------------------------------------ */
enum { US_SYNC1 = 0, US_SYNC2, US_CLASS, US_ID, US_LEN1, US_LEN2, US_PAYLOAD, US_CKA, US_CKB };
enum { NS_IDLE = 0, NS_BODY, NS_CK1, NS_CK2 };

void ubx_parser_init(ubx_parser_t *p) { memset(p, 0, sizeof(*p)); }

static int hexval(char c)
{
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    return -1;
}

/* NMEA checksum-valid sentence detector; runs in parallel with the UBX
 * state machine and never consumes UBX bytes. */
static bool nmea_feed(ubx_parser_t *p, uint8_t c)
{
    switch (p->nmea_state) {
    case NS_IDLE:
        if (c == '$') { p->nmea_state = NS_BODY; p->nmea_idx = 0; p->nmea_ck = 0; }
        return false;
    case NS_BODY:
        if (c == '*') { p->nmea_state = NS_CK1; return false; }
        if (c == '$') { p->nmea_idx = 0; p->nmea_ck = 0; return false; }
        if (c < 0x20 || c > 0x7E || ++p->nmea_idx > 100) { p->nmea_state = NS_IDLE; return false; }
        p->nmea_ck ^= c;
        return false;
    case NS_CK1:
        if (hexval((char)c) < 0) { p->nmea_state = NS_IDLE; return false; }
        p->nmea_hex[0] = (char)c;
        p->nmea_state = NS_CK2;
        return false;
    case NS_CK2:
        p->nmea_state = NS_IDLE;
        if (hexval((char)c) < 0) return false;
        {
            int v = (hexval(p->nmea_hex[0]) << 4) | hexval((char)c);
            if (v == p->nmea_ck && p->nmea_idx >= 5) { p->rx_nmea++; return true; }
        }
        return false;
    }
    p->nmea_state = NS_IDLE;
    return false;
}

ubx_parse_result_t ubx_parse_byte(ubx_parser_t *p, uint8_t c)
{
    p->rx_bytes++;
    bool nmea = nmea_feed(p, c);

    switch (p->state) {
    case US_SYNC1:
        if (c == UBX_SYNC1) p->state = US_SYNC2;
        break;
    case US_SYNC2:
        if (c == UBX_SYNC2) { p->state = US_CLASS; p->ck_a = p->ck_b = 0; }
        else if (c != UBX_SYNC1) p->state = US_SYNC1;
        break;
    case US_CLASS:
        p->cls = c; p->ck_a = (uint8_t)(p->ck_a + c); p->ck_b = (uint8_t)(p->ck_b + p->ck_a);
        p->state = US_ID;
        break;
    case US_ID:
        p->id = c; p->ck_a = (uint8_t)(p->ck_a + c); p->ck_b = (uint8_t)(p->ck_b + p->ck_a);
        p->state = US_LEN1;
        break;
    case US_LEN1:
        p->len = c; p->ck_a = (uint8_t)(p->ck_a + c); p->ck_b = (uint8_t)(p->ck_b + p->ck_a);
        p->state = US_LEN2;
        break;
    case US_LEN2:
        p->len |= (uint16_t)(c << 8); p->ck_a = (uint8_t)(p->ck_a + c); p->ck_b = (uint8_t)(p->ck_b + p->ck_a);
        p->idx = 0;
        if (p->len > UBX_MAX_PAYLOAD) { p->rx_too_long++; p->state = US_SYNC1; }
        else p->state = p->len ? US_PAYLOAD : US_CKA;
        break;
    case US_PAYLOAD:
        p->payload[p->idx++] = c; p->ck_a = (uint8_t)(p->ck_a + c); p->ck_b = (uint8_t)(p->ck_b + p->ck_a);
        if (p->idx >= p->len) p->state = US_CKA;
        break;
    case US_CKA:
        p->state = (c == p->ck_a) ? US_CKB : US_SYNC1;
        if (p->state == US_SYNC1) p->rx_crc_err++;
        break;
    case US_CKB:
        p->state = US_SYNC1;
        if (c == p->ck_b) { p->rx_ok++; return UBX_PARSE_UBX; }
        p->rx_crc_err++;
        break;
    default:
        p->state = US_SYNC1;
        break;
    }
    return nmea ? UBX_PARSE_NMEA : UBX_PARSE_NONE;
}

/* ------------------------------------------------------------------ */
/* decoders                                                           */
/* ------------------------------------------------------------------ */
bool ubx_decode_nav_pvt(const uint8_t *p, uint16_t len, ubx_nav_pvt_t *m)
{
    if (len < 84) return false;   /* protocol < 18 units are 84 bytes, newer 92 */
    memset(m, 0, sizeof(*m));
    m->itow = le_get_u32(p + 0);
    m->year = le_get_u16(p + 4);
    m->month = p[6]; m->day = p[7]; m->hour = p[8]; m->min = p[9]; m->sec = p[10];
    m->valid = p[11];
    m->t_acc = le_get_u32(p + 12);
    m->nano = le_get_i32(p + 16);
    m->fix_type = p[20];
    m->flags = p[21];
    m->flags2 = p[22];
    m->num_sv = p[23];
    m->lon = le_get_i32(p + 24);
    m->lat = le_get_i32(p + 28);
    m->h_ellipsoid = le_get_i32(p + 32);
    m->h_msl = le_get_i32(p + 36);
    m->h_acc = le_get_u32(p + 40);
    m->v_acc = le_get_u32(p + 44);
    m->vel_n = le_get_i32(p + 48);
    m->vel_e = le_get_i32(p + 52);
    m->vel_d = le_get_i32(p + 56);
    m->g_speed = le_get_i32(p + 60);
    m->head_mot = le_get_i32(p + 64);
    m->s_acc = le_get_u32(p + 68);
    m->head_acc = le_get_u32(p + 72);
    m->p_dop = le_get_u16(p + 76);
    m->flags3 = le_get_u16(p + 78);
    if (len >= 92) m->head_veh = le_get_i32(p + 84);
    return true;
}

bool ubx_decode_nav_dop(const uint8_t *p, uint16_t len, ubx_nav_dop_t *m)
{
    if (len < 18) return false;
    m->itow = le_get_u32(p);
    m->g_dop = le_get_u16(p + 4);
    m->p_dop = le_get_u16(p + 6);
    m->t_dop = le_get_u16(p + 8);
    m->v_dop = le_get_u16(p + 10);
    m->h_dop = le_get_u16(p + 12);
    m->n_dop = le_get_u16(p + 14);
    m->e_dop = le_get_u16(p + 16);
    return true;
}

bool ubx_decode_nav_timegps(const uint8_t *p, uint16_t len, ubx_nav_timegps_t *m)
{
    if (len < 16) return false;
    m->itow = le_get_u32(p);
    m->ftow = le_get_i32(p + 4);
    m->week = le_get_i16(p + 8);
    m->leap_s = (int8_t)p[10];
    m->valid = p[11];
    m->t_acc = le_get_u32(p + 12);
    return true;
}

bool ubx_decode_nav_relposned(const uint8_t *p, uint16_t len, ubx_nav_relposned_t *m)
{
    memset(m, 0, sizeof(*m));
    m->version = p[0];
    if (m->version == 0) {
        /* protocol 20.x (M8P) version 0, 40 bytes */
        if (len < 40) return false;
        m->ref_station_id = le_get_u16(p + 2);
        m->itow = le_get_u32(p + 4);
        m->rel_pos_n = le_get_i32(p + 8);
        m->rel_pos_e = le_get_i32(p + 12);
        m->rel_pos_d = le_get_i32(p + 16);
        m->acc_n = le_get_u32(p + 24);
        m->acc_e = le_get_u32(p + 28);
        m->acc_d = le_get_u32(p + 32);
        m->flags = le_get_u32(p + 36);
        return true;
    }
    if (len < 64) return false;
    m->ref_station_id = le_get_u16(p + 2);
    m->itow = le_get_u32(p + 4);
    m->rel_pos_n = le_get_i32(p + 8);
    m->rel_pos_e = le_get_i32(p + 12);
    m->rel_pos_d = le_get_i32(p + 16);
    m->rel_pos_length = le_get_i32(p + 20);
    m->rel_pos_heading = le_get_i32(p + 24);
    m->acc_n = le_get_u32(p + 36);
    m->acc_e = le_get_u32(p + 40);
    m->acc_d = le_get_u32(p + 44);
    m->acc_length = le_get_u32(p + 48);
    m->acc_heading = le_get_u32(p + 52);
    m->flags = le_get_u32(p + 60);
    return true;
}

bool ubx_decode_nav_status(const uint8_t *p, uint16_t len, ubx_nav_status_t *m)
{
    if (len < 16) return false;
    m->itow = le_get_u32(p);
    m->gps_fix = p[4];
    m->flags = p[5];
    m->fix_stat = p[6];
    m->flags2 = p[7];
    m->ttff = le_get_u32(p + 8);
    m->msss = le_get_u32(p + 12);
    return true;
}

bool ubx_decode_mon_ver(const uint8_t *p, uint16_t len, ubx_mon_ver_t *m)
{
    if (len < 40) return false;
    memset(m, 0, sizeof(*m));
    memcpy(m->sw_version, p, 30);
    memcpy(m->hw_version, p + 30, 10);
    int n = (len - 40) / 30;
    if (n > UBX_MON_VER_MAX_EXT) n = UBX_MON_VER_MAX_EXT;
    for (int i = 0; i < n; i++) memcpy(m->extension[i], p + 40 + 30 * i, 30);
    m->num_ext = n;
    return true;
}

bool ubx_decode_cfg_prt(const uint8_t *p, uint16_t len, ubx_cfg_prt_t *m)
{
    if (len < 20) return false;
    m->port_id = p[0];
    m->tx_ready = le_get_u16(p + 2);
    m->mode = le_get_u32(p + 4);
    m->baudrate = le_get_u32(p + 8);
    m->in_proto_mask = le_get_u16(p + 12);
    m->out_proto_mask = le_get_u16(p + 14);
    m->flags = le_get_u16(p + 16);
    return true;
}

bool ubx_decode_cfg_rate(const uint8_t *p, uint16_t len, ubx_cfg_rate_t *m)
{
    if (len < 6) return false;
    m->meas_rate_ms = le_get_u16(p);
    m->nav_rate = le_get_u16(p + 2);
    m->time_ref = le_get_u16(p + 4);
    return true;
}

bool ubx_decode_ack(const uint8_t *p, uint16_t len, ubx_ack_t *m)
{
    if (len < 2) return false;
    m->cls_id = p[0];
    m->msg_id = p[1];
    return true;
}

bool ubx_decode_rxm_rtcm(const uint8_t *p, uint16_t len, ubx_rxm_rtcm_t *m)
{
    if (len < 8) return false;
    m->version = p[0];
    m->flags = p[1];
    m->sub_type = le_get_u16(p + 2);
    m->ref_station = le_get_u16(p + 4);
    m->msg_type = le_get_u16(p + 6);
    return true;
}

/* ------------------------------------------------------------------ */
/* encoders                                                           */
/* ------------------------------------------------------------------ */
size_t ubx_build_cfg_prt(uint8_t *out, size_t out_size, const ubx_cfg_prt_t *prt)
{
    uint8_t p[20];
    memset(p, 0, sizeof(p));
    p[0] = prt->port_id;
    le_put_u16(p + 2, prt->tx_ready);
    le_put_u32(p + 4, prt->mode);
    le_put_u32(p + 8, prt->baudrate);
    le_put_u16(p + 12, prt->in_proto_mask);
    le_put_u16(p + 14, prt->out_proto_mask);
    le_put_u16(p + 16, prt->flags);
    return ubx_frame(UBX_CLASS_CFG, UBX_CFG_PRT, p, sizeof(p), out, out_size);
}

size_t ubx_build_cfg_rate(uint8_t *out, size_t out_size, uint16_t meas_ms, uint16_t nav_rate, uint16_t time_ref)
{
    uint8_t p[6];
    le_put_u16(p, meas_ms);
    le_put_u16(p + 2, nav_rate);
    le_put_u16(p + 4, time_ref);
    return ubx_frame(UBX_CLASS_CFG, UBX_CFG_RATE, p, sizeof(p), out, out_size);
}

/* 3-byte form: applies to the port the command arrives on (what ArduPilot sends) */
size_t ubx_build_cfg_msg(uint8_t *out, size_t out_size, uint8_t cls, uint8_t id, uint8_t rate)
{
    uint8_t p[3] = { cls, id, rate };
    return ubx_frame(UBX_CLASS_CFG, UBX_CFG_MSG, p, sizeof(p), out, out_size);
}

size_t ubx_build_cfg_cfg_save(uint8_t *out, size_t out_size)
{
    uint8_t p[12];
    le_put_u32(p, 0);                 /* clearMask */
    le_put_u32(p + 4, UBX_SAVE_CFG_ALL); /* saveMask */
    le_put_u32(p + 8, 0);             /* loadMask */
    return ubx_frame(UBX_CLASS_CFG, UBX_CFG_CFG, p, sizeof(p), out, out_size);
}

uint8_t ubx_key_size(uint32_t key)
{
    switch ((key >> 28) & 0x7) {
    case 1: case 2: return 1;
    case 3: return 2;
    case 4: return 4;
    case 5: return 8;
    default: return 0;
    }
}

size_t ubx_build_valset(uint8_t *out, size_t out_size, uint8_t layers, uint32_t key, uint64_t value)
{
    uint8_t p[4 + 4 + 8];
    uint8_t vs = ubx_key_size(key);
    if (vs == 0) return 0;
    p[0] = 0;          /* version 0: no transaction */
    p[1] = layers;
    p[2] = 0; p[3] = 0;
    le_put_u32(p + 4, key);
    for (int i = 0; i < vs; i++) p[8 + i] = (uint8_t)(value >> (8 * i));
    return ubx_frame(UBX_CLASS_CFG, UBX_CFG_VALSET, p, (uint16_t)(8 + vs), out, out_size);
}

size_t ubx_build_valget(uint8_t *out, size_t out_size, uint8_t layer, uint32_t key)
{
    uint8_t p[8];
    p[0] = 0;
    p[1] = layer;      /* 0 RAM, 1 BBR, 2 flash, 7 default */
    p[2] = 0; p[3] = 0;
    le_put_u32(p + 4, key);
    return ubx_frame(UBX_CLASS_CFG, UBX_CFG_VALGET, p, sizeof(p), out, out_size);
}

size_t ubx_build_poll(uint8_t *out, size_t out_size, uint8_t cls, uint8_t id)
{
    return ubx_frame(cls, id, NULL, 0, out, out_size);
}

/* $PUBX,41,portId,inProto,outProto,baudrate,autobauding*CS */
size_t ubx_build_pubx41(char *out, size_t out_size, uint8_t port_id, uint16_t in_proto,
                        uint16_t out_proto, uint32_t baud)
{
    char body[64];
    int n = snprintf(body, sizeof(body), "PUBX,41,%u,%04X,%04X,%lu,0",
                     (unsigned)port_id, (unsigned)in_proto, (unsigned)out_proto, (unsigned long)baud);
    if (n <= 0 || (size_t)n >= sizeof(body)) return 0;
    uint8_t ck = 0;
    for (int i = 0; i < n; i++) ck ^= (uint8_t)body[i];
    int m = snprintf(out, out_size, "$%s*%02X\r\n", body, ck);
    if (m <= 0 || (size_t)m >= out_size) return 0;
    return (size_t)m;
}
