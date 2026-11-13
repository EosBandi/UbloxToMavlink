/*
 * mavlink_min.c - minimal MAVLink v2 encoder/decoder (see mavlink_min.h)
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
#include "mavlink_min.h"
#include <string.h>

/* ------------------------------------------------------------------ */
/* little endian helpers                                              */
/* ------------------------------------------------------------------ */
void le_put_u16(uint8_t *b, uint16_t v) { b[0] = v & 0xFF; b[1] = v >> 8; }
void le_put_u32(uint8_t *b, uint32_t v) { for (int i = 0; i < 4; i++) b[i] = (uint8_t)(v >> (8 * i)); }
void le_put_u64(uint8_t *b, uint64_t v) { for (int i = 0; i < 8; i++) b[i] = (uint8_t)(v >> (8 * i)); }
void le_put_i32(uint8_t *b, int32_t v)  { le_put_u32(b, (uint32_t)v); }
void le_put_f32(uint8_t *b, float v)    { uint32_t u; memcpy(&u, &v, 4); le_put_u32(b, u); }
uint16_t le_get_u16(const uint8_t *b) { return (uint16_t)(b[0] | (b[1] << 8)); }
uint32_t le_get_u32(const uint8_t *b) { return (uint32_t)b[0] | ((uint32_t)b[1] << 8) | ((uint32_t)b[2] << 16) | ((uint32_t)b[3] << 24); }
uint64_t le_get_u64(const uint8_t *b) { return (uint64_t)le_get_u32(b) | ((uint64_t)le_get_u32(b + 4) << 32); }
int32_t  le_get_i32(const uint8_t *b) { return (int32_t)le_get_u32(b); }
int16_t  le_get_i16(const uint8_t *b) { return (int16_t)le_get_u16(b); }
float    le_get_f32(const uint8_t *b) { uint32_t u = le_get_u32(b); float f; memcpy(&f, &u, 4); return f; }

/* ------------------------------------------------------------------ */
/* CRC                                                                */
/* ------------------------------------------------------------------ */
void mav_crc_init(uint16_t *crc) { *crc = 0xFFFF; }

void mav_crc_accumulate(uint8_t data, uint16_t *crc)
{
    uint8_t tmp = data ^ (uint8_t)(*crc & 0xFF);
    tmp ^= (uint8_t)(tmp << 4);
    *crc = (uint16_t)((*crc >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4));
}

uint16_t mav_crc_calculate(const uint8_t *buf, size_t len)
{
    uint16_t crc;
    mav_crc_init(&crc);
    while (len--) mav_crc_accumulate(*buf++, &crc);
    return crc;
}

/* ------------------------------------------------------------------ */
/* message table                                                      */
/* ------------------------------------------------------------------ */
struct msg_info { uint32_t id; uint8_t crc_extra; uint8_t len; };

static const struct msg_info msg_table[] = {
    { MAVLINK_MSG_ID_HEARTBEAT,        50,   9 },
    { MAVLINK_MSG_ID_SYSTEM_TIME,     137,  12 },
    { MAVLINK_MSG_ID_GPS_INJECT_DATA, 250, 113 },
    { MAVLINK_MSG_ID_GPS_INPUT,       151,  65 },
    { MAVLINK_MSG_ID_GPS_RTCM_DATA,    35, 182 },
    { MAVLINK_MSG_ID_STATUSTEXT,       83,  54 },
};

static const struct msg_info *find_msg(uint32_t msgid)
{
    for (size_t i = 0; i < sizeof(msg_table) / sizeof(msg_table[0]); i++)
        if (msg_table[i].id == msgid) return &msg_table[i];
    return NULL;
}

int mav_crc_extra(uint32_t msgid) { const struct msg_info *m = find_msg(msgid); return m ? m->crc_extra : -1; }
int mav_msg_len(uint32_t msgid)   { const struct msg_info *m = find_msg(msgid); return m ? m->len : -1; }

/* ------------------------------------------------------------------ */
/* encoder                                                            */
/* ------------------------------------------------------------------ */
size_t mav_pack_frame(mav_channel_t *ch, uint32_t msgid, const uint8_t *payload,
                      size_t len, uint8_t *out, size_t out_size)
{
    int extra = mav_crc_extra(msgid);
    if (extra < 0 || len > MAV_MAX_PAYLOAD) return 0;

    /* MAVLink 2: trim trailing zero bytes, keep at least one byte */
    size_t plen = len;
    while (plen > 1 && payload[plen - 1] == 0) plen--;

    if (out_size < MAV_V2_HDR_LEN + plen + 2) return 0;

    out[0] = MAV_STX_V2;
    out[1] = (uint8_t)plen;
    out[2] = 0;                 /* incompat_flags */
    out[3] = 0;                 /* compat_flags */
    out[4] = ch->seq++;
    out[5] = ch->sysid;
    out[6] = ch->compid;
    out[7] = (uint8_t)(msgid & 0xFF);
    out[8] = (uint8_t)((msgid >> 8) & 0xFF);
    out[9] = (uint8_t)((msgid >> 16) & 0xFF);
    memcpy(&out[10], payload, plen);

    uint16_t crc = mav_crc_calculate(&out[1], MAV_V2_HDR_LEN - 1 + plen);
    mav_crc_accumulate((uint8_t)extra, &crc);
    le_put_u16(&out[MAV_V2_HDR_LEN + plen], crc);
    return MAV_V2_HDR_LEN + plen + 2;
}

size_t mav_pack_heartbeat(mav_channel_t *ch, const mav_heartbeat_t *hb,
                          uint8_t *out, size_t out_size)
{
    uint8_t p[9];
    le_put_u32(&p[0], hb->custom_mode);
    p[4] = hb->type;
    p[5] = hb->autopilot;
    p[6] = hb->base_mode;
    p[7] = hb->system_status;
    p[8] = hb->mavlink_version;
    return mav_pack_frame(ch, MAVLINK_MSG_ID_HEARTBEAT, p, sizeof(p), out, out_size);
}

/*
 * GPS_INPUT wire layout (65 bytes, fields sorted by size, yaw is an extension):
 *  0  time_usec          uint64
 *  8  time_week_ms       uint32
 * 12  lat                int32   degE7
 * 16  lon                int32   degE7
 * 20  alt                float   m (AMSL)
 * 24  hdop               float
 * 28  vdop               float
 * 32  vn                 float   m/s
 * 36  ve                 float   m/s
 * 40  vd                 float   m/s
 * 44  speed_accuracy     float   m/s
 * 48  horiz_accuracy     float   m
 * 52  vert_accuracy      float   m
 * 56  ignore_flags       uint16
 * 58  time_week          uint16
 * 60  gps_id             uint8
 * 61  fix_type           uint8
 * 62  satellites_visible uint8
 * 63  yaw                uint16  cdeg, 0 = unavailable, 36000 = north
 */
size_t mav_pack_gps_input(mav_channel_t *ch, const mav_gps_input_t *gi,
                          uint8_t *out, size_t out_size)
{
    uint8_t p[65];
    le_put_u64(&p[0],  gi->time_usec);
    le_put_u32(&p[8],  gi->time_week_ms);
    le_put_i32(&p[12], gi->lat);
    le_put_i32(&p[16], gi->lon);
    le_put_f32(&p[20], gi->alt);
    le_put_f32(&p[24], gi->hdop);
    le_put_f32(&p[28], gi->vdop);
    le_put_f32(&p[32], gi->vn);
    le_put_f32(&p[36], gi->ve);
    le_put_f32(&p[40], gi->vd);
    le_put_f32(&p[44], gi->speed_accuracy);
    le_put_f32(&p[48], gi->horiz_accuracy);
    le_put_f32(&p[52], gi->vert_accuracy);
    le_put_u16(&p[56], gi->ignore_flags);
    le_put_u16(&p[58], gi->time_week);
    p[60] = gi->gps_id;
    p[61] = gi->fix_type;
    p[62] = gi->satellites_visible;
    le_put_u16(&p[63], gi->yaw);
    return mav_pack_frame(ch, MAVLINK_MSG_ID_GPS_INPUT, p, sizeof(p), out, out_size);
}

/* STATUSTEXT: severity u8 @0, text char[50] @1, id u16 @51, chunk_seq u8 @53 */
size_t mav_pack_statustext(mav_channel_t *ch, uint8_t severity, const char *text,
                           uint8_t *out, size_t out_size)
{
    uint8_t p[54];
    memset(p, 0, sizeof(p));
    p[0] = severity;
    strncpy((char *)&p[1], text, 50);
    return mav_pack_frame(ch, MAVLINK_MSG_ID_STATUSTEXT, p, sizeof(p), out, out_size);
}

/* ------------------------------------------------------------------ */
/* parser                                                             */
/* ------------------------------------------------------------------ */
enum { PS_IDLE = 0, PS_HEADER, PS_BODY };

void mav_parser_init(mav_parser_t *p) { memset(p, 0, sizeof(*p)); }

static bool finish_frame(mav_parser_t *p)
{
    const uint8_t *b = p->buf;
    mav_message_t *m = &p->msg;
    bool v2 = (b[0] == MAV_STX_V2);
    size_t hdr = v2 ? MAV_V2_HDR_LEN : 6;
    size_t plen = b[1];

    m->magic = b[0];
    m->len = (uint8_t)plen;
    if (v2) {
        m->incompat_flags = b[2];
        m->compat_flags = b[3];
        m->seq = b[4];
        m->sysid = b[5];
        m->compid = b[6];
        m->msgid = (uint32_t)b[7] | ((uint32_t)b[8] << 8) | ((uint32_t)b[9] << 16);
    } else {
        m->incompat_flags = 0;
        m->compat_flags = 0;
        m->seq = b[2];
        m->sysid = b[3];
        m->compid = b[4];
        m->msgid = b[5];
        p->rx_v1++;
    }

    int extra = mav_crc_extra(m->msgid);
    if (extra < 0) {
        p->rx_unknown++;
        m->crc_ok = false;
        return false;
    }
    uint16_t crc = mav_crc_calculate(&b[1], hdr - 1 + plen);
    mav_crc_accumulate((uint8_t)extra, &crc);
    uint16_t rx_crc = le_get_u16(&b[hdr + plen]);
    if (crc != rx_crc) {
        p->rx_crc_err++;
        m->crc_ok = false;
        return false;
    }
    memset(m->payload, 0, sizeof(m->payload));
    memcpy(m->payload, &b[hdr], plen);
    m->crc_ok = true;
    p->rx_ok++;
    return true;
}

bool mav_parse_byte(mav_parser_t *p, uint8_t c)
{
    switch (p->state) {
    case PS_IDLE:
        if (c == MAV_STX_V2 || c == MAV_STX_V1) {
            p->buf[0] = c;
            p->idx = 1;
            p->state = PS_HEADER;
        }
        return false;

    case PS_HEADER:
        p->buf[p->idx++] = c;
        if (p->buf[0] == MAV_STX_V2) {
            if (p->idx == MAV_V2_HDR_LEN) {
                size_t sig = (p->buf[2] & 0x01) ? MAV_SIG_LEN : 0;
                p->need = (uint16_t)(MAV_V2_HDR_LEN + p->buf[1] + 2 + sig);
                p->state = PS_BODY;
            }
        } else {
            if (p->idx == 6) {
                p->need = (uint16_t)(6 + p->buf[1] + 2);
                p->state = PS_BODY;
            }
        }
        return false;

    case PS_BODY:
        p->buf[p->idx++] = c;
        if (p->idx >= p->need) {
            p->state = PS_IDLE;
            return finish_frame(p);
        }
        return false;
    }
    p->state = PS_IDLE;
    return false;
}

/* ------------------------------------------------------------------ */
/* decoders                                                           */
/* ------------------------------------------------------------------ */
void mav_decode_heartbeat(const mav_message_t *m, mav_heartbeat_t *hb)
{
    hb->custom_mode = le_get_u32(&m->payload[0]);
    hb->type = m->payload[4];
    hb->autopilot = m->payload[5];
    hb->base_mode = m->payload[6];
    hb->system_status = m->payload[7];
    hb->mavlink_version = m->payload[8];
}

/* GPS_RTCM_DATA: flags u8 @0, len u8 @1, data[180] @2 */
void mav_decode_gps_rtcm_data(const mav_message_t *m, mav_gps_rtcm_data_t *d)
{
    d->flags = m->payload[0];
    d->len = m->payload[1];
    if (d->len > MAVLINK_MSG_GPS_RTCM_DATA_FIELD_DATA_LEN) d->len = MAVLINK_MSG_GPS_RTCM_DATA_FIELD_DATA_LEN;
    memcpy(d->data, &m->payload[2], MAVLINK_MSG_GPS_RTCM_DATA_FIELD_DATA_LEN);
}

/* GPS_INJECT_DATA: target_system u8 @0, target_component u8 @1, len u8 @2, data[110] @3 */
void mav_decode_gps_inject_data(const mav_message_t *m, mav_gps_inject_data_t *d)
{
    d->target_system = m->payload[0];
    d->target_component = m->payload[1];
    d->len = m->payload[2];
    if (d->len > MAVLINK_MSG_GPS_INJECT_DATA_FIELD_DATA_LEN) d->len = MAVLINK_MSG_GPS_INJECT_DATA_FIELD_DATA_LEN;
    memcpy(d->data, &m->payload[3], MAVLINK_MSG_GPS_INJECT_DATA_FIELD_DATA_LEN);
}
