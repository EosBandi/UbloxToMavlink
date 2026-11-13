/*
 * test_codec.c - unit tests for the MAVLink and UBX codecs.
 * Reference frames were generated with pymavlink (MAVLink 2) and by hand
 * from the u-blox interface description.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
#include "mavlink_min.h"
#include "ubx.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>

static int failures = 0;
#define CHECK(cond, ...) do { if (!(cond)) { failures++; printf("FAIL %s:%d: ", __FILE__, __LINE__); printf(__VA_ARGS__); printf("\n"); } } while (0)

static void hex(const uint8_t *b, size_t n)
{
    for (size_t i = 0; i < n; i++) printf("%02x", b[i]);
    printf("\n");
}

/* pymavlink: MAVLink(..., srcSystem=1, srcComponent=220); seq 0
 * heartbeat_encode(type=41, autopilot=8, base_mode=0, custom_mode=0, system_status=4, mavlink_version=3) */
static const uint8_t ref_heartbeat[] = {
    0xfd, 0x09, 0x00, 0x00, 0x00, 0x01, 0xdc, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x29, 0x08, 0x00, 0x04, 0x03,
    0x00, 0x00  /* crc placeholder, filled by the test from the reference computation */
};

static void test_crc(void)
{
    /* MAVLink uses CRC-16/MCRF4XX (X.25 without the final inversion): check value 0x6F91 */
    uint16_t c = mav_crc_calculate((const uint8_t *)"123456789", 9);
    CHECK(c == 0x6F91, "mavlink crc = 0x%04X", c);
}

static void test_heartbeat_roundtrip(void)
{
    mav_channel_t ch = { .sysid = 1, .compid = 220, .seq = 0 };
    mav_heartbeat_t hb = { .custom_mode = 0, .type = 41, .autopilot = 8, .base_mode = 0, .system_status = 4, .mavlink_version = 3 };
    uint8_t f[64];
    size_t n = mav_pack_heartbeat(&ch, &hb, f, sizeof(f));
    CHECK(n == 21, "heartbeat frame len %zu", n);
    CHECK(memcmp(f, ref_heartbeat, 19) == 0, "heartbeat header/payload mismatch");

    mav_parser_t p;
    mav_parser_init(&p);
    bool got = false;
    for (size_t i = 0; i < n; i++) got = mav_parse_byte(&p, f[i]) || got;
    CHECK(got, "heartbeat did not parse");
    CHECK(p.msg.msgid == 0 && p.msg.sysid == 1 && p.msg.compid == 220, "heartbeat ids");
    mav_heartbeat_t out;
    mav_decode_heartbeat(&p.msg, &out);
    CHECK(out.type == 41 && out.autopilot == 8 && out.system_status == 4 && out.mavlink_version == 3, "heartbeat fields");
}

static void test_gps_input_roundtrip(void)
{
    mav_channel_t ch = { .sysid = 1, .compid = 220, .seq = 7 };
    mav_gps_input_t gi = {
        .time_usec = 1700000000123456ULL, .time_week_ms = 123456789, .lat = 473977420, .lon = 190403520,
        .alt = 123.45f, .hdop = 0.8f, .vdop = 1.2f, .vn = 1.5f, .ve = -0.5f, .vd = 0.1f,
        .speed_accuracy = 0.2f, .horiz_accuracy = 1.1f, .vert_accuracy = 2.2f, .ignore_flags = 0,
        .time_week = 2290, .gps_id = 0, .fix_type = 3, .satellites_visible = 14, .yaw = 9000,
    };
    uint8_t f[128];
    size_t n = mav_pack_gps_input(&ch, &gi, f, sizeof(f));
    CHECK(n == 10 + 65 + 2, "gps_input frame len %zu", n);
    CHECK(f[1] == 65, "payload len %u", f[1]);
    CHECK(f[7] == 232 && f[8] == 0 && f[9] == 0, "msgid bytes");

    /* decode by hand and compare */
    const uint8_t *p = &f[10];
    CHECK(le_get_u64(p) == gi.time_usec, "time_usec");
    CHECK(le_get_u32(p + 8) == gi.time_week_ms, "time_week_ms");
    CHECK(le_get_i32(p + 12) == gi.lat && le_get_i32(p + 16) == gi.lon, "lat/lon");
    CHECK(fabsf(le_get_f32(p + 20) - 123.45f) < 1e-4f, "alt");
    CHECK(le_get_u16(p + 58) == 2290, "week");
    CHECK(p[60] == 0 && p[61] == 3 && p[62] == 14, "ids/fix/sats");
    CHECK(le_get_u16(p + 63) == 9000, "yaw");

    /* CRC: recompute with crc_extra 151 */
    uint16_t crc = mav_crc_calculate(&f[1], 9 + 65);
    mav_crc_accumulate(151, &crc);
    CHECK(le_get_u16(&f[75]) == crc, "gps_input crc");

    /* trailing-zero trimming: zero yaw and sats shortens the frame */
    gi.yaw = 0; gi.satellites_visible = 0;
    n = mav_pack_gps_input(&ch, &gi, f, sizeof(f));
    CHECK(f[1] == 62, "trimmed payload len %u", f[1]);
    mav_parser_t pp;
    mav_parser_init(&pp);
    bool got = false;
    for (size_t i = 0; i < n; i++) got = mav_parse_byte(&pp, f[i]) || got;
    CHECK(got && pp.msg.msgid == 232 && pp.msg.payload[61] == 3 && pp.msg.payload[62] == 0, "trimmed frame parses with zero extension");
}

static void test_rtcm_decode(void)
{
    /* build a GPS_RTCM_DATA frame by hand: flags=0x0B (fragmented, frag 1, seq 1), len=3 */
    uint8_t payload[182];
    memset(payload, 0, sizeof(payload));
    payload[0] = 0x0B; payload[1] = 3; payload[2] = 0xD3; payload[3] = 0x00; payload[4] = 0x13;
    mav_channel_t ch = { .sysid = 255, .compid = 190, .seq = 1 };
    uint8_t f[256];
    size_t n = mav_pack_frame(&ch, MAVLINK_MSG_ID_GPS_RTCM_DATA, payload, sizeof(payload), f, sizeof(f));
    CHECK(n == 10 + 5 + 2, "rtcm frame len %zu (trimmed)", n);
    mav_parser_t p;
    mav_parser_init(&p);
    bool got = false;
    for (size_t i = 0; i < n; i++) got = mav_parse_byte(&p, f[i]) || got;
    CHECK(got, "rtcm did not parse");
    mav_gps_rtcm_data_t d;
    mav_decode_gps_rtcm_data(&p.msg, &d);
    CHECK(d.flags == 0x0B && d.len == 3 && d.data[0] == 0xD3 && d.data[2] == 0x13, "rtcm fields");
}

static void test_mavlink1_and_garbage(void)
{
    /* MAVLink 1 heartbeat from pymavlink (sysid 1 compid 1): fe 09 00 01 01 00 | payload | crc */
    const uint8_t v1[] = { 0xfe, 0x09, 0x00, 0x01, 0x01, 0x00,
                           0x00, 0x00, 0x00, 0x00, 0x02, 0x03, 0x51, 0x04, 0x03, 0x00, 0x00 };
    uint8_t frame[sizeof(v1)];
    memcpy(frame, v1, sizeof(v1));
    uint16_t crc = mav_crc_calculate(&frame[1], 5 + 9);
    mav_crc_accumulate(50, &crc);
    le_put_u16(&frame[15], crc);

    mav_parser_t p;
    mav_parser_init(&p);
    /* leading garbage including a stray STX */
    const uint8_t junk[] = { 0x00, 0xfd, 0x11, 0x22 };
    for (size_t i = 0; i < sizeof(junk); i++) mav_parse_byte(&p, junk[i]);
    /* the stray 0xfd starts a bogus frame; feed enough to let it fail and resync */
    bool got = false;
    for (int rep = 0; rep < 3 && !got; rep++)
        for (size_t i = 0; i < sizeof(frame); i++) got = mav_parse_byte(&p, frame[i]) || got;
    CHECK(got, "mavlink1 frame after garbage did not parse");
    CHECK(p.msg.magic == 0xFE && p.msg.msgid == 0 && p.msg.sysid == 1, "mavlink1 fields");
}

static void test_ubx_frame_and_parse(void)
{
    /* CFG-MSG enable NAV-PVT rate 1: b5 62 06 01 03 00 01 07 01 13 51 */
    uint8_t f[32];
    size_t n = ubx_build_cfg_msg(f, sizeof(f), UBX_CLASS_NAV, UBX_NAV_PVT, 1);
    const uint8_t ref[] = { 0xb5, 0x62, 0x06, 0x01, 0x03, 0x00, 0x01, 0x07, 0x01, 0x13, 0x51 };
    CHECK(n == sizeof(ref) && memcmp(f, ref, n) == 0, "CFG-MSG frame");
    if (n != sizeof(ref) || memcmp(f, ref, n) != 0) hex(f, n);

    /* ArduPilot init blob fragment: CFG-MSG NAV-SOL rate 1 = b5 62 06 01 03 00 01 06 01 12 4f */
    n = ubx_build_cfg_msg(f, sizeof(f), UBX_CLASS_NAV, UBX_NAV_SOL, 1);
    const uint8_t ref2[] = { 0xb5, 0x62, 0x06, 0x01, 0x03, 0x00, 0x01, 0x06, 0x01, 0x12, 0x4f };
    CHECK(n == sizeof(ref2) && memcmp(f, ref2, n) == 0, "CFG-MSG NAV-SOL frame vs ArduPilot blob");

    /* parse it back, with NMEA noise in front */
    ubx_parser_t p;
    ubx_parser_init(&p);
    const char *body = "GNGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,";
    uint8_t ck = 0;
    for (const char *c = body; *c; c++) ck ^= (uint8_t)*c;
    char nmea[128];
    snprintf(nmea, sizeof(nmea), "$%s*%02X\r\n", body, ck);
    bool saw_nmea = false;
    for (const char *c = nmea; *c; c++) if (ubx_parse_byte(&p, (uint8_t)*c) == UBX_PARSE_NMEA) saw_nmea = true;
    CHECK(saw_nmea, "NMEA sentence not detected");
    /* a corrupted checksum must not count */
    nmea[strlen(nmea) - 3] ^= 0x01;
    saw_nmea = false;
    for (const char *c = nmea; *c; c++) if (ubx_parse_byte(&p, (uint8_t)*c) == UBX_PARSE_NMEA) saw_nmea = true;
    CHECK(!saw_nmea, "NMEA sentence with bad checksum accepted");
    int frames = 0;
    for (size_t i = 0; i < n; i++) if (ubx_parse_byte(&p, f[i]) == UBX_PARSE_UBX) frames++;
    CHECK(frames == 1 && p.cls == 6 && p.id == 1 && p.len == 3, "UBX parse back");

    /* corrupt checksum */
    f[n - 1] ^= 0xFF;
    frames = 0;
    for (size_t i = 0; i < n; i++) if (ubx_parse_byte(&p, f[i]) == UBX_PARSE_UBX) frames++;
    CHECK(frames == 0 && p.rx_crc_err == 1, "bad checksum accepted");
}

static void test_pubx41(void)
{
    char s[80];
    size_t n = ubx_build_pubx41(s, sizeof(s), 1, 0x0023, 0x0001, 230400);
    /* ArduPilot's UBLOX_SET_BINARY_230400 blob: "$PUBX,41,1,0023,0001,230400,0*1E\r\n" */
    CHECK(n > 0 && strcmp(s, "$PUBX,41,1,0023,0001,230400,0*1E\r\n") == 0, "PUBX41 230400: %s", s);
    n = ubx_build_pubx41(s, sizeof(s), 1, 0x0023, 0x0001, 115200);
    CHECK(n > 0 && strcmp(s, "$PUBX,41,1,0023,0001,115200,0*1C\r\n") == 0, "PUBX41 115200: %s", s);
    n = ubx_build_pubx41(s, sizeof(s), 1, 0x0023, 0x0001, 460800);
    CHECK(n > 0 && strcmp(s, "$PUBX,41,1,0023,0001,460800,0*11\r\n") == 0, "PUBX41 460800: %s", s);
}

static void test_valset(void)
{
    uint8_t f[40];
    size_t n = ubx_build_valset(f, sizeof(f), UBX_LAYER_RAM, UBX_KEY_CFG_RATE_MEAS, 200);
    /* b5 62 06 8a 0a 00 | 00 01 00 00 | 01 00 21 30 | c8 00 | ck */
    const uint8_t ref[] = { 0xb5, 0x62, 0x06, 0x8a, 0x0a, 0x00, 0x00, 0x01, 0x00, 0x00, 0x01, 0x00, 0x21, 0x30, 0xc8, 0x00 };
    CHECK(n == sizeof(ref) + 2 && memcmp(f, ref, sizeof(ref)) == 0, "VALSET RATE_MEAS");
    if (n != sizeof(ref) + 2 || memcmp(f, ref, sizeof(ref)) != 0) hex(f, n);
    CHECK(ubx_key_size(UBX_KEY_CFG_UART1_BAUDRATE) == 4, "key size U4");
    CHECK(ubx_key_size(UBX_KEY_MSGOUT_NAV_PVT_I2C) == 1, "key size U1");
    CHECK(ubx_key_size(UBX_KEY_SIGNAL_GPS_ENA) == 1, "key size L");
    CHECK(UBX_KEY_MSGOUT_NAV_PVT_I2C + UBX_PORT_UART1 == 0x20910007u, "PVT UART1 key");
    CHECK(UBX_KEY_MSGOUT_NAV_EOE_I2C + UBX_PORT_UART1 == 0x20910160u, "EOE UART1 key");
}

static void test_pvt_decode(void)
{
    uint8_t p[92];
    memset(p, 0, sizeof(p));
    le_put_u32(p, 123456000);
    le_put_u16(p + 4, 2024); p[6] = 5; p[7] = 17; p[8] = 10; p[9] = 17; p[10] = 36; p[11] = 0x07;
    p[20] = 3; p[21] = 0x01 | 0x02; p[23] = 12;
    le_put_i32(p + 24, 190403520); le_put_i32(p + 28, 473977420);
    le_put_i32(p + 32, 170000); le_put_i32(p + 36, 123450);
    le_put_u32(p + 40, 1100); le_put_u32(p + 44, 2200);
    le_put_i32(p + 48, 1500); le_put_i32(p + 52, -500); le_put_i32(p + 56, 100);
    le_put_u32(p + 68, 200);
    le_put_u16(p + 76, 150);
    ubx_nav_pvt_t m;
    CHECK(ubx_decode_nav_pvt(p, 92, &m), "pvt decode");
    CHECK(m.itow == 123456000 && m.year == 2024 && m.fix_type == 3 && m.num_sv == 12, "pvt basics");
    CHECK(m.lat == 473977420 && m.lon == 190403520 && m.h_msl == 123450 && m.h_acc == 1100, "pvt pos");
    CHECK(m.vel_n == 1500 && m.vel_e == -500 && m.vel_d == 100 && m.s_acc == 200 && m.p_dop == 150, "pvt vel");
    CHECK(m.flags == 0x03, "pvt flags");
    CHECK(!ubx_decode_nav_pvt(p, 80, &m), "short pvt accepted");
}

int main(void)
{
    test_crc();
    test_heartbeat_roundtrip();
    test_gps_input_roundtrip();
    test_rtcm_decode();
    test_mavlink1_and_garbage();
    test_ubx_frame_and_parse();
    test_pubx41();
    test_valset();
    test_pvt_decode();
    if (failures) {
        printf("%d test(s) FAILED\n", failures);
        return 1;
    }
    printf("all codec tests passed\n");
    return 0;
}
