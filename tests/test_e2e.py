#!/usr/bin/env python3
"""
End-to-end test for ubx2mav using two pseudo terminals.

One pty pair plays a u-blox NEO-M8N (answers MON-VER / CFG polls, ACKs
configuration, streams NAV-PVT/DOP/TIMEGPS/EOE epochs and records injected
RTCM bytes). The other pty pair plays an ArduPilot flight controller using
pymavlink (sends heartbeats, forwards GPS_RTCM_DATA, collects GPS_INPUT).

Usage: test_e2e.py path/to/ubx2mav
Requires: pymavlink (pip install pymavlink)
"""
import os
import pty
import struct
import subprocess
import sys
import threading
import time
import tty

from pymavlink.dialects.v20 import ardupilotmega as mavlink2

BIN = sys.argv[1] if len(sys.argv) > 1 else "build/ubx2mav"
FC_SYSID = 7

# ---------------------------------------------------------------- UBX helpers
def ubx(cls, mid, payload=b""):
    body = bytes([cls, mid]) + struct.pack("<H", len(payload)) + payload
    a = b = 0
    for x in body:
        a = (a + x) & 0xFF
        b = (b + a) & 0xFF
    return b"\xb5\x62" + body + bytes([a, b])


def ack(cls, mid):
    return ubx(0x05, 0x01, bytes([cls, mid]))


class UbxStream:
    def __init__(self):
        self.buf = b""

    def feed(self, data):
        """yield (cls, id, payload) for complete frames; also yields ('raw', bytes) for non-UBX data"""
        self.buf += data
        out = []
        while True:
            i = self.buf.find(b"\xb5\x62")
            if i < 0:
                if self.buf:
                    out.append(("raw", self.buf))
                    self.buf = b""
                break
            if i > 0:
                out.append(("raw", self.buf[:i]))
                self.buf = self.buf[i:]
            if len(self.buf) < 8:
                break
            ln = struct.unpack("<H", self.buf[4:6])[0]
            if len(self.buf) < 8 + ln:
                break
            frame = self.buf[: 8 + ln]
            self.buf = self.buf[8 + ln:]
            out.append((frame[2], frame[3], frame[6:6 + ln]))
        return out


# ---------------------------------------------------------------- GPS simulator
class GpsSim(threading.Thread):
    """
    module = "m8"    NEO-M8N FW3.01 (PROTVER 18): legacy CFG, has NAV-EOE
    module = "m8old" NEO-M8N FW2.01 (PROTVER 15): legacy CFG, NAKs NAV-EOE, never sends EOE
    module = "m10"   MAX-M10S (PROTVER 34): VALSET only, NAKs every legacy CFG message
    """
    def __init__(self, fd, module="m8"):
        super().__init__(daemon=True)
        self.fd = fd
        self.module = module
        self.stream = UbxStream()
        self.rates = {}
        self.valset = {}
        self.rate_ms = 1000
        self.dynmodel = 0
        self.prt_out = 0x03
        self.prt_in = 0x07
        self.rtcm = b""
        self.epochs_sent = 0
        self.itow = 500_000_000  # ms into the week
        self.week = 2290
        self.run_flag = True
        self.log = []
        self.stream_started = False
        self.polls = []
        self.has_eoe = module != "m8old"

    def send(self, data):
        os.write(self.fd, data)

    def run(self):
        while self.run_flag:
            try:
                data = os.read(self.fd, 4096)
            except OSError:
                break
            for item in self.stream.feed(data):
                if item[0] == "raw":
                    raw = item[1]
                    # ignore NMEA ($PUBX...) but record RTCM (0xD3 preamble)
                    if raw.startswith(b"\xd3") or self.rtcm:
                        self.rtcm += raw
                    continue
                self.handle(*item)

    def handle(self, cls, mid, payload):
        self.log.append((cls, mid, len(payload)))
        if cls == 0x0A and mid == 0x04:                       # MON-VER poll
            self.polls.append("MON-VER")
            if self.module == "m10":
                sw = b"ROM SPG 5.10 (7b202e)".ljust(30, b"\0")
                hw = b"000A0000".ljust(10, b"\0")
                ext = b"PROTVER=34.10".ljust(30, b"\0") + b"MOD=MAX-M10S".ljust(30, b"\0") + b"GPS;GLO;GAL;BDS".ljust(30, b"\0")
            elif self.module == "m8old":
                sw = b"2.01 (75331)".ljust(30, b"\0")
                hw = b"00080000".ljust(10, b"\0")
                ext = b"PROTVER 15.00".ljust(30, b"\0") + b"MOD=NEO-M8N-0".ljust(30, b"\0")
            else:
                sw = b"ROM CORE 3.01 (107888)".ljust(30, b"\0")
                hw = b"00080000".ljust(10, b"\0")
                ext = b"PROTVER=18.00".ljust(30, b"\0") + b"MOD=NEO-M8N-0".ljust(30, b"\0") + b"FIS=0xEF4015 (100111)".ljust(30, b"\0")
            self.send(ubx(0x0A, 0x04, sw + hw + ext))
        elif cls == 0x06 and mid == 0x8A:                     # CFG-VALSET (single key)
            if self.module != "m10":
                self.send(ubx(0x05, 0x00, bytes([cls, mid])))
                return
            key = struct.unpack("<I", payload[4:8])[0]
            size = {1: 1, 2: 1, 3: 2, 4: 4, 5: 8}[(key >> 28) & 7]
            value = int.from_bytes(payload[8:8 + size], "little")
            self.valset[key] = value
            if key == 0x30210001:
                self.rate_ms = value
            if key == 0x20110021:
                self.dynmodel = value
            if key == 0x20910007 and value == 1:               # CFG-MSGOUT-UBX_NAV_PVT_UART1
                self.stream_started = True
            self.send(ack(cls, mid))
        elif cls == 0x06 and self.module == "m10":            # M10 has no legacy CFG messages
            self.send(ubx(0x05, 0x00, bytes([cls, mid])))
        elif cls == 0x06 and mid == 0x01 and len(payload) == 3 and payload[:2] == b"\x01\x61" and not self.has_eoe:
            self.send(ubx(0x05, 0x00, bytes([cls, mid])))     # no NAV-EOE on old firmware
        elif cls == 0x06 and mid == 0x00:                     # CFG-PRT
            if len(payload) == 0:
                p = struct.pack("<BBHIIHHHH", 1, 0, 0, 0x8D0, 230400, self.prt_in, self.prt_out, 0, 0)
                self.send(ubx(0x06, 0x00, p))
            else:
                _, _, _, _, baud, self.prt_in, self.prt_out, _, _ = struct.unpack("<BBHIIHHHH", payload)
            self.send(ack(cls, mid))
        elif cls == 0x06 and mid == 0x08:                     # CFG-RATE
            if len(payload) == 0:
                self.send(ubx(0x06, 0x08, struct.pack("<HHH", self.rate_ms, 1, 1)))
            else:
                self.rate_ms, nav, ref = struct.unpack("<HHH", payload)
            self.send(ack(cls, mid))
        elif cls == 0x06 and mid == 0x24:                     # CFG-NAV5
            if len(payload) == 0:
                p = bytearray(36)
                p[2] = self.dynmodel
                p[3] = 3
                p[12] = 5
                self.send(ubx(0x06, 0x24, bytes(p)))
            else:
                mask = struct.unpack("<H", payload[:2])[0]
                if mask & 1:
                    self.dynmodel = payload[2]
            self.send(ack(cls, mid))
        elif cls == 0x06 and mid == 0x01:                     # CFG-MSG
            if len(payload) == 3:
                self.rates[(payload[0], payload[1])] = payload[2]
                if (payload[0], payload[1]) == (0x01, 0x07) and payload[2] == 1:
                    self.stream_started = True
            self.send(ack(cls, mid))
        elif cls == 0x06:
            self.send(ack(cls, mid))

    def epoch(self, fix=3, flags=0x03, sats=14):
        """one navigation epoch: PVT, DOP, TIMEGPS, EOE"""
        self.itow += self.rate_ms
        secs = self.itow // 1000
        # UTC of this iTOW (leap 18 s): week 2290 starts 2023-11-19 00:00:00 GPS
        gps_unix = 315964800 + self.week * 604800 + secs - 18
        t = time.gmtime(gps_unix)
        pvt = struct.pack("<IHBBBBBBIiBBBBiiiiIIiiiiiIIHHI",
                          self.itow, t.tm_year, t.tm_mon, t.tm_mday, t.tm_hour, t.tm_min, t.tm_sec,
                          0x07, 50, 0, fix, flags, 0, sats,
                          190403520, 473977420, 170000, 123450, 1100, 2200,
                          1500, -500, 100, 1581, 12345678, 200, 500000, 150, 0, 0)
        pvt += struct.pack("<ihH", 0, 0, 0)
        assert len(pvt) == 92, len(pvt)
        dop = struct.pack("<IHHHHHHH", self.itow, 180, 150, 90, 120, 80, 60, 50)
        timegps = struct.pack("<IihbBI", self.itow, 0, self.week, 18, 0x07, 50)
        eoe = struct.pack("<I", self.itow)
        frames = ubx(0x01, 0x07, pvt) + ubx(0x01, 0x04, dop) + ubx(0x01, 0x20, timegps)
        if self.has_eoe:
            frames += ubx(0x01, 0x61, eoe)
        self.send(frames)
        self.epochs_sent += 1


# ---------------------------------------------------------------- FC simulator
class FcSim(threading.Thread):
    def __init__(self, fd):
        super().__init__(daemon=True)
        self.fd = fd
        self.mav = mavlink2.MAVLink(self, srcSystem=FC_SYSID, srcComponent=1)
        self.mav.robust_parsing = True
        self.gps_inputs = []
        self.heartbeats = []
        self.statustexts = []
        self.run_flag = True
        self.lock = threading.Lock()

    def write(self, data):           # file-like for MAVLink()
        os.write(self.fd, bytes(data))

    def run(self):
        while self.run_flag:
            try:
                data = os.read(self.fd, 4096)
            except OSError:
                break
            for c in data:
                try:
                    m = self.mav.parse_char(bytes([c]))
                except Exception:
                    m = None
                if m is None:
                    continue
                with self.lock:
                    t = m.get_type()
                    if t == "GPS_INPUT":
                        self.gps_inputs.append((time.monotonic(), m))
                    elif t == "HEARTBEAT":
                        self.heartbeats.append(m)
                    elif t == "STATUSTEXT":
                        self.statustexts.append(m.text)

    def heartbeat(self):
        with self.lock:
            self.mav.heartbeat_send(mavlink2.MAV_TYPE_QUADROTOR, mavlink2.MAV_AUTOPILOT_ARDUPILOTMEGA, 0, 0, 3)

    def rtcm(self, data, seq):
        """send data as fragmented GPS_RTCM_DATA like MAVProxy / Mission Planner"""
        with self.lock:
            if len(data) <= 180:
                self.mav.gps_rtcm_data_send(0, len(data), data.ljust(180, b"\0"))
                return
            frags = [data[i:i + 180] for i in range(0, len(data), 180)]
            if len(frags[-1]) == 180:
                frags.append(b"")
            for i, f in enumerate(frags):
                flags = 1 | (i << 1) | ((seq & 0x1F) << 3)
                self.mav.gps_rtcm_data_send(flags, len(f), f.ljust(180, b"\0"))


# ---------------------------------------------------------------- test
def run_scenario(module, failures):
    print(f"=== scenario: {module} ===")
    gps_master, gps_slave = pty.openpty()
    fc_master, fc_slave = pty.openpty()
    for fd in (gps_master, fc_master):
        tty.setraw(fd)
    gps = GpsSim(gps_master, module)
    fc = FcSim(fc_master)
    gps.start()
    fc.start()

    cmd = [BIN, "--gps-dev", os.ttyname(gps_slave), "--fc", os.ttyname(fc_slave),
           "--stats", "0", "-v", "--eoe-timeout", "40"]
    proc = subprocess.Popen(cmd, stderr=subprocess.PIPE, text=True)
    log_lines = []

    def drain():
        for line in proc.stderr:
            log_lines.append(line.rstrip())
    threading.Thread(target=drain, daemon=True).start()

    def check(cond, msg):
        if not cond:
            failures.append(f"[{module}] {msg}")
            print("FAIL:", msg)

    try:
        # wait for configuration to reach the point where NAV-PVT is enabled
        t0 = time.monotonic()
        while not gps.stream_started and time.monotonic() - t0 < 15:
            fc.heartbeat()
            time.sleep(0.2)
        check(gps.stream_started, "bridge never enabled NAV-PVT (configuration did not run)")
        time.sleep(1.5)   # let the remaining configuration finish

        check(gps.rate_ms == 200, f"measurement rate not set to 200 ms (got {gps.rate_ms})")
        check(gps.dynmodel == 8, f"dynamic model not set to 8 (got {gps.dynmodel})")
        if module == "m10":
            vs = gps.valset
            check(vs.get(0x30210002) == 1 and vs.get(0x20210003) == 0, "RATE-NAV/TIMEREF not set")
            check(vs.get(0x10740002) == 0 and vs.get(0x10740001) == 1, "UART1 OUTPROT not UBX-only")
            check(vs.get(0x10730001) == 1 and vs.get(0x10730004) == 1, "UART1 INPROT lacks UBX|RTCM3X")
            for key, want in {0x20910007: 1, 0x20910039: 1, 0x20910048: 1, 0x20910160: 1,
                              0x2091001b: 0, 0x2091002a: 0, 0x20910043: 0}.items():
                check(vs.get(key) == want, f"VALSET 0x{key:08x} = {vs.get(key)} != {want}")
            check(all(k != 0x20910006 for k in vs), "I2C port keys were written instead of UART1")
        else:
            check(gps.prt_out == 0x01, f"port output protocol not UBX-only (0x{gps.prt_out:04x})")
            check(gps.prt_in & 0x21 == 0x21, f"port input protocol lacks UBX|RTCM3 (0x{gps.prt_in:04x})")
            want_rates = {(1, 7): 1, (1, 4): 1, (1, 0x20): 1, (1, 2): 0, (1, 3): 0, (1, 0x12): 0}
            if gps.has_eoe:
                want_rates[(1, 0x61)] = 1
            for key, want in want_rates.items():
                check(gps.rates.get(key) == want, f"CFG-MSG {key[0]:02x}/{key[1]:02x} rate {gps.rates.get(key)} != {want}")

        # stream 15 epochs at 5 Hz while heart-beating (30 for the no-EOE module so
        # the bridge has time to give up waiting for NAV-EOE and switch to send-on-PVT)
        n_epochs = 15 if gps.has_eoe else 30
        for i in range(n_epochs):
            gps.epoch(fix=3, flags=0x03, sats=14)
            if i % 5 == 0:
                fc.heartbeat()
            time.sleep(0.2)
        time.sleep(0.3)

        with fc.lock:
            gi = list(fc.gps_inputs)
            hbs = list(fc.heartbeats)
        check(len(gi) >= n_epochs - 2, f"expected ~{n_epochs} GPS_INPUT, got {len(gi)}")
        if not gps.has_eoe and len(gi) > 25:
            # after 20 epochs without NAV-EOE the bridge must send straight from NAV-PVT:
            # the GPS_INPUT then arrives well before the 40 ms EOE timeout
            late_gaps = [gi[i + 1][0] - gi[i][0] for i in range(len(gi) - 6, len(gi) - 1)]
            check(all(0.15 < g < 0.25 for g in late_gaps), f"irregular spacing on no-EOE module: {late_gaps}")
        if gi:
            _, m = gi[-1]
            check(m.get_srcSystem() == FC_SYSID, f"GPS_INPUT sysid {m.get_srcSystem()} != adopted {FC_SYSID}")
            check(m.get_srcComponent() == 220, f"compid {m.get_srcComponent()} != 220")
            check(m.fix_type == 4, f"fix_type {m.fix_type} != 4 (3D + diffSoln -> DGPS)")
            check(m.lat == 473977420 and m.lon == 190403520, f"lat/lon {m.lat}/{m.lon}")
            check(abs(m.alt - 123.45) < 1e-3, f"alt {m.alt}")
            check(abs(m.hdop - 0.80) < 1e-3 and abs(m.vdop - 1.20) < 1e-3, f"hdop/vdop {m.hdop}/{m.vdop}")
            check(abs(m.vn - 1.5) < 1e-3 and abs(m.ve + 0.5) < 1e-3 and abs(m.vd - 0.1) < 1e-3, f"vel {m.vn},{m.ve},{m.vd}")
            check(abs(m.speed_accuracy - 0.2) < 1e-3, f"sacc {m.speed_accuracy}")
            check(abs(m.horiz_accuracy - 1.1) < 1e-3 and abs(m.vert_accuracy - 2.2) < 1e-3, f"hacc/vacc {m.horiz_accuracy}/{m.vert_accuracy}")
            check(m.satellites_visible == 14, f"sats {m.satellites_visible}")
            check(m.time_week == 2290, f"week {m.time_week}")
            check(m.time_week_ms == gps.itow, f"itow {m.time_week_ms} != {gps.itow}")
            check(m.ignore_flags == 0, f"ignore_flags {m.ignore_flags}")
            check(m.yaw == 0, f"yaw {m.yaw} should be 0 without RELPOSNED")
            # UTC epoch of the last fix encoded in time_usec
            want_us = (315964800 + 2290 * 604800 + gps.itow // 1000 - 18) * 1_000_000 + (gps.itow % 1000) * 1000
            check(m.time_usec == want_us, f"time_usec {m.time_usec} != {want_us}")
            # latency: GPS_INPUT must follow the epoch quickly (EOE path, not the timeout)
            if len(gi) >= 2:
                gaps = [gi[i + 1][0] - gi[i][0] for i in range(len(gi) - 1)]
                avg = sum(gaps) / len(gaps)
                check(0.15 < avg < 0.25, f"average GPS_INPUT interval {avg:.3f}s not ~0.2s")
            itows = [x[1].time_week_ms for x in gi]
            check(itows == sorted(set(itows)), "duplicate or out-of-order iTOW in GPS_INPUT")
        check(len(hbs) >= 2, f"expected heartbeats from the bridge, got {len(hbs)}")
        if hbs:
            check(hbs[-1].type == 41 and hbs[-1].autopilot == 8, f"heartbeat type/autopilot {hbs[-1].type}/{hbs[-1].autopilot}")

        # no-fix epoch must be reported as fix_type 1, never 0
        gps.epoch(fix=0, flags=0x00, sats=3)
        time.sleep(0.3)
        with fc.lock:
            m = fc.gps_inputs[-1][1] if fc.gps_inputs else None
        check(m is not None and m.fix_type == 1 and m.time_week_ms == gps.itow, f"no-fix epoch fix_type {getattr(m, 'fix_type', None)}")

        # RTK fixed epoch
        gps.epoch(fix=3, flags=0x01 | 0x02 | 0x80, sats=20)
        time.sleep(0.3)
        with fc.lock:
            m = fc.gps_inputs[-1][1]
        check(m.fix_type == 6, f"RTK fixed epoch fix_type {m.fix_type} != 6")

        # RTCM injection: one unfragmented and one 3-fragment message
        rtcm1 = b"\xd3\x00\x13" + bytes(range(19)) + b"\x11\x22\x33"
        rtcm2 = b"\xd3\x01\x90" + bytes((i * 7) & 0xFF for i in range(400)) + b"\xaa\xbb\xcc"
        gps.rtcm = b""
        fc.rtcm(rtcm1, seq=1)
        time.sleep(0.2)
        fc.rtcm(rtcm2, seq=2)
        time.sleep(0.5)
        check(gps.rtcm == rtcm1 + rtcm2, f"RTCM injection mismatch: got {len(gps.rtcm)} bytes, want {len(rtcm1) + len(rtcm2)}")

        # STATUSTEXT announcing the module
        with fc.lock:
            texts = list(fc.statustexts)
        want_mod = "MAX-M10S" if module == "m10" else "NEO-M8N"
        check(any(want_mod in t for t in texts), f"no module STATUSTEXT seen: {texts}")

    finally:
        proc.terminate()
        try:
            proc.wait(timeout=3)
        except subprocess.TimeoutExpired:
            proc.kill()
        gps.run_flag = False
        fc.run_flag = False
        for fd in (gps_master, gps_slave, fc_master, fc_slave):
            try:
                os.close(fd)
            except OSError:
                pass

    if failures or "-v" in sys.argv:
        print(f"---- ubx2mav log ({module}) ----")
        print("\n".join(log_lines[-80:]))
    print(f"scenario {module}: {len(fc.gps_inputs)} GPS_INPUT, {gps.epochs_sent} epochs, {len(gps.rtcm)} RTCM bytes")


def main():
    failures = []
    for module in ("m8", "m10", "m8old"):
        before = len(failures)
        run_scenario(module, failures)
        if len(failures) > before:
            print(f"scenario {module} FAILED")
    if failures:
        print(f"{len(failures)} e2e check(s) FAILED")
        sys.exit(1)
    print("e2e test passed (m8 legacy, m10 VALSET, m8 without NAV-EOE)")


if __name__ == "__main__":
    main()
