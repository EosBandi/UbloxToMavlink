# ubx2mav - u-blox GPS to ArduPilot MAVLink bridge

A small C service for a Raspberry Pi Zero 2 (or any Linux box) that takes a
u-blox GNSS receiver on a serial port and feeds an ArduPilot flight controller
with MAVLink `GPS_INPUT` messages, so the autopilot sees it as `GPS_TYPE = 14`
(MAVLink GPS). The receiver is configured the same way ArduPilot's own
`AP_GPS_UBLOX` driver configures a directly connected module, and the data is
forwarded with the same field mapping ArduPilot uses internally, so the result
is as close to a directly connected GPS as the `GPS_INPUT` message allows.

No external libraries: MAVLink v2 and UBX are implemented in-tree
(`src/mavlink_min.c`, `src/ubx.c`).

## What it does

**On the GPS side** (mirrors `AP_GPS_UBLOX.cpp` with `GPS_AUTO_CONFIG=1`):

- Finds the module at any common baud rate and moves it to 230400 baud
  (ArduPilot's default). Uses `CFG-PRT`, the `$PUBX,41` sentence from
  ArduPilot's init blob, and `CFG-UART1-BAUDRATE` so every generation works.
- Identifies the module with `MON-VER` (u-blox 6/7/M8/M9/F9/M10, `PROTVER`,
  `MOD=`), then configures it with legacy `CFG-*` messages (M8 and older,
  M9) or `CFG-VALSET` keys (F9, M10), exactly as ArduPilot does.
- Port protocols: UBX + RTCM3 in, UBX only out (NMEA switched off).
- `CFG-RATE` 200 ms / nav rate 1 / UTC time reference (`GPS_RATE_MS`).
- `CFG-NAV5` dynamic model 8 "airborne <4g" (`GPS_NAVFILTER`), optional
  minimum elevation (`GPS_MIN_ELEV`).
- Optional constellation mask (`GPS_GNSS_MODE` algorithm incl. the ZED/NEO
  second-band keys on F9) and SBAS mode (`GPS_SBAS_MODE`).
- Enables `NAV-PVT`, `NAV-DOP`, `NAV-TIMEGPS`, `NAV-EOE` (and `NAV-RELPOSNED`
  when yaw is requested); disables `NAV-POSLLH/STATUS/SOL/VELNED/SAT/SVINFO`.
- Optional save to non-volatile memory (`GPS_SAVE_CFG`).
- Re-detects and re-configures the receiver after a data timeout or if the
  device disappears (USB unplug), without restarting the service.

**On the flight controller side** (matches what `AP_GPS_MAV.cpp` consumes):

- One `GPS_INPUT` per navigation epoch, sent right after `NAV-EOE` so the
  position, DOP and time of the *same* epoch travel together (falls back to
  a short timeout or send-on-PVT on modules without `NAV-EOE`).
- Field mapping identical to ArduPilot's u-blox driver: fix type from
  `NAV-PVT` `fixType`/`flags` (DGPS, RTK float, RTK fixed), lat/lon,
  altitude AMSL, hDOP/vDOP from `NAV-DOP` (pDOP until the first DOP arrives),
  NED velocity, speed/horizontal/vertical accuracy, satellite count, GPS
  week + iTOW (week from `NAV-TIMEGPS`, derived from the PVT UTC date until it
  arrives, rolled over on iTOW wrap). "No fix" is always sent as fix type 1,
  because `AP_GPS_MAV` casts the value straight into its fix enum where 0
  means "no GPS detected".
- Optional GPS yaw from a moving-baseline rover's `NAV-RELPOSNED`, using the
  same validity mask as ArduPilot (fixed carrier solution, heading valid,
  moving, no missing reference data).
- Heartbeats as `MAV_TYPE_GPS` / `MAV_COMP_ID_GPS` (220). The bridge adopts
  the autopilot's system id, so it is treated as a component of the vehicle
  (`SYSID_ENFORCE` safe) and ArduPilot's router learns a route to it.
- **RTK corrections**: because the bridge is a known route, ArduPilot forwards
  broadcast `GPS_RTCM_DATA` / `GPS_INJECT_DATA` from the GCS to it. Fragments
  are re-assembled with the same algorithm as `AP_GPS::handle_gps_rtcm_fragment`
  and injected into the receiver; `RXM-RTCM` is enabled so the log shows
  whether the module actually used them.
- `STATUSTEXT` to the GCS when the module is detected ("u-blox NEO-M8N ..."),
  on configuration, and on data loss.
- Periodic statistics line (fix, sats, HDOP, rates, RTCM counters, CRC
  errors) in the journal.

### What `GPS_INPUT` cannot carry

Compared to a directly connected u-blox, ArduPilot will not get: geoid
undulation, RTK baseline/age fields, raw measurements for logging,
jamming/noise indicators (`MON-HW`), and the driver-specific lag estimate.
Set `GPS_DELAY_MS=120` on the flight controller to use the lag ArduPilot
assumes for M8/F9 modules (the MAVLink backend otherwise assumes 200 ms).

## Hardware setup on a Pi Zero 2 W

The Zero 2 W exposes a single UART pair on the 40-pin header (GPIO14 TXD /
GPIO15 RXD, 3.3 V). The recommended wiring is:

| Connection | Pi side | Notes |
|---|---|---|
| u-blox receiver | header UART `/dev/ttyAMA0` | GPS TX to GPIO15, GPS RX to GPIO14, 3.3 V logic |
| flight controller | FC USB port into the Pi's USB OTG port, `/dev/ttyACM0` | MAVLink2 is on by default on the FC USB port |

Alternatives: a USB-serial adapter for either side (`/dev/ttyUSB0`), a u-blox
with USB (`/dev/ttyACM1`), or a network link (`--fc udp://host:port`) when a
MAVLink router already runs on the Pi.

Free the header UART for the GPS and give it the good PL011 UART:

```
# /boot/firmware/config.txt (or /boot/config.txt on older images)
enable_uart=1
dtoverlay=disable-bt
```

Remove `console=serial0,115200` from `/boot/firmware/cmdline.txt`, then:

```bash
sudo systemctl disable --now hciuart serial-getty@ttyAMA0.service
```

## Build and install

On the Pi (Raspberry Pi OS, `sudo apt install build-essential`):

```bash
make
sudo make install
sudo systemctl enable --now ubx2mav
journalctl -fu ubx2mav
```

Cross compiling on a Debian/Ubuntu host (`apt install gcc-arm-linux-gnueabihf`
or `gcc-aarch64-linux-gnu`):

```bash
make CROSS=arm-linux-gnueabihf-    # 32-bit Raspberry Pi OS
make CROSS=aarch64-linux-gnu-      # 64-bit Raspberry Pi OS
```

Tests (host, needs `pip install pymavlink`): `make test` runs the codec unit
tests and an end-to-end test that emulates an M8 receiver and an ArduPilot
flight controller on two pseudo terminals.

## Configuration

Options go in `/etc/default/ubx2mav`:

```
UBX2MAV_OPTS="--gps-dev /dev/ttyAMA0 --fc /dev/ttyACM0 --fc-baud 115200"
```

Run `ubx2mav --help` for the full list. The defaults reproduce ArduPilot's
defaults (230400 baud, 5 Hz, airborne 4g, constellations and SBAS left as
configured, nothing saved to the module). Useful extras:

| Option | Purpose |
|---|---|
| `--gnss-mode 71` | GPS + SBAS + Galileo + GLONASS, same bitmask as `GPS_GNSS_MODE` |
| `--save-config` | persist the configuration in the module (`GPS_SAVE_CFG`) |
| `--yaw-relposned` | ZED-F9P moving-baseline rover: forward heading as GPS yaw |
| `--gps-id 1` | feed the second GPS instance (`GPS2_TYPE=14`) |
| `--send-on pvt` | lowest latency on modules without `NAV-EOE` |
| `--require-fix-ok` | also demand the u-blox `gnssFixOK` flag for a fix |
| `-v` / `-vv` | per-message debug / byte-level trace |

## Flight controller parameters

| Parameter | Value | Notes |
|---|---|---|
| `GPS_TYPE` (`GPS1_TYPE` on 4.6+) | 14 | MAVLink GPS |
| `SERIALn_PROTOCOL` | 2 | MAVLink2 on the port wired to the Pi (the USB port already is) |
| `SERIALn_BAUD` | 115 | or whatever `--fc-baud` is set to |
| `GPS_DELAY_MS` (`GPS1_DELAY_MS`) | 120 | optional, lag ArduPilot uses for M8/F9 |
| `GPS_AUTO_CONFIG` | any | irrelevant for the MAVLink backend; the bridge does the configuring |
| `GPS_POS_X/Y/Z` | antenna offset | still applies |
| `GPS_RATE_MS` | 200 | not sent to the bridge; use `--rate-ms` instead |

With `GPS_TYPE=14` ArduPilot instantiates the MAVLink backend immediately and
waits for `GPS_INPUT` with `gps_id` equal to the instance (0 for GPS1).
Mission Planner / QGC show the usual GPS status, HDOP, satellite count and
fix type, and RTK injection from the GCS works through the FC to the Pi.

## Design notes

- `src/gps.c` - detection, configuration, UBX frame handling, epoch assembly.
- `src/main.c` - I/O loop, `GPS_INPUT` builder, RTCM re-assembly, heartbeats.
- `src/mavlink_min.c` - MAVLink v2 framing with the exact field layout and
  CRC extras of `HEARTBEAT`, `GPS_INPUT`, `GPS_RTCM_DATA`, `GPS_INJECT_DATA`,
  `STATUSTEXT` (verified against pymavlink).
- `src/ubx.c` - UBX framing, message decoders/encoders, VALSET keys.
- `src/serial.c` - termios setup, custom baud rates, UDP links.

Latency: at 230400 baud a full epoch (PVT+DOP+TIMEGPS+EOE, 154 bytes) takes
~7 ms on the wire; `GPS_INPUT` (77 bytes) another ~7 ms at 115200 to the FC.
The service runs with realtime scheduling (see the unit file) so the Pi adds
well under a millisecond on top.

## License

GPL-3.0-or-later, because configuration logic and message handling were
derived from ArduPilot's `AP_GPS` library.
