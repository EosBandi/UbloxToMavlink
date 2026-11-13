# ubx2mav - u-blox GPS to ArduPilot MAVLink GPS_INPUT bridge
#
# Native build (on the Pi):      make
# Cross build (Debian/Ubuntu):   make CROSS=arm-linux-gnueabihf-   (32-bit Raspberry Pi OS)
#                                make CROSS=aarch64-linux-gnu-     (64-bit Raspberry Pi OS)
# Tests (host):                  make test
# Install (on the Pi):           sudo make install && sudo systemctl enable --now ubx2mav

CROSS   ?=
CC      := $(CROSS)gcc
CFLAGS  ?= -O2 -g
CFLAGS  += -std=gnu11 -Wall -Wextra -Wshadow -Wformat=2 -Wno-format-nonliteral -D_FILE_OFFSET_BITS=64
LDFLAGS ?=
LDLIBS  := -lm
PREFIX  ?= /usr/local
DESTDIR ?=

SRC_COMMON := src/mavlink_min.c src/ubx.c src/log.c
SRC_MAIN   := src/main.c src/gps.c src/serial.c src/serial_baud.c $(SRC_COMMON)
BUILD      := build
OBJ_MAIN   := $(patsubst src/%.c,$(BUILD)/%.o,$(SRC_MAIN))

all: $(BUILD)/ubx2mav

$(BUILD)/ubx2mav: $(OBJ_MAIN)
	$(CC) $(LDFLAGS) -o $@ $^ $(LDLIBS)

$(BUILD)/%.o: src/%.c src/*.h | $(BUILD)
	$(CC) $(CFLAGS) -c -o $@ $<

$(BUILD):
	mkdir -p $(BUILD)

# ---- tests (run on the build host) ----
TEST_CFLAGS := $(CFLAGS) -Isrc
$(BUILD)/test_codec: tests/test_codec.c $(SRC_COMMON) src/*.h | $(BUILD)
	$(CC) $(TEST_CFLAGS) -o $@ tests/test_codec.c $(SRC_COMMON) $(LDLIBS)

test: $(BUILD)/test_codec $(BUILD)/ubx2mav
	$(BUILD)/test_codec
	python3 tests/test_e2e.py $(BUILD)/ubx2mav

# ---- install ----
install: $(BUILD)/ubx2mav
	install -D -m 0755 $(BUILD)/ubx2mav $(DESTDIR)$(PREFIX)/bin/ubx2mav
	install -D -m 0644 systemd/ubx2mav.service $(DESTDIR)/etc/systemd/system/ubx2mav.service
	@if [ ! -f $(DESTDIR)/etc/default/ubx2mav ]; then \
		install -D -m 0644 systemd/ubx2mav.default $(DESTDIR)/etc/default/ubx2mav; \
	else echo "keeping existing /etc/default/ubx2mav"; fi
	-systemctl daemon-reload 2>/dev/null

uninstall:
	-systemctl disable --now ubx2mav 2>/dev/null
	rm -f $(DESTDIR)$(PREFIX)/bin/ubx2mav $(DESTDIR)/etc/systemd/system/ubx2mav.service

clean:
	rm -rf $(BUILD)

.PHONY: all test install uninstall clean
