// ---------------------------------------------------------------------------------------------
// LisaNet support for the LisaFPGA board's onboard ESProFile -- NOT part of upstream ESProFile. Written with an AI
// assistant. See the LisaFPGA fork: docs/superpowers/specs/2026-10-10-esp32-wifi-mode-design.md and
// docs/lisanet_link_protocol.md.
//
// The FPGA's on-screen menu has an ESP32 MODE row (FLOPPY / WIFI FDD / WIFI HDD). The board's own ESP32s have no
// antenna, so WIFI modes use an external WiFi board on the FPGA's J21 header. In WIFI HDD this ESP32 becomes its SD-card
// server: the FPGA raises MODE (FPGA_0, GPIO36) and wires FPGA_1 (GPIO37) and FPGA_2 (GPIO38) through to the WiFi
// board. The Lisa has no hard disk while that lasts. At boot ESProFile reads MODE and runs either as usual or as the
// SD server; the emulator also watches MODE between commands, and the server restarts the ESP32 when MODE drops.
//
// Older LisaFPGA bitstreams (and upstream's) hold these three lines at a constant 3'b101 -- MODE high, link input LOW.
// A real WIFI HDD request always has the link input idling HIGH (the FPGA passes the WiFi board's idle UART line, or
// its pull-up, straight through). So a request is MODE high for LISANET_MODE_STABLE_MS with the link input seen high
// at least once in that time; the constant pattern never qualifies, and this firmware stays an ordinary ProFile.
// ---------------------------------------------------------------------------------------------
#pragma once
#include <Arduino.h>

#define LISANET_MODE_PIN    36   // FPGA_0: MODE, 1 = serve the SD card (WIFI HDD)
#define LISANET_LINK_RX_PIN 37   // FPGA_1: from the WiFi board (through the FPGA)
#define LISANET_LINK_TX_PIN 38   // FPGA_2: to the WiFi board (the FPGA never drives this line)
#define LISANET_LINK_BAUD   2000000

// MODE must hold steady this long before either personality acts on it. While the FPGA is still configuring at
// power-up its pins float or are weakly pulled up, so a single read could see a request that isn't really there.
#define LISANET_MODE_STABLE_MS 300

// True if the FPGA is asking for the SD server (see above). Call before ESProFile configures anything.
bool lisanetModeRequestedAtBoot();

// True once MODE has been low continuously for LISANET_MODE_STABLE_MS. Polled by the SD server.
bool lisanetModeStableAt(bool level);

// The SD-server personality (sdServer.cpp). Never returns: restarts the ESP32 when MODE drops.
[[noreturn]] void sdServerMain();
