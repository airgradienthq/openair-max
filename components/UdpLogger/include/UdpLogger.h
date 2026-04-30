/**
 * AirGradient
 * https://airgradient.com
 *
 * CC BY-SA 4.0 Attribution-ShareAlike 4.0 International License
 *
 * UdpLogger
 * Mirrors every ESP_LOG line over Wi-Fi UDP to a fixed host:port. Used for
 * field debugging when the device is hung outdoors and only cellular (or no)
 * console access is available.
 *
 * Wire format per UDP datagram (one log line per packet):
 *   <serial>\t<uptime_ms>\t<raw_log_line>
 *
 * The raw log line is whatever ESP-IDF's logger produced, including any ANSI
 * color codes. The dashboard splits on the first two tabs and treats the rest
 * as the log payload.
 *
 * Designed to coexist with the cellular flow only — ESP32-C6 has a single Wi-Fi
 * radio so do not start this while the device is in WiFi network mode.
 */

#ifndef UDP_LOGGER_H
#define UDP_LOGGER_H

#include <cstdint>

namespace UdpLogger {

// Connect to Wi-Fi STA and start mirroring ESP_LOG output via UDP. Idempotent.
// Returns true on successful Wi-Fi init kick-off; the connection itself is
// asynchronous and the sender task buffers logs until the link is up.
bool start(const char *serial, const char *ssid, const char *password,
           const char *host, uint16_t port);

// Tear down sender task, socket, and Wi-Fi STA.
void stop();

// True after we've received an IP and the UDP socket is open.
bool isReady();

} // namespace UdpLogger

#endif // UDP_LOGGER_H
