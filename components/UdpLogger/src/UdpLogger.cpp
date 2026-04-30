/**
 * AirGradient
 * https://airgradient.com
 *
 * CC BY-SA 4.0 Attribution-ShareAlike 4.0 International License
 */

#include "UdpLogger.h"

#include <cstdarg>
#include <cstdio>
#include <cstring>
#include <algorithm>

#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "lwip/sockets.h"
#include "lwip/netdb.h"

namespace {

constexpr size_t kEntryTextLen = 256;
constexpr size_t kQueueDepth = 64;
constexpr size_t kSerialMax = 32;
constexpr size_t kHostMax = 64;

// Heap-allocated by the vprintf hook, freed by the sender task. We queue
// pointers (not values) so the hook's stack footprint stays tiny — important
// because ESP-IDF's sys_evt task only has ~2.8KB of stack and the hook runs
// inside event-handler code paths.
struct LogEntry {
  uint32_t tsMs;
  uint16_t len;
  char text[kEntryTextLen];
};

vprintf_like_t s_prevVprintf = nullptr;
QueueHandle_t s_queue = nullptr;
TaskHandle_t s_senderTask = nullptr;

volatile bool s_started = false;
volatile bool s_networkReady = false;
int s_socket = -1;

char s_serial[kSerialMax] = {0};
char s_host[kHostMax] = {0};
uint16_t s_port = 0;
struct sockaddr_in s_destAddr = {};

esp_event_handler_instance_t s_wifiHandlerInstance = nullptr;
esp_event_handler_instance_t s_ipHandlerInstance = nullptr;

// Custom vprintf hook installed via esp_log_set_vprintf. Runs in whatever task
// called ESP_LOGx — including ESP-IDF's sys_evt task, which has only ~2.8KB
// of stack. So we keep stack footprint to ~32 bytes by heap-allocating each
// entry and queueing only a pointer.
extern "C" int udpVprintfHook(const char *fmt, va_list args) {
  va_list copy;
  va_copy(copy, args);

  int forwarded = 0;
  if (s_prevVprintf) {
    forwarded = s_prevVprintf(fmt, args);
  } else {
    forwarded = vprintf(fmt, args);
  }

  if (s_started && s_queue) {
    LogEntry *e = static_cast<LogEntry *>(malloc(sizeof(LogEntry)));
    if (e) {
      e->tsMs = static_cast<uint32_t>(esp_timer_get_time() / 1000);
      int n = vsnprintf(e->text, sizeof(e->text), fmt, copy);
      if (n < 0) {
        free(e);
      } else {
        size_t copyLen = static_cast<size_t>(n);
        if (copyLen >= sizeof(e->text)) {
          copyLen = sizeof(e->text) - 1;
        }
        e->text[copyLen] = '\0';
        e->len = static_cast<uint16_t>(copyLen);
        if (xQueueSend(s_queue, &e, 0) != pdTRUE) {
          // Queue full — drop the oldest entry so freshest logs always reach
          // the dashboard.
          LogEntry *old = nullptr;
          if (xQueueReceive(s_queue, &old, 0) == pdTRUE && old) {
            free(old);
          }
          if (xQueueSend(s_queue, &e, 0) != pdTRUE) {
            free(e);
          }
        }
      }
    }
  }
  va_end(copy);

  return forwarded;
}

void senderTask(void *) {
  for (;;) {
    if (!s_networkReady || s_socket < 0) {
      vTaskDelay(pdMS_TO_TICKS(200));
      continue;
    }

    LogEntry *e = nullptr;
    if (xQueueReceive(s_queue, &e, pdMS_TO_TICKS(500)) != pdTRUE || !e) {
      continue;
    }

    char pkt[kEntryTextLen + kSerialMax + 32];
    int n = snprintf(pkt, sizeof(pkt), "%s\t%lu\t%s", s_serial,
                     static_cast<unsigned long>(e->tsMs), e->text);
    if (n > 0) {
      if (n >= static_cast<int>(sizeof(pkt))) {
        n = sizeof(pkt) - 1;
      }
      sendto(s_socket, pkt, n, 0,
             reinterpret_cast<struct sockaddr *>(&s_destAddr), sizeof(s_destAddr));
    }
    free(e);
  }
}

void wifiEventHandler(void *, esp_event_base_t base, int32_t id, void *) {
  if (base == WIFI_EVENT && id == WIFI_EVENT_STA_START) {
    esp_wifi_connect();
  } else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
    s_networkReady = false;
    if (s_socket >= 0) {
      close(s_socket);
      s_socket = -1;
    }
    // Auto-reconnect — keep retrying so the dashboard reattaches as soon as
    // the AP is back in range.
    esp_wifi_connect();
  }
}

void ipEventHandler(void *, esp_event_base_t, int32_t, void *event_data) {
  auto *event = static_cast<ip_event_got_ip_t *>(event_data);
  if (s_socket >= 0) {
    close(s_socket);
    s_socket = -1;
  }
  s_socket = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
  if (s_socket < 0) {
    return;
  }
  s_destAddr.sin_family = AF_INET;
  s_destAddr.sin_port = htons(s_port);
  s_destAddr.sin_addr.s_addr = inet_addr(s_host);
  s_networkReady = true;

  // Emit one breadcrumb so the dashboard knows we're online.
  ESP_LOGI("UdpLog", "UDP logger online: ip=" IPSTR " -> %s:%u",
           IP2STR(&event->ip_info.ip), s_host, s_port);
}

} // namespace

namespace UdpLogger {

bool start(const char *serial, const char *ssid, const char *password,
           const char *host, uint16_t port) {
  if (s_started) {
    return true;
  }
  if (!serial || !ssid || !password || !host) {
    return false;
  }

  strncpy(s_serial, serial, sizeof(s_serial) - 1);
  strncpy(s_host, host, sizeof(s_host) - 1);
  s_port = port;

  s_queue = xQueueCreate(kQueueDepth, sizeof(LogEntry *));
  if (!s_queue) {
    return false;
  }

  // Bring up netif/event loop if not already done by another component
  // (WiFiManager owns these in WiFi mode; in cellular mode we own them).
  esp_err_t err = esp_netif_init();
  if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
    return false;
  }
  err = esp_event_loop_create_default();
  if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
    return false;
  }

  if (!esp_netif_get_handle_from_ifkey("WIFI_STA_DEF")) {
    esp_netif_create_default_wifi_sta();
  }

  wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
  err = esp_wifi_init(&cfg);
  if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
    return false;
  }
  // RAM-only storage so we never clobber the user's saved Wi-Fi creds.
  esp_wifi_set_storage(WIFI_STORAGE_RAM);

  esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
                                      &wifiEventHandler, nullptr,
                                      &s_wifiHandlerInstance);
  esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP,
                                      &ipEventHandler, nullptr,
                                      &s_ipHandlerInstance);

  wifi_config_t wc = {};
  strncpy(reinterpret_cast<char *>(wc.sta.ssid), ssid, sizeof(wc.sta.ssid) - 1);
  strncpy(reinterpret_cast<char *>(wc.sta.password), password,
          sizeof(wc.sta.password) - 1);
  wc.sta.threshold.authmode = WIFI_AUTH_OPEN;
  wc.sta.pmf_cfg.capable = true;

  esp_wifi_set_mode(WIFI_MODE_STA);
  esp_wifi_set_config(WIFI_IF_STA, &wc);
  esp_wifi_start();

  s_prevVprintf = esp_log_set_vprintf(&udpVprintfHook);

  xTaskCreate(senderTask, "UdpLogSend", 4096, nullptr, 4, &s_senderTask);

  s_started = true;
  return true;
}

void stop() {
  if (!s_started) {
    return;
  }
  s_started = false;

  if (s_prevVprintf) {
    esp_log_set_vprintf(s_prevVprintf);
    s_prevVprintf = nullptr;
  }

  if (s_senderTask) {
    vTaskDelete(s_senderTask);
    s_senderTask = nullptr;
  }

  if (s_socket >= 0) {
    close(s_socket);
    s_socket = -1;
  }

  if (s_wifiHandlerInstance) {
    esp_event_handler_instance_unregister(WIFI_EVENT, ESP_EVENT_ANY_ID,
                                          s_wifiHandlerInstance);
    s_wifiHandlerInstance = nullptr;
  }
  if (s_ipHandlerInstance) {
    esp_event_handler_instance_unregister(IP_EVENT, IP_EVENT_STA_GOT_IP,
                                          s_ipHandlerInstance);
    s_ipHandlerInstance = nullptr;
  }

  esp_wifi_disconnect();
  esp_wifi_stop();

  if (s_queue) {
    LogEntry *e = nullptr;
    while (xQueueReceive(s_queue, &e, 0) == pdTRUE) {
      if (e) free(e);
    }
    vQueueDelete(s_queue);
    s_queue = nullptr;
  }
  s_networkReady = false;
}

bool isReady() { return s_networkReady; }

} // namespace UdpLogger
