#pragma once

#include "sdkconfig.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_netif.h"
#include <string>
#include <functional>

#ifdef __cplusplus
extern "C" {
#endif

// Component name for logging
#define WM_TAG "WiFiManager"

// Logging macros
#if CONFIG_WM_LOG_LEVEL >= 5
#define WM_LOGV(format, ...) ESP_LOGV(WM_TAG, format, ##__VA_ARGS__)
#else
#define WM_LOGV(format, ...)
#endif

#if CONFIG_WM_LOG_LEVEL >= 4
#define WM_LOGD(format, ...) ESP_LOGD(WM_TAG, format, ##__VA_ARGS__)
#else
#define WM_LOGD(format, ...)
#endif

#if CONFIG_WM_LOG_LEVEL >= 3
#define WM_LOGI(format, ...) ESP_LOGI(WM_TAG, format, ##__VA_ARGS__)
#else
#define WM_LOGI(format, ...)
#endif

#if CONFIG_WM_LOG_LEVEL >= 2
#define WM_LOGW(format, ...) ESP_LOGW(WM_TAG, format, ##__VA_ARGS__)
#else
#define WM_LOGW(format, ...)
#endif

#if CONFIG_WM_LOG_LEVEL >= 1
#define WM_LOGE(format, ...) ESP_LOGE(WM_TAG, format, ##__VA_ARGS__)
#else
#define WM_LOGE(format, ...)
#endif

// WiFiManager states
typedef enum {
    WM_STATE_INIT = 0,
    WM_STATE_TRY_STA,
    WM_STATE_RUN_STA,
    WM_STATE_START_PORTAL,
    WM_STATE_RUN_PORTAL,
    WM_STATE_PORTAL_ABORT,
    WM_STATE_PORTAL_TIMEOUT
} wm_state_t;

// Connection results (mapped from ESP-IDF wifi_err_reason_t)
typedef enum {
    WL_IDLE_STATUS = 0,
    WL_NO_SSID_AVAIL,
    WL_SCAN_COMPLETED,
    WL_CONNECTED,
    WL_CONNECT_FAILED,
    WL_CONNECTION_LOST,
    WL_WRONG_PASSWORD,
    WL_DISCONNECTED
} wl_status_t;

// Menu page options
typedef enum {
    MENU_WIFI = 0,
    MENU_INFO,
    MENU_EXIT,
    MENU_ERASE,
    MENU_RESTART,
    MENU_PARAM,
    MENU_CLOSE,
    MENU_DARK
} menu_page_t;

// Parameter types
typedef enum {
    WMP_TYPE_TEXT = 0,
    WMP_TYPE_PASSWORD,
    WMP_TYPE_NUMBER,
    WMP_TYPE_CHECKBOX,
    WMP_TYPE_RADIO,
    WMP_TYPE_SELECT,
    WMP_TYPE_TEXTAREA,
    WMP_TYPE_HIDDEN
} wm_parameter_type_t;

// Constants
#define WM_MAX_SSID_LEN 32
#define WM_MAX_PASSWORD_LEN 64
#define WM_MAX_HOSTNAME_LEN 32
#define WM_MAX_CUSTOM_HTML_LEN 1024
#define WM_MAX_CUSTOM_PARAMS CONFIG_WM_MAX_CUSTOM_PARAMS
#define WM_MAX_SCAN_RESULTS 20

// Default values
#define WM_DEFAULT_AP_CHANNEL 1
#define WM_DEFAULT_CONNECT_TIMEOUT CONFIG_WM_DEFAULT_CONNECT_TIMEOUT
#define WM_DEFAULT_PORTAL_TIMEOUT CONFIG_WM_DEFAULT_PORTAL_TIMEOUT
#define WM_MIN_QUALITY CONFIG_WM_MIN_SIGNAL_QUALITY

// HTTP server
#define WM_HTTP_PORT 80
#define WM_HTTP_MAX_HANDLERS 20

// DNS server
#define WM_DNS_PORT 53
#define WM_DNS_MAX_CLIENTS 4

#ifdef __cplusplus
}
#endif 