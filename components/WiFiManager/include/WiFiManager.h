#pragma once

#include "wm_config.h"
#include "WiFiManagerParameter.h"
#include "esp_wifi.h"
#include "esp_netif.h"
#include "esp_event.h"
#include "esp_http_server.h"
#include "lwip/ip4_addr.h"
#include <string>
#include <string_view>
#include <vector>
#include <memory>
#include <mutex>
#include <functional>

#define NETWORK_MODE_CELLULAR_STR "cellular"
#define NETWORK_MODE_WIFI_STR "wifi"

// Forward declarations
class WiFiManager;

// Callback types
typedef std::function<void(WiFiManager *)> APCallback;
typedef std::function<void()> SaveConfigCallback;
typedef std::function<void()> ConfigModeCallback;
typedef std::function<void()> WebServerModeCallback;

/**
 * WiFi network information structure
 */
struct WiFiNetwork {
  std::string ssid;
  int32_t rssi;
  wifi_auth_mode_t encryptionType;
  int32_t channel;
  bool isHidden;
};

struct SettingsForm {
  std::string networkMode;
  std::string ssid;
  std::string password;
  std::string apn;
  std::string httpDomain;
  // Add more here
};

/**
 * WiFiManager class - Main entry point for WiFi configuration management
 * API-compatible with Arduino WiFiManager
 */
class WiFiManager {
public:
  // Constructor / Destructor
  WiFiManager();
  ~WiFiManager();

  // Core connection methods
  bool autoConnect(bool skipPortal = false);
  bool autoConnect(const char *apName);
  bool autoConnect(const char *apName, const char *apPassword);
  bool autoConnect(const char *apName, const char *apPassword, bool skipPortal);
  bool startConfigPortal();
  bool startConfigPortal(const char *apName);
  bool startConfigPortal(const char *apName, const char *apPassword);

  // Web portal control
  void startWebPortal();
  void stopWebPortal();
  void setSettings(SettingsForm settings);
  SettingsForm getSettings();

  // Manual server control
  void stopServers();
  bool process(); // Non-blocking maintenance

  // WiFi credential management
  bool resetSettings();

  // WiFi connection management
  bool isWiFiConnected();
  bool reconnectWiFi();
  bool reconnectWiFi(uint32_t timeoutSeconds);
  int getSignal();

  // Timeout configuration
  void setConfigPortalTimeout(uint32_t seconds);
  void setConnectTimeout(uint32_t seconds);
  void setConfigPortalBlocking(bool shouldBlock);
  void setBreakAfterConfig(bool shouldBreak);

  // Callback registration
  void setAPCallback(APCallback callback);
  void setSaveConfigCallback(SaveConfigCallback callback);
  void setConfigModeCallback(ConfigModeCallback callback);
  void setWebServerModeCallback(WebServerModeCallback callback);

  // Scanning and filtering
  void setMinimumSignalQuality(int percent = 8);
  void setRemoveDuplicateAPs(bool remove = true);
  bool preloadWiFiScan(bool enable = true);
  void setScanDispPerc(bool showPercent = false);

  // Captive portal behavior
  void setCaptivePortalEnable(bool enable = true);
  void setCaptivePortalClientCheck(bool enable = true);
  void setWebPortalClientCheck(bool enable = true);

  // IP configuration
  void setAPStaticIPConfig(ip4_addr_t ip, ip4_addr_t gw, ip4_addr_t netmask);
  void setSTAStaticIPConfig(ip4_addr_t ip, ip4_addr_t gw, ip4_addr_t netmask);
  void setSTAStaticIPConfig(ip4_addr_t ip, ip4_addr_t gw, ip4_addr_t netmask, ip4_addr_t dns);

  // Hostname and UI customization
  bool setHostname(const char *hostname);
  bool setHostname(const std::string &hostname);
  void setMenu(const menu_page_t *menu, uint8_t size);
  void setClass(const char *cssClass);
  void setCustomHeadElement(const char *html);

  // Custom parameters
  void addParameter(WiFiManagerParameter *parameter);
  WiFiManagerParameter *getParameters();
  int getParametersCount() const;

  // Diagnostics and helpers
  wl_status_t getLastConxResult() const;
  const char *getWLStatusString(wl_status_t status) const;
  const char *getModeString(wifi_mode_t mode) const;
  bool getWiFiIsSaved() const;
  std::string getSSID() const;
  std::string getPassword() const;

  // WiFi control
  void setWiFiAutoReconnect(bool autoReconnect = true);
  bool disconnect(bool wifioff = false);
  bool erase();

  // Debug information
  void debugPlatformInfo() const;
  void debugSoftAPConfig() const;

  // Status getters
  wm_state_t getState() const { return _state; }
  bool isConfigPortalActive() const;
  bool isWebPortalActive() const;

private:
  // State management
  wm_state_t _state;
  std::mutex _mutex;

  // Configuration
  std::string _apName;
  std::string _apPassword;
  std::string _hostname;
  uint32_t _connectTimeout;
  uint32_t _configPortalTimeout;
  bool _configPortalBlocking;
  bool _breakAfterConfig;

  // Network configuration
  bool _apStaticIPSet;
  bool _staStaticIPSet;
  ip4_addr_t _apIP, _apGW, _apNetmask;
  ip4_addr_t _staIP, _staGW, _staNetmask, _staDNS;

  // Scanning and filtering
  int _minimumQuality;
  bool _removeDuplicateAPs;
  bool _scanDispPerc;
  std::vector<WiFiNetwork> _scanResults;
  std::vector<wifi_ap_record_t> _rawScanResults;
  int64_t _lastScanTime;
  bool _scanInProgress;

  // Captive portal settings
  bool _captivePortalEnable;
  bool _captivePortalClientCheck;
  bool _webPortalClientCheck;

  // UI customization
  std::string _customHeadElement;
  std::string _cssClass;
  std::vector<menu_page_t> _menuPages;

  // Custom parameters
  std::vector<std::unique_ptr<WiFiManagerParameter>> _params;

  // Callbacks
  APCallback _apCallback;
  SaveConfigCallback _saveConfigCallback;
  ConfigModeCallback _configModeCallback;
  WebServerModeCallback _webServerModeCallback;

  // ESP-IDF handles
  esp_netif_t *_apNetif;
  esp_netif_t *_staNetif;
  httpd_handle_t _httpServer;
  esp_event_handler_instance_t _wifiEventHandler;
  esp_event_handler_instance_t _ipEventHandler;

  // Internal state
  wl_status_t _lastConxResult;
  bool _portalAbortResult;
  int64_t _configPortalStart;
  int64_t _connectStart;

  // Private methods
  void init();
  void cleanup();

  // WiFi management
  bool setupWiFi();
  bool startSTA();
  bool startAP(const char *ssid, const char *password);
  void stopWiFi();

  // State machine
  void updateState();
  bool handleSTAConnection();
  bool handlePortalMode();

  // HTTP server
  bool startHTTPServer();
  void stopHTTPServer();

  // Internal methods (no mutex locking)
  bool startConfigPortalInternal(const char *apName, const char *apPassword);

  static esp_err_t handleRoot(httpd_req_t *req);
  static esp_err_t handleSettings(httpd_req_t *req);
  static esp_err_t handleFetchSettings(httpd_req_t *req);
  static esp_err_t handleStatus(httpd_req_t *req);
  static esp_err_t handleScan(httpd_req_t *req);
  static esp_err_t handleSettingsSave(httpd_req_t *req);
  static esp_err_t handleInfo(httpd_req_t *req);
  static esp_err_t handleReset(httpd_req_t *req);
  static esp_err_t handleExit(httpd_req_t *req);
  static esp_err_t handleCaptivePortal(httpd_req_t *req);
  static WiFiManager *getManagerFromRequest(httpd_req_t *req);

  SettingsForm _settings;
  static std::string urlDecode(const std::string &src); // decode a URL-encoded string
  static SettingsForm parseFormParams(char *buf);
  static esp_err_t performSettingCellular(httpd_req_t *req, const SettingsForm &settings);
  static esp_err_t performSettingWifi(httpd_req_t *req, const SettingsForm &settings);

  // DNS server
  bool startDNSServer();
  void stopDNSServer();
  static void dnsServerTask(void *pvParameters);

  // DNS server private members
  TaskHandle_t _dnsTaskHandle;
  int _dnsSocket;
  bool _dnsRunning;

  // Initialization state
  bool _initialized;
  bool _cleanupInProgress;

  // Scanning
  bool scanWiFiNetworks();
  void filterScanResults();
  void performWiFiScan(bool async = false);
  bool isDuplicateSSID(const char *ssid, const std::vector<wifi_ap_record_t> &results);
  int calculateSignalQuality(int rssi);
  void sortScanResultsBySignal();
  std::vector<wifi_ap_record_t> getFilteredScanResults();

  // Event handlers
  static void wifiEventHandler(void *arg, esp_event_base_t event_base, int32_t event_id,
                               void *event_data);
  static void ipEventHandler(void *arg, esp_event_base_t event_base, int32_t event_id,
                             void *event_data);

  // Utility methods
  std::string generateDefaultAPName();
  bool isValidIPString(const char *ip);
  wl_status_t mapESPResult(wifi_err_reason_t reason);
};
