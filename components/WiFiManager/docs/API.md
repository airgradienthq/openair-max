# WiFiManager API Reference

Complete API documentation for the ESP-IDF WiFiManager component.

## Table of Contents

- [Core Classes](#core-classes)
- [Connection Methods](#connection-methods)
- [Configuration Methods](#configuration-methods)
- [Callback Methods](#callback-methods)
- [Information Methods](#information-methods)
- [Manual Control Methods](#manual-control-methods)
- [Parameter Management](#parameter-management)
- [Types and Enums](#types-and-enums)

## Core Classes

### WiFiManager

Main class for WiFi configuration management.

```cpp
#include "WiFiManager.h"

WiFiManager wifiManager;
```

**Constructor:**
```cpp
WiFiManager();
```

**Destructor:**
```cpp
~WiFiManager();
```

### WiFiManagerParameter

Class for custom configuration parameters.

```cpp
#include "WiFiManagerParameter.h"

WiFiManagerParameter param("id", "label", "defaultValue", length);
```

## Connection Methods

### autoConnect

Automatically connect using saved credentials or start captive portal.

```cpp
bool autoConnect(const char* apName = nullptr, const char* apPassword = nullptr);
```

**Parameters:**
- `apName` - Access Point SSID (default: auto-generated from MAC)
- `apPassword` - Access Point password (default: open network)

**Returns:** `true` if connected to WiFi, `false` on failure or timeout

**Example:**
```cpp
// Simple usage
if (wifiManager.autoConnect()) {
    printf("Connected!\n");
}

// With custom AP name
if (wifiManager.autoConnect("MyDevice-Setup")) {
    printf("Connected!\n");
}

// With AP name and password
if (wifiManager.autoConnect("MyDevice-Setup", "password123")) {
    printf("Connected!\n");
}
```

### startConfigPortal

Manually start configuration portal without attempting saved credentials.

```cpp
bool startConfigPortal();
bool startConfigPortal(const char* apName);
bool startConfigPortal(const char* apName, const char* apPassword);
```

**Parameters:**
- `apName` - Access Point SSID (default: auto-generated)
- `apPassword` - Access Point password (default: open)

**Returns:** `true` if connected, `false` on timeout or failure

**Example:**
```cpp
// Start portal immediately
if (wifiManager.startConfigPortal("Setup-Portal")) {
    printf("Connected via portal!\n");
}
```

## Configuration Methods

### setConfigPortalTimeout

Set timeout for captive portal in seconds.

```cpp
void setConfigPortalTimeout(uint32_t seconds);
```

**Parameters:**
- `seconds` - Timeout in seconds (0 = no timeout)

**Default:** 180 seconds (3 minutes)

**Example:**
```cpp
wifiManager.setConfigPortalTimeout(300); // 5 minutes
```

### setConnectTimeout

Set timeout for WiFi connection attempts.

```cpp
void setConnectTimeout(uint32_t seconds);
```

**Parameters:**
- `seconds` - Connection timeout in seconds

**Default:** 30 seconds

### setConfigPortalBlocking

Set blocking/non-blocking mode for portal operation.

```cpp
void setConfigPortalBlocking(bool shouldBlock);
```

**Parameters:**
- `shouldBlock` - `true` for blocking, `false` for non-blocking

**Default:** `true` (blocking)

**Example:**
```cpp
// Non-blocking mode
wifiManager.setConfigPortalBlocking(false);
wifiManager.autoConnect();

while (true) {
    wifiManager.process(); // Handle requests
    // Your other tasks...
    vTaskDelay(pdMS_TO_TICKS(100));
}
```

### setBreakAfterConfig

Exit portal after successful configuration.

```cpp
void setBreakAfterConfig(bool shouldBreak);
```

**Parameters:**
- `shouldBreak` - `true` to exit after config, `false` to continue

**Default:** `true`

### WiFi Scanning Configuration

#### setMinimumSignalQuality

Filter networks by minimum signal quality.

```cpp
void setMinimumSignalQuality(int percent = 8);
```

**Parameters:**
- `percent` - Minimum signal quality percentage (0-100)

#### setRemoveDuplicateAPs

Remove duplicate SSIDs from scan results.

```cpp
void setRemoveDuplicateAPs(bool remove = true);
```

#### setScanDispPerc

Display signal strength as percentage instead of dBm.

```cpp
void setScanDispPerc(bool showPercent = false);
```

## Callback Methods

### setAPCallback

Called when Access Point mode starts.

```cpp
void setAPCallback(APCallback callback);
```

**Callback Signature:**
```cpp
typedef std::function<void(WiFiManager*)> APCallback;
```

**Example:**
```cpp
void onAPMode(WiFiManager* myWiFiManager) {
    printf("AP Mode started: %s\n", "MyDevice-AP");
    printf("IP: 192.168.4.1\n");
}

wifiManager.setAPCallback(onAPMode);
```

### setSaveConfigCallback

Called after successful WiFi connection.

```cpp
void setSaveConfigCallback(SaveConfigCallback callback);
```

**Callback Signature:**
```cpp
typedef std::function<void()> SaveConfigCallback;
```

**Example:**
```cpp
void onSaveConfig() {
    printf("Configuration saved!\n");
    // Save custom parameters here
}

wifiManager.setSaveConfigCallback(onSaveConfig);
```

### setConfigModeCallback

Called when entering configuration mode.

```cpp
void setConfigModeCallback(ConfigModeCallback callback);
```

**Callback Signature:**
```cpp
typedef std::function<void(WiFiManager*)> ConfigModeCallback;
```

### setWebServerModeCallback

Called when web server starts/stops.

```cpp
void setWebServerModeCallback(WebServerModeCallback callback);
```

## Information Methods

### getSSID

Get currently connected WiFi SSID.

```cpp
std::string getSSID() const;
```

**Returns:** SSID string, empty if not connected

### getLastConxResult

Get result of last connection attempt.

```cpp
wl_status_t getLastConxResult() const;
```

**Returns:** Connection status (see `wl_status_t` enum)

### getWiFiIsSaved

Check if WiFi credentials are saved.

```cpp
bool getWiFiIsSaved() const;
```

**Returns:** `true` if credentials exist, `false` otherwise

### getModeString

Get human-readable string for WiFi mode.

```cpp
const char* getModeString(wifi_mode_t mode) const;
```

**Parameters:**
- `mode` - WiFi mode enum

**Returns:** Mode string ("STA", "AP", "STA+AP", etc.)

### Status Check Methods

```cpp
bool isConfigPortalActive() const;  // Portal running?
bool isWebPortalActive() const;     // Web server running?
```

## Manual Control Methods

### startWebPortal / stopWebPortal

Manual web portal control.

```cpp
void startWebPortal();
void stopWebPortal();
```

### stopServers

Stop HTTP and DNS servers manually.

```cpp
void stopServers();
```

**Use Case:** Free memory after successful connection

**Example:**
```cpp
if (wifiManager.autoConnect()) {
    printf("Connected! Stopping servers to save memory...\n");
    wifiManager.stopServers();
}
```

### resetSettings

Clear saved WiFi credentials.

```cpp
bool resetSettings();
```

**Returns:** `true` if successful, `false` on error

**Example:**
```cpp
if (wifiManager.resetSettings()) {
    printf("WiFi settings cleared. Restarting...\n");
    esp_restart();
}
```

### process

Process requests in non-blocking mode.

```cpp
bool process();
```

**Returns:** `true` if still processing, `false` if completed

**Usage:** Call regularly (every 100ms) in non-blocking mode

## Parameter Management

### addParameter

Add custom parameter to configuration portal.

```cpp
void addParameter(WiFiManagerParameter* parameter);
```

**Example:**
```cpp
WiFiManagerParameter serverParam("server", "API Server", "api.example.com", 40);
wifiManager.addParameter(&serverParam);

// After connection, get value:
printf("Server: %s\n", serverParam.getValue());
```

### WiFiManagerParameter Class

Constructor for custom parameters:

```cpp
WiFiManagerParameter(const char* id, const char* placeholder, const char* defaultValue, int length);
```

**Parameters:**
- `id` - Parameter ID (used as form name)
- `placeholder` - Display label
- `defaultValue` - Default value
- `length` - Maximum length

**Methods:**
```cpp
const char* getID() const;        // Get parameter ID
const char* getValue() const;     // Get current value
const char* getLabel() const;     // Get display label
void setValue(const char* value); // Set new value
```

## Types and Enums

### wl_status_t

WiFi connection status values:

```cpp
typedef enum {
    WL_NO_SHIELD = 255,
    WL_IDLE_STATUS = 0,
    WL_NO_SSID_AVAIL = 1,
    WL_SCAN_COMPLETED = 2,
    WL_CONNECTED = 3,
    WL_CONNECT_FAILED = 4,
    WL_CONNECTION_LOST = 5,
    WL_DISCONNECTED = 6
} wl_status_t;
```

### wifi_mode_t

ESP-IDF WiFi modes:

```cpp
typedef enum {
    WIFI_MODE_NULL = 0,  // Disabled
    WIFI_MODE_STA,       // Station mode
    WIFI_MODE_AP,        // Access Point mode  
    WIFI_MODE_APSTA,     // Station + AP mode
    WIFI_MODE_MAX
} wifi_mode_t;
```

## Configuration Options (Kconfig)

Configure via `idf.py menuconfig` → Component config → WiFiManager:

| Option | Default | Description |
|--------|---------|-------------|
| `CONFIG_WM_DEFAULT_AP_SSID` | `"ESP-WiFiManager"` | Default AP SSID prefix |
| `CONFIG_WM_AP_IP` | `"192.168.4.1"` | AP IP address |
| `CONFIG_WM_AP_GW` | `"192.168.4.1"` | AP gateway address |
| `CONFIG_WM_AP_NETMASK` | `"255.255.255.0"` | AP netmask |
| `CONFIG_WM_HTTP_PORT` | `80` | HTTP server port |
| `CONFIG_WM_DNS_PORT` | `53` | DNS server port |
| `CONFIG_WM_MAX_PARAMS` | `20` | Maximum custom parameters |
| `CONFIG_WM_DEBUG` | `false` | Enable debug logging |

## Error Handling

Most methods return boolean success/failure. For detailed error information, check ESP-IDF logs:

```cpp
// Enable verbose WiFiManager logging
esp_log_level_set("WiFiManager", ESP_LOG_DEBUG);
```

## Thread Safety

WiFiManager uses internal mutexes for thread safety. All public methods are safe to call from any task.

## Memory Usage

| Component | RAM Usage (Active) | Notes |
|-----------|-------------------|--------|
| WiFiManager Instance | ~2KB | Core object |
| HTTP Server | ~25KB | When portal active |
| DNS Server | ~8KB | When portal active |
| Custom Parameters | ~100B each | User configurable |
| **Total Active** | **~35KB** | During configuration |
| **Total Idle** | **~2KB** | Normal operation |

## Performance Notes

- **Portal Response Time**: ~100-500ms typical
- **WiFi Scan Time**: 2-10 seconds depending on environment
- **Connection Time**: 5-30 seconds typical
- **Memory**: Portal uses ~35KB RAM when active
- **CPU Usage**: <1% when idle, ~5-10% when active

## Migration from Arduino WiFiManager

Most Arduino WiFiManager code works with minimal changes:

```cpp
// Arduino
#include <WiFiManager.h>
WiFiManager wifiManager;

// ESP-IDF (same!)
#include "WiFiManager.h"
WiFiManager wifiManager;
```

**Key Differences:**
- Use `ESP_LOGI()` instead of `Serial.print()`
- Use `std::string` instead of `String`
- Use `vTaskDelay()` instead of `delay()`
- Custom parameters need manual persistence (no automatic NVS save)

## Best Practices

1. **Memory Management**: Call `stopServers()` after successful connection
2. **Non-blocking**: Use `process()` for responsive applications
3. **Timeouts**: Set reasonable timeouts for your use case
4. **Error Handling**: Always check return values
5. **Custom Parameters**: Implement your own persistence layer
6. **Debugging**: Enable debug logs during development
7. **Testing**: Test with various WiFi environments and devices 