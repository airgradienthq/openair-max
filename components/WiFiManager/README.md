# WiFiManager for ESP-IDF

> Vibe code all the way through. But still need assist here and there to figure out problems/bugs. Reference: [tzapu/WiFiManager](https://github.com/tzapu/WiFiManager) 

---

[![ESP-IDF](https://img.shields.io/badge/ESP--IDF-5.x-blue.svg)](https://github.com/espressif/esp-idf)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)

A comprehensive WiFi configuration manager for ESP-IDF 5.x, providing an Arduino WiFiManager-compatible API with captive portal functionality.

## ✨ Features

- **🔄 Arduino WiFiManager API Compatibility** - Direct port with 1:1 API mapping
- **📱 Captive Portal** - Automatic redirection to configuration page
- **🎨 Modern Web UI** - Clean, responsive interface for all devices  
- **🔍 WiFi Scanning** - Live network discovery with signal strength
- **⚙️ Custom Parameters** - Add your own configuration fields
- **💾 Persistent Storage** - WiFi credentials saved automatically
- **🚫 Non-blocking Operation** - Optional async mode with `process()`
- **🎯 Manual Control** - Full control over portal lifecycle
- **🔒 Security** - WPA/WPA2/WPA3 support with validation
- **📊 Diagnostics** - Connection status and debugging info

## 🚀 Quick Start

### Installation

1. Copy the `wifimanager` folder to your project's `components/` directory
2. Add to your `CMakeLists.txt`:
```cmake
# In your main component's CMakeLists.txt
set(COMPONENT_REQUIRES wifimanager)
```

### Basic Usage

```cpp
#include "WiFiManager.h"

void app_main() {
    WiFiManager wifiManager;
    
    // Try to connect with saved credentials or start captive portal
    if (wifiManager.autoConnect("MyDevice-AP")) {
        printf("✅ Connected to WiFi!\n");
        // Your application code here
    } else {
        printf("❌ Failed to connect\n");
    }
}
```

## 📖 Examples

| Example | Description |
|---------|-------------|
| [**basic**](examples/basic/) | Simple autoConnect usage |
| [**advanced**](examples/advanced/) | Custom parameters, callbacks, timeouts |
| [**non_blocking**](examples/non_blocking/) | Async operation with `process()` |
| [**custom_html**](examples/custom_html/) | Custom styling and branding |

## 🔧 API Reference

### Core Methods

#### `bool autoConnect(const char* apName = nullptr, const char* apPassword = nullptr)`
Attempts to connect using saved credentials. If no credentials exist or connection fails, starts captive portal.

**Parameters:**
- `apName` - Access Point name (default: auto-generated)
- `apPassword` - Access Point password (default: open)

**Returns:** `true` if connected to WiFi, `false` on failure

---

#### `bool startConfigPortal(const char* apName = nullptr, const char* apPassword = nullptr)`
Manually start the configuration portal without attempting saved credentials.

---

#### `void stopServers()`
Manually stop HTTP and DNS servers. Useful for memory cleanup after successful connection.

---

#### `bool resetSettings()`  
Clear saved WiFi credentials. Device will need reconfiguration on next boot.

### Configuration Methods

#### `void setConfigPortalTimeout(uint32_t seconds)`
Set timeout for captive portal (default: 180 seconds)

#### `void setConnectTimeout(uint32_t seconds)`  
Set timeout for WiFi connection attempts (default: 30 seconds)

#### `void setConfigPortalBlocking(bool shouldBlock)`
Set blocking/non-blocking mode (default: blocking)

#### `void addParameter(WiFiManagerParameter* parameter)`
Add custom configuration parameter to the portal

### Callback Methods

#### `void setAPCallback(APCallback callback)`
Called when AP mode starts

#### `void setSaveConfigCallback(SaveConfigCallback callback)`  
Called after successful WiFi connection

#### `void setConfigModeCallback(ConfigModeCallback callback)`
Called when entering config mode

### Information Methods

#### `std::string getSSID() const`
Get currently connected SSID

#### `wl_status_t getLastConxResult() const`
Get last connection attempt result

#### `bool getWiFiIsSaved() const`
Check if WiFi credentials are saved

## 🎛️ Configuration Options

Configure via `idf.py menuconfig` → Component config → WiFiManager

| Option | Default | Description |
|--------|---------|-------------|
| `CONFIG_WM_DEFAULT_AP_SSID` | `"ESP-WiFiManager"` | Default AP name prefix |
| `CONFIG_WM_AP_IP` | `"192.168.4.1"` | AP IP address |
| `CONFIG_WM_HTTP_PORT` | `80` | HTTP server port |
| `CONFIG_WM_MAX_PARAMS` | `20` | Max custom parameters |

## 🎨 Custom Parameters

Add your own configuration fields to the captive portal:

```cpp
// Create custom parameters
WiFiManagerParameter customField("server", "API Server", "api.example.com", 40);
WiFiManagerParameter customPort("port", "Port", "443", 6);

// Add to WiFiManager
wifiManager.addParameter(&customField);  
wifiManager.addParameter(&customPort);

// After connection, access values
if (wifiManager.autoConnect()) {
    printf("Server: %s\n", customField.getValue());
    printf("Port: %s\n", customPort.getValue());
    
    // Save to your preferred storage (NVS, SPIFFS, etc.)
}
```

## 🔄 Non-Blocking Operation

For applications that need to run other tasks during configuration:

```cpp
wifiManager.setConfigPortalBlocking(false);
wifiManager.autoConnect("MyDevice");

// Your main loop
while (true) {
    wifiManager.process();  // Handle portal requests
    
    // Your other tasks
    doOtherWork();
    
    vTaskDelay(pdMS_TO_TICKS(100));
}
```

## 🐛 Debugging

Enable debug logging:

```cpp
// In your main CMakeLists.txt
idf_component_get_property(wifimanager wifimanager COMPONENT_LIB)
target_compile_definitions(${wifimanager} PRIVATE WM_DEBUG=1)
```

Or via menuconfig: `Component config → WiFiManager → Enable debug logging`

## 🏗️ Architecture

```
┌─────────────────┐    ┌──────────────┐    ┌─────────────┐
│   Application   │────│ WiFiManager  │────│   ESP-IDF   │
└─────────────────┘    └──────────────┘    └─────────────┘
                              │
                    ┌─────────┼─────────┐
                    │         │         │
            ┌───────▼──┐ ┌────▼────┐ ┌──▼─────┐
            │HTTP Server│ │DNS Server│ │WiFi/NVS│
            └───────────┘ └─────────┘ └────────┘
```

## 🤝 Contributing

Contributions welcome! Please ensure:
- ESP-IDF 5.x compatibility
- API compatibility with Arduino WiFiManager
- Comprehensive testing
- Documentation updates

## 📄 License

MIT License - see [LICENSE](LICENSE) file for details.

## 🙏 Acknowledgments

- Based on [tzapu/WiFiManager](https://github.com/tzapu/WiFiManager) for Arduino
- Uses [ESP-IDF](https://github.com/espressif/esp-idf) framework
- Web UI inspired by modern captive portal designs

---

**Made with ❤️ for the ESP-IDF community** 
