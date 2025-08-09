# Getting Started with WiFiManager

A step-by-step guide to using the ESP-IDF WiFiManager component in your project.

## 📋 Prerequisites

- **ESP-IDF 5.x**: Download from [ESP-IDF GitHub](https://github.com/espressif/esp-idf)
- **ESP32 Hardware**: Any ESP32, ESP32-S2, ESP32-S3, or ESP32-C3 board
- **Development Environment**: VS Code with ESP-IDF extension, or command line

## 🚀 Installation

### Method 1: Direct Copy (Recommended)

1. **Copy the component** to your project:
   ```bash
   # In your ESP-IDF project root
   cp -r /path/to/wifimanager components/
   ```

2. **Add to your CMakeLists.txt**:
   ```cmake
   # In your main component's CMakeLists.txt
   idf_component_register(
       SRCS "main.c"
       INCLUDE_DIRS "."
       REQUIRES wifimanager  # Add this line
   )
   ```

### Method 2: Git Submodule

1. **Add as submodule**:
   ```bash
   cd components
   git submodule add <repository-url> wifimanager
   git submodule update --init --recursive
   ```

2. **Update your CMakeLists.txt** (same as Method 1)

## 🎯 Your First WiFiManager App

### Step 1: Create Basic main.c

```c
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "WiFiManager.h"

static const char* TAG = "main";

void app_main(void) {
    ESP_LOGI(TAG, "Starting WiFiManager Example");
    
    // Create WiFiManager instance
    WiFiManager wifiManager;
    
    // Attempt connection
    if (wifiManager.autoConnect("MyDevice-Setup")) {
        ESP_LOGI(TAG, "✅ Connected to WiFi: %s", wifiManager.getSSID().c_str());
        
        // Your application code here
        while (1) {
            ESP_LOGI(TAG, "💚 Application running...");
            vTaskDelay(pdMS_TO_TICKS(5000));
        }
    } else {
        ESP_LOGE(TAG, "❌ Failed to connect");
        esp_restart();
    }
}
```

### Step 2: Build and Flash

```bash
# Build the project
idf.py build

# Flash to your device
idf.py -p /dev/ttyUSB0 flash monitor
```

### Step 3: First Run Experience

1. **Flash completes** → Device restarts
2. **No saved WiFi** → Creates "MyDevice-Setup" access point
3. **Connect your phone/computer** to "MyDevice-Setup"
4. **Browser opens automatically** → Shows WiFi configuration page
5. **Select your WiFi** → Enter password → Submit
6. **Device connects** to your WiFi → Continues with your app

## 📱 Using the Captive Portal

### What You'll See

1. **Access Point List**: All nearby WiFi networks with signal strength
2. **WiFi Credentials**: SSID selection and password field
3. **Custom Parameters**: Any additional fields you've added
4. **Status Information**: Device info, IP address, etc.

### Expected Behavior

- **Automatic Redirect**: Any web request redirects to config page
- **Live Scanning**: Refresh button updates network list
- **Responsive Design**: Works on phones, tablets, computers
- **Progress Feedback**: Shows connection attempts and results

## ⚙️ Configuration Options

### Via menuconfig

```bash
idf.py menuconfig
# Navigate to: Component config → WiFiManager
```

**Available Options:**
- Default AP SSID prefix
- AP IP address and network settings
- HTTP and DNS server ports
- Maximum custom parameters
- Debug logging level

### Via Code

```cpp
// Timeouts
wifiManager.setConfigPortalTimeout(300);  // 5 minutes
wifiManager.setConnectTimeout(30);        // 30 seconds

// WiFi Scanning
wifiManager.setMinimumSignalQuality(8);   // Filter weak signals
wifiManager.setRemoveDuplicateAPs(true);  // Remove duplicate SSIDs

// Portal Behavior
wifiManager.setConfigPortalBlocking(false); // Non-blocking mode
```

## 🔧 Adding Custom Parameters

Add your own configuration fields to the portal:

```cpp
#include "WiFiManagerParameter.h"

// Create parameters
WiFiManagerParameter serverParam("server", "API Server", "api.example.com", 40);
WiFiManagerParameter portParam("port", "Port", "443", 6);

// Add to WiFiManager
wifiManager.addParameter(&serverParam);
wifiManager.addParameter(&portParam);

// After connection, access values
printf("Server: %s\n", serverParam.getValue());
printf("Port: %s\n", portParam.getValue());

// Save to your preferred storage (NVS, SPIFFS, etc.)
```

## 📞 Using Callbacks

React to WiFiManager events:

```cpp
// Called when AP mode starts
void onAPMode(WiFiManager* manager) {
    printf("🎯 Portal started: connect to MyDevice-Setup\n");
    printf("🌐 Open browser to: http://192.168.4.1\n");
}

// Called after successful configuration
void onSaveConfig() {
    printf("💾 WiFi configured successfully!\n");
    // Save custom parameters here
}

// Set callbacks
wifiManager.setAPCallback(onAPMode);
wifiManager.setSaveConfigCallback(onSaveConfig);
```

## 🚫 Non-Blocking Mode

For applications that need to run other tasks during configuration:

```cpp
// Enable non-blocking mode
wifiManager.setConfigPortalBlocking(false);
wifiManager.autoConnect("MyDevice");

// Main loop
while (1) {
    wifiManager.process();  // Handle portal requests
    
    // Your other tasks
    updateLEDs();
    readSensors();
    handleButtons();
    
    vTaskDelay(pdMS_TO_TICKS(100));
}
```

## 🔄 Reset and Manual Control

### Reset WiFi Settings

```cpp
// Clear saved credentials
if (wifiManager.resetSettings()) {
    printf("Settings cleared, restarting...\n");
    esp_restart();
}
```

### Manual Portal Control

```cpp
// Start portal manually (without autoConnect)
if (buttonPressed()) {
    wifiManager.startConfigPortal("Manual-Setup");
}

// Stop servers to save memory
if (wifiManager.autoConnect()) {
    wifiManager.stopServers(); // Frees ~35KB RAM
}
```

## 🐛 Debugging

### Enable Debug Logging

```cpp
// In your main.c
esp_log_level_set("WiFiManager", ESP_LOG_DEBUG);
```

### Common Issues and Solutions

#### 1. "Component not found"
```
ERROR: Unknown component 'wifimanager'
```
**Solution**: Ensure `wifimanager` folder is in `components/` directory

#### 2. "Portal not appearing"
```
I (1234) WiFiManager: AP started but no captive portal
```
**Solution**: Check your device's WiFi - connect to the AP manually and go to `192.168.4.1`

#### 3. "Connection timeout"
```
W (5678) WiFiManager: Connection timeout after 30 seconds
```
**Solution**: Verify WiFi password, check signal strength, increase timeout:
```cpp
wifiManager.setConnectTimeout(60); // 60 seconds
```

#### 4. "Memory allocation failed"
```
E (9999) WiFiManager: Failed to start HTTP server
```
**Solution**: Ensure sufficient free heap memory (>50KB recommended)

### Debug Output Example

```
I (1234) WiFiManager: 🚀 Initializing WiFiManager
I (1234) WiFiManager: 📡 Starting WiFi subsystem
I (1234) WiFiManager: 🔍 Checking for saved credentials
I (1234) WiFiManager: 📱 No saved WiFi, starting portal
I (1234) WiFiManager: 🌐 AP started: MyDevice-Setup
I (1234) WiFiManager: 🌍 DNS server started on port 53
I (1234) WiFiManager: 📄 HTTP server started on port 80
I (1234) WiFiManager: 💡 Portal ready: http://192.168.4.1
```

## 📚 Next Steps

### Explore Examples

1. **[Basic Example](../examples/basic/)**: Simple autoConnect usage
2. **[Advanced Example](../examples/advanced/)**: Custom parameters and callbacks
3. **[Non-Blocking Example](../examples/non_blocking/)**: Async operation
4. **[Custom HTML Example](../examples/custom_html/)**: UI customization

### Learn More

- **[API Reference](API.md)**: Complete method documentation
- **[Component README](../README.md)**: Feature overview and examples
- **[ESP-IDF Documentation](https://docs.espressif.com/projects/esp-idf/)**: ESP-IDF framework docs

### Integration Ideas

- **IoT Sensors**: Add sensor readings to the portal
- **LED Status**: Show WiFi status with LEDs
- **Button Control**: Use buttons for reset/manual portal
- **Display Integration**: Show status on OLED/LCD screens
- **OTA Updates**: Combine with over-the-air update system

## 🎉 Success!

You now have a fully functional WiFi configuration system! Your device will:

1. **✅ Auto-connect** to saved WiFi on boot
2. **📱 Start captive portal** if no WiFi configured
3. **🔄 Handle reconnection** if WiFi is lost
4. **⚙️ Accept custom parameters** for your application
5. **📊 Provide status information** for debugging

**Happy coding with ESP-IDF WiFiManager!** 🚀 